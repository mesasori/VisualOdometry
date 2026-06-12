"""Event-based live-визуализатор VO-прогона, встраиваемый в Tk."""
from __future__ import annotations

import time
import tkinter as tk
from tkinter import ttk
from typing import List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("TkAgg")  # noqa: E402  (важно до import pyplot/figure)

import numpy as np  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from src.vo.interface import FrameData, Pose  # noqa: E402


_FRAME_FALLBACK_HEIGHT = 376
_FRAME_FALLBACK_WIDTH = 1241


# Перевод коротких статус-кодов Runner на русский — только для отображения
# в overlay-надписях; во внутренней логике статусы остаются английскими
# (см. контракт src/eval/runner.py).
_STATUS_RU = {
    "ok": "ок",
    "lost": "потеряно",
    "cancelled": "остановлено",
    "done": "готово",
    "failed": "сбой",
    "init": "инициализация",
}


def _ru_status(status: object) -> str:
    if isinstance(status, str):
        return _STATUS_RU.get(status, status)
    return "?"


def _xs_zs(poses: Sequence[Pose]) -> Tuple[List[float], List[float]]:
    """Top-down view координаты (X, Z) — стандарт KITTI: forward = +Z."""
    xs = [float(p.t[0, 0]) for p in poses]
    zs = [float(p.t[2, 0]) for p in poses]
    return xs, zs


class LiveVisualizer(ttk.Frame):
    """Tk-фрейм с встроенным matplotlib-холстом для live-режима VO-прогона.

    Жизненный цикл:
        viz = LiveVisualizer(parent, show_frame=True)
        viz.pack(...)
        viz.reset("kitti_odometry_05", gt_poses=gt, n_frames_hint=2761)
        # из worker'a:
        viz.push_frame(frame_data, pose, status)   # на каждом кадре
        viz.set_metrics({"success_rate": 0.98, "ms_per_frame": 17.3})
        viz.freeze()  # после окончания — финальная отрисовка

    Безопасность потоков: LiveVisualizer **не thread-safe**. Все методы
    должны вызываться из главного Tk-потока (внутри poll-callback'а
    chooser'а). Worker кладёт данные в очередь, polling забирает и зовёт
    push_frame/set_metrics здесь.
    """

    def __init__(
        self,
        parent: tk.Widget,
        show_frame: bool = True,
        min_redraw_interval_ms: int = 50,
        max_traj_points: Optional[int] = None,
    ) -> None:
        super().__init__(parent)
        self._show_frame = bool(show_frame)
        self._min_redraw_interval = max(1, int(min_redraw_interval_ms)) / 1000.0
        self._max_traj_points = max_traj_points

        if self._show_frame:
            self._fig = Figure(
                figsize=(12, 5), dpi=100, tight_layout=True, facecolor="white"
            )
            self._ax_frame = self._fig.add_subplot(1, 2, 1)
            self._ax_traj = self._fig.add_subplot(1, 2, 2)
        else:
            self._fig = Figure(
                figsize=(8, 6), dpi=100, tight_layout=True, facecolor="white"
            )
            self._ax_frame = None
            self._ax_traj = self._fig.add_subplot(1, 1, 1)

        for _ax in (self._ax_frame, self._ax_traj):
            if _ax is not None:
                _ax.set_facecolor("white")

        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        tk_widget = self._canvas.get_tk_widget()
        tk_widget.configure(bg="white", highlightthickness=0)
        tk_widget.pack(fill=tk.BOTH, expand=True)

        # state
        self._traj_xs: List[float] = []
        self._traj_zs: List[float] = []
        self._gt_xs: List[float] = []
        self._gt_zs: List[float] = []
        self._n_frames_hint = 0
        self._dataset_name = ""
        self._frame_count = 0
        self._last_redraw_t = 0.0
        self._lost_count = 0

        # mpl artists, создаются в reset()
        self._frame_image = None
        self._traj_line = None
        self._traj_point = None
        self._gt_line = None
        self._traj_info_text = None
        self._frame_info_text = None

        self._init_axes()

    # ------------------------------------------------------------------
    # Init / reset
    # ------------------------------------------------------------------
    def _init_axes(self) -> None:
        """Первичная отрисовка пустых осей + создание Line2D/AxesImage."""
        if self._ax_frame is not None:
            self._ax_frame.set_title("Текущий кадр")
            self._ax_frame.axis("off")
            dummy = np.zeros(
                (_FRAME_FALLBACK_HEIGHT, _FRAME_FALLBACK_WIDTH), dtype=np.uint8
            )
            self._frame_image = self._ax_frame.imshow(
                dummy, cmap="gray", vmin=0, vmax=255, aspect="auto"
            )
            self._frame_info_text = self._ax_frame.text(
                0.02,
                0.98,
                "",
                transform=self._ax_frame.transAxes,
                fontsize=10,
                verticalalignment="top",
                bbox=dict(
                    boxstyle="round", facecolor="white", edgecolor="#c8c8c8", alpha=0.95
                ),
            )

        ax = self._ax_traj
        ax.set_title("Траектория (вид сверху)")
        ax.set_xlabel("X, м")
        ax.set_ylabel("Z, м  (вперёд)")
        # adjustable="box" — matplotlib подгоняет axes box, а не data limits,
        # поэтому ручной set_xlim/set_ylim в _fit_limits не вызывает warning
        # "Ignoring fixed y limits to fulfill fixed data aspect".
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        (self._traj_line,) = ax.plot(
            [], [], "b-", linewidth=2, label="Оценка VO"
        )
        (self._traj_point,) = ax.plot(
            [], [], "ro", markersize=8, label="Текущая позиция"
        )
        (self._gt_line,) = ax.plot(
            [], [], "g--", linewidth=1.2, alpha=0.7, label="Эталон (GT)"
        )
        ax.legend(loc="best", fontsize=9, framealpha=0.95, facecolor="white")
        self._traj_info_text = ax.text(
            0.02,
            0.02,
            "",
            transform=ax.transAxes,
            fontsize=9,
            verticalalignment="bottom",
            bbox=dict(
                boxstyle="round", facecolor="white", edgecolor="#c8c8c8", alpha=0.95
            ),
        )

    def reset(
        self,
        dataset_name: str,
        gt_poses: Optional[Sequence[Pose]] = None,
        n_frames_hint: int = 0,
        title: Optional[str] = None,
    ) -> None:
        """Сбрасывает внутреннее состояние и перерисовывает GT-кривую сразу.

        gt_poses — для отрисовки заранее (статичная зелёная пунктирная линия).
        n_frames_hint — сколько кадров будет в прогоне (для отрисовки
        полной GT-кривой; если меньше — урежется при первом push_frame).
        """
        self._traj_xs.clear()
        self._traj_zs.clear()
        self._dataset_name = dataset_name
        self._n_frames_hint = int(n_frames_hint)
        self._frame_count = 0
        self._lost_count = 0
        self._last_redraw_t = 0.0

        self._traj_line.set_data([], [])
        self._traj_point.set_data([], [])
        if gt_poses:
            self._gt_xs, self._gt_zs = _xs_zs(list(gt_poses))
            self._gt_line.set_data(self._gt_xs, self._gt_zs)
        else:
            self._gt_xs = []
            self._gt_zs = []
            self._gt_line.set_data([], [])

        if self._traj_info_text is not None:
            self._traj_info_text.set_text("")
        if self._frame_info_text is not None:
            self._frame_info_text.set_text("")

        ax = self._ax_traj
        ax_title = title or dataset_name
        ax.set_title(ax_title)
        if self._gt_xs and self._gt_zs:
            self._fit_limits()

        self._fig.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Push events from worker
    # ------------------------------------------------------------------
    def push_frame(
        self,
        frame: Optional[FrameData],
        pose: Pose,
        status: str,
    ) -> None:
        """Один кадр VO-прогона: обновить траекторию, опц. кадр, метрики."""
        self._frame_count += 1
        if status != "ok":
            self._lost_count += 1

        x = float(pose.t[0, 0])
        z = float(pose.t[2, 0])
        self._traj_xs.append(x)
        self._traj_zs.append(z)

        if (
            self._max_traj_points is not None
            and len(self._traj_xs) > self._max_traj_points
        ):
            self._traj_xs = self._traj_xs[-self._max_traj_points :]
            self._traj_zs = self._traj_zs[-self._max_traj_points :]

        if self._show_frame and frame is not None and frame.left is not None:
            img = frame.left
            if img.ndim == 3:
                img = img[..., 0]
            self._frame_image.set_data(img)
            self._frame_image.set_extent(
                [-0.5, img.shape[1] - 0.5, img.shape[0] - 0.5, -0.5]
            )

        now = time.perf_counter()
        if now - self._last_redraw_t < self._min_redraw_interval:
            return
        self._last_redraw_t = now
        self._redraw()

    def set_metrics(self, info: dict) -> None:
        """Записывает в overlay-текст бегущие метрики.

        Принимаемые ключи: success_rate (0..1), ms_per_frame, idx, total,
        ATE rmse и т.п. — выводим только те, что есть.
        """
        lines: List[str] = []
        if "idx" in info and "total" in info:
            total_repr = info["total"] if info["total"] and info["total"] > 0 else "?"
            lines.append(f"кадр {info['idx']} / {total_repr}")
        if "success_rate" in info:
            lines.append(f"успешных: {info['success_rate'] * 100:.1f}%")
        if "ms_per_frame" in info:
            lines.append(f"{info['ms_per_frame']:.1f} мс/кадр")
        if self._traj_xs:
            x_now, z_now = self._traj_xs[-1], self._traj_zs[-1]
            lines.append(f"позиция: ({x_now:.1f}, {z_now:.1f}) м")
        if self._traj_info_text is not None:
            self._traj_info_text.set_text("\n".join(lines))

        if self._frame_info_text is not None and "status" in info:
            self._frame_info_text.set_text(
                f"кадр {info.get('idx', '?')}\n"
                f"статус: {_ru_status(info.get('status'))}"
            )

    def freeze(self) -> None:
        """Финальная отрисовка после окончания прогона.

        Гарантирует, что последний кадр и метрики отрисованы, даже если
        throttling «съел» последний push.
        """
        self._redraw()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _redraw(self) -> None:
        if self._traj_xs:
            self._traj_line.set_data(self._traj_xs, self._traj_zs)
            self._traj_point.set_data([self._traj_xs[-1]], [self._traj_zs[-1]])
            self._fit_limits()
        self._fig.canvas.draw_idle()

    def _fit_limits(self) -> None:
        """Подгоняет xlim/ylim под текущие данные с небольшим запасом."""
        all_x = list(self._traj_xs) + list(self._gt_xs)
        all_z = list(self._traj_zs) + list(self._gt_zs)
        if not all_x or not all_z:
            return
        min_x, max_x = min(all_x), max(all_x)
        min_z, max_z = min(all_z), max(all_z)
        pad_x = max(10.0, (max_x - min_x) * 0.1)
        pad_z = max(10.0, (max_z - min_z) * 0.1)
        self._ax_traj.set_xlim(min_x - pad_x, max_x + pad_x)
        self._ax_traj.set_ylim(min_z - pad_z, max_z + pad_z)
