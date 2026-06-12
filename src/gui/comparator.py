"""Comparator — наложение N сохранённых VO-траекторий поверх GT."""
from __future__ import annotations

import json
import re
import tkinter as tk
from collections import defaultdict
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np

from src.datasets import DATASETS, import_all_datasets
from src.eval.metrics import (
    compute_ate,
    compute_drift,
    compute_rpe,
    load_kitti_trajectory,
)
from src.vo.interface import Pose


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RESULTS_DIR = PROJECT_ROOT / "results"
TRAJECTORIES_DIR = RESULTS_DIR / "trajectories"
METRICS_DIR = RESULTS_DIR / "metrics"
KITTI_DIR = PROJECT_ROOT / "kitti_data"


# ---------------------------------------------------------------------------
# Парсинг имени файла траектории и dataset-key → конкретный Dataset
# ---------------------------------------------------------------------------
def split_algo_dataset(stem: str) -> Tuple[str, str]:
    """`A4__kitti_raw_2011_09_30_0018` → ('A4', 'kitti_raw_2011_09_30_0018')."""
    if "__" not in stem:
        raise ValueError(f"Не удаётся распарсить имя траектории: {stem!r}")
    algo, _, dataset = stem.partition("__")
    return algo, dataset


def dataset_key_for(dataset_name: str) -> Optional[str]:
    """`kitti_raw_2011_09_30_0018` → 'kitti_raw'; неизвестный → None."""
    if dataset_name.startswith("kitti_raw_"):
        return "kitti_raw"
    if dataset_name.startswith("kitti_odometry_"):
        return "kitti_odometry"
    return None


_RAW_RE = re.compile(r"^kitti_raw_(\d{4}_\d{2}_\d{2})_(\d{4})(?:__cancelled)?$")
_ODO_RE = re.compile(r"^kitti_odometry_(\d{2})(?:__cancelled)?$")


def make_dataset_for(dataset_name: str):
    """Возвращает сконфигурированный Dataset для подгрузки GT в comparator.

    Поддерживает только то, что зарегистрировано в [src/datasets/](../datasets/):
    kitti_odometry и kitti_raw. Возвращает None для неизвестных
    (например, кастомных тегов).
    """
    import_all_datasets()
    m = _RAW_RE.match(dataset_name)
    if m:
        ds_cls = DATASETS.get("kitti_raw")
        if ds_cls is None:
            return None
        date, drive = m.group(1), m.group(2)
        return ds_cls(str(KITTI_DIR / "raw"), date, drive)
    m = _ODO_RE.match(dataset_name)
    if m:
        ds_cls = DATASETS.get("kitti_odometry")
        if ds_cls is None:
            return None
        return ds_cls(str(KITTI_DIR), m.group(1))
    return None


# ---------------------------------------------------------------------------
# Базовые рендер-утилиты
# ---------------------------------------------------------------------------
def _xs_zs(poses: Sequence[Pose]) -> Tuple[List[float], List[float]]:
    xs = [float(p.t[0, 0]) for p in poses]
    zs = [float(p.t[2, 0]) for p in poses]
    return xs, zs


def _try_align(traj: List[Pose], gt: List[Pose]) -> Tuple[List[Pose], bool]:
    """Umeyama-выравнивание `traj` под `gt` (через evo PosePath3D.align).

    Возвращает (выровненные позы, True), если получилось; (исходные, False) —
    если evo не смог (вырожденная ковариация и т.п.).
    """
    if not traj or not gt:
        return traj, False
    try:
        from evo.core.trajectory import PosePath3D

        n = min(len(traj), len(gt))
        ref = PosePath3D(poses_se3=[p.to_matrix() for p in gt[:n]])
        est = PosePath3D(poses_se3=[p.to_matrix() for p in traj[:n]])
        est.align(ref, correct_scale=False)
        aligned = [Pose.from_matrix(T) for T in est.poses_se3]
        return aligned, True
    except Exception:  # noqa: BLE001
        return traj, False


def _format_metric_value(v) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if abs(v) < 1e-3 or abs(v) > 1e5:
            return f"{v:.3e}"
        return f"{v:.3f}"
    if isinstance(v, bool):
        return "True" if v else "False"
    return str(v)


def _load_metrics_for(traj_path: Path) -> Optional[dict]:
    """Грузит metrics.json, парный к траектории (тот же stem)."""
    metrics_path = METRICS_DIR / f"{traj_path.stem}.json"
    if not metrics_path.exists():
        return None
    try:
        return json.loads(metrics_path.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return None


# ---------------------------------------------------------------------------
# Plot N траекторий
# ---------------------------------------------------------------------------
def plot_overlay_multi(
    traj_paths: Sequence[str | Path],
    gt: Optional[Iterable[Pose]] = None,
    title: Optional[str] = None,
    align_to_gt: bool = True,
    block: bool = True,
    metrics_per_traj: Optional[Sequence[Optional[dict]]] = None,
) -> None:
    """Открывает matplotlib-окно с N траекториями VO поверх GT.

    Args:
        traj_paths: список путей к KITTI-форматам траекторий.
        gt: ground truth (общий для всех — все траектории должны быть на
            одной сцене).
        title: заголовок окна.
        align_to_gt: Umeyama-align каждой к GT перед рендером.
        block: блокировать поток до закрытия окна (True для CLI/GUI).
        metrics_per_traj: предзагруженные метрики для каждой траектории
            (тот же индекс). Если None — пересчитываются по траектории и GT.
    """
    if not traj_paths:
        raise ValueError("Нужна хотя бы одна траектория для отрисовки.")
    traj_paths = [Path(p) for p in traj_paths]
    for p in traj_paths:
        if not p.exists():
            raise FileNotFoundError(p)

    trajectories: List[List[Pose]] = [load_kitti_trajectory(p) for p in traj_paths]
    gt_list: Optional[List[Pose]] = list(gt) if gt is not None else None

    metrics_rows: List[List[str]] = []
    columns = [
        "Алгоритм",
        "ATE rmse, м",
        "ATE max, м",
        "RPE_t rmse, м",
        "Дрейф, м",
        "Успешно",
        "мс/кадр",
    ]

    fig, (ax_traj, ax_table) = plt.subplots(
        2,
        1,
        figsize=(11, 9),
        gridspec_kw={"height_ratios": [4, 1]},
        facecolor="white",
    )
    ax_traj.set_facecolor("white")
    ax_table.set_facecolor("white")
    fig.suptitle(title or "Наложение траекторий", fontsize=14, fontweight="bold")

    colors = plt.cm.tab10(np.linspace(0, 1, max(10, len(traj_paths))))

    if gt_list:
        gx, gz = _xs_zs(gt_list)
        ax_traj.plot(gx, gz, "k--", linewidth=1.4, alpha=0.7, label="Эталон (GT)")

    for i, (path, traj) in enumerate(zip(traj_paths, trajectories)):
        algo_name, _ = split_algo_dataset(path.stem)
        traj_for_plot = traj
        aligned_flag = False
        if align_to_gt and gt_list:
            traj_for_plot, aligned_flag = _try_align(traj, gt_list)

        xs, zs = _xs_zs(traj_for_plot)
        color = colors[i % len(colors)]
        label = path.stem
        ax_traj.plot(xs, zs, "-", linewidth=2, color=color, label=label)
        if xs and zs:
            ax_traj.plot(xs[0], zs[0], "o", color=color, markersize=6, markeredgecolor="black")
            ax_traj.plot(xs[-1], zs[-1], "s", color=color, markersize=7, markeredgecolor="black")

        # метрики
        m: Optional[dict]
        if metrics_per_traj is not None and i < len(metrics_per_traj):
            m = metrics_per_traj[i]
        else:
            m = None
        if m is None and gt_list:
            try:
                ate = compute_ate(traj, gt_list)
                rpe = compute_rpe(traj, gt_list, delta=1, relation="translation_part")
                drift = compute_drift(traj, gt_list)
                m = {
                    "ate": ate,
                    "rpe_translation_1": rpe,
                    "final_drift_m": drift,
                }
            except Exception:  # noqa: BLE001
                m = None

        ate = (m or {}).get("ate") or {}
        rpe = (m or {}).get("rpe_translation_1") or {}
        row = [
            algo_name,
            _format_metric_value(ate.get("rmse")),
            _format_metric_value(ate.get("max")),
            _format_metric_value(rpe.get("rmse")),
            _format_metric_value((m or {}).get("final_drift_m")),
            _format_metric_value(
                None if m is None else (m.get("success_rate") if "success_rate" in m else None)
            ),
            _format_metric_value(
                None if m is None else (m.get("time_per_frame_ms") if "time_per_frame_ms" in m else None)
            ),
        ]
        metrics_rows.append(row)

    ax_traj.set_xlabel("X, м")
    ax_traj.set_ylabel("Z, м  (вперёд)")
    ax_traj.set_aspect("equal", adjustable="box")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.legend(loc="best", fontsize=9, framealpha=0.95, facecolor="white")
    if align_to_gt and gt_list:
        ax_traj.set_title(
            "Выровнено по GT (Umeyama, без масштабирования)", fontsize=10
        )
    else:
        ax_traj.set_title("Без выравнивания (raw)", fontsize=10)

    # Таблица метрик
    ax_table.axis("off")
    if metrics_rows:
        table = ax_table.table(
            cellText=metrics_rows,
            colLabels=columns,
            loc="upper center",
            cellLoc="center",
            colLoc="center",
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.0, 1.4)
        # Принудительно белый фон ячеек таблицы — чтобы скриншоты для
        # пояснительной записки оставались на сплошном белом фоне.
        for (row_idx, _col_idx), cell in table.get_celld().items():
            cell.set_facecolor("white")
            cell.set_edgecolor("#c8c8c8")
            if row_idx == 0:
                cell.set_text_props(fontweight="bold")

    plt.tight_layout()
    plt.show(block=block)


# ---------------------------------------------------------------------------
# Backwards-compat: одна траектория + GT
# ---------------------------------------------------------------------------
def plot_overlay(
    traj_path: str | Path,
    gt: Optional[Iterable[Pose]] = None,
    title: Optional[str] = None,
    extra_metrics: Optional[dict] = None,
    block: bool = True,
) -> None:
    """Тонкая обёртка для одной траектории — оставлена для совместимости."""
    metrics = [extra_metrics] if extra_metrics is not None else None
    plot_overlay_multi(
        [traj_path],
        gt=gt,
        title=title or Path(traj_path).stem,
        align_to_gt=True,
        block=block,
        metrics_per_traj=metrics,
    )


# ---------------------------------------------------------------------------
# Discover saved trajectories + grouping
# ---------------------------------------------------------------------------
def discover_trajectories() -> Dict[str, List[Path]]:
    """Сканирует results/trajectories/ и группирует пути по dataset_name.

    Возвращает {dataset_name: [path_to_traj.txt, ...]}.
    """
    out: Dict[str, List[Path]] = defaultdict(list)
    if not TRAJECTORIES_DIR.is_dir():
        return out
    for p in sorted(TRAJECTORIES_DIR.glob("*.txt")):
        try:
            _, dataset = split_algo_dataset(p.stem)
        except ValueError:
            continue
        out[dataset].append(p)
    return out


# ---------------------------------------------------------------------------
# ComparatorTab — Treeview с группами + чек-боксы + overlay
# ---------------------------------------------------------------------------
class ComparatorTab(ttk.Frame):
    """Tk-фрейм со списком сохранённых траекторий, сгруппированных по сцене.

    Внутри одной сцены траектории можно отметить чек-боксами и наложить.
    Выбор из разных сцен запрещён (валидное сравнение требует общий GT).
    """

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, padding=(10, 8))
        self._check_vars: Dict[str, tk.BooleanVar] = {}
        self._item_to_path: Dict[str, Path] = {}
        self._build_ui()
        self.refresh()

    def _build_ui(self) -> None:
        ttk.Label(
            self,
            text="Сравнение траекторий",
            font=("TkDefaultFont", 13, "bold"),
        ).pack(anchor=tk.W, pady=(0, 4))
        ttk.Label(
            self,
            text=(
                "Отметьте несколько траекторий ОДНОЙ сцены и нажмите "
                "«Показать наложение». Чтобы переключить выбор траектории, "
                "кликните по галочке. Сравнение допустимо только в рамках "
                "одной сцены — иначе сравнивались бы разные дороги."
            ),
            foreground="#555555",
            wraplength=760,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(0, 8))

        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(toolbar, text="🔄 Обновить", command=self.refresh).pack(side=tk.LEFT)
        ttk.Button(
            toolbar,
            text="Снять выбор со всех",
            command=self._unselect_all,
        ).pack(side=tk.LEFT, padx=(6, 0))

        self.align_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            toolbar,
            text="Выровнять по GT (Umeyama)",
            variable=self.align_var,
        ).pack(side=tk.LEFT, padx=(20, 0))

        self.show_button = ttk.Button(
            toolbar,
            text="📊 Показать наложение",
            command=self._on_show,
        )
        self.show_button.pack(side=tk.RIGHT)

        tree_frame = ttk.Frame(self)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        self.tree = ttk.Treeview(
            tree_frame,
            columns=("sel", "file", "ate"),
            show="tree headings",
            selectmode="none",
        )
        self.tree.heading("#0", text="Группа / файл")
        self.tree.heading("sel", text="✓")
        self.tree.heading("file", text="Файл")
        self.tree.heading("ate", text="ATE rmse, м")
        self.tree.column("#0", width=320, anchor=tk.W)
        self.tree.column("sel", width=40, anchor=tk.CENTER, stretch=False)
        self.tree.column("file", width=260, anchor=tk.W)
        self.tree.column("ate", width=110, anchor=tk.E)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        vsb = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=vsb.set)

        self.tree.bind("<Button-1>", self._on_tree_click)

        self.status_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.status_var, foreground="#555555").pack(
            anchor=tk.W, pady=(6, 0)
        )

    # ------------------------------------------------------------------
    # Refresh / state
    # ------------------------------------------------------------------
    def refresh(self) -> None:
        """Перечитывает results/trajectories/, перестраивает Treeview."""
        for item in self.tree.get_children():
            self.tree.delete(item)
        self._check_vars.clear()
        self._item_to_path.clear()

        groups = discover_trajectories()
        if not groups:
            self.status_var.set(
                f"Нет сохранённых прогонов в {TRAJECTORIES_DIR.relative_to(PROJECT_ROOT)}/"
            )
            return

        n_files = 0
        for dataset in sorted(groups.keys()):
            group_id = self.tree.insert(
                "",
                tk.END,
                text=dataset,
                values=("", f"{len(groups[dataset])} файлов", ""),
                open=True,
            )
            for path in sorted(groups[dataset]):
                metrics = _load_metrics_for(path)
                ate_str = ""
                if metrics:
                    ate = metrics.get("ate") or {}
                    ate_str = _format_metric_value(ate.get("rmse"))
                var = tk.BooleanVar(value=False)
                item_id = self.tree.insert(
                    group_id,
                    tk.END,
                    text="",
                    values=("☐", path.name, ate_str),
                )
                self._check_vars[item_id] = var
                self._item_to_path[item_id] = path
                n_files += 1
        self.status_var.set(
            f"Найдено {n_files} файлов в {len(groups)} группах. "
            "По умолчанию траектории выравниваются по GT (Umeyama)."
        )

    def select_trajectory_by_path(self, path: Path) -> bool:
        """Поставить галку у траектории по её пути. True если найдено.

        Используется при auto-jump из chooser после прогона.
        """
        for item_id, p in self._item_to_path.items():
            if p == path:
                self._check_vars[item_id].set(True)
                self.tree.item(item_id, values=("☑", p.name, self.tree.item(item_id, "values")[2]))
                parent = self.tree.parent(item_id)
                if parent:
                    self.tree.item(parent, open=True)
                self.tree.see(item_id)
                return True
        return False

    def _unselect_all(self) -> None:
        for item_id, var in self._check_vars.items():
            var.set(False)
            current_vals = self.tree.item(item_id, "values")
            self.tree.item(item_id, values=("☐", current_vals[1], current_vals[2]))

    def _on_tree_click(self, event) -> None:
        """Перехват клика — на колонке `sel` тогглим чек-бокс."""
        region = self.tree.identify_region(event.x, event.y)
        if region != "cell":
            return
        col = self.tree.identify_column(event.x)
        item_id = self.tree.identify_row(event.y)
        if not item_id or item_id not in self._check_vars:
            return
        if col != "#1":  # колонка sel = #1 (после #0 = тree column)
            return
        var = self._check_vars[item_id]
        var.set(not var.get())
        current_vals = self.tree.item(item_id, "values")
        new_check = "☑" if var.get() else "☐"
        self.tree.item(item_id, values=(new_check, current_vals[1], current_vals[2]))

    # ------------------------------------------------------------------
    # Show overlay
    # ------------------------------------------------------------------
    def _collect_selection(self) -> List[Path]:
        return [
            self._item_to_path[item_id]
            for item_id, var in self._check_vars.items()
            if var.get()
        ]

    def _on_show(self) -> None:
        selected = self._collect_selection()
        if not selected:
            messagebox.showinfo(
                "Сравнение траекторий",
                "Не выбрано ни одной траектории. Отметьте хотя бы одну "
                "галочкой в столбце ✓.",
            )
            return

        datasets = {split_algo_dataset(p.stem)[1] for p in selected}
        # Нормализуем __cancelled-варианты — они принадлежат той же сцене,
        # что и обычная траектория
        normalized = {d.removesuffix("__cancelled") for d in datasets}
        if len(normalized) > 1:
            messagebox.showerror(
                "Сравнение траекторий",
                "Выбраны траектории разных сцен:\n  "
                + "\n  ".join(sorted(normalized))
                + "\n\nСравнивать можно только в рамках одной сцены "
                "(общий эталон GT).",
            )
            return

        dataset_name = next(iter(normalized))
        gt = None
        try:
            ds = make_dataset_for(dataset_name)
            if ds is not None:
                gt = ds.ground_truth()
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Не удалось загрузить GT: {exc}")

        metrics_per_traj = [_load_metrics_for(p) for p in selected]

        title = (
            f"Наложение траекторий × {dataset_name}  "
            f"({len(selected)} прогонов)"
        )
        try:
            plot_overlay_multi(
                selected,
                gt=gt,
                title=title,
                align_to_gt=bool(self.align_var.get()),
                block=False,
                metrics_per_traj=metrics_per_traj,
            )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror(
                "Сравнение траекторий", f"{type(exc).__name__}: {exc}"
            )


# ---------------------------------------------------------------------------
# Legacy shim: python -m src.gui.comparator → MainApp на вкладке comparator
# ---------------------------------------------------------------------------
def main() -> int:
    """Shortcut: python -m src.gui.comparator открывает MainApp с табом comparator."""
    from src.gui.main_app import MainApp

    app = MainApp(initial_tab="comparator")
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
