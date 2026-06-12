"""Chooser — конфигуратор одного прогона VO-алгоритма на датасете."""
from __future__ import annotations

import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable, List, Optional, Tuple

from src.algorithms import ALGORITHMS, import_all_algorithms
from src.datasets import DATASETS, import_all_datasets
from src.eval.runner import RunResult, Runner
from src.gui.live_visualizer import LiveVisualizer
from src.vo.interface import FrameData, Pose, is_compatible


PROJECT_ROOT = Path(__file__).resolve().parents[2]
KITTI_DIR = PROJECT_ROOT / "kitti_data"


# ---------------------------------------------------------------------------
# Discover available KITTI Odometry sequences и Raw drives
# ---------------------------------------------------------------------------
def discover_kitti_odometry_sequences() -> List[Tuple[str, int, bool]]:
    """Возвращает [(seq_name, n_frames, has_gt), ...]."""
    out: List[Tuple[str, int, bool]] = []
    seq_dir = KITTI_DIR / "sequences"
    poses_dir = KITTI_DIR / "poses"
    if not seq_dir.is_dir():
        return out
    for d in sorted(p for p in seq_dir.iterdir() if p.is_dir()):
        image_0 = d / "image_0"
        if not image_0.is_dir():
            continue
        n = sum(1 for f in image_0.iterdir() if f.suffix.lower() == ".png")
        if n == 0:
            continue
        has_gt = (poses_dir / f"{d.name}.txt").exists()
        out.append((d.name, n, has_gt))
    return out


def discover_kitti_raw_drives() -> List[Tuple[str, str, int]]:
    """Возвращает [(date, drive, n_frames), ...]."""
    out: List[Tuple[str, str, int]] = []
    raw_dir = KITTI_DIR / "raw"
    if not raw_dir.is_dir():
        return out
    for date_dir in sorted(p for p in raw_dir.iterdir() if p.is_dir()):
        for drive_dir in sorted(d for d in date_dir.iterdir() if d.is_dir()):
            if "_drive_" not in drive_dir.name or not drive_dir.name.endswith("_sync"):
                continue
            cam0 = drive_dir / "image_00" / "data"
            if not cam0.is_dir():
                continue
            n = sum(1 for f in cam0.iterdir() if f.suffix.lower() == ".png")
            if n == 0:
                continue
            drive_id = drive_dir.name.split("_drive_")[1].replace("_sync", "")
            out.append((date_dir.name, drive_id, n))
    return out


# ---------------------------------------------------------------------------
# Helpers: создать датасет по (key, sequence_or_drive)
# ---------------------------------------------------------------------------
def _make_dataset(ds_key: str, seq_key: str):
    """Создаёт сконфигурированный Dataset по ключу реестра и id сцены."""
    ds_cls = DATASETS[ds_key]
    if ds_key == "kitti_odometry":
        return ds_cls(str(KITTI_DIR), seq_key)
    if ds_key == "kitti_raw":
        date, drive = seq_key.split("/")
        return ds_cls(str(KITTI_DIR / "raw"), date, drive)
    raise ValueError(f"Unsupported dataset key: {ds_key}")


# ---------------------------------------------------------------------------
# Worker thread (Runner + progress + frame callback)
# ---------------------------------------------------------------------------
class _WorkerSignal:
    """Простейшая обёртка над queue для передачи прогресса и кадров между потоками.

    События в очереди (kind, payload):
      * ("progress", (idx, total, status))
      * ("frame", (frame_data, pose, status))   — для LiveVisualizer
      * ("log", str)
      * ("done", RunResult)
      * ("error", BaseException)
    """

    def __init__(self) -> None:
        self.q: "queue.Queue[Tuple[str, object]]" = queue.Queue()

    def progress(self, idx: int, total: int, status: str) -> None:
        self.q.put(("progress", (idx, total, status)))

    def frame(self, frame_data: FrameData, pose: Pose, status: str) -> None:
        self.q.put(("frame", (frame_data, pose, status)))

    def log(self, msg: str) -> None:
        self.q.put(("log", msg))

    def done(self, payload: object) -> None:
        self.q.put(("done", payload))

    def error(self, exc: BaseException) -> None:
        self.q.put(("error", exc))


def _worker(
    algo_key: str,
    dataset_key: str,
    sequence_key: str,
    max_frames: Optional[int],
    seed: int,
    cancel_event: threading.Event,
    push_frames: bool,
    signal: _WorkerSignal,
) -> None:
    try:
        # Идемпотентно — реестры заполняются один раз через декораторы
        import_all_algorithms()
        import_all_datasets()
        algo_cls = ALGORITHMS[algo_key]
        algo = algo_cls()
        dataset = _make_dataset(dataset_key, sequence_key)

        signal.log(
            f"алгоритм={algo_key}  датасет={dataset_key}  "
            f"сцена={sequence_key}  макс_кадров={max_frames}  "
            f"seed={seed}\n"
        )

        runner = Runner(seed=seed)

        def on_progress(idx: int, total: int, _pose: Pose, status: str) -> None:
            signal.progress(idx, total, status)

        def on_frame(frame: FrameData, pose: Pose, status: str) -> None:
            signal.frame(frame, pose, status)

        frame_cb = on_frame if push_frames else None
        result = runner.run(
            algo,
            dataset,
            max_frames=max_frames,
            progress_callback=on_progress,
            cancel_event=cancel_event,
            frame_callback=frame_cb,
        )
        signal.done(result)
    except Exception as exc:  # noqa: BLE001
        signal.error(exc)


# ---------------------------------------------------------------------------
# ChooserTab — основной фрейм для вкладки «Запустить прогон»
# ---------------------------------------------------------------------------
class ChooserTab(ttk.Frame):
    """Tk-фрейм с конфигуратором прогона и live-визуализацией.

    Встраивается в [main_app.MainApp](main_app.py) как одна из вкладок
    Notebook. Снаружи можно передать `on_run_finished(result)` коллбэк,
    чтобы родитель показал диалог «Открыть в Comparator?» и переключил
    вкладку. Если коллбэк не задан, ChooserTab после прогона ничего
    дополнительно не делает (полезно для standalone-режима, который
    приходит из python -m src.gui.chooser).
    """

    POLL_MS = 50

    def __init__(
        self,
        parent: tk.Widget,
        on_run_finished: Optional[Callable[[RunResult], None]] = None,
    ) -> None:
        super().__init__(parent)
        self._on_run_finished = on_run_finished

        import_all_algorithms()
        import_all_datasets()

        self._kitti_odo = discover_kitti_odometry_sequences()
        self._kitti_raw = discover_kitti_raw_drives()

        self.algo_var = tk.StringVar()
        self.dataset_var = tk.StringVar()
        self.sequence_var = tk.StringVar()
        self.max_frames_var = tk.StringVar(value="")
        self.seed_var = tk.StringVar(value="42")
        self.show_frame_var = tk.BooleanVar(value=True)

        self._signal: Optional[_WorkerSignal] = None
        self._worker_thread: Optional[threading.Thread] = None
        self._cancel_event: Optional[threading.Event] = None
        self._last_result: Optional[RunResult] = None
        self._sequence_keys: List[str] = []
        self._sequence_labels: List[str] = []

        self._build_ui()
        self._on_dataset_change()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        # Левая колонка — форма с параметрами; правая — LiveVisualizer.
        # Layout: PanedWindow с двумя фреймами.
        paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(paned, padding=(12, 8))
        right = ttk.Frame(paned, padding=(8, 8))
        paned.add(left, weight=1)
        paned.add(right, weight=3)

        ttk.Label(
            left,
            text="Стенд исследования визуальной одометрии",
            font=("TkDefaultFont", 13, "bold"),
        ).pack(anchor=tk.W, pady=(0, 4))
        ttk.Label(
            left,
            text=(
                "Выберите алгоритм, датасет и последовательность —\n"
                "и нажмите «Запустить». Живая визуализация появится справа."
            ),
            foreground="#555555",
            wraplength=320,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(0, 10))

        form = ttk.LabelFrame(left, text="Параметры прогона", padding=10)
        form.pack(fill=tk.X)

        dataset_keys = [k for k in DATASETS.keys() if k in ("kitti_odometry", "kitti_raw")]
        if not dataset_keys:
            dataset_keys = ["<нет датасетов>"]
        if not self.dataset_var.get():
            self.dataset_var.set(dataset_keys[0])
        ttk.Label(form, text="Датасет:").grid(
            row=0, column=0, sticky=tk.W, padx=(0, 8), pady=4
        )
        self.dataset_combo = ttk.Combobox(
            form,
            textvariable=self.dataset_var,
            values=dataset_keys,
            state="readonly",
            width=22,
        )
        self.dataset_combo.grid(row=0, column=1, sticky=tk.W, pady=4)
        self.dataset_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_dataset_change())

        ttk.Label(form, text="Алгоритм:").grid(
            row=1, column=0, sticky=tk.W, padx=(0, 8), pady=4
        )
        self.algo_combo = ttk.Combobox(
            form, textvariable=self.algo_var, state="readonly", width=22
        )
        self.algo_combo.grid(row=1, column=1, sticky=tk.W, pady=4)

        ttk.Label(form, text="Последовательность / заезд:").grid(
            row=2, column=0, sticky=tk.W, padx=(0, 8), pady=4
        )
        self.sequence_combo = ttk.Combobox(
            form, textvariable=self.sequence_var, state="readonly", width=42
        )
        self.sequence_combo.grid(row=2, column=1, sticky=tk.W, pady=4)

        ttk.Label(form, text="Макс. кадров:").grid(
            row=3, column=0, sticky=tk.W, padx=(0, 8), pady=4
        )
        ttk.Entry(form, textvariable=self.max_frames_var, width=10).grid(
            row=3, column=1, sticky=tk.W, pady=4
        )

        ttk.Label(form, text="Зерно генератора (seed):").grid(
            row=4, column=0, sticky=tk.W, padx=(0, 8), pady=4
        )
        ttk.Entry(form, textvariable=self.seed_var, width=10).grid(
            row=4, column=1, sticky=tk.W, pady=4
        )

        ttk.Checkbutton(
            form,
            text="Показывать кадр (выкл. — быстрее на длинных прогонах)",
            variable=self.show_frame_var,
        ).grid(row=5, column=0, columnspan=2, sticky=tk.W, pady=(8, 0))

        bar = ttk.Frame(left, padding=(0, 10, 0, 0))
        bar.pack(fill=tk.X)
        self.run_button = ttk.Button(bar, text="▶ Запустить", command=self._on_run)
        self.run_button.pack(side=tk.LEFT)
        self.cancel_button = ttk.Button(
            bar, text="⏹ Остановить", command=self._on_cancel, state=tk.DISABLED
        )
        self.cancel_button.pack(side=tk.LEFT, padx=(8, 0))

        self.progress_var = tk.IntVar(value=0)
        ttk.Progressbar(
            left, variable=self.progress_var, mode="determinate", maximum=100
        ).pack(fill=tk.X, pady=(8, 4))

        log_frame = ttk.LabelFrame(left, text="Журнал прогона", padding=6)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        self.log_text = tk.Text(
            log_frame,
            height=14,
            wrap=tk.WORD,
            font=("TkFixedFont", 9),
            background="#ffffff",
            foreground="#1a1a1a",
            insertbackground="#1a1a1a",
            relief="solid",
            borderwidth=1,
            highlightthickness=0,
        )
        self.log_text.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scroll.set, state=tk.DISABLED)

        # right pane — LiveVisualizer
        self.live = LiveVisualizer(right, show_frame=True)
        self.live.pack(fill=tk.BOTH, expand=True)

        # default algo выбор после того, как мы знаем dataset → отфильтрованный список
        self._refresh_algorithms()

    def _on_dataset_change(self) -> None:
        self._refresh_algorithms()
        self._refresh_sequences()
        self.show_frame_var.set(True)

    def _refresh_algorithms(self) -> None:
        """Перестраивает список Algorithm под выбранный Dataset.

        Фильтрация через is_compatible(req, caps) из src/vo/interface.py —
        ровно та же логика, что использует Runner для финальной валидации.
        """
        ds_key = self.dataset_var.get()
        keys: List[str] = []
        try:
            ds = _make_dataset(ds_key, self._first_seq_for(ds_key))
            caps = ds.capabilities()
        except Exception:  # noqa: BLE001
            caps = None

        for key, algo_cls in ALGORITHMS.items():
            try:
                req = algo_cls().requirements()
            except Exception:  # noqa: BLE001
                continue
            if caps is None or is_compatible(req, caps):
                keys.append(key)

        if not keys:
            keys = ["<нет совместимых алгоритмов>"]
        self.algo_combo["values"] = keys
        if self.algo_var.get() not in keys:
            self.algo_var.set(keys[0])

    def _first_seq_for(self, ds_key: str) -> str:
        """Возвращает первый доступный seq_key для probe capabilities."""
        if ds_key == "kitti_odometry" and self._kitti_odo:
            return self._kitti_odo[0][0]
        if ds_key == "kitti_raw" and self._kitti_raw:
            d, dr, _ = self._kitti_raw[0]
            return f"{d}/{dr}"
        raise ValueError(f"no sequences available for {ds_key}")

    def _refresh_sequences(self) -> None:
        ds_key = self.dataset_var.get()
        if ds_key == "kitti_odometry":
            values = [
                f"{name}  ({n} кадров{', есть GT' if has_gt else ''})"
                for name, n, has_gt in self._kitti_odo
            ]
            keys = [name for name, _, _ in self._kitti_odo]
        elif ds_key == "kitti_raw":
            values = [
                f"{date}/drive_{drive}  ({n} кадров)"
                for date, drive, n in self._kitti_raw
            ]
            keys = [f"{date}/{drive}" for date, drive, _ in self._kitti_raw]
        else:
            values, keys = [], []

        self.sequence_combo["values"] = values
        self._sequence_keys = keys
        self._sequence_labels = values
        if values:
            self.sequence_combo.current(0)
        else:
            self.sequence_var.set("")

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    def _log(self, msg: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg if msg.endswith("\n") else msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------
    def _resolve_sequence_key(self) -> str:
        label = self.sequence_var.get()
        if not label:
            raise ValueError("Не выбрана последовательность.")
        try:
            idx = self._sequence_labels.index(label)
        except ValueError as exc:
            raise ValueError(
                f"Не понимаю выбранную последовательность: {label!r}"
            ) from exc
        return self._sequence_keys[idx]

    def _on_run(self) -> None:
        try:
            algo_key = self.algo_var.get()
            ds_key = self.dataset_var.get()
            seq_key = self._resolve_sequence_key()
            raw_max = self.max_frames_var.get().strip()
            max_frames = int(raw_max) if raw_max else None
            raw_seed = self.seed_var.get().strip()
            seed = int(raw_seed) if raw_seed else 42
            if algo_key not in ALGORITHMS:
                raise ValueError(f"Алгоритм {algo_key!r} не зарегистрирован.")
            if ds_key not in DATASETS:
                raise ValueError(f"Датасет {ds_key!r} не зарегистрирован.")
        except ValueError as exc:
            messagebox.showerror("Ошибка ввода", str(exc))
            return

        # подготовим LiveVisualizer: пересоберём с актуальным show_frame
        self._rebuild_live(self.show_frame_var.get())

        # подгрузим GT заранее, чтобы LiveVisualizer показал зелёный пунктир сразу
        gt_poses = None
        try:
            ds_probe = _make_dataset(ds_key, seq_key)
            gt_poses = ds_probe.ground_truth()
            if gt_poses and max_frames is not None:
                gt_poses = gt_poses[:max_frames]
        except Exception as exc:  # noqa: BLE001
            self._log(f"  Предзагрузка GT не удалась: {exc}")

        self.live.reset(
            dataset_name=f"{algo_key} × {ds_key}/{seq_key}",
            gt_poses=gt_poses,
            n_frames_hint=(max_frames or 0),
            title=f"{algo_key} on {ds_key}/{seq_key}",
        )

        self.run_button.configure(state=tk.DISABLED)
        self.cancel_button.configure(state=tk.NORMAL)
        self.dataset_combo.configure(state=tk.DISABLED)
        self.algo_combo.configure(state=tk.DISABLED)
        self.sequence_combo.configure(state=tk.DISABLED)
        self.progress_var.set(0)
        self._log("=" * 60)
        self._log(f"Запуск: {algo_key} × {ds_key} × {seq_key}  (seed={seed})")
        if max_frames:
            self._log(f"Макс. кадров: {max_frames}")

        self._signal = _WorkerSignal()
        self._cancel_event = threading.Event()
        self._worker_thread = threading.Thread(
            target=_worker,
            args=(
                algo_key,
                ds_key,
                seq_key,
                max_frames,
                seed,
                self._cancel_event,
                bool(self.show_frame_var.get()),
                self._signal,
            ),
            daemon=True,
        )
        self._worker_thread.start()
        self.after(self.POLL_MS, self._poll_signal)

    def _rebuild_live(self, show_frame: bool) -> None:
        """Пересобирает LiveVisualizer, если изменился флаг show_frame.

        FigureCanvasTkAgg не поддерживает динамическое
        добавление/удаление subplot'ов — проще пересобрать виджет.
        """
        if getattr(self.live, "_show_frame", None) == bool(show_frame):
            return
        parent = self.live.master
        self.live.destroy()
        self.live = LiveVisualizer(parent, show_frame=bool(show_frame))
        self.live.pack(fill=tk.BOTH, expand=True)

    def _on_cancel(self) -> None:
        if self._cancel_event is not None:
            self._cancel_event.set()
            self.cancel_button.configure(state=tk.DISABLED)
            self._log("⏹ Запрошена остановка прогона...")

    def _poll_signal(self) -> None:
        if self._signal is None:
            return
        drained = False
        while True:
            try:
                kind, payload = self._signal.q.get_nowait()
            except queue.Empty:
                break
            drained = True
            if kind == "progress":
                idx, total, status = payload  # type: ignore[misc]
                if total and total > 0:
                    self.progress_var.set(int(round(idx / total * 100)))
                else:
                    self.progress_var.set(0)
                if idx % 100 == 0 or status != "ok":
                    self._log(f"  кадр {idx}/{total or '?'}  статус={status}")
                self.live.set_metrics(
                    {"idx": idx, "total": total, "status": status}
                )
            elif kind == "frame":
                frame_data, pose, status = payload  # type: ignore[misc]
                self.live.push_frame(frame_data, pose, status)
            elif kind == "log":
                self._log(str(payload))
            elif kind == "done":
                self._on_done(payload)  # type: ignore[arg-type]
                self._signal = None
                return
            elif kind == "error":
                self._on_error(payload)  # type: ignore[arg-type]
                self._signal = None
                return

        if self._worker_thread is not None and self._worker_thread.is_alive():
            self.after(self.POLL_MS, self._poll_signal)
        elif drained:
            return
        else:
            self.after(self.POLL_MS, self._poll_signal)

    # ------------------------------------------------------------------
    # On done / error
    # ------------------------------------------------------------------
    def _on_done(self, result: RunResult) -> None:
        self._last_result = result
        self.run_button.configure(state=tk.NORMAL)
        self.cancel_button.configure(state=tk.DISABLED)
        self.dataset_combo.configure(state="readonly")
        self.algo_combo.configure(state="readonly")
        self.sequence_combo.configure(state="readonly")

        cancelled = bool(result.metrics.get("cancelled"))
        self._log("-" * 60)
        if cancelled:
            self._log(f"⏹ Прогон остановлен. Кадров обработано: {result.n_frames}")
        else:
            self._log(
                f"Готово. Кадров: {result.n_frames}  "
                f"сбоев: {result.failed_frames}  "
                f"успешных: {result.success_rate * 100:.1f}%  "
                f"мс/кадр: {result.time_per_frame_ms:.1f}"
            )
        ate = result.metrics.get("ate") or {}
        if "rmse" in ate:
            self._log(
                f"ATE rmse={ate['rmse']:.3f} м  среднее={ate.get('mean', 0):.3f} м  "
                f"макс={ate.get('max', 0):.3f} м  выровнено={ate.get('aligned')}"
            )
        if "final_drift_m" in result.metrics:
            self._log(f"итоговый дрейф = {result.metrics['final_drift_m']:.2f} м")
        if result.trajectory_path:
            self._log(f"сохранена траектория: {result.trajectory_path}")
        if result.metrics_path:
            self._log(f"сохранены метрики:    {result.metrics_path}")

        # Финальная отрисовка LiveVisualizer
        self.live.set_metrics(
            {
                "idx": result.n_frames,
                "total": result.n_frames,
                "success_rate": result.success_rate,
                "ms_per_frame": result.time_per_frame_ms,
                "status": "cancelled" if cancelled else "done",
            }
        )
        self.live.freeze()

        if self._on_run_finished is not None and result.trajectory_path is not None:
            try:
                self._on_run_finished(result)
            except Exception as exc:  # noqa: BLE001
                self._log(f"  ошибка обработчика завершения: {exc}")

    def _on_error(self, exc: BaseException) -> None:
        self.run_button.configure(state=tk.NORMAL)
        self.cancel_button.configure(state=tk.DISABLED)
        self.dataset_combo.configure(state="readonly")
        self.algo_combo.configure(state="readonly")
        self.sequence_combo.configure(state="readonly")
        self._log(f"Ошибка прогона: {type(exc).__name__}: {exc}")
        messagebox.showerror("Ошибка прогона", f"{type(exc).__name__}: {exc}")


# ---------------------------------------------------------------------------
# Legacy shim: python -m src.gui.chooser → открыть MainApp на вкладке chooser
# ---------------------------------------------------------------------------
def main() -> int:
    """Shortcut: python -m src.gui.chooser открывает MainApp с табом chooser."""
    from src.gui.main_app import MainApp

    app = MainApp(initial_tab="chooser")
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
