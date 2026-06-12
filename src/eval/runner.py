"""Runner: один прогон алгоритма на датасете → траектория + метрики на диск.

Принимает (VOAlgorithm, Dataset), валидирует совместимость requirements↔
capabilities, итерирует FrameData → process(), копит траекторию,
сохраняет в results/trajectories/ и results/metrics/.

Контракт:
- Если algo.process() вернул None — runner повторяет последнюю успешную
  позу (в траектории не появляется «дырки» — KITTI evaluator ожидает
  одну строку на кадр) и инкрементирует счётчик failed_frames.
- progress_callback(frame_idx, total, pose, status) — для GUI/CLI прогрессбара.
- Корневой каталог результатов: results/{trajectories, metrics}/.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

import cv2

from src.datasets.base import Dataset
from src.eval.metrics import (
    compute_ate,
    compute_drift,
    compute_rpe,
    save_kitti_trajectory,
    save_metrics_json,
)
from src.vo.interface import (
    AlgorithmRequirements,
    DatasetCapabilities,
    Pose,
    VOAlgorithm,
    is_compatible,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_DIR = PROJECT_ROOT / "results"
DEFAULT_RNG_SEED = 42


ProgressCallback = Callable[[int, int, Pose, str], None]


@dataclass
class RunResult:
    """Сводка одного прогона алгоритма на датасете."""

    algo_name: str
    dataset_name: str
    trajectory: List[Pose] = field(default_factory=list)
    n_frames: int = 0
    failed_frames: int = 0
    total_time_s: float = 0.0
    metrics: dict = field(default_factory=dict)
    trajectory_path: Optional[Path] = None
    metrics_path: Optional[Path] = None

    @property
    def success_rate(self) -> float:
        if self.n_frames == 0:
            return 0.0
        return 1.0 - self.failed_frames / self.n_frames

    @property
    def time_per_frame_ms(self) -> float:
        if self.n_frames == 0:
            return 0.0
        return self.total_time_s / self.n_frames * 1000.0


def _validate_compatibility(
    requirements: AlgorithmRequirements, capabilities: DatasetCapabilities
) -> None:
    """Бросает ValueError, если датасет не покрывает требования алгоритма.

    Тонкая обёртка над is_compatible() с понятным сообщением для CLI/GUI.
    """
    if is_compatible(requirements, capabilities):
        return
    if requirements.needs_stereo and not capabilities.has_stereo:
        raise ValueError(
            "Алгоритм требует stereo, у датасета has_stereo=False"
        )
    if requirements.needs_imu and not capabilities.has_imu:
        raise ValueError(
            "Алгоритм требует IMU, у датасета has_imu=False"
        )
    raise ValueError("Алгоритм несовместим с датасетом (см. capabilities)")


class Runner:
    """Прогонщик одного алгоритма на одном датасете.

    Args:
        results_dir: куда писать results/trajectories и results/metrics.
        seed: RNG seed для воспроизводимости. Применяется через
            cv2.setRNGSeed() перед каждым прогоном — фиксирует поведение
            cv2.findEssentialMat (A1), cv2.solvePnPRansac (A2),
            cv2.calcOpticalFlowPyrLK (A1/A2). По умолчанию DEFAULT_RNG_SEED=42.
    """

    def __init__(
        self,
        results_dir: Optional[Path] = None,
        seed: int = DEFAULT_RNG_SEED,
    ) -> None:
        self.results_dir = Path(results_dir) if results_dir else DEFAULT_RESULTS_DIR
        self.trajectories_dir = self.results_dir / "trajectories"
        self.metrics_dir = self.results_dir / "metrics"
        self.trajectories_dir.mkdir(parents=True, exist_ok=True)
        self.metrics_dir.mkdir(parents=True, exist_ok=True)
        self.seed = int(seed)

    def run(
        self,
        algorithm: VOAlgorithm,
        dataset: Dataset,
        max_frames: Optional[int] = None,
        progress_callback: Optional[ProgressCallback] = None,
        save: bool = True,
        result_tag: Optional[str] = None,
        cancel_event: Optional[threading.Event] = None,
        frame_callback: Optional[Callable[[object, Pose, str], None]] = None,
    ) -> RunResult:
        """Один прогон.

        Args:
            algorithm: уже сконструированный объект VOAlgorithm (без reset).
            dataset:   объект Dataset.
            max_frames: обрезать датасет (None = все).
            progress_callback: вызывается на каждом кадре, не блокирует.
            save: писать ли results/trajectories/<...>.txt и metrics/<...>.json.
            result_tag: суффикс к имени файла; если None — берётся dataset.name().
            cancel_event: опциональный threading.Event для прерывания прогона
                из другого потока (GUI «Стоп»). Если set() — runner выходит
                после текущего кадра, пишет result.metrics["cancelled"]=True.
                Дефолт None — CLI-поведение без изменений.
            frame_callback: опциональный callback для GUI live-визуализатора;
                принимает (frame_data, pose, status). Вызывается на каждом
                кадре после process(). Дефолт None — runner не передаёт
                FrameData никому (CLI не нужны сырые кадры).

        Returns:
            RunResult со списком поз, метриками и путями к файлам.
        """
        _validate_compatibility(algorithm.requirements(), dataset.capabilities())
        cv2.setRNGSeed(self.seed)
        algorithm.reset(dataset.calibration())

        result = RunResult(algo_name=algorithm.name, dataset_name=dataset.name())

        last_pose = Pose.identity()
        gt_list: List[Pose] = []
        total = len(dataset) if hasattr(dataset, "__len__") else -1
        if max_frames is not None and total > 0:
            total = min(total, max_frames)

        cancelled = False
        t0 = time.perf_counter()
        for frame in dataset.frames():
            if cancel_event is not None and cancel_event.is_set():
                cancelled = True
                break
            if max_frames is not None and result.n_frames >= max_frames:
                break

            t_frame = time.perf_counter()
            try:
                pose = algorithm.process(frame)
                status = "ok" if pose is not None else "lost"
            except Exception as exc:  # noqa: BLE001
                pose = None
                status = f"error: {type(exc).__name__}: {exc}"

            if pose is None:
                pose = last_pose
                result.failed_frames += 1
            else:
                last_pose = pose

            result.trajectory.append(pose)
            if frame.gt_pose is not None:
                gt_list.append(frame.gt_pose)
            result.n_frames += 1

            if progress_callback is not None:
                try:
                    progress_callback(result.n_frames, total, pose, status)
                except Exception:  # noqa: BLE001
                    pass

            if frame_callback is not None:
                try:
                    frame_callback(frame, pose, status)
                except Exception:  # noqa: BLE001
                    pass

            _ = t_frame
        result.total_time_s = time.perf_counter() - t0

        # Метрики (только если есть GT — либо из датасета, либо из frame.gt_pose)
        gt_full: Optional[List[Pose]] = None
        if dataset.capabilities().has_ground_truth:
            gt_dataset = dataset.ground_truth()
            if gt_dataset is not None:
                gt_full = gt_dataset[: result.n_frames]
            elif gt_list:
                gt_full = gt_list

        if gt_full and len(gt_full) >= 2 and len(result.trajectory) >= 2:
            try:
                result.metrics["ate"] = compute_ate(result.trajectory, gt_full)
            except Exception as exc:  # noqa: BLE001
                result.metrics["ate_error"] = f"{type(exc).__name__}: {exc}"
            try:
                result.metrics["rpe_translation_1"] = compute_rpe(
                    result.trajectory, gt_full, delta=1, relation="translation_part"
                )
            except Exception as exc:  # noqa: BLE001
                result.metrics["rpe_error"] = f"{type(exc).__name__}: {exc}"
            try:
                drift = compute_drift(result.trajectory, gt_full)
                if drift is not None:
                    result.metrics["final_drift_m"] = drift
            except Exception as exc:  # noqa: BLE001
                result.metrics["drift_error"] = f"{type(exc).__name__}: {exc}"

        result.metrics["success_rate"] = result.success_rate
        result.metrics["failed_frames"] = result.failed_frames
        result.metrics["n_frames"] = result.n_frames
        result.metrics["time_per_frame_ms"] = result.time_per_frame_ms
        result.metrics["total_time_s"] = result.total_time_s
        if cancelled:
            result.metrics["cancelled"] = True

        if save:
            tag = result_tag or dataset.name()
            cancel_suffix = "__cancelled" if cancelled else ""
            stem = f"{algorithm.name}__{tag}{cancel_suffix}"
            traj_path = self.trajectories_dir / f"{stem}.txt"
            metrics_path = self.metrics_dir / f"{stem}.json"
            save_kitti_trajectory(result.trajectory, traj_path)
            save_metrics_json(result.metrics, metrics_path)
            result.trajectory_path = traj_path
            result.metrics_path = metrics_path

        return result


# ---------------------------------------------------------------------------
# Self-test (Этап 1 sanity: mock algo + mock dataset → ATE = 0)
# ---------------------------------------------------------------------------
def _self_test() -> None:
    """Sanity-тест: identity-алгоритм на dataset из 10 пустых кадров с GT=identity."""
    import numpy as np

    from src.vo.interface import (
        AlgorithmRequirements,
        Calibration,
        DatasetCapabilities,
        FrameData,
        Pose,
        VOAlgorithm,
    )

    class _IdentityAlgo(VOAlgorithm):
        def reset(self, calibration: Calibration) -> None:
            self._calib = calibration

        def process(self, frame: FrameData) -> Optional[Pose]:
            return Pose.identity()

        def requirements(self) -> AlgorithmRequirements:
            return AlgorithmRequirements()

    class _MockDataset(Dataset):
        def name(self) -> str:
            return "mock_identity"

        def calibration(self) -> Calibration:
            K = np.array(
                [[718.86, 0, 607.19], [0, 718.86, 185.22], [0, 0, 1]], dtype=float
            )
            return Calibration(K_left=K, image_size=(1241, 376))

        def frames(self):
            for i in range(10):
                yield FrameData(
                    index=i,
                    timestamp=float(i) * 0.1,
                    left=np.zeros((376, 1241), dtype=np.uint8),
                    gt_pose=Pose.identity(),
                )

        def ground_truth(self):
            return [Pose.identity() for _ in range(10)]

        def capabilities(self) -> DatasetCapabilities:
            return DatasetCapabilities(has_ground_truth=True)

        def __len__(self) -> int:
            return 10

    runner = Runner()
    result = runner.run(
        _IdentityAlgo(),
        _MockDataset(),
        save=False,
    )

    print(f"frames:   {result.n_frames}")
    print(f"failed:   {result.failed_frames}")
    print(f"success:  {result.success_rate:.3f}")
    print(f"time/fr:  {result.time_per_frame_ms:.3f} ms")
    print("ATE rmse:", result.metrics.get("ate", {}).get("rmse"))
    print("RPE rmse:", result.metrics.get("rpe_translation_1", {}).get("rmse"))
    print("drift_m:", result.metrics.get("final_drift_m"))

    ate_rmse = result.metrics.get("ate", {}).get("rmse")
    assert ate_rmse is not None and ate_rmse < 1e-9, f"ATE rmse must be ~0, got {ate_rmse}"
    drift = result.metrics.get("final_drift_m")
    assert drift is not None and drift < 1e-9, f"drift must be ~0, got {drift}"
    print("OK — sanity test passed")


def _cli() -> int:
    """CLI: один прогон алгоритма на датасете.

    Примеры:
        python -m src.eval.runner --selftest
        python -m src.eval.runner --algo A1 --dataset kitti_odometry --sequence 04
        python -m src.eval.runner --algo A1 --dataset kitti_odometry --sequence 05 \\
            --max-frames 1000
        python -m src.eval.runner --algo A1 --dataset kitti_raw \\
            --date 2011_09_26 --drive 0001
        python -m src.eval.runner --algo A3 --dataset kitti_raw \\
            --date 2011_09_30 --drive 0018 --seed 42
    """
    import argparse

    from src.algorithms import ALGORITHMS, import_all_algorithms
    from src.datasets import DATASETS, import_all_datasets

    parser = argparse.ArgumentParser(description="VO Stand runner CLI")
    parser.add_argument("--algo", default="A1", help="ключ алгоритма (по умолчанию A1)")
    parser.add_argument(
        "--dataset",
        default="kitti_odometry",
        choices=["kitti_odometry", "kitti_raw"],
    )
    parser.add_argument(
        "--sequence",
        help="для kitti_odometry: '00'..'21'",
    )
    parser.add_argument("--date", help="для kitti_raw: '2011_09_26' и т.п.")
    parser.add_argument("--drive", help="для kitti_raw: '0001', '0018' и т.п.")
    parser.add_argument(
        "--basedir",
        default="kitti_data",
        help="корень kitti_data (по умолчанию ./kitti_data)",
    )
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_RNG_SEED,
        help=f"RNG seed для cv2 (RANSAC, LK); по умолчанию {DEFAULT_RNG_SEED}",
    )
    parser.add_argument(
        "--selftest", action="store_true", help="запустить mock-тест Этапа 1"
    )
    args = parser.parse_args()

    if args.selftest:
        _self_test()
        return 0

    import_all_algorithms()
    import_all_datasets()

    if args.algo not in ALGORITHMS:
        parser.error(f"Unknown --algo {args.algo!r}; available: {list(ALGORITHMS)}")
    if args.dataset not in DATASETS:
        parser.error(f"Unknown --dataset {args.dataset!r}; available: {list(DATASETS)}")

    algo_cls = ALGORITHMS[args.algo]
    ds_cls = DATASETS[args.dataset]

    if args.dataset == "kitti_odometry":
        if not args.sequence:
            parser.error("--sequence required for kitti_odometry")
        ds = ds_cls(args.basedir, args.sequence)
    else:
        if not args.date or not args.drive:
            parser.error("--date and --drive required for kitti_raw")
        from pathlib import Path

        ds = ds_cls(str(Path(args.basedir) / "raw"), args.date, args.drive)

    algo = algo_cls()

    def cb(idx: int, total: int, _pose: Pose, _status: str) -> None:
        if total > 0 and (idx % 200 == 0 or idx == total):
            print(f"  frame {idx:5d}/{total}  {idx / total * 100:5.1f}%")

    runner = Runner(seed=args.seed)
    result = runner.run(algo, ds, max_frames=args.max_frames, progress_callback=cb)

    print("=" * 70)
    print(f"algo:      {result.algo_name}")
    print(f"dataset:   {result.dataset_name}")
    print(
        f"frames:    {result.n_frames}  failed={result.failed_frames}  "
        f"success={result.success_rate * 100:.1f}%"
    )
    print(f"time/frame: {result.time_per_frame_ms:.1f} ms")
    ate = result.metrics.get("ate") or {}
    if "rmse" in ate:
        print(
            f"ATE rmse={ate['rmse']:.3f} m  mean={ate.get('mean', 0):.3f} m  "
            f"max={ate.get('max', 0):.3f} m  aligned={ate.get('aligned')}"
        )
    if "final_drift_m" in result.metrics:
        print(f"final drift: {result.metrics['final_drift_m']:.2f} m")
    if result.trajectory_path:
        print(f"saved trajectory: {result.trajectory_path}")
    if result.metrics_path:
        print(f"saved metrics:    {result.metrics_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_cli())
