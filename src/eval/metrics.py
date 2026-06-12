"""Тонкая обёртка над evo: ATE, RPE, drift, KITTI I/O траекторий.

Цель — изолировать всю работу с evo в одном месте, чтобы алгоритмы
и runner оперировали нашим типом Pose, а в evo заходили только при
сериализации/подсчёте метрик.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List, Optional

import numpy as np

from evo.core import metrics, sync
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.tools import file_interface

from src.vo.interface import Pose


# ---------------------------------------------------------------------------
# Сериализация траекторий
# ---------------------------------------------------------------------------
def _poses_to_se3_list(poses: Iterable[Pose]) -> List[np.ndarray]:
    return [p.to_matrix() for p in poses]


def save_kitti_trajectory(poses: List[Pose], path: str | Path) -> None:
    """Сохраняет траекторию в KITTI-формате (12 чисел на строку, row-major)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not poses:
        path.write_text("", encoding="utf-8")
        return
    traj = PosePath3D(poses_se3=_poses_to_se3_list(poses))
    file_interface.write_kitti_poses_file(str(path), traj, confirm_overwrite=False)


def load_kitti_trajectory(path: str | Path) -> List[Pose]:
    """Загружает KITTI-траекторию обратно в список Pose."""
    traj = file_interface.read_kitti_poses_file(str(path))
    return [Pose.from_matrix(T) for T in traj.poses_se3]


def _build_path(poses: List[Pose]) -> PosePath3D:
    return PosePath3D(poses_se3=_poses_to_se3_list(poses))


# ---------------------------------------------------------------------------
# Метрики
# ---------------------------------------------------------------------------
def compute_ate(
    traj: List[Pose],
    gt: List[Pose],
    align: bool = True,
    correct_scale: bool = False,
) -> dict:
    """Absolute Trajectory Error (translation part) поверх Umeyama-выравнивания.

    Возвращает словарь со статистиками evo (mean, median, rmse, std, min, max).
    align=True — выравнивает оценочную траекторию по reference (стандартная
    практика для VO без абсолютной локализации). Если выравнивание невозможно
    (например, обе траектории — identity, дегенеративная ковариация) — считаем
    raw ATE без выравнивания и помечаем aligned=False.
    """
    if not traj or not gt:
        return {"error": "empty trajectory or gt"}
    n = min(len(traj), len(gt))
    traj_est = _build_path(list(traj)[:n])
    traj_ref = _build_path(list(gt)[:n])
    aligned_ok = False
    if align:
        try:
            traj_est.align(traj_ref, correct_scale=correct_scale)
            aligned_ok = True
        except Exception:  # noqa: BLE001  — GeometryException на дегенеративных
            aligned_ok = False
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((traj_ref, traj_est))
    stats = ape.get_all_statistics()
    stats["aligned"] = aligned_ok
    stats["correct_scale"] = bool(correct_scale)
    stats["n_poses"] = n
    return stats


def compute_rpe(
    traj: List[Pose],
    gt: List[Pose],
    delta: int = 1,
    relation: str = "translation_part",
) -> dict:
    """Relative Pose Error на парах кадров (k, k+delta).

    relation — 'translation_part' (по умолчанию) или 'rotation_angle_deg'.
    """
    if not traj or not gt:
        return {"error": "empty trajectory or gt"}
    n = min(len(traj), len(gt))
    traj_est = _build_path(list(traj)[:n])
    traj_ref = _build_path(list(gt)[:n])
    rel = (
        metrics.PoseRelation.rotation_angle_deg
        if relation == "rotation_angle_deg"
        else metrics.PoseRelation.translation_part
    )
    rpe = metrics.RPE(rel, delta=delta, delta_unit=metrics.Unit.frames, all_pairs=False)
    rpe.process_data((traj_ref, traj_est))
    stats = rpe.get_all_statistics()
    stats["delta_frames"] = delta
    stats["relation"] = relation
    stats["n_poses"] = n
    return stats


def compute_drift(traj: List[Pose], gt: List[Pose]) -> Optional[float]:
    """Финальный drift в метрах: ||t_traj_last - t_gt_last||."""
    if not traj or not gt:
        return None
    n = min(len(traj), len(gt))
    return float(np.linalg.norm(traj[n - 1].t - gt[n - 1].t))


# ---------------------------------------------------------------------------
# Сводка метрик
# ---------------------------------------------------------------------------
def save_metrics_json(metrics_dict: dict, path: str | Path) -> None:
    """JSON-сохранение метрик с приведением numpy-типов к стандартным."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    def _coerce(obj):
        if isinstance(obj, dict):
            return {k: _coerce(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [_coerce(v) for v in obj]
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    path.write_text(json.dumps(_coerce(metrics_dict), indent=2), encoding="utf-8")
