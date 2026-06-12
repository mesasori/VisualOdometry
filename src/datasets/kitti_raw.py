"""KITTI Raw — синхронизированные камеры + IMU (oxts) для visual-inertial алгоритмов.

Используется для A3, A4 (mono+IMU, stereo+IMU). GT-поза вычисляется из OXTS
(GPS+IMU), переводится в локальную систему первого кадра — совместимо с
форматом KITTI Odometry poses/XX.txt.

Особенность: в `*_sync/`-версии, которую мы качали, на каждый кадр приходится
ровно один OXTS-семпл (синхронно). Это даёт IMU-частоту ~10 Hz между кадрами,
что меньше, чем 100 Hz из большого плана. Для loosely-coupled EKF это
работает, но для tightly-coupled нужны unsync-данные. Зафиксировано в
stages/04_a1_implementation.md как открытый вопрос для Этапа 5/6.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Iterator, List, Optional

import numpy as np
import pykitti

from src.datasets import register_dataset
from src.datasets.base import Dataset
from src.vo.interface import (
    Calibration,
    DatasetCapabilities,
    FrameData,
    IMUSample,
    Pose,
)


def _ts_to_float(ts: datetime, t0: datetime) -> float:
    """Перевод datetime в offset от t0 в секундах."""
    return (ts - t0).total_seconds()


def _oxts_to_imu_sample(packet, timestamp: float) -> IMUSample:
    """OXTS-пакет → IMUSample.

    KITTI OXTS: af/al/au — линейное ускорение по forward/left/up в IMU frame
    (м/с²); wf/wl/wu — угловая скорость (рад/с) по тем же осям.
    """
    return IMUSample(
        timestamp=timestamp,
        acc=np.array([packet.af, packet.al, packet.au], dtype=np.float64),
        gyro=np.array([packet.wf, packet.wl, packet.wu], dtype=np.float64),
    )


@register_dataset("kitti_raw")
class KittiRaw(Dataset):
    """KITTI Raw drive loader через pykitti.raw.

    Args:
        basedir: путь к корню raw-данных (kitti_data/raw).
        date:    "2011_09_26", "2011_09_30", "2011_10_03".
        drive:   "0001", "0018", "0042" — последние 4 цифры drive id.
    """

    def __init__(self, basedir: str | Path, date: str, drive: str):
        self.basedir = Path(basedir)
        self.date = date
        self.drive = drive
        self._data = pykitti.raw(str(self.basedir), date, drive)

        # Calibration
        K_left = np.array(self._data.calib.K_cam0, dtype=np.float64)
        K_right = np.array(self._data.calib.K_cam1, dtype=np.float64)
        baseline = float(self._data.calib.b_gray)
        # T_imu_cam = матрица перехода из IMU в cam0
        # pykitti даёт T_cam0_imu (cam0 ← imu); нам нужно T_imu_cam0 = (T_cam0_imu)^-1
        T_cam0_imu = np.array(self._data.calib.T_cam0_imu, dtype=np.float64)
        T_imu_cam0 = np.linalg.inv(T_cam0_imu)

        # image_size
        first_img = np.array(self._data.get_cam0(0))
        h, w = first_img.shape[:2]

        self._calibration = Calibration(
            K_left=K_left,
            image_size=(w, h),
            K_right=K_right,
            baseline_m=baseline,
            T_imu_cam=T_imu_cam0,
        )

        # Timestamps (float, секунды от первого кадра)
        ts_list = list(self._data.timestamps)
        if not ts_list:
            raise ValueError("pykitti returned empty timestamps")
        self._t0 = ts_list[0]
        self._timestamps: List[float] = [_ts_to_float(t, self._t0) for t in ts_list]

        # GT через OXTS — переводим из мировой ENU в систему cam0 первого кадра
        self._gt_poses: Optional[List[Pose]] = self._build_gt_poses(T_cam0_imu)

        self._n = len(self._data.cam0_files)
        self._has_stereo = len(self._data.cam1_files) == self._n

    # ------------------------------------------------------------------
    # GT
    # ------------------------------------------------------------------
    def _build_gt_poses(self, T_cam0_imu: np.ndarray) -> Optional[List[Pose]]:
        """OXTS T_w_imu → KITTI-стиль gt в системе cam0[0].

        T_w_cam0_i = T_w_imu_i @ T_imu_cam0  — поза cam0 кадра i в мире.
        gt_i = (T_w_cam0_0)^-1 @ T_w_cam0_i  — поза cam0_i в системе cam0_0.
        Это совпадает с формой kitti_data/poses/XX.txt.
        """
        oxts_list = list(self._data.oxts)
        if not oxts_list:
            return None
        T_imu_cam0 = np.linalg.inv(T_cam0_imu)
        T_w_cam0: List[np.ndarray] = []
        for oxts in oxts_list:
            T_w_cam0.append(oxts.T_w_imu @ T_imu_cam0)
        T0_inv = np.linalg.inv(T_w_cam0[0])
        return [Pose.from_matrix(T0_inv @ T) for T in T_w_cam0]

    # ------------------------------------------------------------------
    # IMU bursts
    # ------------------------------------------------------------------
    def _imu_burst(self, frame_idx: int) -> List[IMUSample]:
        """В sync-режиме — ровно 1 семпл на кадр (с timestamp кадра)."""
        oxts_list = self._data.oxts
        if frame_idx >= len(oxts_list):
            return []
        ts = self._timestamps[frame_idx] if frame_idx < len(self._timestamps) else 0.0
        return [_oxts_to_imu_sample(oxts_list[frame_idx].packet, ts)]

    # ------------------------------------------------------------------
    # Dataset API
    # ------------------------------------------------------------------
    def name(self) -> str:
        return f"kitti_raw_{self.date}_{self.drive}"

    def calibration(self) -> Calibration:
        return self._calibration

    def capabilities(self) -> DatasetCapabilities:
        return DatasetCapabilities(
            has_stereo=self._has_stereo,
            has_imu=True,
            has_ground_truth=self._gt_poses is not None,
        )

    def ground_truth(self) -> Optional[List[Pose]]:
        return list(self._gt_poses) if self._gt_poses is not None else None

    def frames(self) -> Iterator[FrameData]:
        for i in range(self._n):
            left = np.array(self._data.get_cam0(i))
            right = np.array(self._data.get_cam1(i)) if self._has_stereo else None
            gt = self._gt_poses[i] if self._gt_poses is not None and i < len(self._gt_poses) else None
            yield FrameData(
                index=i,
                timestamp=self._timestamps[i],
                left=left,
                right=right,
                imu=self._imu_burst(i),
                gt_pose=gt,
            )

    def __len__(self) -> int:
        return self._n


# ---------------------------------------------------------------------------
# Sanity-print helper (Этап 2 sanity)
# ---------------------------------------------------------------------------
def _print_first_frames(
    basedir: str = "kitti_data/raw",
    date: str = "2011_09_26",
    drive: str = "0001",
    n: int = 3,
) -> None:
    ds = KittiRaw(basedir, date, drive)
    print(f"=== KittiRaw {date}/drive_{drive} ===")
    print(f"frames:       {len(ds)}")
    print(f"capabilities: {ds.capabilities()}")
    K = ds.calibration().K_left
    print(f"K_left:\n{K}")
    print(f"baseline_m:   {ds.calibration().baseline_m}")
    print(f"image_size:   {ds.calibration().image_size}")
    print(f"T_imu_cam:\n{ds.calibration().T_imu_cam}")
    gt = ds.ground_truth()
    print(f"GT count:     {len(gt) if gt is not None else 'no GT'}")
    print()
    for i, frame in enumerate(ds.frames()):
        if i >= n:
            break
        gt_t = frame.gt_pose.t.flatten() if frame.gt_pose is not None else None
        imu_n = len(frame.imu) if frame.imu else 0
        first_imu = frame.imu[0] if frame.imu else None
        print(
            f"  frame {frame.index:04d}  t={frame.timestamp:8.3f}s  "
            f"L={frame.left.shape}  R={None if frame.right is None else frame.right.shape}  "
            f"imu_n={imu_n}  gt_t={gt_t}"
        )
        if first_imu is not None:
            print(f"    IMU acc={first_imu.acc}  gyro={first_imu.gyro}")


if __name__ == "__main__":
    _print_first_frames()
