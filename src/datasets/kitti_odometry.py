"""KITTI Odometry benchmark — sequences/<XX>/{image_0, image_1, calib.txt, times.txt}.

Используется для A1, A2 (без IMU). Для seq 00..10 доступен GT в poses/XX.txt.
Структура `kitti_data/` зафиксирована в stages/03_data_layout.md.
"""
from __future__ import annotations

from pathlib import Path
from typing import Iterator, List, Optional

import cv2
import numpy as np

from src.datasets import register_dataset
from src.datasets.base import Dataset
from src.vo.interface import (
    Calibration,
    DatasetCapabilities,
    FrameData,
    Pose,
)


def _parse_calib_txt(path: Path) -> dict:
    """Парсит KITTI Odometry calib.txt: 5 строк P0, P1, P2, P3, Tr.

    Возвращает {"P0": (3,4), "P1": (3,4), "P2": (3,4), "P3": (3,4), "Tr": (3,4)}.
    """
    out: dict = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            key, _, vals = line.partition(":")
            nums = [float(v) for v in vals.split()]
            if len(nums) == 12:
                out[key.strip()] = np.array(nums, dtype=np.float64).reshape(3, 4)
    return out


def _parse_kitti_poses(path: Path) -> List[Pose]:
    poses: List[Pose] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            poses.append(Pose.from_kitti_row(line))
    return poses


@register_dataset("kitti_odometry")
class KittiOdometry(Dataset):
    """KITTI Odometry sequence loader.

    Args:
        basedir: путь к корню kitti_data/ (внутри ожидается sequences/, poses/).
        sequence: строка "00".."21".
    """

    def __init__(self, basedir: str | Path, sequence: str):
        self.basedir = Path(basedir)
        self.sequence = str(sequence).zfill(2)

        self.seq_dir = self.basedir / "sequences" / self.sequence
        if not self.seq_dir.is_dir():
            raise FileNotFoundError(f"KITTI Odometry sequence not found: {self.seq_dir}")

        self.image_left_dir = self.seq_dir / "image_0"
        self.image_right_dir = self.seq_dir / "image_1"
        if not self.image_left_dir.is_dir():
            raise FileNotFoundError(f"image_0/ not found in {self.seq_dir}")

        self.left_files: List[Path] = sorted(
            p for p in self.image_left_dir.iterdir() if p.suffix.lower() == ".png"
        )
        self.right_files: List[Path] = (
            sorted(p for p in self.image_right_dir.iterdir() if p.suffix.lower() == ".png")
            if self.image_right_dir.is_dir()
            else []
        )
        if not self.left_files:
            raise ValueError(f"No PNG frames in {self.image_left_dir}")

        # Calibration
        self._calib_data = _parse_calib_txt(self.seq_dir / "calib.txt")
        P0 = self._calib_data["P0"]
        P1 = self._calib_data.get("P1")
        K_left = P0[:, :3].copy()
        K_right = P1[:, :3].copy() if P1 is not None else None
        # baseline = -P1[0,3] / P0[0,0] — расстояние между cam0 и cam1 в метрах
        baseline = float(-P1[0, 3] / P0[0, 0]) if P1 is not None else None

        # image size (берём из первой картинки, чтобы не лениться сравнивать
        # с предполагаемыми 1241x376 — у KITTI размеры по seq немного разные)
        first_img = cv2.imread(str(self.left_files[0]), cv2.IMREAD_GRAYSCALE)
        if first_img is None:
            raise RuntimeError(f"Cannot read first image {self.left_files[0]}")
        h, w = first_img.shape[:2]

        self._calibration = Calibration(
            K_left=K_left,
            image_size=(w, h),
            K_right=K_right,
            baseline_m=baseline,
            T_imu_cam=None,
        )

        # Timestamps (offset от начала, секунды)
        times_path = self.seq_dir / "times.txt"
        if times_path.exists():
            self._timestamps: List[float] = [
                float(s) for s in times_path.read_text().split() if s
            ]
        else:
            self._timestamps = [i * 0.1 for i in range(len(self.left_files))]
        if len(self._timestamps) < len(self.left_files):
            # Подравниваем — иногда times.txt короче, чем число кадров
            self._timestamps += [
                self._timestamps[-1] + 0.1 * (k + 1)
                for k in range(len(self.left_files) - len(self._timestamps))
            ]

        # Ground truth (только для seq 00..10)
        gt_path = self.basedir / "poses" / f"{self.sequence}.txt"
        self._gt_poses: Optional[List[Pose]] = None
        if gt_path.exists():
            self._gt_poses = _parse_kitti_poses(gt_path)

        self._has_stereo = len(self.right_files) == len(self.left_files) and K_right is not None

    # ------------------------------------------------------------------
    # Dataset API
    # ------------------------------------------------------------------
    def name(self) -> str:
        return f"kitti_odometry_{self.sequence}"

    def calibration(self) -> Calibration:
        return self._calibration

    def capabilities(self) -> DatasetCapabilities:
        return DatasetCapabilities(
            has_stereo=self._has_stereo,
            has_imu=False,
            has_ground_truth=self._gt_poses is not None,
        )

    def ground_truth(self) -> Optional[List[Pose]]:
        return list(self._gt_poses) if self._gt_poses is not None else None

    def frames(self) -> Iterator[FrameData]:
        n = len(self.left_files)
        for i in range(n):
            left = cv2.imread(str(self.left_files[i]), cv2.IMREAD_GRAYSCALE)
            if left is None:
                raise RuntimeError(f"Failed to read {self.left_files[i]}")
            right = None
            if self._has_stereo:
                right = cv2.imread(str(self.right_files[i]), cv2.IMREAD_GRAYSCALE)

            gt = self._gt_poses[i] if self._gt_poses is not None and i < len(self._gt_poses) else None

            yield FrameData(
                index=i,
                timestamp=self._timestamps[i] if i < len(self._timestamps) else float(i) * 0.1,
                left=left,
                right=right,
                imu=None,
                gt_pose=gt,
            )

    def __len__(self) -> int:
        return len(self.left_files)


# ---------------------------------------------------------------------------
# Sanity-print helper (Этап 2 sanity)
# ---------------------------------------------------------------------------
def _print_first_frames(basedir: str = "kitti_data", sequence: str = "04", n: int = 3) -> None:
    ds = KittiOdometry(basedir, sequence)
    print(f"=== KittiOdometry seq {sequence} ===")
    print(f"frames:       {len(ds)}")
    print(f"capabilities: {ds.capabilities()}")
    K = ds.calibration().K_left
    print(f"K_left:\n{K}")
    print(f"baseline_m:   {ds.calibration().baseline_m}")
    print(f"image_size:   {ds.calibration().image_size}")
    gt = ds.ground_truth()
    print(f"GT count:     {len(gt) if gt is not None else 'no GT'}")
    print()
    for i, frame in enumerate(ds.frames()):
        if i >= n:
            break
        gt_t = frame.gt_pose.t.flatten() if frame.gt_pose is not None else None
        print(
            f"  frame {frame.index:04d}  t={frame.timestamp:8.3f}s  "
            f"L={frame.left.shape}  R={None if frame.right is None else frame.right.shape}  "
            f"gt_t={gt_t}"
        )


if __name__ == "__main__":
    _print_first_frames()
