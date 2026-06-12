"""Базовые интерфейсы и типы данных Visual Odometry research stand."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Iterator, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Pose
# ---------------------------------------------------------------------------
@dataclass
class Pose:
    """SE(3)-поза: вращение R (3x3) + сдвиг t (3x1).

    Семантика: KITTI-совместимая. Каждая поза T = [R | t] переводит
    точку из системы координат камеры в систему координат frame 0
    (world frame). Это совпадает с poses/XX.txt в KITTI.
    """

    R: np.ndarray
    t: np.ndarray

    def __post_init__(self) -> None:
        self.R = np.asarray(self.R, dtype=np.float64).reshape(3, 3)
        self.t = np.asarray(self.t, dtype=np.float64).reshape(3, 1)

    @classmethod
    def identity(cls) -> "Pose":
        return cls(R=np.eye(3, dtype=np.float64), t=np.zeros((3, 1), dtype=np.float64))

    @classmethod
    def from_kitti_row(cls, row: str) -> "Pose":
        """Парсит строку KITTI poses (12 чисел: 3x4 row-major)."""
        vals = row.strip().split()
        if len(vals) != 12:
            raise ValueError(f"KITTI pose row must have 12 values, got {len(vals)}")
        T = np.array([float(v) for v in vals], dtype=np.float64).reshape(3, 4)
        return cls(R=T[:, :3], t=T[:, 3:4])

    @classmethod
    def from_matrix(cls, T: np.ndarray) -> "Pose":
        """Создаёт Pose из 4x4 (или 3x4) матрицы преобразования."""
        T = np.asarray(T, dtype=np.float64)
        if T.shape == (4, 4):
            return cls(R=T[:3, :3], t=T[:3, 3:4])
        if T.shape == (3, 4):
            return cls(R=T[:, :3], t=T[:, 3:4])
        raise ValueError(f"Expected (3,4) or (4,4), got {T.shape}")

    def to_kitti_row(self) -> str:
        """Сериализация в одну строку KITTI poses (12 чисел)."""
        T = np.hstack([self.R, self.t])
        return " ".join(f"{v:.9e}" for v in T.flatten())

    def to_matrix(self) -> np.ndarray:
        """Возвращает 4x4 однородную матрицу преобразования."""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.R
        T[:3, 3:4] = self.t
        return T

    def compose(self, rel: "Pose") -> "Pose":
        """SE(3) composition: T_new = self @ rel."""
        return Pose(R=self.R @ rel.R, t=self.R @ rel.t + self.t)

    def relative_to(self, prev: "Pose") -> "Pose":
        """T_rel = prev^{-1} @ self — относительная поза от prev к self."""
        R_inv = prev.R.T
        return Pose(R=R_inv @ self.R, t=R_inv @ (self.t - prev.t))

    def translation_distance(self, other: "Pose") -> float:
        """Евклидова дистанция между трансляциями двух поз."""
        return float(np.linalg.norm(self.t - other.t))

    def position(self) -> Tuple[float, float, float]:
        return float(self.t[0, 0]), float(self.t[1, 0]), float(self.t[2, 0])


# ---------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class IMUSample:
    """Один IMU-семпл: ускорение (3,) м/с^2 и угловая скорость (3,) рад/с.

    Конвенция осей — IMU (body) frame, как в KITTI OXTS:
    x — forward, y — left, z — up.
    """

    timestamp: float
    acc: np.ndarray
    gyro: np.ndarray


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Calibration:
    """Все параметры сенсоров для одного датасета/последовательности.

    Минимально требуемое — K_left и image_size; всё остальное опционально
    и заполняется только если у датасета есть стерео/IMU.
    """

    K_left: np.ndarray
    image_size: Tuple[int, int]
    K_right: Optional[np.ndarray] = None
    baseline_m: Optional[float] = None
    T_imu_cam: Optional[np.ndarray] = None

    def focal_x(self) -> float:
        return float(self.K_left[0, 0])

    def focal_y(self) -> float:
        return float(self.K_left[1, 1])

    def principal_point(self) -> Tuple[float, float]:
        return float(self.K_left[0, 2]), float(self.K_left[1, 2])

    def has_stereo(self) -> bool:
        return self.K_right is not None and self.baseline_m is not None

    def has_imu(self) -> bool:
        return self.T_imu_cam is not None


# ---------------------------------------------------------------------------
# Frame data
# ---------------------------------------------------------------------------
@dataclass
class FrameData:
    """Унифицированный пакет данных одного кадра, который видит алгоритм.

    Алгоритмы используют только нужные им поля: A1 — left + gt_pose (для scale);
    A2 — left + right; A3/A4 — left/right + imu; неподдерживаемые поля просто
    игнорируются.
    """

    index: int
    timestamp: float
    left: np.ndarray
    right: Optional[np.ndarray] = None
    imu: Optional[List[IMUSample]] = field(default_factory=list)
    gt_pose: Optional[Pose] = None


# ---------------------------------------------------------------------------
# Capabilities
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class AlgorithmRequirements:
    """Что алгоритм требует от датасета."""

    needs_stereo: bool = False
    needs_imu: bool = False


@dataclass(frozen=True)
class DatasetCapabilities:
    """Что датасет умеет отдавать."""

    has_stereo: bool = False
    has_imu: bool = False
    has_ground_truth: bool = False


def is_compatible(
    requirements: AlgorithmRequirements, capabilities: DatasetCapabilities
) -> bool:
    """True, если датасет покрывает все требования алгоритма.

    Используется и в Runner (для валидации с понятной ошибкой через
    _validate_compatibility), и в GUI chooser (для фильтрации списка
    алгоритмов под выбранный датасет до старта прогона).
    """
    if requirements.needs_stereo and not capabilities.has_stereo:
        return False
    if requirements.needs_imu and not capabilities.has_imu:
        return False
    return True


# ---------------------------------------------------------------------------
# VOAlgorithm ABC
# ---------------------------------------------------------------------------
class VOAlgorithm(ABC):
    """Базовый класс для всех VO-алгоритмов в стенде.

    Жизненный цикл:
        algo = MyAlgo(...)
        algo.reset(dataset.calibration())
        for frame in dataset.frames():
            pose = algo.process(frame)   # абсолютная Pose или None
            ...
    """

    @abstractmethod
    def reset(self, calibration: Calibration) -> None:
        """Сбросить состояние, подгрузить калибровку перед прогоном."""

    @abstractmethod
    def process(self, frame: FrameData) -> Optional[Pose]:
        """Один кадр в, накопленная **абсолютная** поза наружу.

        Возвращает None, если алгоритм потерял трекинг на этом кадре —
        runner запишет в траекторию последнюю успешную позу и инкрементирует
        счётчик failed_frames.
        """

    @abstractmethod
    def requirements(self) -> AlgorithmRequirements:
        """Что алгоритму нужно от датасета."""

    @property
    def name(self) -> str:
        return self.__class__.__name__
