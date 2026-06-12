"""Базовый интерфейс датасета и общие хелперы для всех загрузчиков."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterator, List, Optional

from src.vo.interface import (
    Calibration,
    DatasetCapabilities,
    FrameData,
    Pose,
)


class Dataset(ABC):
    """Каждый датасет (KITTI Odometry, KITTI Raw, …) реализует этот контракт.

    Алгоритм/runner ничего не знает про конкретный формат — берёт `frames()`,
    `calibration()` и опционально `ground_truth()`.
    """

    @abstractmethod
    def name(self) -> str:
        """Человекочитаемое имя для логов и имён файлов результатов."""

    @abstractmethod
    def calibration(self) -> Calibration:
        """Калибровка камер (+ опц. IMU) для этого прогона."""

    @abstractmethod
    def frames(self) -> Iterator[FrameData]:
        """Итератор по кадрам датасета. Не должен пересоздавать состояние —
        одна последовательность = один проход."""

    @abstractmethod
    def ground_truth(self) -> Optional[List[Pose]]:
        """Список GT-поз в KITTI-семантике (T_k = pose камеры в frame 0)
        или None, если GT отсутствует."""

    @abstractmethod
    def capabilities(self) -> DatasetCapabilities:
        """Какие модальности данных умеет отдавать датасет."""

    def __len__(self) -> int:
        """Опционально — число кадров, если известно заранее.
        Дефолт: -1, если датасет stream-only."""
        return -1
