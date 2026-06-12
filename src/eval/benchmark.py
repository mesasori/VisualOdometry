"""Маппинг KITTI Odometry↔Raw и место для будущего benchmark-обхода матрицы."""
from __future__ import annotations

from typing import Optional, Tuple


KITTI_ODO_RAW_PAIRS: dict[str, Tuple[str, str]] = {
    "01": ("2011_10_03", "0042"),
    "04": ("2011_09_30", "0016"),
    "05": ("2011_09_30", "0018"),
    "06": ("2011_09_30", "0020"),
}


def raw_for_odo(seq: str) -> Optional[Tuple[str, str]]:
    """Для Odometry sequence (e.g. '05') вернуть (date, drive) Raw drive.

    Возвращает None, если эквивалента нет (seq 00, 02, 03, 07, 08, 09, 10).
    """
    return KITTI_ODO_RAW_PAIRS.get(str(seq).zfill(2))


def odo_for_raw(date: str, drive: str) -> Optional[str]:
    """Для Raw drive (date, drive) вернуть Odometry sequence или None."""
    for seq, pair in KITTI_ODO_RAW_PAIRS.items():
        if pair == (date, drive):
            return seq
    return None


__all__ = [
    "KITTI_ODO_RAW_PAIRS",
    "raw_for_odo",
    "odo_for_raw",
]
