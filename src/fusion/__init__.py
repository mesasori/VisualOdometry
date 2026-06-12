"""fusion: общая EKF-инфраструктура для visual-inertial алгоритмов A3 и A4.

Публичный API (через ленивые импорты — чтобы `python -m src.fusion.ekf`
не выдавал RuntimeWarning о двойном импорте):
    EKF              — основной класс loosely-coupled VIO-фильтра.
    ImuState         — состояние [p, q, v, b_a, b_g].
    propagate        — одна итерация IMU-модели (без EKF-обвязки).
    GRAVITY_WORLD    — гравитационный вектор по умолчанию (0, 0, -9.81).

Использование:
    from src.fusion import EKF, ImuState
    # либо напрямую:
    from src.fusion.ekf import EKF
"""
from __future__ import annotations

__all__ = [
    "EKF",
    "ImuState",
    "propagate",
    "GRAVITY_WORLD",
]


def __getattr__(name: str):
    if name == "EKF":
        from src.fusion.ekf import EKF
        return EKF
    if name in ("ImuState", "propagate", "GRAVITY_WORLD"):
        from src.fusion import imu_model
        return getattr(imu_model, name)
    raise AttributeError(f"module 'src.fusion' has no attribute {name!r}")
