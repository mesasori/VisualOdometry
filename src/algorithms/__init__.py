"""algorithms: реализации Visual Odometry, реализующие интерфейс VOAlgorithm."""

ALGORITHMS: dict = {}


def register_algorithm(key: str):
    """Декоратор для регистрации алгоритма в общем реестре.

    Используется в mono_fast_lk, stereo_sgbm_pnp, mono_imu_ekf и т.п.
    """
    def decorator(cls):
        ALGORITHMS[key] = cls
        return cls
    return decorator


def import_all_algorithms() -> None:
    """Импортирует все известные модули алгоритмов, чтобы их декораторы
    зарегистрировались в ALGORITHMS."""
    from . import mono_fast_lk  # noqa: F401
    from . import stereo_sgbm_pnp  # noqa: F401
    from . import mono_imu_ekf  # noqa: F401
    from . import stereo_imu_ekf  # noqa: F401


__all__ = [
    "ALGORITHMS",
    "register_algorithm",
    "import_all_algorithms",
]
