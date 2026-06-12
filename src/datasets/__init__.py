"""datasets: загрузчики датасетов и реестр для GUI/runner."""
from .base import Dataset

DATASETS: dict = {}


def register_dataset(key: str):
    """Декоратор для регистрации датасета в общем реестре.

    Используется внутри модулей kitti_odometry, kitti_raw, samsung_s23.
    Делать импорты модулей в `__init__` нельзя (циркуляр + опц. зависимости),
    регистрация происходит ленива — импортировать модуль вручную или
    через `import_all_datasets()`.
    """
    def decorator(cls):
        DATASETS[key] = cls
        return cls
    return decorator


def import_all_datasets() -> None:
    """Принудительно импортирует все известные модули датасетов,
    чтобы их декораторы зарегистрировались в DATASETS."""
    from . import kitti_odometry  # noqa: F401
    from . import kitti_raw  # noqa: F401


__all__ = [
    "Dataset",
    "DATASETS",
    "register_dataset",
    "import_all_datasets",
]
