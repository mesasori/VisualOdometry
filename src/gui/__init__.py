"""gui: единый Tkinter-интерфейс — chooser для запуска и comparator для сравнения.

Точки входа:
* `python -m src.gui` — главное окно с двумя вкладками (см.
  [main_app.MainApp](main_app.py)).
* `python -m src.gui.chooser` — shortcut на вкладку chooser
  (см. [chooser.main](chooser.py)).
* `python -m src.gui.comparator` — shortcut на вкладку comparator
  (см. [comparator.main](comparator.py)).
"""

__all__ = [
    "MainApp",
    "ChooserTab",
    "ComparatorTab",
    "LiveVisualizer",
]


def __getattr__(name: str):
    """Lazy-import чтобы Tk-импорт случайно не происходил при
    `from src.gui import ...` в местах, где он не нужен."""
    if name == "MainApp":
        from src.gui.main_app import MainApp

        return MainApp
    if name == "ChooserTab":
        from src.gui.chooser import ChooserTab

        return ChooserTab
    if name == "ComparatorTab":
        from src.gui.comparator import ComparatorTab

        return ComparatorTab
    if name == "LiveVisualizer":
        from src.gui.live_visualizer import LiveVisualizer

        return LiveVisualizer
    raise AttributeError(f"module 'src.gui' has no attribute {name!r}")
