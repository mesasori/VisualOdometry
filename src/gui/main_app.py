"""MainApp — главное окно `python -m src.gui` с двумя вкладками.

Слой над ChooserTab + ComparatorTab. Содержит ttk.Notebook с двумя
фреймами, общий стиль, и интеграцию «после прогона предложить перейти
в comparator с предзаполненной галкой».
"""
from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk

from src.eval.runner import RunResult
from src.gui.chooser import ChooserTab
from src.gui.comparator import ComparatorTab


# Цвета общей светлой темы — единый стандарт для скриншотов в диплом
# (по требованиям РПЗ изображения вставляются только на белом фоне).
BG_COLOR = "#ffffff"
FG_COLOR = "#1a1a1a"
MUTED_FG = "#555555"
ACCENT_COLOR = "#0a66c2"
BORDER_COLOR = "#c8c8c8"
HOVER_BG = "#eef2f7"
SELECTED_BG = "#dbe7f5"


def apply_white_theme(root: tk.Misc) -> ttk.Style:
    """Применяет единый «белый» ttk-стиль к окну.

    Использует тему ``clam`` (надёжно перекрывается на macOS/Linux/Windows)
    и переопределяет фон/текст у всех ttk-виджетов, которые встречаются
    в стенде. Возвращает объект Style на случай, если вызывающий код
    хочет добавить дополнительные настройки.
    """
    root.configure(bg=BG_COLOR)
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass

    style.configure(
        ".",
        background=BG_COLOR,
        foreground=FG_COLOR,
        fieldbackground=BG_COLOR,
        bordercolor=BORDER_COLOR,
        lightcolor=BG_COLOR,
        darkcolor=BORDER_COLOR,
        troughcolor=BG_COLOR,
    )
    style.configure("TFrame", background=BG_COLOR)
    style.configure("TLabel", background=BG_COLOR, foreground=FG_COLOR)
    style.configure(
        "TLabelframe",
        background=BG_COLOR,
        bordercolor=BORDER_COLOR,
        relief="solid",
    )
    style.configure(
        "TLabelframe.Label", background=BG_COLOR, foreground=FG_COLOR
    )
    style.configure(
        "TButton",
        background=BG_COLOR,
        foreground=FG_COLOR,
        bordercolor=BORDER_COLOR,
        relief="solid",
        padding=(8, 4),
    )
    style.map(
        "TButton",
        background=[("active", HOVER_BG), ("pressed", SELECTED_BG)],
        foreground=[("disabled", "#9a9a9a")],
    )
    style.configure(
        "TCheckbutton", background=BG_COLOR, foreground=FG_COLOR
    )
    style.map(
        "TCheckbutton",
        background=[("active", BG_COLOR)],
    )
    style.configure(
        "TCombobox",
        fieldbackground=BG_COLOR,
        background=BG_COLOR,
        foreground=FG_COLOR,
        bordercolor=BORDER_COLOR,
        arrowcolor=FG_COLOR,
    )
    style.map(
        "TCombobox",
        fieldbackground=[("readonly", BG_COLOR), ("disabled", "#f4f4f4")],
        background=[("readonly", BG_COLOR)],
        foreground=[("disabled", "#9a9a9a")],
    )
    style.configure(
        "TEntry",
        fieldbackground=BG_COLOR,
        foreground=FG_COLOR,
        bordercolor=BORDER_COLOR,
    )
    style.configure(
        "TNotebook", background=BG_COLOR, bordercolor=BORDER_COLOR
    )
    style.configure(
        "TNotebook.Tab",
        background=BG_COLOR,
        foreground=FG_COLOR,
        padding=(14, 6),
        bordercolor=BORDER_COLOR,
    )
    style.map(
        "TNotebook.Tab",
        background=[("selected", BG_COLOR), ("active", HOVER_BG)],
        foreground=[("selected", ACCENT_COLOR)],
    )
    style.configure(
        "Horizontal.TProgressbar",
        background=ACCENT_COLOR,
        troughcolor="#eaeaea",
        bordercolor=BORDER_COLOR,
        lightcolor=ACCENT_COLOR,
        darkcolor=ACCENT_COLOR,
    )
    style.configure(
        "TPanedwindow", background=BG_COLOR, bordercolor=BORDER_COLOR
    )
    style.configure("TScrollbar", background=BG_COLOR, troughcolor=BG_COLOR)
    style.configure(
        "Treeview",
        background=BG_COLOR,
        fieldbackground=BG_COLOR,
        foreground=FG_COLOR,
        bordercolor=BORDER_COLOR,
    )
    style.configure(
        "Treeview.Heading",
        background=BG_COLOR,
        foreground=FG_COLOR,
        bordercolor=BORDER_COLOR,
        relief="flat",
    )
    style.map(
        "Treeview",
        background=[("selected", SELECTED_BG)],
        foreground=[("selected", FG_COLOR)],
    )
    return style


class MainApp(tk.Tk):
    """Главное окно: ttk.Notebook(ChooserTab, ComparatorTab).

    Args:
        initial_tab: 'chooser' или 'comparator' — какая вкладка активна
            при запуске. Используется для legacy shortcuts
            python -m src.gui.chooser / python -m src.gui.comparator.
    """

    def __init__(self, initial_tab: str = "chooser") -> None:
        super().__init__()
        self.title("Стенд исследования визуальной одометрии")
        self.geometry("1280x780")
        self.minsize(1100, 680)

        apply_white_theme(self)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.chooser_tab = ChooserTab(
            self.notebook, on_run_finished=self._on_run_finished
        )
        self.comparator_tab = ComparatorTab(self.notebook)

        self.notebook.add(self.chooser_tab, text="Запустить прогон")
        self.notebook.add(self.comparator_tab, text="Сравнить траектории")

        # Когда пользователь активирует вкладку comparator — рефрешим список
        # на случай если за время работы chooser появились новые файлы.
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        if initial_tab == "comparator":
            self.notebook.select(self.comparator_tab)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_tab_changed(self, _event=None) -> None:
        if self.notebook.select() == str(self.comparator_tab):
            self.comparator_tab.refresh()

    def _on_run_finished(self, result: RunResult) -> None:
        """Вызывается ChooserTab после успешного прогона.

        Предлагает открыть comparator с предзаполненной галкой на свежий
        прогон, чтобы пользователь сразу увидел overlay.
        """
        if result.trajectory_path is None:
            return
        answer = messagebox.askyesno(
            "Прогон завершён",
            f"Сохранено: {result.trajectory_path.name}\n\n"
            "Открыть «Сравнить траектории» и предвыбрать этот прогон?",
        )
        if not answer:
            return
        self.comparator_tab.refresh()
        self.comparator_tab.select_trajectory_by_path(result.trajectory_path)
        self.notebook.select(self.comparator_tab)


def main(initial_tab: str = "chooser") -> int:
    app = MainApp(initial_tab=initial_tab)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
