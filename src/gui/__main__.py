"""Точка входа `python -m src.gui` — открывает MainApp с табами.

Поддерживает `--tab chooser|comparator` для прямого открытия конкретной
вкладки (используется legacy shortcut'ами python -m src.gui.chooser
и python -m src.gui.comparator).
"""
from __future__ import annotations

import argparse
import sys

from src.gui.main_app import main


def _cli() -> int:
    parser = argparse.ArgumentParser(prog="python -m src.gui", description="VO Stand GUI")
    parser.add_argument(
        "--tab",
        choices=["chooser", "comparator"],
        default="chooser",
        help="Какую вкладку открыть при старте (по умолчанию chooser).",
    )
    args = parser.parse_args()
    return main(initial_tab=args.tab)


if __name__ == "__main__":
    raise SystemExit(_cli())
