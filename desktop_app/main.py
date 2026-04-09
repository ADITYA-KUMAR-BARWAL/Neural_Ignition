"""
Stewart Platform 6-DOF Mission Control — Entry Point.
"""

import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QFont
from styles import NEON_STYLESHEET
from ui_layout import MainWindow


def main():
    app = QApplication(sys.argv)

    # Apply neon cyberpunk theme
    app.setStyleSheet(NEON_STYLESHEET)

    # Set monospace font
    font = QFont("Consolas", 10)
    font.setStyleHint(QFont.Monospace)
    app.setFont(font)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
