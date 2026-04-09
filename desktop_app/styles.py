"""
Neon Cyberpunk Stylesheet for Stewart Platform GUI.
Matches the reference design: dark bg, neon cyan primary, neon pink secondary.
"""

COLORS = {
    "bg_darkest": "#0b0f19",
    "bg_dark": "#111827",
    "bg_panel": "#161b22",
    "bg_input": "#1a2332",
    "cyan": "#00ffcc",
    "cyan_dim": "#00aa88",
    "pink": "#ff2d6a",
    "pink_dim": "#cc2255",
    "orange": "#ff6a00",
    "green": "#33ff66",
    "yellow": "#ffcc00",
    "white": "#e0e0e0",
    "gray": "#555555",
}

NEON_STYLESHEET = """
/* ===== Main Window ===== */
QMainWindow {
    background-color: #0b0f19;
}

QWidget#centralWidget {
    background-color: #0b0f19;
}

/* ===== Splitter ===== */
QSplitter::handle {
    background-color: #1a2332;
    width: 2px;
}

/* ===== Scroll Area ===== */
QScrollArea {
    background-color: #0b0f19;
    border: none;
}
QScrollArea > QWidget > QWidget {
    background-color: #0b0f19;
}
QScrollBar:vertical {
    background: #111827;
    width: 8px;
    border-radius: 4px;
}
QScrollBar::handle:vertical {
    background: #00aa88;
    border-radius: 4px;
    min-height: 30px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* ===== Group Boxes ===== */
QGroupBox {
    background-color: #111827;
    border: 1px solid #00aa88;
    border-radius: 8px;
    margin-top: 14px;
    padding: 18px 10px 10px 10px;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 13px;
    font-weight: bold;
    color: #00ffcc;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 12px;
    background-color: #111827;
    border: 1px solid #00ffcc;
    border-radius: 4px;
    color: #00ffcc;
    font-size: 12px;
    letter-spacing: 1px;
}

/* ===== Labels ===== */
QLabel {
    color: #00ffcc;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 13px;
    font-weight: bold;
    background: transparent;
}
QLabel#sectionLabel {
    color: #ff2d6a;
    font-size: 15px;
    letter-spacing: 2px;
}
QLabel#valueLabel {
    color: #ffcc00;
    font-size: 12px;
}
QLabel#statusConnected {
    color: #33ff66;
    font-size: 12px;
}
QLabel#statusDisconnected {
    color: #ff2d6a;
    font-size: 12px;
}

/* ===== Line Edits ===== */
QLineEdit {
    background-color: #1a2332;
    color: #00ffcc;
    border: 2px solid #00aa88;
    border-radius: 5px;
    padding: 4px 6px;
    font-family: 'Consolas', monospace;
    font-size: 13px;
    font-weight: bold;
    selection-background-color: #00ffcc;
    selection-color: #0b0f19;
}
QLineEdit:focus {
    border-color: #00ffcc;
}
QLineEdit#pinkInput {
    border-color: #cc2255;
    color: #ff2d6a;
}
QLineEdit#pinkInput:focus {
    border-color: #ff2d6a;
}

/* ===== Sliders ===== */
QSlider::groove:horizontal {
    border: none;
    height: 6px;
    background: #1a2332;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
        fx:0.5, fy:0.3, stop:0 #00ffcc, stop:1 #00aa88);
    border: 2px solid #00ffcc;
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 9px;
}
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #00aa88, stop:1 #00ffcc);
    border-radius: 3px;
}
QSlider#pinkSlider::handle:horizontal {
    background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
        fx:0.5, fy:0.3, stop:0 #ff2d6a, stop:1 #cc2255);
    border: 2px solid #ff2d6a;
}
QSlider#pinkSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #cc2255, stop:1 #ff2d6a);
}

/* ===== Push Buttons ===== */
QPushButton {
    background-color: #1a2332;
    color: #00ffcc;
    border: 2px solid #00ffcc;
    border-radius: 6px;
    padding: 6px 16px;
    font-family: 'Consolas', monospace;
    font-size: 12px;
    font-weight: bold;
    letter-spacing: 1px;
}
QPushButton:hover {
    background-color: #00ffcc;
    color: #0b0f19;
}
QPushButton:pressed {
    background-color: #00aa88;
}
QPushButton#pinkButton {
    border-color: #ff2d6a;
    color: #ff2d6a;
}
QPushButton#pinkButton:hover {
    background-color: #ff2d6a;
    color: #0b0f19;
}

/* ===== Table Widget ===== */
QTableWidget {
    background-color: #111827;
    color: #00ffcc;
    border: 1px solid #00aa88;
    border-radius: 4px;
    gridline-color: #1a2332;
    font-family: 'Consolas', monospace;
    font-size: 12px;
    selection-background-color: #1a2332;
}
QTableWidget::item {
    padding: 4px;
    border: none;
}
QHeaderView::section {
    background-color: #161b22;
    color: #ff2d6a;
    border: 1px solid #1a2332;
    padding: 4px;
    font-family: 'Consolas', monospace;
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 1px;
}

/* ===== Status Bar ===== */
QStatusBar {
    background-color: #111827;
    color: #00ffcc;
    border-top: 1px solid #00aa88;
    font-family: 'Consolas', monospace;
    font-size: 11px;
}

/* ===== Menu Bar ===== */
QMenuBar {
    background-color: #111827;
    color: #00ffcc;
    border-bottom: 1px solid #00aa88;
    font-family: 'Consolas', monospace;
    font-size: 12px;
}
QMenuBar::item:selected {
    background-color: #1a2332;
}
QMenu {
    background-color: #161b22;
    color: #00ffcc;
    border: 1px solid #00aa88;
}
QMenu::item:selected {
    background-color: #00ffcc;
    color: #0b0f19;
}
"""
