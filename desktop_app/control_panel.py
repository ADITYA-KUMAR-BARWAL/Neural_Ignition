"""
Left-side control panel: dimension inputs, 6 DOF sliders, servo angle table.
"""

import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QSlider, QPushButton, QGroupBox,
    QTableWidget, QTableWidgetItem, QScrollArea, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, Signal


class _DOFSlider(QWidget):
    """Single DOF control: label + slider + value box."""
    valueChanged = Signal(float)

    def __init__(self, name: str, min_val: float, max_val: float,
                 unit: str = "mm", resolution: int = 10, pink: bool = False):
        super().__init__()
        self._res = resolution
        self._min = min_val
        self._max = max_val
        self._block = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)

        lbl = QLabel(f"{name}:")
        lbl.setFixedWidth(30)
        layout.addWidget(lbl)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(min_val * resolution), int(max_val * resolution))
        self.slider.setValue(0)
        if pink:
            self.slider.setObjectName("pinkSlider")
        layout.addWidget(self.slider, 1)

        self.value_box = QLineEdit("0.0")
        self.value_box.setFixedWidth(60)
        self.value_box.setAlignment(Qt.AlignCenter)
        if pink:
            self.value_box.setObjectName("pinkInput")
        layout.addWidget(self.value_box)

        unit_lbl = QLabel(unit)
        unit_lbl.setFixedWidth(22)
        layout.addWidget(unit_lbl)

        self.slider.valueChanged.connect(self._on_slider)
        self.value_box.editingFinished.connect(self._on_text)

    def _on_slider(self, raw):
        if self._block:
            return
        val = raw / self._res
        self._block = True
        self.value_box.setText(f"{val:.1f}")
        self._block = False
        self.valueChanged.emit(val)

    def _on_text(self):
        if self._block:
            return
        try:
            val = float(self.value_box.text())
            val = max(self._min, min(self._max, val))
            self._block = True
            self.slider.setValue(int(val * self._res))
            self.value_box.setText(f"{val:.1f}")
            self._block = False
            self.valueChanged.emit(val)
        except ValueError:
            pass

    def set_value(self, v: float):
        self._block = True
        v = max(self._min, min(self._max, v))
        self.slider.setValue(int(v * self._res))
        self.value_box.setText(f"{v:.1f}")
        self._block = False

    def value(self) -> float:
        return self.slider.value() / self._res


class ControlPanel(QWidget):
    """Left panel containing dimension controls, IK sliders, and servo readout."""
    poseChanged = Signal(float, float, float, float, float, float)  # x y z roll pitch yaw
    geometryChanged = Signal(float, float, float, float)  # base_r plat_r horn rod

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(380)
        self._build_ui()

    def _build_ui(self):
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)

        container = QWidget()
        main_lay = QVBoxLayout(container)
        main_lay.setSpacing(10)
        main_lay.setContentsMargins(10, 10, 10, 10)

        # ── Title ────
        title = QLabel("⬡ STEWART PLATFORM")
        title.setObjectName("sectionLabel")
        title.setAlignment(Qt.AlignCenter)
        main_lay.addWidget(title)

        # ── Dimensions ────
        dim_group = QGroupBox("✦ DIMENSIONS")
        dim_grid = QGridLayout()
        dim_grid.setSpacing(6)

        dim_labels = ["base_r", "plat_r", "horn", "rod"]
        dim_defaults = [80.0, 50.0, 25.0, 130.0]
        self.dim_inputs = {}

        for col, (name, default) in enumerate(zip(dim_labels, dim_defaults)):
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            dim_grid.addWidget(lbl, 0, col)
            inp = QLineEdit(str(default))
            inp.setFixedWidth(65)
            inp.setAlignment(Qt.AlignCenter)
            inp.setObjectName("pinkInput")
            inp.editingFinished.connect(self._on_dim_changed)
            dim_grid.addWidget(inp, 1, col)
            self.dim_inputs[name] = inp

        dim_group.setLayout(dim_grid)
        main_lay.addWidget(dim_group)

        # ── Inverse Kinematics ────
        ik_group = QGroupBox("✦ INVERSE KINEMATICS")
        ik_lay = QVBoxLayout()
        ik_lay.setSpacing(4)

        # Translation sliders
        t_label = QLabel("— Translation —")
        t_label.setObjectName("valueLabel")
        t_label.setAlignment(Qt.AlignCenter)
        ik_lay.addWidget(t_label)

        self.sl_x = _DOFSlider("tx", -30, 30, "mm")
        self.sl_y = _DOFSlider("ty", -30, 30, "mm")
        self.sl_z = _DOFSlider("tz", -20, 20, "mm")
        for s in (self.sl_x, self.sl_y, self.sl_z):
            s.valueChanged.connect(self._on_pose_changed)
            ik_lay.addWidget(s)

        # Rotation sliders (pink accented)
        r_label = QLabel("— Rotation —")
        r_label.setObjectName("valueLabel")
        r_label.setAlignment(Qt.AlignCenter)
        ik_lay.addWidget(r_label)

        self.sl_roll  = _DOFSlider("rx", -25, 25, "°", pink=True)
        self.sl_pitch = _DOFSlider("ry", -25, 25, "°", pink=True)
        self.sl_yaw   = _DOFSlider("rz", -25, 25, "°", pink=True)
        for s in (self.sl_roll, self.sl_pitch, self.sl_yaw):
            s.valueChanged.connect(self._on_pose_changed)
            ik_lay.addWidget(s)

        ik_group.setLayout(ik_lay)
        main_lay.addWidget(ik_group)

        # ── Reset button ────
        btn_row = QHBoxLayout()
        self.reset_btn = QPushButton("⟲  RESET")
        self.reset_btn.setObjectName("pinkButton")
        self.reset_btn.clicked.connect(self.reset_pose)
        btn_row.addWidget(self.reset_btn)
        main_lay.addLayout(btn_row)

        # ── Servo Angles Table ────
        servo_group = QGroupBox("✦ SERVO ANGLES")
        servo_lay = QVBoxLayout()

        self.servo_table = QTableWidget(6, 2)
        self.servo_table.setHorizontalHeaderLabels(["SERVO", "ANGLE (°)"])
        self.servo_table.horizontalHeader().setStretchLastSection(True)
        self.servo_table.verticalHeader().setVisible(False)
        self.servo_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.servo_table.setSelectionMode(QTableWidget.NoSelection)
        self.servo_table.setMaximumHeight(210)

        for i in range(6):
            name_item = QTableWidgetItem(f"  S{i+1}")
            angle_item = QTableWidgetItem("90.0")
            angle_item.setTextAlignment(Qt.AlignCenter)
            self.servo_table.setItem(i, 0, name_item)
            self.servo_table.setItem(i, 1, angle_item)

        servo_lay.addWidget(self.servo_table)
        servo_group.setLayout(servo_lay)
        main_lay.addWidget(servo_group)

        # ── Connection status ────
        self.conn_label = QLabel("● OFFLINE")
        self.conn_label.setObjectName("statusDisconnected")
        self.conn_label.setAlignment(Qt.AlignCenter)
        main_lay.addWidget(self.conn_label)

        main_lay.addStretch()

        scroll.setWidget(container)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    # ── Slots ───────────────────────────────────────────────────────
    def _on_pose_changed(self, _=None):
        self.poseChanged.emit(
            self.sl_x.value(), self.sl_y.value(), self.sl_z.value(),
            self.sl_roll.value(), self.sl_pitch.value(), self.sl_yaw.value()
        )

    def _on_dim_changed(self):
        try:
            vals = [float(self.dim_inputs[k].text())
                    for k in ("base_r", "plat_r", "horn", "rod")]
            self.geometryChanged.emit(*vals)
        except ValueError:
            pass

    def reset_pose(self):
        for s in (self.sl_x, self.sl_y, self.sl_z,
                  self.sl_roll, self.sl_pitch, self.sl_yaw):
            s.set_value(0.0)
        self._on_pose_changed()

    # ── Public ──────────────────────────────────────────────────────
    def update_servo_angles(self, angles, valid: bool):
        for i, a in enumerate(angles[:6]):
            item = self.servo_table.item(i, 1)
            if item:
                item.setText(f"{a:.1f}")

    def set_connection_status(self, connected: bool):
        if connected:
            self.conn_label.setText("● CONNECTED — COM4")
            self.conn_label.setObjectName("statusConnected")
        else:
            self.conn_label.setText("● OFFLINE")
            self.conn_label.setObjectName("statusDisconnected")
        self.conn_label.style().unpolish(self.conn_label)
        self.conn_label.style().polish(self.conn_label)
