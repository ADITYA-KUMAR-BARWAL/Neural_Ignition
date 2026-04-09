"""
Main window layout: assembles control panel + 3D view with signal wiring.
"""

from PySide6.QtWidgets import QMainWindow, QSplitter, QWidget, QHBoxLayout
from PySide6.QtCore import Qt

from control_panel import ControlPanel
from visualization import StewartPlatformView
from kinematics import StewartKinematics
from communication import SerialComm


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STEWART PLATFORM — 6-DOF MISSION CONTROL")
        self.setMinimumSize(1200, 700)
        self.resize(1400, 800)

        # Core modules
        self.kin = StewartKinematics()
        self.serial = SerialComm(port="COM4", baud=9600)

        # Widgets
        self.panel = ControlPanel()
        self.view3d = StewartPlatformView()

        # Layout: splitter with control panel left, 3D view right
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.panel)
        splitter.addWidget(self.view3d)
        splitter.setStretchFactor(0, 0)   # panel fixed width
        splitter.setStretchFactor(1, 1)   # 3D view stretches
        splitter.setHandleWidth(2)

        central = QWidget()
        central.setObjectName("centralWidget")
        lay = QHBoxLayout(central)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(splitter)
        self.setCentralWidget(central)

        # Status bar
        self.statusBar().showMessage("Ready  |  Drag to orbit  |  Scroll to zoom")

        # Menu bar
        self._build_menu()

        # Wire signals
        self.panel.poseChanged.connect(self._on_pose)
        self.panel.geometryChanged.connect(self._on_geometry)

        # Initial computation
        self._on_pose(0, 0, 0, 0, 0, 0)

        # Try to connect to Arduino
        self._try_connect()

    def _build_menu(self):
        menu = self.menuBar()

        file_menu = menu.addMenu("File")
        file_menu.addAction("Exit", self.close)

        conn_menu = menu.addMenu("Connection")
        conn_menu.addAction("Connect", self._try_connect)
        conn_menu.addAction("Disconnect", self._disconnect)

        view_menu = menu.addMenu("View")
        view_menu.addAction("Reset Camera", self.view3d.reset_camera)
        view_menu.addAction("Reset Pose", self.panel.reset_pose)

    def _try_connect(self):
        ok = self.serial.connect()
        self.panel.set_connection_status(ok)
        if ok:
            self.statusBar().showMessage("Connected to COM4")
        else:
            self.statusBar().showMessage("Arduino not found — running offline")

    def _disconnect(self):
        self.serial.disconnect()
        self.panel.set_connection_status(False)
        self.statusBar().showMessage("Disconnected")

    def _on_pose(self, x, y, z, roll, pitch, yaw):
        """Recompute IK and update everything."""
        angles, base, horns, plat, center, valid = \
            self.kin.inverse_kinematics(x, y, z, roll, pitch, yaw)

        self.view3d.update_platform(base, horns, plat, center, valid)
        self.panel.update_servo_angles(angles, valid)
        self.serial.send_angles(angles.tolist())

        status = "VALID" if valid else "⚠ OUT OF RANGE"
        self.statusBar().showMessage(
            f"Pose: ({x:.1f}, {y:.1f}, {z:.1f}) mm  "
            f"({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)  |  {status}"
        )

    def _on_geometry(self, base_r, plat_r, horn, rod):
        """Update geometry parameters and recompute."""
        try:
            self.kin.update_params(
                base_radius=base_r,
                platform_radius=plat_r,
                horn_length=horn,
                rod_length=rod
            )
            self.view3d.cam_target[2] = self.kin.home_height / 2
            # Recompute with current pose
            self._on_pose(
                self.panel.sl_x.value(),
                self.panel.sl_y.value(),
                self.panel.sl_z.value(),
                self.panel.sl_roll.value(),
                self.panel.sl_pitch.value(),
                self.panel.sl_yaw.value()
            )
        except ValueError as e:
            self.statusBar().showMessage(f"Geometry error: {e}")

    def closeEvent(self, event):
        self.serial.disconnect()
        event.accept()
