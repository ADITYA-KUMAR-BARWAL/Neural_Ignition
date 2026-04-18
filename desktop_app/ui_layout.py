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
        self.serial = SerialComm(port="COM4", baud=115200)

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
        self.panel.manualServoOverride.connect(self._on_manual_servo)
        self.panel.zeroHardware.connect(self._on_zero_hardware)

        # Serial send is suppressed until user actively moves a slider.
        # This prevents the GUI from overriding the Arduino's safe home
        # position (50°) on startup.
        self._armed = False

        # Initial computation (visualization only, no serial send)
        self._on_pose(0, 0, 0, 0, 0, 0)

        # Override table to match firmware safe home (127°)
        self.panel.update_servo_angles([127.0] * 6, True)

        # Now arm for future interactions
        self._armed = True

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

        self.view3d.update_platform(
            base, horns, plat, center, valid,
            servo_angles=angles,
            base_angles=self.kin.base_angles_rad,
            servo_dir=self.kin.servo_dir,
            horn_length=self.kin.horn_length,
        )
        self.panel.update_servo_angles(angles, valid)

        # Only send to Arduino after user has actively changed a slider
        if self._armed:
            self.serial.send_angles(angles.tolist())

        status = "VALID" if valid else "⚠ OUT OF RANGE"
        self.statusBar().showMessage(
            f"Pose: ({x:.1f}, {y:.1f}, {z:.1f}) mm  "
            f"({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)  |  {status}"
        )

    def _on_geometry(self, horn, rod):
        """Update geometry parameters and recompute."""
        try:
            self.kin.update_params(
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

    def _on_manual_servo(self, angles):
        """User manually typed servo angles — send directly to Arduino."""
        self.serial.send_angles(angles)
        self.statusBar().showMessage(
            f"Manual Override: [{', '.join(f'{a:.1f}°' for a in angles)}]"
        )

    def _on_zero_hardware(self):
        """Re-sync the digital twin to physical home position.

        Resets all IK sliders to zero and recomputes angles to home,
        but does NOT send any serial commands. The user is expected to
        have manually positioned the physical servos to the level/home
        pose before clicking this button.
        """
        # Temporarily disarm serial to suppress any sends during reset
        was_armed = self._armed
        self._armed = False

        # Reset all sliders to zero (home pose)
        self.panel.reset_pose()

        # Recompute IK at home pose — updates visualization + table
        self._on_pose(0, 0, 0, 0, 0, 0)

        # Restore armed state
        self._armed = was_armed

        self.statusBar().showMessage(
            "⌖ Hardware zeroed — digital twin re-synced to home position"
        )

    def closeEvent(self, event):
        self.serial.disconnect()
        event.accept()
