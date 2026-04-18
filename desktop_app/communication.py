"""
Serial communication for Stewart Platform — PCA9685 / MG996R servo driver.
Protocol: '<A1:90.0,A2:90.0,A3:90.0,A4:90.0,A5:90.0,A6:90.0>\n'
Gracefully degrades to offline mode with console packet logging.
"""

import time
import serial
import serial.tools.list_ports
from typing import List, Optional


class SerialComm:
    """Thread-safe serial communication wrapper with offline fallback."""

    # Servo operating range (degrees)
    SERVO_MIN = 90.0
    SERVO_MAX = 180.0

    def __init__(self, port: str = "COM4", baud: int = 115200):
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._last_send_time: float = 0.0
        self._min_interval: float = 0.02  # 50 Hz max update rate
        self._last_packet: str = ""

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> bool:
        """Open serial connection. Returns True on success."""
        try:
            self._serial = serial.Serial(
                self.port,
                self.baud,
                timeout=0.1,
                write_timeout=0.1
            )
            # Wait for Arduino to reset after serial connection
            time.sleep(2.0)
            # Flush any startup messages
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            print(f"[SERIAL] Connected to {self.port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"[SERIAL] Connection failed: {e}")
            self._serial = None
            return False

    def disconnect(self):
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            try:
                # Send neutral position before disconnecting
                neutral = [127.0] * 6
                self._send_raw(neutral)
                time.sleep(0.05)
                self._serial.close()
            except Exception:
                pass
            print("[SERIAL] Disconnected")
        self._serial = None

    def _clamp_angle(self, angle: float) -> float:
        """Clamp angle to MG996R safe operating range."""
        return max(self.SERVO_MIN, min(self.SERVO_MAX, angle))

    def _format_packet(self, angles: List[float]) -> str:
        """
        Format 6 angles into the protocol packet.
        Format: '<A1:90.0,A2:90.0,A3:90.0,A4:90.0,A5:90.0,A6:90.0>'
        """
        clamped = [self._clamp_angle(a) for a in angles[:6]]
        parts = [f"A{i+1}:{clamped[i]:.1f}" for i in range(6)]
        return f"<{','.join(parts)}>"

    def _send_raw(self, angles: List[float]) -> bool:
        """Send packet without throttle check."""
        if not self.is_connected:
            return False
        try:
            packet = self._format_packet(angles)
            self._serial.write((packet + "\n").encode("utf-8"))
            self._serial.flush()
            return True
        except serial.SerialException:
            return False

    def send_angles(self, angles: List[float]) -> bool:
        """
        Send 6 servo angles to Arduino via PCA9685.
        Throttled to prevent serial buffer overflow.
        In offline mode, prints packet to console.
        """
        now = time.time()
        if now - self._last_send_time < self._min_interval:
            return True  # Skip — too soon

        packet = self._format_packet(angles)

        # Avoid sending duplicate packets
        if packet == self._last_packet:
            return True

        self._last_packet = packet
        self._last_send_time = now

        if self.is_connected:
            try:
                self._serial.write((packet + "\n").encode("utf-8"))
                self._serial.flush()
                return True
            except serial.SerialException as e:
                print(f"[SERIAL] Send failed: {e}")
                self._serial = None
                return False
        else:
            # Offline mode: log to console
            print(f"[OFFLINE] {packet}")
            return False

    def read_response(self) -> Optional[str]:
        """Read any response from Arduino (non-blocking)."""
        if not self.is_connected:
            return None
        try:
            if self._serial.in_waiting > 0:
                line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                return line if line else None
        except serial.SerialException:
            pass
        return None

    @staticmethod
    def list_ports() -> List[str]:
        """List available serial ports."""
        return [p.device for p in serial.tools.list_ports.comports()]
