"""
Serial communication wrapper for Arduino Uno (COM4, 9600 baud).
Protocol: sends servo angles as <a0,a1,a2,a3,a4,a5>\n
"""

import serial
import serial.tools.list_ports
from typing import List, Optional


class SerialComm:
    def __init__(self, port: str = "COM4", baud: int = 9600):
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> bool:
        """Open serial connection. Returns True on success."""
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"[SERIAL] Connected to {self.port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"[SERIAL] Connection failed: {e}")
            self._serial = None
            return False

    def disconnect(self):
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            print("[SERIAL] Disconnected")
        self._serial = None

    def send_angles(self, angles: List[float]) -> bool:
        """
        Send 6 servo angles to Arduino.
        Format: <a0,a1,a2,a3,a4,a5>\n
        Angles are rounded to 1 decimal place.
        """
        if not self.is_connected:
            return False
        try:
            angles_str = ",".join(f"{a:.1f}" for a in angles[:6])
            command = f"<{angles_str}>\n"
            self._serial.write(command.encode("utf-8"))
            return True
        except serial.SerialException as e:
            print(f"[SERIAL] Send failed: {e}")
            return False

    @staticmethod
    def list_ports() -> List[str]:
        """List available serial ports."""
        return [p.device for p in serial.tools.list_ports.comports()]
