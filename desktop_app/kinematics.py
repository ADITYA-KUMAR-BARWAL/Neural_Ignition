"""
Inverse Kinematics engine for a 6-DOF rotary Stewart platform.
Computes servo angles from desired platform pose (x, y, z, roll, pitch, yaw).
"""

import numpy as np
from typing import Tuple


class StewartKinematics:
    def __init__(self,
                 base_radius: float = 80.0,
                 platform_radius: float = 50.0,
                 horn_length: float = 25.0,
                 rod_length: float = 130.0,
                 base_spread: float = 7.5,
                 platform_spread: float = 20.0):
        self.base_radius = base_radius
        self.platform_radius = platform_radius
        self.horn_length = horn_length
        self.rod_length = rod_length
        self.base_spread = base_spread
        self.platform_spread = platform_spread
        self._compute_geometry()

    def _compute_geometry(self):
        """Compute anchor positions, servo directions, and home height."""
        # Base anchor angles: 3 pairs at 0°, 120°, 240° with ±spread
        self.base_angles_deg = []
        for i in range(3):
            center = i * 120.0
            self.base_angles_deg.append(center - self.base_spread)
            self.base_angles_deg.append(center + self.base_spread)

        # Platform anchor angles: 3 pairs at 60°, 180°, 300° with ±spread
        # Ordering matches base pairing for leg connections
        self.platform_angles_deg = []
        for i in range(3):
            center = 60.0 + i * 120.0
            self.platform_angles_deg.append(center + self.platform_spread)
            self.platform_angles_deg.append(center - self.platform_spread)

        self.base_angles_rad = np.radians(self.base_angles_deg)
        self.platform_angles_rad = np.radians(self.platform_angles_deg)

        # 3D positions
        self.base_anchors = np.zeros((6, 3))
        self.platform_anchors_local = np.zeros((6, 3))
        for i in range(6):
            self.base_anchors[i] = [
                self.base_radius * np.cos(self.base_angles_rad[i]),
                self.base_radius * np.sin(self.base_angles_rad[i]),
                0.0
            ]
            self.platform_anchors_local[i] = [
                self.platform_radius * np.cos(self.platform_angles_rad[i]),
                self.platform_radius * np.sin(self.platform_angles_rad[i]),
                0.0
            ]

        # Servo horn sweep direction: alternating for paired servos
        self.servo_dir = np.array([1, -1, 1, -1, 1, -1], dtype=float)

        # Home height (average across all legs)
        d_sq = 0.0
        for i in range(6):
            dx = self.base_anchors[i, 0] - self.platform_anchors_local[i, 0]
            dy = self.base_anchors[i, 1] - self.platform_anchors_local[i, 1]
            d_sq += dx * dx + dy * dy
        d_sq_avg = d_sq / 6.0

        disc = self.rod_length ** 2 - d_sq_avg
        if disc < 0:
            raise ValueError("Rod too short for geometry! Increase rod_length.")
        self.home_height = self.horn_length + np.sqrt(disc)

    @staticmethod
    def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        """ZYX Euler rotation matrix from degrees."""
        r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])

    def inverse_kinematics(self, x: float, y: float, z: float,
                           roll: float, pitch: float, yaw: float
                           ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]:
        """
        Compute inverse kinematics for desired pose.

        Returns:
            servo_angles (6,): degrees [0-180]
            base_positions (6,3): base anchor XYZ
            horn_tips (6,3): horn tip XYZ
            platform_positions (6,3): platform anchor XYZ in world frame
            platform_center (3,): center of moved platform
            valid: True if all angles in range
        """
        R = self.rotation_matrix(roll, pitch, yaw)
        T = np.array([x, y, z + self.home_height])

        platform_world = np.zeros((6, 3))
        horn_tips = np.zeros((6, 3))
        servo_angles = np.zeros(6)
        valid = True

        for i in range(6):
            # Platform anchor in world frame
            Ti = T + R @ self.platform_anchors_local[i]
            platform_world[i] = Ti

            # Leg vector from base anchor to platform anchor
            L = Ti - self.base_anchors[i]
            L_sq = np.dot(L, L)

            # Radial direction for this servo
            beta = self.base_angles_rad[i]
            r_hat = np.array([np.cos(beta), np.sin(beta), 0.0])
            d = self.servo_dir[i]

            # Solve: g = e*cos(α) + f*sin(α)
            e = d * np.dot(r_hat, L)
            f = L[2]
            g = (self.horn_length ** 2 + L_sq - self.rod_length ** 2) / (2.0 * self.horn_length)

            ef_mag = np.sqrt(e * e + f * f)
            if ef_mag < 1e-6:
                valid = False
                servo_angles[i] = 90.0
                horn_tips[i] = self.base_anchors[i] + np.array([0, 0, self.horn_length])
                continue

            cos_arg = np.clip(g / ef_mag, -1.0, 1.0)
            if abs(g / ef_mag) > 1.0:
                valid = False

            base_a = np.arctan2(f, e)
            offset = np.arccos(cos_arg)

            # Two candidate solutions
            a1 = np.degrees(base_a + offset)
            a2 = np.degrees(base_a - offset)

            # Pick solution closest to 90° within [0, 180]
            best = 90.0
            best_dist = 999.0
            for cand in [a1, a2, a1 + 360, a2 + 360, a1 - 360, a2 - 360]:
                if -5 <= cand <= 185:
                    dist = abs(cand - 90.0)
                    if dist < best_dist:
                        best_dist = dist
                        best = cand

            servo_angles[i] = np.clip(best, 0.0, 180.0)

            # Compute horn tip position
            alpha_r = np.radians(servo_angles[i])
            horn_tips[i] = (self.base_anchors[i]
                            + self.horn_length * np.cos(alpha_r) * d * r_hat
                            + self.horn_length * np.sin(alpha_r) * np.array([0, 0, 1]))

        platform_center = T.copy()
        return servo_angles, self.base_anchors.copy(), horn_tips, platform_world, platform_center, valid

    def update_params(self, **kwargs):
        """Update geometry parameters and recompute."""
        for k, v in kwargs.items():
            if hasattr(self, k):
                setattr(self, k, v)
        self._compute_geometry()
