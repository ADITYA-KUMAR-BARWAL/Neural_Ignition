"""
Inverse Kinematics engine for a 6-DOF rotary Stewart platform.
Computes servo angles from desired platform pose (x, y, z, roll, pitch, yaw).

Hardware Geometry (all dimensions in mm):
  - Base: Truncated equilateral triangle
      Height (bottom edge to top flat): 119.87 mm
      Flat edge (truncation): 49.98 mm
      Servo pair spacing: ±10 mm from flat midpoint
      3 pairs on flats at 90°, 210°, 330°
  - Top platform: Hexagonal (truncated triangle)
      Long edge: 119 mm, Short edge: 21 mm
      Material width: 10 mm → 5 mm inset for ball-joint centers
      Rotated 60° from base for crossed-rod pattern
  - Each leg: servo horn of length horn_length + connecting rod of length rod_length
  - Servos alternate direction (outward/inward) for paired mounting.
"""

import numpy as np
from typing import Tuple


# ═══════════════════════════════════════════════════════════════════
# Hardware geometry constants (derived from CAD drawings)
# ═══════════════════════════════════════════════════════════════════

# ── Base truncated triangle ──────────────────────────────────────
# Original equilateral triangle circumradius (from height equation:
#   height = 3R/2 - t√3/2 = 119.87 → R = 108.77 mm)
_BASE_R = (119.87 + 49.98 * np.sqrt(3) / 2) * 2.0 / 3.0  # ≈ 108.77
_BASE_T = 49.98  # flat edge length (truncation)
_BASE_D_FLAT = _BASE_R - _BASE_T * np.sqrt(3) / 2.0       # ≈ 65.49 mm
_SERVO_PAIR_OFFSET = 10.0  # mm from flat midpoint to each servo pivot

# ── Top platform hexagon ─────────────────────────────────────────
_PLAT_LONG_EDGE = 119.0   # mm
_PLAT_SHORT_EDGE = 21.0   # mm
_PLAT_MATERIAL_W = 10.0   # mm (inset = 5 mm for ball-joint bore center)
_PLAT_S = _PLAT_LONG_EDGE + 2.0 * _PLAT_SHORT_EDGE  # original triangle side = 161 mm
_PLAT_R = _PLAT_S / np.sqrt(3)                        # circumradius ≈ 92.95 mm
_PLAT_T = _PLAT_SHORT_EDGE                            # truncation = 21 mm


def _build_base_anchors() -> np.ndarray:
    """
    Compute 6 servo pivot positions on the truncated-triangle base.

    The base has 3 flat edges (truncation cuts) facing 90°, 210°, 330°.
    Two servos sit on each flat, offset ±10 mm from the midpoint along the edge.

    Servo numbering (pairs, center ± spread):
      S1, S2 on 90°  flat  →  angles ≈  81°,  99°
      S3, S4 on 210° flat  →  angles ≈ 201°, 219°
      S5, S6 on 330° flat  →  angles ≈ 321°, 339°
    """
    anchors = np.zeros((6, 3))

    # Flat edge outward-normal directions (where original vertices were)
    flat_dirs = [90.0, 210.0, 330.0]

    # Tangent directions along each flat edge (perpendicular to outward normal)
    # For flat at θ°, the tangent along the edge in the H_start→H_end direction:
    flat_tangents = {
        90.0:  np.array([1.0, 0.0]),          # along +x
        210.0: np.array([-0.5, np.sqrt(3)/2]),  # along H4→H5
        330.0: np.array([-0.5, -np.sqrt(3)/2]), # along H2→H3
    }

    for pair_idx, theta_deg in enumerate(flat_dirs):
        theta = np.radians(theta_deg)
        midpoint = _BASE_D_FLAT * np.array([np.cos(theta), np.sin(theta)])
        tangent = flat_tangents[theta_deg]

        # S_odd (center - spread) and S_even (center + spread)
        # S_odd = midpoint + offset * tangent  (smaller angle from center)
        # S_even = midpoint - offset * tangent (larger angle from center)
        s_odd = midpoint + _SERVO_PAIR_OFFSET * tangent
        s_even = midpoint - _SERVO_PAIR_OFFSET * tangent

        # Ensure S_odd has the smaller angle within the pair
        angle_odd = np.degrees(np.arctan2(s_odd[1], s_odd[0])) % 360
        angle_even = np.degrees(np.arctan2(s_even[1], s_even[0])) % 360

        # For δ from center: pick the one closer to (center - spread)
        center_angle = theta_deg % 360
        d_odd = min(abs(angle_odd - center_angle), 360 - abs(angle_odd - center_angle))
        d_even = min(abs(angle_even - center_angle), 360 - abs(angle_even - center_angle))

        # Both should be roughly equal spread, assign by which is CW vs CCW
        # "center - spread" = smaller modular angle for most cases
        # Use angular order: whichever is CCW-earlier from center
        idx_a = pair_idx * 2      # S_odd index
        idx_b = pair_idx * 2 + 1  # S_even index

        anchors[idx_a, :2] = s_odd
        anchors[idx_b, :2] = s_even

    return anchors


def _build_platform_anchors() -> np.ndarray:
    """
    Compute 6 rod connection points on the hexagonal top platform.

    The platform is a truncated equilateral triangle (long edge 119 mm,
    short edge 21 mm). Rod ball-joints are at the hexagon vertices,
    inset by 5 mm (half material width) toward center.

    The platform is rotated 60° from the base to create the Stewart
    platform's characteristic crossed-rod pattern.

    Returns coordinates already cross-mapped to match base servo indices:
      S1 → nearest point on adjacent platform pair (38°)
      S2 → nearest point on opposite adjacent pair (142°)
      etc.
    """
    # Hexagon vertices (apex-up orientation, before rotation)
    # Using truncated triangle formulas with R_plat and t_plat
    R = _PLAT_R
    t = _PLAT_T
    half_t = t / 2.0
    t_s3_2 = t * np.sqrt(3) / 2.0
    R_s3_2 = R * np.sqrt(3) / 2.0
    R_half = R / 2.0

    # Vertices going CW from top-right (same labeling as base derivation)
    hex_verts = np.array([
        [half_t,           R - t_s3_2],         # H1: top-right
        [R_s3_2 - half_t, -R_half + t_s3_2],    # H2: right
        [R_s3_2 - t,      -R_half],             # H3: bottom-right
        [-(R_s3_2 - t),   -R_half],             # H4: bottom-left
        [-(R_s3_2 - half_t), -R_half + t_s3_2], # H5: left
        [-half_t,          R - t_s3_2],          # H6: top-left
    ])

    # Inset toward center by 5 mm (half material width)
    vertex_radius = np.sqrt(R**2 - R * t * np.sqrt(3) + t**2)
    inset_scale = (vertex_radius - _PLAT_MATERIAL_W / 2.0) / vertex_radius

    inset_verts = hex_verts * inset_scale

    # Rotate by 60° for Stewart platform cross-pattern
    cos60, sin60 = np.cos(np.radians(60)), np.sin(np.radians(60))
    rot60 = np.array([[cos60, -sin60],
                      [sin60,  cos60]])

    rotated = (rot60 @ inset_verts.T).T  # (6, 2)

    # After rotation, the 6 points are at approximate angles:
    #   P1'≈142°, P2'≈38°, P3'≈22°, P4'≈278°, P5'≈262°, P6'≈158°
    #
    # Short-edge pairs (close together):
    #   Pair at ~30°:  P2'(38°), P3'(22°)
    #   Pair at ~150°: P6'(158°), P1'(142°)
    #   Pair at ~270°: P4'(278°), P5'(262°)
    #
    # Cross-mapping: each base servo → nearest platform point on adjacent pair
    #   S1(81°)  → P2'(38°)    S2(99°)  → P1'(142°)
    #   S3(201°) → P6'(158°)   S4(219°) → P5'(262°)
    #   S5(321°) → P4'(278°)   S6(339°) → P3'(22°)
    #
    # Index mapping: rotated[original_hex_idx] → base_servo_idx
    cross_map = [1, 0, 5, 4, 3, 2]  # P2', P1', P6', P5', P4', P3'

    platform = np.zeros((6, 3))
    for servo_idx, hex_idx in enumerate(cross_map):
        platform[servo_idx, 0] = rotated[hex_idx, 0]
        platform[servo_idx, 1] = rotated[hex_idx, 1]
        # Z = 0 (local platform frame)

    return platform


class StewartKinematics:
    """Complete inverse kinematics solver for a rotary Stewart platform."""

    def __init__(self,
                 horn_length: float = 25.0,
                 rod_length: float = 130.0):
        self.horn_length = horn_length
        self.rod_length = rod_length
        self._compute_geometry()

    def _compute_geometry(self):
        """Compute anchor positions, servo directions, and home height."""
        # ── Fixed hardware geometry ──────────────────────────────────
        self.base_anchors = _build_base_anchors()
        self.platform_anchors_local = _build_platform_anchors()

        # Base anchor angles (for tangential horn direction in IK solver)
        self.base_angles_rad = np.array([
            np.arctan2(self.base_anchors[i, 1], self.base_anchors[i, 0])
            for i in range(6)
        ])
        self.base_angles_deg = np.degrees(self.base_angles_rad)

        # Servo horn sweep direction: alternating for paired servos
        self.servo_dir = np.array([1, -1, 1, -1, 1, -1], dtype=float)

        # ── Home height calculation ──────────────────────────────────
        # Scan α ∈ [0°, 180°] for each leg to find max achievable height,
        # then offset downward to center the Z workspace.
        min_max_height = float('inf')
        for i in range(6):
            beta = self.base_angles_rad[i]
            r_hat = np.array([np.cos(beta), np.sin(beta)])
            t_hat = np.array([-np.sin(beta), np.cos(beta)])
            d = self.servo_dir[i]

            delta_xy = (self.platform_anchors_local[i, :2]
                        - self.base_anchors[i, :2])
            rad_comp = np.dot(delta_xy, r_hat)
            tang_comp = np.dot(delta_xy, t_hat)

            # Scan alpha to find max achievable height for this leg
            best_h = 0.0
            for alpha_deg in range(0, 181):
                alpha = np.radians(alpha_deg)
                eff_tang = tang_comp - self.horn_length * np.cos(alpha) * d
                horiz_sq = rad_comp ** 2 + eff_tang ** 2
                vert_sq = self.rod_length ** 2 - horiz_sq
                if vert_sq < 0:
                    continue
                h = self.horn_length * np.sin(alpha) + np.sqrt(vert_sq)
                best_h = max(best_h, h)

            if best_h < 1e-6:
                raise ValueError("Rod too short for geometry! Increase rod_length.")
            min_max_height = min(min_max_height, best_h)

        # Offset below the peak to center the workspace.
        # 40% of horn_length gives roughly equal Z travel up and down.
        self.home_height = min_max_height - 0.40 * self.horn_length

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

        Parameters:
            x, y, z: translation in mm relative to home position
            roll, pitch, yaw: rotation in degrees (ZYX Euler convention)

        Returns:
            servo_angles (6,): degrees [0-180]
            base_positions (6,3): base anchor XYZ
            horn_tips (6,3): horn tip XYZ
            platform_positions (6,3): platform anchor XYZ in world frame
            platform_center (3,): center of moved platform
            valid: True if all angles are within [0, 180]
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

            # Tangential direction for this servo (perpendicular to radial, in XY plane)
            beta = self.base_angles_rad[i]
            t_hat = np.array([-np.sin(beta), np.cos(beta), 0.0])
            d = self.servo_dir[i]

            # Solve the equation: g = e*cos(α) + f*sin(α)
            # where α is the servo horn angle
            e = d * np.dot(t_hat, L)
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

            # Two candidate solutions — pick the one closest to 90° (neutral)
            a1 = np.degrees(base_a + offset)
            a2 = np.degrees(base_a - offset)

            best = 90.0
            best_dist = 999.0
            for cand in [a1, a2, a1 + 360, a2 + 360, a1 - 360, a2 - 360]:
                if -5 <= cand <= 185:
                    dist = abs(cand - 90.0)
                    if dist < best_dist:
                        best_dist = dist
                        best = cand

            servo_angles[i] = np.clip(best, 0.0, 180.0)

            # Compute horn tip position for visualization
            alpha_r = np.radians(servo_angles[i])
            horn_tips[i] = (self.base_anchors[i]
                            + self.horn_length * np.cos(alpha_r) * d * t_hat
                            + self.horn_length * np.sin(alpha_r) * np.array([0, 0, 1]))

        platform_center = T.copy()
        return servo_angles, self.base_anchors.copy(), horn_tips, platform_world, platform_center, valid

    def update_params(self, **kwargs):
        """Update geometry parameters and recompute.

        Only horn_length and rod_length are adjustable.
        Base and platform anchors are fixed to hardware dimensions.
        """
        for k, v in kwargs.items():
            if k in ('horn_length', 'rod_length'):
                setattr(self, k, v)
        self._compute_geometry()

    def get_workspace_limits(self) -> dict:
        """Return approximate workspace boundaries for slider range guidance."""
        return {
            "trans_xy": self.horn_length * 0.7,
            "trans_z": self.horn_length * 0.5,
            "rot_max": 25.0,
            "home_height": self.home_height,
        }
