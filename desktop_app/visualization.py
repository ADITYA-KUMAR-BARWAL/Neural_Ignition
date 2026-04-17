"""
3D OpenGL visualization of the Stewart Platform.
Renders:
  - Solid hexagonal base plate with neon-glow edges
  - 6 × MG996R servo blocks oriented radially on the base
  - Servo horn arms driven by computed α_i angles
  - Chrome connecting rods with ball-joint spheres
  - Translucent top platform with glow edges
  - Dark Industry / Neon aesthetic

Uses QOpenGLWidget with legacy OpenGL for maximum compatibility.
Coordinate system: Z-up, base fixed at origin.
"""

import math
import numpy as np
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import Qt, QPoint
from OpenGL.GL import *
from OpenGL.GLU import *


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Color Palette — "Dark Industry Neon"
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
COL_BG          = (0.03, 0.05, 0.08)            # near-black blue
COL_GRID        = (0.06, 0.10, 0.18, 0.40)      # subtle blue grid
COL_GRID_AXIS   = (0.10, 0.18, 0.30, 0.60)      # brighter axis lines

COL_BASE_EDGE   = (1.0, 0.42, 0.10, 1.0)        # orange glow edge
COL_BASE_GLOW   = (1.0, 0.42, 0.10, 0.15)       # orange glow fill
COL_BASE_FILL   = (0.08, 0.08, 0.14, 0.50)      # dark translucent fill

COL_SERVO_BODY  = (0.10, 0.10, 0.10, 1.0)       # dark matte black
COL_SERVO_EDGE  = (0.25, 0.25, 0.25, 1.0)       # subtle edge highlight
COL_SERVO_LABEL = (0.85, 0.55, 0.05, 1.0)       # gold label accent

COL_HORN        = (0.75, 0.78, 0.82, 1.0)       # metallic silver
COL_HORN_PIVOT  = (0.90, 0.50, 0.05, 1.0)       # orange pivot

COL_ROD         = (0.82, 0.84, 0.86, 1.0)       # chrome
COL_ROD_INVALID = (1.0, 0.10, 0.25, 1.0)        # red = out of range

COL_JOINT_BASE  = (1.0, 0.42, 0.10, 1.0)        # orange ball joint
COL_JOINT_PLAT  = (0.0, 0.90, 1.0, 1.0)         # cyan ball joint

COL_PLAT_EDGE   = (0.0, 0.90, 1.0, 1.0)         # bright cyan edge
COL_PLAT_GLOW   = (0.0, 0.90, 1.0, 0.12)        # cyan translucent fill
COL_PLAT_FILL   = (0.0, 0.90, 1.0, 0.06)        # very subtle fill

COL_CENTER      = (0.0, 1.0, 0.55, 1.0)         # bright green center


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Drawing primitives
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _draw_sphere(cx, cy, cz, r, slices=16, stacks=12):
    """Draw a solid sphere at (cx, cy, cz) with radius r."""
    for i in range(stacks):
        lat0 = math.pi * (-0.5 + i / stacks)
        lat1 = math.pi * (-0.5 + (i + 1) / stacks)
        z0, zr0 = math.sin(lat0) * r, math.cos(lat0) * r
        z1, zr1 = math.sin(lat1) * r, math.cos(lat1) * r
        glBegin(GL_QUAD_STRIP)
        for j in range(slices + 1):
            lng = 2 * math.pi * j / slices
            x, y = math.cos(lng), math.sin(lng)
            glNormal3f(x * zr0, y * zr0, z0)
            glVertex3f(cx + x * zr0, cy + y * zr0, cz + z0)
            glNormal3f(x * zr1, y * zr1, z1)
            glVertex3f(cx + x * zr1, cy + y * zr1, cz + z1)
        glEnd()


def _draw_cylinder(x0, y0, z0, x1, y1, z1, radius, slices=12):
    """Draw a solid cylinder between two 3D points with end caps."""
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    if length < 1e-6:
        return

    # Build orthonormal basis aligned with the cylinder axis
    az = np.array([dx, dy, dz]) / length
    if abs(az[2]) < 0.99:
        ax = np.cross(az, np.array([0, 0, 1]))
    else:
        ax = np.cross(az, np.array([1, 0, 0]))
    ax /= np.linalg.norm(ax)
    ay = np.cross(az, ax)

    # Side faces
    glBegin(GL_QUAD_STRIP)
    for i in range(slices + 1):
        angle = 2 * math.pi * i / slices
        ca, sa = math.cos(angle), math.sin(angle)
        nx = ax[0] * ca + ay[0] * sa
        ny = ax[1] * ca + ay[1] * sa
        nz = ax[2] * ca + ay[2] * sa
        glNormal3f(nx, ny, nz)
        glVertex3f(x0 + nx * radius, y0 + ny * radius, z0 + nz * radius)
        glVertex3f(x1 + nx * radius, y1 + ny * radius, z1 + nz * radius)
    glEnd()

    # End cap at p0
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(-az[0], -az[1], -az[2])
    glVertex3f(x0, y0, z0)
    for i in range(slices + 1):
        angle = 2 * math.pi * i / slices
        ca, sa = math.cos(angle), math.sin(angle)
        nx = ax[0] * ca + ay[0] * sa
        ny = ax[1] * ca + ay[1] * sa
        nz = ax[2] * ca + ay[2] * sa
        glVertex3f(x0 + nx * radius, y0 + ny * radius, z0 + nz * radius)
    glEnd()

    # End cap at p1
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(az[0], az[1], az[2])
    glVertex3f(x1, y1, z1)
    for i in range(slices, -1, -1):
        angle = 2 * math.pi * i / slices
        ca, sa = math.cos(angle), math.sin(angle)
        nx = ax[0] * ca + ay[0] * sa
        ny = ax[1] * ca + ay[1] * sa
        nz = ax[2] * ca + ay[2] * sa
        glVertex3f(x1 + nx * radius, y1 + ny * radius, z1 + nz * radius)
    glEnd()


def _draw_box(cx, cy, cz, sx, sy, sz):
    """Draw axis-aligned box centered at (cx, cy, cz) with half-sizes (sx, sy, sz)."""
    # 6 faces of a cuboid
    vertices = [
        # Front (+Y)
        ((-1, 1, -1), (-1, 1, 1), (1, 1, 1), (1, 1, -1), (0, 1, 0)),
        # Back (-Y)
        ((1, -1, -1), (1, -1, 1), (-1, -1, 1), (-1, -1, -1), (0, -1, 0)),
        # Right (+X)
        ((1, 1, -1), (1, 1, 1), (1, -1, 1), (1, -1, -1), (1, 0, 0)),
        # Left (-X)
        ((-1, -1, -1), (-1, -1, 1), (-1, 1, 1), (-1, 1, -1), (-1, 0, 0)),
        # Top (+Z)
        ((-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1), (0, 0, 1)),
        # Bottom (-Z)
        ((-1, 1, -1), (1, 1, -1), (1, -1, -1), (-1, -1, -1), (0, 0, -1)),
    ]
    glBegin(GL_QUADS)
    for v0, v1, v2, v3, n in vertices:
        glNormal3f(*n)
        glVertex3f(cx + v0[0]*sx, cy + v0[1]*sy, cz + v0[2]*sz)
        glVertex3f(cx + v1[0]*sx, cy + v1[1]*sy, cz + v1[2]*sz)
        glVertex3f(cx + v2[0]*sx, cy + v2[1]*sy, cz + v2[2]*sz)
        glVertex3f(cx + v3[0]*sx, cy + v3[1]*sy, cz + v3[2]*sz)
    glEnd()


def _draw_box_edges(cx, cy, cz, sx, sy, sz):
    """Draw wireframe edges of a box for edge highlight effect."""
    corners = []
    for dx in (-1, 1):
        for dy in (-1, 1):
            for dz in (-1, 1):
                corners.append((cx + dx*sx, cy + dy*sy, cz + dz*sz))

    edges = [
        (0,1),(2,3),(4,5),(6,7),  # Z edges
        (0,2),(1,3),(4,6),(5,7),  # Y edges
        (0,4),(1,5),(2,6),(3,7),  # X edges
    ]
    glBegin(GL_LINES)
    for a, b in edges:
        glVertex3f(*corners[a])
        glVertex3f(*corners[b])
    glEnd()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Main Widget
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class StewartPlatformView(QOpenGLWidget):
    """Real-time 3D OpenGL visualization of a Stewart platform.

    Coordinate system:
      - Z axis points UP
      - Base plate sits at Z = 0
      - Platform floats above at Z = home_height + tz
    """

    # MG996R servo block half-dimensions (scaled to visualization units)
    SERVO_HX = 10.0   # half-width along tangent
    SERVO_HY = 5.0    # half-depth along radial
    SERVO_HZ = 7.0    # half-height (Z)

    def __init__(self, parent=None):
        super().__init__(parent)

        # Camera
        self.cam_az = 45.0
        self.cam_el = 25.0
        self.cam_dist = 400.0
        self.cam_target = np.array([0.0, 0.0, 70.0])
        self._last_mouse = QPoint()

        # Platform geometry state
        self.base_anchors = np.zeros((6, 3))
        self.horn_tips = np.zeros((6, 3))
        self.plat_anchors = np.zeros((6, 3))
        self.plat_center = np.array([0.0, 0.0, 140.0])
        self.valid = True

        # Servo angle + geometry data for rendering
        self.servo_angles = np.full(6, 50.0)
        self.base_angles_rad = np.zeros(6)
        self.servo_dir = np.ones(6)
        self.horn_length = 25.0

        self.setMinimumSize(400, 400)
        self.setFocusPolicy(Qt.StrongFocus)

    # ── Public API ─────────────────────────────────────────────────
    def update_platform(self, base, horns, plat, center, valid,
                        servo_angles=None, base_angles=None,
                        servo_dir=None, horn_length=None):
        """Update all platform state for rendering.

        Args:
            base:         (6,3) base anchor positions
            horns:        (6,3) horn tip positions (computed from IK)
            plat:         (6,3) platform anchor positions in world frame
            center:       (3,)  platform center position
            valid:        bool  whether all servo angles are in range
            servo_angles: (6,)  computed servo angles in degrees
            base_angles:  (6,)  base anchor angles in radians
            servo_dir:    (6,)  servo sweep directions (+1/-1)
            horn_length:  float horn arm length
        """
        self.base_anchors = base
        self.horn_tips = horns
        self.plat_anchors = plat
        self.plat_center = center
        self.valid = valid

        if servo_angles is not None:
            self.servo_angles = np.asarray(servo_angles)
        if base_angles is not None:
            self.base_angles_rad = np.asarray(base_angles)
        if servo_dir is not None:
            self.servo_dir = np.asarray(servo_dir)
        if horn_length is not None:
            self.horn_length = horn_length

        self.update()

    # ── OpenGL callbacks ───────────────────────────────────────────
    def initializeGL(self):
        glClearColor(*COL_BG, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glEnable(GL_POINT_SMOOTH)
        glEnable(GL_MULTISAMPLE)

        # ── Two-point lighting ──
        glEnable(GL_LIGHTING)

        # Key light (warm, from upper-right-front)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [250, 150, 400, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1.0, 0.95, 0.90, 1])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1.0, 1.0, 1.0, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.08, 0.08, 0.12, 1])

        # Fill light (cool, from lower-left-back)
        glEnable(GL_LIGHT1)
        glLightfv(GL_LIGHT1, GL_POSITION, [-200, -100, 100, 1])
        glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.25, 0.30, 0.40, 1])
        glLightfv(GL_LIGHT1, GL_SPECULAR, [0.2, 0.2, 0.3, 1])
        glLightfv(GL_LIGHT1, GL_AMBIENT,  [0.02, 0.02, 0.04, 1])

        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        # Global ambient
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.06, 0.06, 0.10, 1.0])

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / max(h, 1)
        gluPerspective(45, aspect, 1.0, 2000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        self._apply_camera()

        self._draw_grid()
        self._draw_axes()
        self._draw_base_plate()
        self._draw_servos()
        self._draw_horn_arms()
        self._draw_connecting_rods()
        self._draw_top_platform()

    # ── Camera ─────────────────────────────────────────────────────
    def _apply_camera(self):
        az = math.radians(self.cam_az)
        el = math.radians(self.cam_el)
        cx = self.cam_dist * math.cos(el) * math.cos(az)
        cy = self.cam_dist * math.cos(el) * math.sin(az)
        cz = self.cam_dist * math.sin(el)
        t = self.cam_target
        gluLookAt(cx + t[0], cy + t[1], cz + t[2],
                  t[0], t[1], t[2],
                  0, 0, 1)

    # ── Mouse interaction ──────────────────────────────────────────
    def mousePressEvent(self, event):
        self._last_mouse = event.position().toPoint()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            pos = event.position().toPoint()
            dx = pos.x() - self._last_mouse.x()
            dy = pos.y() - self._last_mouse.y()
            self.cam_az -= dx * 0.5
            self.cam_el += dy * 0.5
            self.cam_el = max(-89, min(89, self.cam_el))
            self._last_mouse = pos
            self.update()
        elif event.buttons() & Qt.RightButton:
            pos = event.position().toPoint()
            dy = pos.y() - self._last_mouse.y()
            self.cam_target[2] += dy * 0.5
            self._last_mouse = pos
            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self.cam_dist -= delta * 0.3
        self.cam_dist = max(100, min(1500, self.cam_dist))
        self.update()

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Drawing: Grid & Axes
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        size = 200
        step = 20
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for i in range(-size, size + 1, step):
            if i == 0:
                glColor4f(*COL_GRID_AXIS)
            else:
                glColor4f(*COL_GRID)
            glVertex3f(i, -size, 0)
            glVertex3f(i, size, 0)
            glVertex3f(-size, i, 0)
            glVertex3f(size, i, 0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_axes(self):
        glDisable(GL_LIGHTING)
        glLineWidth(2.5)
        length = 50
        glBegin(GL_LINES)
        # X = red
        glColor3f(0.9, 0.15, 0.15)
        glVertex3f(0, 0, 0.5)
        glVertex3f(length, 0, 0.5)
        # Y = green
        glColor3f(0.15, 0.85, 0.15)
        glVertex3f(0, 0, 0.5)
        glVertex3f(0, length, 0.5)
        # Z = blue
        glColor3f(0.2, 0.4, 1.0)
        glVertex3f(0, 0, 0.5)
        glVertex3f(0, 0, length)
        glEnd()

        # Axis labels (small spheres at tips)
        glEnable(GL_LIGHTING)
        glColor4f(0.9, 0.15, 0.15, 1.0)
        _draw_sphere(length + 3, 0, 0.5, 2.0, 8, 6)
        glColor4f(0.15, 0.85, 0.15, 1.0)
        _draw_sphere(0, length + 3, 0.5, 2.0, 8, 6)
        glColor4f(0.2, 0.4, 1.0, 1.0)
        _draw_sphere(0, 0, length + 3, 2.0, 8, 6)

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Drawing: Base Plate
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _draw_base_plate(self):
        ba = self.base_anchors
        order = sorted(range(6), key=lambda i: math.atan2(ba[i][1], ba[i][0]))

        glDisable(GL_LIGHTING)

        # Solid fill (very translucent dark)
        glColor4f(*COL_BASE_FILL)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, 0)  # Center of base
        for i in order:
            glVertex3f(ba[i][0], ba[i][1], ba[i][2])
        glVertex3f(ba[order[0]][0], ba[order[0]][1], ba[order[0]][2])
        glEnd()

        # Outer glow edge (wide, dim)
        glLineWidth(6.0)
        glColor4f(COL_BASE_EDGE[0], COL_BASE_EDGE[1], COL_BASE_EDGE[2], 0.20)
        glBegin(GL_LINE_LOOP)
        for i in order:
            glVertex3f(ba[i][0], ba[i][1], ba[i][2])
        glEnd()

        # Sharp edge (narrow, bright)
        glLineWidth(2.5)
        glColor4f(*COL_BASE_EDGE)
        glBegin(GL_LINE_LOOP)
        for i in order:
            glVertex3f(ba[i][0], ba[i][1], ba[i][2])
        glEnd()

        glEnable(GL_LIGHTING)

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Drawing: Servo Blocks (MG996R)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _draw_servos(self):
        """Draw 6 servo blocks positioned at base anchors, oriented radially."""
        ba = self.base_anchors

        for i in range(6):
            bx, by, bz = ba[i]
            angle_deg = math.degrees(self.base_angles_rad[i])

            glPushMatrix()
            # Position servo so its TOP surface is at the base anchor Z.
            # The servo body hangs below the base plane (mechanically accurate).
            glTranslatef(bx, by, bz - self.SERVO_HZ)
            # Rotate so the servo body is tangent to the base circle
            glRotatef(angle_deg + 90, 0, 0, 1)

            # ── Servo body (dark matte box) ──
            glColor4f(*COL_SERVO_BODY)
            _draw_box(0, 0, 0, self.SERVO_HX, self.SERVO_HY, self.SERVO_HZ)

            # ── Edge highlight ──
            glDisable(GL_LIGHTING)
            glLineWidth(1.5)
            glColor4f(*COL_SERVO_EDGE)
            _draw_box_edges(0, 0, 0, self.SERVO_HX, self.SERVO_HY, self.SERVO_HZ)

            # ── Label accent stripe (gold line on front face) ──
            glLineWidth(2.0)
            glColor4f(*COL_SERVO_LABEL)
            stripe_y = self.SERVO_HY + 0.1
            glBegin(GL_LINES)
            glVertex3f(-self.SERVO_HX * 0.6, stripe_y, -self.SERVO_HZ * 0.3)
            glVertex3f( self.SERVO_HX * 0.6, stripe_y, -self.SERVO_HZ * 0.3)
            glEnd()

            glEnable(GL_LIGHTING)
            glPopMatrix()

        # ── Pivot spheres at base anchor points (top of servo) ──
        glColor4f(*COL_HORN_PIVOT)
        for i in range(6):
            bx, by, bz = ba[i]
            _draw_sphere(bx, by, bz, 2.5)

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Drawing: Horn Arms (driven by α_i)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _draw_horn_arms(self):
        """Draw metallic horn arms from servo pivots to horn tip points."""
        ba = self.base_anchors
        ht = self.horn_tips

        glEnable(GL_LIGHTING)
        glColor4f(*COL_HORN)

        for i in range(6):
            # Pivot point = base anchor (IK origin for this horn)
            px, py, pz = ba[i]

            # Horn tip from IK computation
            hx, hy, hz = ht[i]

            # Draw horn arm as cylinder
            _draw_cylinder(px, py, pz, hx, hy, hz, 2.0)

            # Horn tip sphere
            glColor4f(*COL_HORN)
            _draw_sphere(hx, hy, hz, 2.5)

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Drawing: Connecting Rods (horn tip → platform anchor)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _draw_connecting_rods(self):
        """Draw chrome connecting rods with ball joints at both ends."""
        ht = self.horn_tips
        pa = self.plat_anchors

        rod_col = COL_ROD if self.valid else COL_ROD_INVALID

        # ── Neon glow lines (drawn before solid rods for depth) ──
        glDisable(GL_LIGHTING)
        # Wide glow
        glLineWidth(5.0)
        glColor4f(rod_col[0], rod_col[1], rod_col[2], 0.12)
        glBegin(GL_LINES)
        for i in range(6):
            glVertex3f(*ht[i])
            glVertex3f(*pa[i])
        glEnd()
        # Narrow core
        glLineWidth(1.5)
        glColor4f(rod_col[0], rod_col[1], rod_col[2], 0.40)
        glBegin(GL_LINES)
        for i in range(6):
            glVertex3f(*ht[i])
            glVertex3f(*pa[i])
        glEnd()

        # ── Solid 3D cylinder rods ──
        glEnable(GL_LIGHTING)
        glColor4f(*rod_col)
        for i in range(6):
            _draw_cylinder(ht[i][0], ht[i][1], ht[i][2],
                           pa[i][0], pa[i][1], pa[i][2], 1.2)

        # ── Ball joints at horn tips (orange) ──
        glColor4f(*COL_JOINT_BASE)
        for i in range(6):
            _draw_sphere(ht[i][0], ht[i][1], ht[i][2], 3.0)

        # ── Ball joints at platform anchors (cyan) ──
        glColor4f(*COL_JOINT_PLAT)
        for i in range(6):
            _draw_sphere(pa[i][0], pa[i][1], pa[i][2], 3.0)

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Drawing: Top Platform
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _draw_top_platform(self):
        """Draw the moving top platform with glow edges and translucent fill."""
        pa = self.plat_anchors
        pc = self.plat_center
        order = sorted(range(6), key=lambda i: math.atan2(
            pa[i][1] - pc[1], pa[i][0] - pc[0]))

        glDisable(GL_LIGHTING)

        # ── Translucent fill ──
        glColor4f(*COL_PLAT_FILL)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(*pc)
        for i in order:
            glVertex3f(*pa[i])
        glVertex3f(*pa[order[0]])
        glEnd()

        # ── Outer glow edge ──
        glLineWidth(6.0)
        glColor4f(COL_PLAT_EDGE[0], COL_PLAT_EDGE[1], COL_PLAT_EDGE[2], 0.18)
        glBegin(GL_LINE_LOOP)
        for i in order:
            glVertex3f(*pa[i])
        glEnd()

        # ── Sharp bright edge ──
        glLineWidth(2.5)
        glColor4f(*COL_PLAT_EDGE)
        glBegin(GL_LINE_LOOP)
        for i in order:
            glVertex3f(*pa[i])
        glEnd()

        # ── Cross-braces (structural lines from center to each anchor) ──
        glLineWidth(1.0)
        glColor4f(COL_PLAT_EDGE[0], COL_PLAT_EDGE[1], COL_PLAT_EDGE[2], 0.25)
        glBegin(GL_LINES)
        for i in range(6):
            glVertex3f(*pc)
            glVertex3f(*pa[i])
        glEnd()

        glEnable(GL_LIGHTING)

        # ── Center sphere ──
        glColor4f(*COL_CENTER)
        _draw_sphere(pc[0], pc[1], pc[2], 4.0)

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Reset
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def reset_camera(self):
        self.cam_az = 45.0
        self.cam_el = 25.0
        self.cam_dist = 400.0
        self.cam_target = np.array([0.0, 0.0, 70.0])
        self.update()
