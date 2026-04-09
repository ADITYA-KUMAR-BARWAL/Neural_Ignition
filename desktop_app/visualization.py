"""
3D OpenGL visualization of the Stewart Platform.
Renders base plate, top platform, servo horns, connecting rods, and grid floor.
Uses QOpenGLWidget with legacy OpenGL for simplicity.
"""

import math
import numpy as np
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import Qt, QPoint
from OpenGL.GL import *
from OpenGL.GLU import *


# ── Color constants (RGBA) ──────────────────────────────────────────
COL_GRID       = (0.10, 0.22, 0.36, 0.5)
COL_GRID_AXIS  = (0.15, 0.30, 0.45, 0.7)
COL_BASE       = (1.0, 0.42, 0.20, 1.0)   # orange
COL_BASE_PT    = (1.0, 0.18, 0.42, 1.0)   # pink
COL_HORN       = (1.0, 0.80, 0.0, 1.0)    # yellow
COL_ROD        = (0.2, 1.0, 0.4, 1.0)     # green
COL_PLAT       = (0.0, 1.0, 0.8, 1.0)     # cyan
COL_PLAT_PT    = (0.0, 1.0, 0.8, 1.0)     # cyan
COL_CENTER     = (0.0, 1.0, 0.53, 1.0)    # bright green
COL_BG         = (0.04, 0.086, 0.15)       # dark navy
COL_INVALID    = (1.0, 0.0, 0.0, 1.0)     # red


def _draw_sphere(cx, cy, cz, r, slices=12, stacks=8):
    """Draw a solid sphere at (cx,cy,cz) with radius r."""
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


class StewartPlatformView(QOpenGLWidget):
    """Real-time 3D OpenGL visualization of a Stewart platform."""

    def __init__(self, parent=None):
        super().__init__(parent)
        # Camera
        self.cam_az = 45.0
        self.cam_el = 25.0
        self.cam_dist = 400.0
        self.cam_target = np.array([0.0, 0.0, 70.0])
        self._last_mouse = QPoint()

        # Platform state
        self.base_anchors = np.zeros((6, 3))
        self.horn_tips = np.zeros((6, 3))
        self.plat_anchors = np.zeros((6, 3))
        self.plat_center = np.array([0.0, 0.0, 140.0])
        self.valid = True

        self.setMinimumSize(400, 400)
        self.setFocusPolicy(Qt.StrongFocus)

    # ── Public API ──────────────────────────────────────────────────
    def update_platform(self, base, horns, plat, center, valid):
        self.base_anchors = base
        self.horn_tips = horns
        self.plat_anchors = plat
        self.plat_center = center
        self.valid = valid
        self.update()

    # ── OpenGL callbacks ────────────────────────────────────────────
    def initializeGL(self):
        glClearColor(*COL_BG, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glEnable(GL_POINT_SMOOTH)

        # Simple lighting
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [200, 200, 400, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1, 1, 1, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.35, 1])
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

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
        self._draw_platform()

    # ── Camera ──────────────────────────────────────────────────────
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

    # ── Mouse interaction ───────────────────────────────────────────
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

    # ── Drawing helpers ─────────────────────────────────────────────
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
        glBegin(GL_LINES)
        # X = red
        glColor3f(1, 0.2, 0.2)
        glVertex3f(0, 0, 0.5)
        glVertex3f(40, 0, 0.5)
        # Y = green
        glColor3f(0.2, 1, 0.2)
        glVertex3f(0, 0, 0.5)
        glVertex3f(0, 40, 0.5)
        # Z = blue
        glColor3f(0.3, 0.5, 1)
        glVertex3f(0, 0, 0.5)
        glVertex3f(0, 0, 40)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_platform(self):
        ba = self.base_anchors
        ht = self.horn_tips
        pa = self.plat_anchors
        pc = self.plat_center

        # ── Base outline (orange) ────
        glDisable(GL_LIGHTING)
        # Glow layer
        glLineWidth(5.0)
        glColor4f(COL_BASE[0], COL_BASE[1], COL_BASE[2], 0.25)
        order_b = sorted(range(6), key=lambda i: math.atan2(ba[i][1], ba[i][0]))
        glBegin(GL_LINE_LOOP)
        for i in order_b:
            glVertex3f(*ba[i])
        glEnd()
        # Sharp layer
        glLineWidth(2.0)
        glColor4f(*COL_BASE)
        glBegin(GL_LINE_LOOP)
        for i in order_b:
            glVertex3f(*ba[i])
        glEnd()

        # ── Base anchor spheres (pink) ────
        glEnable(GL_LIGHTING)
        glColor4f(*COL_BASE_PT)
        for i in range(6):
            _draw_sphere(ba[i][0], ba[i][1], ba[i][2], 3.0)

        # ── Servo horns (yellow) ────
        glDisable(GL_LIGHTING)
        glLineWidth(3.5)
        glColor4f(*COL_HORN)
        glBegin(GL_LINES)
        for i in range(6):
            glVertex3f(*ba[i])
            glVertex3f(*ht[i])
        glEnd()

        # Horn tip spheres
        glEnable(GL_LIGHTING)
        glColor4f(*COL_HORN)
        for i in range(6):
            _draw_sphere(ht[i][0], ht[i][1], ht[i][2], 2.5)

        # ── Connecting rods (green) with glow ────
        glDisable(GL_LIGHTING)
        rod_col = COL_ROD if self.valid else COL_INVALID
        # Glow
        glLineWidth(5.0)
        glColor4f(rod_col[0], rod_col[1], rod_col[2], 0.20)
        glBegin(GL_LINES)
        for i in range(6):
            glVertex3f(*ht[i])
            glVertex3f(*pa[i])
        glEnd()
        # Sharp
        glLineWidth(2.0)
        glColor4f(*rod_col)
        glBegin(GL_LINES)
        for i in range(6):
            glVertex3f(*ht[i])
            glVertex3f(*pa[i])
        glEnd()

        # ── Platform outline (cyan) ────
        order_p = sorted(range(6), key=lambda i: math.atan2(pa[i][1], pa[i][0]))
        # Glow
        glLineWidth(5.0)
        glColor4f(COL_PLAT[0], COL_PLAT[1], COL_PLAT[2], 0.25)
        glBegin(GL_LINE_LOOP)
        for i in order_p:
            glVertex3f(*pa[i])
        glEnd()
        # Sharp
        glLineWidth(2.5)
        glColor4f(*COL_PLAT)
        glBegin(GL_LINE_LOOP)
        for i in order_p:
            glVertex3f(*pa[i])
        glEnd()

        # ── Platform anchor spheres (cyan) ────
        glEnable(GL_LIGHTING)
        glColor4f(*COL_PLAT_PT)
        for i in range(6):
            _draw_sphere(pa[i][0], pa[i][1], pa[i][2], 3.0)

        # ── Platform center sphere (bright green) ────
        glColor4f(*COL_CENTER)
        _draw_sphere(pc[0], pc[1], pc[2], 5.0)

    def reset_camera(self):
        self.cam_az = 45.0
        self.cam_el = 25.0
        self.cam_dist = 400.0
        self.cam_target = np.array([0.0, 0.0, 70.0])
        self.update()
