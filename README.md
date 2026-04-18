# ⬡ Stewart Platform — 6-DOF Mission Control

A complete **6 Degrees-of-Freedom Stewart Platform** system featuring a real-time inverse-kinematics desktop application and an Arduino-based firmware controller for continuous-rotation servos via PCA9685.

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)
![PySide6](https://img.shields.io/badge/PySide6-6.5+-41CD52?logo=qt&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D?logo=arduino&logoColor=white)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Build-FF7F00?logo=platformio&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-blue)

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Hardware](#hardware)
- [Desktop Application](#desktop-application)
- [Firmware](#firmware)
- [Serial Protocol](#serial-protocol)
- [Installation & Setup](#installation--setup)
- [Usage](#usage)
- [Kinematics Reference](#kinematics-reference)
- [Tuning Guide](#tuning-guide)
- [Troubleshooting](#troubleshooting)

---

## Overview

This project implements a **rotary Stewart platform** — a parallel manipulator capable of translation (X, Y, Z) and rotation (Roll, Pitch, Yaw) in six independent degrees of freedom. The system is split into two main components:

| Component | Language | Description |
|-----------|----------|-------------|
| **Desktop App** | Python (PySide6 + OpenGL) | Real-time IK solver, 3D visualization, and control GUI |
| **Firmware** | C++ (Arduino / PlatformIO) | Open-loop dead-reckoning servo controller via PCA9685 |

The desktop application computes inverse kinematics for a desired platform pose and sends servo angle commands over serial. The firmware translates these into timed PWM pulses for continuous-rotation servos.

---

## Features

### Desktop Application
- **Inverse Kinematics Engine** — Full 6-DOF IK solver with ZYX Euler rotation, custom truncated-triangle base, and hexagonal platform geometry
- **Real-Time 3D Visualization** — OpenGL-rendered view with physically accurate servo blocks, horn arms, connecting rods, and ball joints
- **Dark Industry Neon UI** — Cyberpunk-themed interface with neon cyan/pink color scheme
- **6-DOF Sliders** — Translation (±30 mm XY, ±20 mm Z) and rotation (±25°) controls
- **Editable Servo Table** — Real-time angle readout with manual override capability
- **Live Serial Communication** — Auto-connect to Arduino with 50 Hz throttled updates
- **Offline Mode** — Full IK + visualization runs without hardware; serial packets logged to console
- **Zero Hardware Button** — Re-syncs digital twin to physical home position without sending serial commands

### Firmware
- **Open-Loop Dead Reckoning** — "Spin and Kill" strategy for 360° continuous-rotation servos
- **Non-Blocking Movement** — All 6 servos move simultaneously using `millis()` timers
- **Dead Zone Filtering** — Ignores angle deltas < 0.5° to prevent micro-jitter
- **Serial Acknowledgement** — Returns delta info for debugging
- **Timeout Protection** — Discards incomplete serial messages after 100 ms

---

## Project Structure

```
innovathon_pca/
├── desktop_app/
│   ├── main.py              # Entry point — launches the PySide6 application
│   ├── ui_layout.py         # MainWindow layout: wires control panel + 3D view + IK + serial
│   ├── control_panel.py     # Left panel: dimension inputs, 6-DOF sliders, servo table
│   ├── kinematics.py        # Inverse kinematics engine (truncated-triangle geometry)
│   ├── visualization.py     # OpenGL 3D renderer (servo blocks, rods, platform)
│   ├── communication.py     # Thread-safe serial wrapper with offline fallback
│   └── styles.py            # Neon Cyberpunk QSS stylesheet
│
├── firmware/
│   ├── platformio.ini       # PlatformIO config (Arduino Uno, PCA9685 library)
│   └── src/
│       └── main.cpp         # Open-loop dead reckoning servo controller
│
└── requirements.txt         # Python dependencies
```

### Module Responsibilities

| Module | Role |
|--------|------|
| `main.py` | Application bootstrap — creates `QApplication`, applies neon stylesheet, launches window |
| `ui_layout.py` | Assembles the `MainWindow` with splitter layout; wires IK, serial, and UI signals |
| `control_panel.py` | Builds the left panel with dimension inputs (horn/rod length), 6 DOF sliders, reset/zero buttons, servo angle table, and connection status indicator |
| `kinematics.py` | Defines `StewartKinematics` class with hardware geometry constants, base/platform anchor computation, ZYX rotation matrix, and the core IK solver |
| `visualization.py` | `StewartPlatformView` (QOpenGLWidget) — renders grid, axes, base plate, servo blocks, horn arms, connecting rods, ball joints, and top platform with orbit/zoom camera |
| `communication.py` | `SerialComm` class — serial connection management, packet formatting, 50 Hz throttle, duplicate suppression, and offline console logging |
| `styles.py` | Complete QSS stylesheet defining the "Dark Industry Neon" theme for all Qt widgets |

---

## Hardware

### Bill of Materials

| Component | Specification |
|-----------|---------------|
| Microcontroller | Arduino Uno |
| PWM Driver | PCA9685 16-channel (I²C, 0x40) |
| Servos (×6) | MG996R or equivalent continuous-rotation |
| Base Plate | Truncated equilateral triangle (119.87 mm height, 49.98 mm flat edges) |
| Top Platform | Hexagonal truncated triangle (119 mm long edge, 21 mm short edge) |
| Servo Horn Arms | 25 mm (default, adjustable in GUI) |
| Connecting Rods | 130 mm (default, adjustable in GUI) |
| Ball Joints | 6× base-side + 6× platform-side |

### Geometry Layout

**Base (truncated triangle):**
- 3 flat edges at **90°, 210°, 330°**
- 2 servos per flat, offset **±10 mm** from midpoint
- Servo pairs: S1/S2, S3/S4, S5/S6
- Alternating sweep direction: `[+1, -1, +1, -1, +1, -1]`

**Platform (hexagonal, 60° rotated):**
- Rotated 60° from base for crossed-rod pattern
- Rod endpoints inset 5 mm from hexagon vertices (ball-joint bore centers)
- Cross-mapped to base servos for proper Stewart geometry

### PCA9685 Channel Mapping

| Servo | PCA9685 Channel |
|-------|-----------------|
| S1 | 0 |
| S2 | 4 |
| S3 | 6 |
| S4 | 9 |
| S5 | 11 |
| S6 | 15 |

---

## Desktop Application

### Architecture

```
┌──────────────────────────────────────────────────────────┐
│                      MainWindow                          │
│  ┌─────────────┐  ┌──────────────────────────────────┐   │
│  │ ControlPanel │  │    StewartPlatformView (OpenGL)  │   │
│  │             │  │                                   │   │
│  │ Dimensions  │  │    ┌──────────────────────────┐   │   │
│  │ IK Sliders  │──▶   │  Real-time 3D Rendering  │   │   │
│  │ Servo Table │  │    │  • Base plate + servos   │   │   │
│  │ Status      │  │    │  • Horn arms + rods      │   │   │
│  └──────┬──────┘  │    │  • Top platform          │   │   │
│         │         │    └──────────────────────────┘   │   │
│         ▼         └──────────────────────────────────┘   │
│  ┌──────────────┐                                        │
│  │  Kinematics  │◀── StewartKinematics (IK solver)       │
│  └──────┬───────┘                                        │
│         ▼                                                │
│  ┌──────────────┐                                        │
│  │  SerialComm  │── Serial @ 115200 baud ──▶ Arduino     │
│  └──────────────┘                                        │
└──────────────────────────────────────────────────────────┘
```

### Camera Controls

| Action | Control |
|--------|---------|
| Orbit | Left-click drag |
| Zoom | Scroll wheel |
| Pan (vertical) | Right-click drag |

### 3D Visualization Elements

| Element | Color | Description |
|---------|-------|-------------|
| Base plate | Orange edge / dark fill | Truncated triangle outline |
| Servo blocks | Matte black + gold stripe | Positioned radially at base anchors |
| Pivot spheres | Orange | Servo output shaft |
| Horn arms | Metallic silver | Servo pivot → horn tip cylinder |
| Connecting rods | Chrome (or red if invalid) | Horn tip → platform anchor |
| Base ball joints | Orange | At horn tip positions |
| Platform ball joints | Cyan | At platform anchor positions |
| Top platform | Cyan edge / translucent fill | Hexagonal moving platform |
| Platform center | Green sphere | Center of mass marker |

---

## Firmware

### Control Strategy — "Spin and Kill"

The firmware uses an **open-loop dead reckoning** approach for 360° continuous-rotation servos:

```
1. Receive absolute target angles from Python IK engine
2. Calculate: delta = targetAngle - currentAngle
3. Choose direction: delta > 0 → PULSE_FORWARD, delta < 0 → PULSE_BACKWARD
4. Spin for: |delta| × TIME_PER_DEGREE_MS milliseconds
5. Kill PWM (set to 0) to stop servo
6. Update: currentAngle = targetAngle (dead reckoning)
```

All 6 servos run independently using non-blocking `millis()` timers, enabling simultaneous movement.

### Key Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `TIME_PER_DEGREE_MS` | 5 | Spin time per degree of rotation |
| `PULSE_FORWARD` | 400 | PCA9685 tick for CW spin |
| `PULSE_BACKWARD` | 200 | PCA9685 tick for CCW spin |
| `DEAD_ZONE_DEG` | 0.5° | Minimum delta to trigger movement |
| `SAFE_HOME_ANGLE` | 127.0° | Assumed startup position |
| `UPDATE_INTERVAL_MS` | 10 | Servo check frequency (100 Hz) |

---

## Serial Protocol

### Packet Format

```
<A1:angle1,A2:angle2,A3:angle3,A4:angle4,A5:angle5,A6:angle6>\n
```

**Example:**
```
<A1:135.0,A2:120.5,A3:140.2,A4:125.0,A5:138.7,A6:122.3>
```

### Firmware Responses

| Response | Meaning |
|----------|---------|
| `OK:135.0(d8.0),120.5(d-6.5),...` | Acknowledged with angle + delta per servo |
| `ERR:expected 6 angles, got N` | Incomplete packet |
| `ERR:message timeout` | Packet not terminated within 100 ms |
| `S1: 127.0° → 135.0° (Δ8.0° = 40ms)` | Per-servo movement log |
| `S1: STOPPED at 135.0°` | Servo reached target |

### Communication Parameters

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Default port | COM4 |
| Max update rate | 50 Hz (20 ms throttle) |
| Angle range | 90.0° – 180.0° |

---

## Installation & Setup

### Prerequisites

- **Python** 3.10+
- **PlatformIO** CLI or IDE extension (for firmware flashing)
- **Arduino Uno** with PCA9685 connected via I²C

### Desktop Application

```bash
# Clone the repository
git clone https://github.com/your-org/innovathon_pca.git
cd innovathon_pca

# Install Python dependencies
pip install -r requirements.txt

# Launch the application
cd desktop_app
python main.py
```

### Firmware

```bash
# Using PlatformIO CLI
cd firmware
pio run --target upload

# Or use PlatformIO IDE (VS Code extension)
# 1. Open the firmware/ folder
# 2. Click Upload (→) in the PlatformIO toolbar
```

### Dependencies

**Python (`requirements.txt`):**

| Package | Version | Purpose |
|---------|---------|---------|
| PySide6 | ≥ 6.5.0 | Qt6 GUI framework |
| pyserial | ≥ 3.5 | Serial communication |
| PyOpenGL | ≥ 3.1.7 | OpenGL bindings for 3D rendering |
| numpy | ≥ 1.24.0 | Numerical computation for kinematics |

**Firmware (`platformio.ini`):**

| Library | Version | Purpose |
|---------|---------|---------|
| Adafruit PWM Servo Driver | ≥ 2.4.1 | PCA9685 I²C PWM driver |

---

## Usage

### Quick Start

1. **Flash firmware** to Arduino Uno via PlatformIO
2. **Connect hardware**: Arduino ↔ PCA9685 (I²C), PCA9685 ↔ 6 servos
3. **Launch the desktop app**: `python desktop_app/main.py`
4. The app auto-detects COM4; if not found, runs in **offline mode**
5. Adjust **Translation** sliders (X/Y/Z) or **Rotation** sliders (Roll/Pitch/Yaw)
6. Observe real-time 3D visualization and servo angle updates
7. Servo commands are sent to Arduino at up to 50 Hz

### Manual Servo Override

Double-click any angle in the **Servo Angles** table to type a custom value (90°–180°). This bypasses the IK solver and sends the angle directly to the Arduino.

### Zero Hardware

If the physical servos drift out of sync with the digital twin:
1. Manually level the physical platform
2. Click **⌖ ZERO HARDWARE**
3. This resets sliders and re-syncs the digital model — **no serial commands are sent**

### Adjusting Geometry

Edit the **horn** and **rod** dimension fields to match your physical hardware. The IK solver and home height automatically recompute.

---

## Kinematics Reference

### Inverse Kinematics Algorithm

For each of the 6 legs, the IK solver:

1. Computes the **platform anchor** in world frame: `P_i = T + R · p_i(local)`
2. Calculates the **leg vector**: `L_i = P_i - B_i`
3. Projects onto the **tangential horn plane** using the servo's tangent direction
4. Solves the trigonometric equation: `g = e·cos(α) + f·sin(α)`
   - `e = d · (t̂ · L)` — tangential component
   - `f = L_z` — vertical component
   - `g = (h² + |L|² - r²) / 2h` — geometric constraint
5. Selects the solution **closest to 135°** (midpoint of 90°-180° range)

### Coordinate System

- **Z-up**: Base plate at Z = 0, platform floats above
- **ZYX Euler** rotation convention (roll → pitch → yaw)
- All dimensions in **millimeters**, angles in **degrees**

### Workspace Limits

| DOF | Range | Unit |
|-----|-------|------|
| X translation | ±30 | mm |
| Y translation | ±30 | mm |
| Z translation | ±20 | mm |
| Roll | ±25 | deg |
| Pitch | ±25 | deg |
| Yaw | ±25 | deg |

---

## Tuning Guide

### Firmware Tuning

The most critical constant is `TIME_PER_DEGREE_MS`:

```
Measure your servo's rotation speed:
  1. Send a known delta (e.g., 90°)
  2. Time how long the physical servo takes to rotate 90°
  3. TIME_PER_DEGREE_MS = measured_time_ms / 90
```

Adjust `PULSE_FORWARD` / `PULSE_BACKWARD` if your servos spin the wrong direction or at inconsistent speed. The values depend on the servo's neutral point calibration.

### Desktop Tuning

- **Horn / Rod lengths** — Edit in the GUI dimension fields to match physical hardware
- **Slider ranges** — Modify `_DOFSlider` min/max in `control_panel.py`
- **Serial port** — Change the `port` parameter in `ui_layout.py` (`SerialComm(port="COM4")`)

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Arduino not found — running offline" | Check COM port in Device Manager; update `port` in `ui_layout.py` |
| Servos jitter at rest | Increase `DEAD_ZONE_DEG` in firmware; reduce slider sensitivity |
| Platform drifts over time | Recalibrate `TIME_PER_DEGREE_MS`; use Zero Hardware button to re-sync |
| "Rod too short for geometry!" | Increase `rod_length` or decrease platform pose range |
| Servo angles show red (⚠) | Pose is near mechanical limits; reduce slider values |
| OpenGL rendering issues | Ensure PyOpenGL is installed: `pip install PyOpenGL PyOpenGL_accelerate` |
| Servos spin wrong direction | Swap `PULSE_FORWARD` and `PULSE_BACKWARD` values in firmware |

---

## License

This project was developed for the **Innovathon** hackathon. See the repository for licensing details
