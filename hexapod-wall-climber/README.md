# 🦟 Hexapod Wall Climber

A small, lightweight six-legged robot capable of climbing vertical surfaces using suction-based adhesion. Built around a 3-DOF-per-leg design with a tripod gait planner and real-time inverse kinematics.

![Build Status](https://img.shields.io/badge/build-in%20progress-yellow)
![License](https://img.shields.io/badge/license-MIT-blue)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-red)

---

## Overview

This project explores bio-inspired locomotion combined with vacuum adhesion to create a robot that can traverse smooth vertical surfaces (glass, painted metal, tiles). The design prioritises low weight, modularity, and ease of fabrication with consumer-grade 3D printing and off-the-shelf electronics.

**Key specs (target):**
| Parameter | Value |
|---|---|
| Total weight | < 500 g |
| Number of legs | 6 |
| DOF per leg | 3 (coxa, femur, tibia) |
| Adhesion method | Miniature vacuum suction |
| Suction force (target) | ≥ 750 g total |
| Max wall speed | ~5 cm/s |
| Battery life | ~30 min (3S LiPo 2200 mAh) |
| Controller | Raspberry Pi 5 + PCA9685 servo HAT |

---

## Repository Structure

```
hexapod-wall-climber/
├── firmware/               # Low-level servo and pump control (C/C++ or MicroPython)
│   ├── src/
│   ├── include/
│   └── lib/
├── software/               # High-level Python control software
│   ├── gait/               # Gait planners (tripod, wave, ripple)
│   ├── kinematics/         # Forward & inverse kinematics
│   ├── vision/             # Optional camera-based wall detection
│   ├── utils/              # Helpers, logging, calibration
│   └── tests/              # Unit and integration tests
├── hardware/
│   ├── cad/                # 3D printable STL / Fusion 360 files
│   ├── schematics/         # Wiring diagrams and PCB layouts
│   └── bom/                # Bill of materials
├── config/                 # YAML config files for robot parameters
├── scripts/                # Utility shell scripts (flash, setup, calibrate)
├── docs/                   # Documentation and research references
└── README.md
```

---

## Getting Started

### 1. Hardware you'll need

See [`hardware/bom/bom.csv`](hardware/bom/bom.csv) for the full list. Core components:

- Raspberry Pi 5 (4 GB)
- PCA9685 16-channel servo HAT
- 18× MG90S metal-gear servos (or DS3225 for more torque)
- 6× silicone suction cups (30–40 mm diameter)
- 1× miniature 12V vacuum pump
- 6× solenoid valves (normally-open, 12V)
- MPU-6050 IMU breakout
- 3S LiPo battery (2200 mAh) + BEC/regulator
- 3D-printed chassis (PETG or ASA recommended)

### 2. Software setup

```bash
# Clone the repo
git clone https://github.com/YOUR_USERNAME/hexapod-wall-climber.git
cd hexapod-wall-climber

# Create a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Copy and edit the config
cp config/robot_default.yaml config/robot.yaml
nano config/robot.yaml
```

### 3. Calibrate servos

```bash
python scripts/calibrate_servos.py --config config/robot.yaml
```

### 4. Run a basic gait test (flat ground)

```bash
python -m software.gait.run --gait tripod --surface flat --speed 0.3
```

### 5. Run wall-climbing mode

```bash
python -m software.gait.run --gait tripod --surface vertical --speed 0.1
```

---

## Adhesion System

The robot uses a centralised vacuum pump connected to each foot via silicone tubing and individual solenoid valves. This lets the controller independently activate suction on each foot in sync with the gait cycle.

```
Pump → Manifold → Valve 1 → Foot 1
                → Valve 2 → Foot 2
                → ...
                → Valve 6 → Foot 6
```

The controller opens a valve just before a foot is planted, holds suction while the foot bears weight, then closes the valve and briefly vents the cup before the lift phase. Timing is critical — see [`software/gait/adhesion_sync.py`](software/gait/adhesion_sync.py).

---

## Kinematics

Each leg uses a 3-DOF serial chain: **coxa → femur → tibia**. Inverse kinematics are solved analytically (closed-form), which keeps computation fast enough for real-time control.

```
software/kinematics/
├── leg.py          # Single leg IK/FK
├── body.py         # Body pose to foot position mapping
└── workspace.py    # Reachable workspace visualiser
```

For wall-climbing, the body orientation from the IMU is used to rotate the workspace so foot targets are always perpendicular to the wall surface.

---

## Gait Planner

Three gaits are implemented:

| Gait | Stability | Speed | Notes |
|---|---|---|---|
| Tripod | Medium | Fast | 3 legs move at once. Best for flat ground |
| Wave | High | Slow | 1 leg at a time. Best for walls |
| Ripple | High | Medium | 2 legs at a time. Good compromise |

Wall-climbing defaults to **wave gait** for maximum stability margin.

---

## Roadmap

- [x] Leg IK/FK solver
- [x] Tripod gait (flat ground)
- [x] Wave gait (flat ground)
- [ ] Suction valve sync with gait
- [ ] Wall-climbing wave gait
- [ ] IMU-based orientation correction
- [ ] Slip detection via foot force sensors
- [ ] Autonomous wall navigation
- [ ] ROS2 integration
- [ ] Web-based telemetry dashboard

---

## Contributing

Pull requests are welcome. Please open an issue first to discuss significant changes. See [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines.

---

## License

MIT — see [`LICENSE`](LICENSE).

---

## References

- Haynes, G.C. & Rizzi, A.A. — *Gaits and gait transitions for legged robots*
- Autumn, K. et al. — *Gecko adhesion: structure, function, and applications* (for dry adhesive background)
- Neville, N. & Buehler, M. — *Towards bipedal running of a six-legged robot*
- RHex robot platform — [BostonDynamics / Kodlab CMU](https://kodlab.seas.upenn.edu/robots/rhex/)
