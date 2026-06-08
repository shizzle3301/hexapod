# Contributing

Thanks for your interest in contributing to this project!

## Getting started

1. Fork the repo and clone your fork
2. Create a branch: `git checkout -b feature/my-feature`
3. Set up the dev environment:
   ```bash
   python3 -m venv .venv && source .venv/bin/activate
   pip install -r requirements.txt
   ```
4. Make your changes with tests
5. Run tests: `pytest software/tests/ -v`
6. Open a pull request against `main`

## Areas that need help

- **CAD**: Parametric chassis design in Fusion 360 / OpenSCAD
- **Gait**: Adaptive gait switching (detect when a leg slips and recover)
- **Sensing**: IMU-based tilt compensation for wall climbing
- **Vision**: Camera-based edge detection to know when the robot reaches the top
- **ROS2**: Bridge between the Python controller and ROS2 topics
- **Testing**: Hardware-in-the-loop test harness

## Code style

- Python: follow PEP 8, use type hints, keep functions under ~50 lines
- Docstrings: NumPy style
- Commit messages: conventional commits (`feat:`, `fix:`, `docs:`, `test:`)
- All new code should have corresponding tests

## Opening issues

Please include:
- What you expected to happen
- What actually happened
- Steps to reproduce
- Hardware configuration if relevant (which servos, which Pi version, etc.)
