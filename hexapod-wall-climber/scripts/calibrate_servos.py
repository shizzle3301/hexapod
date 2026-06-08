#!/usr/bin/env python3
"""
scripts/calibrate_servos.py

Interactive servo calibration utility.
Walks through each servo, lets you manually move it to min/neutral/max,
and saves the pulse widths to config/robot.yaml.

Usage
-----
python scripts/calibrate_servos.py --config config/robot.yaml
python scripts/calibrate_servos.py --leg 0  # calibrate one leg only
"""

import argparse
import sys
from pathlib import Path

import yaml

JOINT_NAMES = ["coxa", "femur", "tibia"]
LEG_NAMES   = [
    "front-right", "mid-right", "rear-right",
    "front-left",  "mid-left",  "rear-left",
]


def prompt_us(prompt: str, default: int) -> int:
    while True:
        raw = input(f"  {prompt} [default {default}]: ").strip()
        if not raw:
            return default
        try:
            v = int(raw)
            if 400 <= v <= 2600:
                return v
            print("  Value must be between 400 and 2600 µs.")
        except ValueError:
            print("  Please enter an integer.")


def calibrate_leg(leg_idx: int, existing: dict) -> dict:
    name = LEG_NAMES[leg_idx]
    print(f"\n── Leg {leg_idx}: {name} ──")
    result = {}
    for joint in JOINT_NAMES:
        ex = existing.get(joint, {})
        print(f"  Joint: {joint}")
        neutral = prompt_us("Neutral (0°) pulse µs", ex.get("neutral_us", 1500))
        min_us  = prompt_us("Min limit pulse µs",    ex.get("min_us",     500))
        max_us  = prompt_us("Max limit pulse µs",    ex.get("max_us",     2500))
        rev_raw = input(f"  Reversed? [y/N]: ").strip().lower()
        reversed_ = rev_raw in ("y", "yes")
        result[joint] = {
            "neutral_us": neutral,
            "min_us":     min_us,
            "max_us":     max_us,
            "reversed":   reversed_,
        }
    return result


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="config/robot.yaml")
    p.add_argument("--leg",    type=int, default=None, help="Calibrate only this leg (0–5)")
    args = p.parse_args()

    config_path = Path(args.config)
    if config_path.exists():
        with open(config_path) as f:
            cfg = yaml.safe_load(f) or {}
    else:
        cfg = {}

    cfg.setdefault("servo_hat", {}).setdefault("calibration", {})
    cal = cfg["servo_hat"]["calibration"]

    legs = [args.leg] if args.leg is not None else list(range(6))

    print("Servo calibration utility")
    print("Move each servo to the requested position and enter the pulse width.")
    print("Press Enter to keep the existing value.\n")

    for i in legs:
        key = f"leg_{i}"
        cal[key] = calibrate_leg(i, cal.get(key, {}))

    config_path.parent.mkdir(parents=True, exist_ok=True)
    with open(config_path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)

    print(f"\nCalibration saved to {config_path}.")


if __name__ == "__main__":
    main()
