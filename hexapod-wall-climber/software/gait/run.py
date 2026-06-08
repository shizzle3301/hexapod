"""
software/gait/run.py

Main entry point for running the hexapod.

Usage
-----
python -m software.gait.run --gait wave --surface vertical --speed 0.5
python -m software.gait.run --gait tripod --surface flat --steps 10
python -m software.gait.run --calibrate

Arguments
---------
--gait      : tripod | wave | ripple  (default: wave)
--surface   : flat | vertical         (default: flat)
--speed     : 0.0–1.0 speed scale     (default: 0.3)
--steps     : number of steps, 0=infinite (default: 0)
--config    : path to robot.yaml      (default: config/robot.yaml)
--simulate  : force simulation mode even with GPIO available
--calibrate : run servo calibration routine and exit
--verbose   : enable debug logging
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from software.kinematics.body import Body, BodyConfig, BodyPose
from software.gait.planner import GaitParams, make_gait
from software.gait.adhesion_sync import AdhesionController, AdhesionConfig
from software.utils.servo_controller import ServoController, ServoControllerConfig

log = logging.getLogger("hexapod")


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def load_config(path: str) -> dict:
    p = Path(path)
    if not p.exists():
        log.warning("Config not found at %s — using defaults.", path)
        return {}
    with open(p) as f:
        return yaml.safe_load(f) or {}


def build_gait_params(cfg: dict, speed_scale: float, surface: str) -> GaitParams:
    g = cfg.get("gait", {})
    p = GaitParams(
        step_length=g.get("step_length", 0.04) * speed_scale,
        step_height=g.get("step_height", 0.025),
        step_duration=g.get("step_duration", 0.6) / max(speed_scale, 0.1),
        control_freq=g.get("control_freq", 50.0),
    )
    if surface == "vertical":
        # Shorter steps, higher duty factor for wall stability
        p.step_length *= 0.6
        p.step_height *= 0.8
        p.duty_factor = 5 / 6
    return p


# ------------------------------------------------------------------
# Main loop
# ------------------------------------------------------------------

def run(args: argparse.Namespace) -> None:
    cfg = load_config(args.config)

    # --- Kinematics setup ---
    body = Body(BodyConfig())
    default_feet = body.default_foot_positions_world()

    if args.surface == "vertical":
        # Body pitched back slightly to press feet into wall
        pose = BodyPose(
            position=np.array([0.0, 0.0, 0.0]),
            pitch=np.radians(5),
        )
        default_feet = body.default_foot_positions_world(pose)
        log.info("Vertical surface mode: body pitched 5° toward wall.")
    else:
        pose = BodyPose()

    # --- Gait planner ---
    params = build_gait_params(cfg, args.speed, args.surface)
    n_steps = args.steps if args.steps > 0 else None
    planner = make_gait(args.gait, default_feet, params)
    log.info("Gait: %s  Surface: %s  Speed scale: %.2f", args.gait, args.surface, args.speed)

    # --- Hardware controllers ---
    sim = args.simulate or cfg.get("simulate", False)
    adhesion_cfg = AdhesionConfig(simulate=sim)
    servo_cfg = ServoControllerConfig(simulate=sim)

    # --- Graceful shutdown ---
    _running = True
    def _handle_sigint(sig, frame):
        nonlocal _running
        log.info("Interrupted — shutting down.")
        _running = False
    signal.signal(signal.SIGINT, _handle_sigint)

    with AdhesionController(adhesion_cfg) as adhesion, \
         ServoController(servo_cfg) as servos:

        log.info("Starting gait loop. Press Ctrl-C to stop.")
        tick = 0

        for frame in planner.frames(n_steps=n_steps):
            if not _running:
                break

            t_start = time.monotonic()

            # Solve body IK for this frame
            joint_angles = body.solve(pose, frame.foot_targets)

            # Drive servos
            servos.set_all(joint_angles)

            # Drive suction valves
            adhesion.apply(frame)

            # Logging every 50 ticks (1 second at 50 Hz)
            if tick % 50 == 0:
                log.info("t=%.1fs  %s", frame.t, adhesion.status_str())

            # Rate limiting
            elapsed = time.monotonic() - t_start
            sleep = (1.0 / params.control_freq) - elapsed
            if sleep > 0:
                time.sleep(sleep)

            tick += 1

    log.info("Hexapod stopped cleanly.")


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Hexapod wall climber controller")
    p.add_argument("--gait",     default="wave",         choices=["tripod","wave","ripple"])
    p.add_argument("--surface",  default="flat",          choices=["flat","vertical"])
    p.add_argument("--speed",    default=0.3,             type=float)
    p.add_argument("--steps",    default=0,               type=int)
    p.add_argument("--config",   default="config/robot.yaml")
    p.add_argument("--simulate", action="store_true")
    p.add_argument("--verbose",  action="store_true")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s  %(name)s  %(levelname)s  %(message)s",
    )
    run(args)
