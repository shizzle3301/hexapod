"""
software/kinematics/leg.py

Analytic inverse kinematics and forward kinematics for a single 3-DOF leg.

Leg coordinate frame:
    - Origin at the coxa joint (mounted on the body)
    - X-axis points outward along the default leg direction
    - Z-axis points up (away from the surface the robot walks on)

Joints:
    q1 = coxa  (rotation about Z, swings leg forward/backward)
    q2 = femur (rotation about Y, raises/lowers leg)
    q3 = tibia (rotation about Y, extends/retracts lower leg)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class LegConfig:
    """Physical dimensions of one leg (metres)."""
    coxa_length: float = 0.030   # m — body mount to coxa pivot
    femur_length: float = 0.060  # m — coxa pivot to knee
    tibia_length: float = 0.080  # m — knee to foot tip

    # Joint limits (radians)
    coxa_min:  float = -math.radians(60)
    coxa_max:  float =  math.radians(60)
    femur_min: float = -math.radians(90)
    femur_max: float =  math.radians(90)
    tibia_min: float = -math.radians(120)
    tibia_max: float =  math.radians(10)


@dataclass
class JointAngles:
    """Joint angles for one leg (radians)."""
    coxa:  float = 0.0
    femur: float = 0.0
    tibia: float = 0.0

    def to_degrees(self) -> tuple[float, float, float]:
        return (
            math.degrees(self.coxa),
            math.degrees(self.femur),
            math.degrees(self.tibia),
        )

    def clamp(self, cfg: LegConfig) -> "JointAngles":
        """Return a copy with angles clamped to joint limits."""
        return JointAngles(
            coxa=float(np.clip(self.coxa, cfg.coxa_min, cfg.coxa_max)),
            femur=float(np.clip(self.femur, cfg.femur_min, cfg.femur_max)),
            tibia=float(np.clip(self.tibia, cfg.tibia_min, cfg.tibia_max)),
        )


class Leg:
    """
    3-DOF leg kinematics.

    Usage
    -----
    leg = Leg(LegConfig())

    # Forward kinematics: angles → foot position
    pos = leg.forward_kinematics(JointAngles(0.1, -0.5, 0.8))

    # Inverse kinematics: foot position → angles
    angles, ok = leg.inverse_kinematics(np.array([0.12, 0.0, -0.05]))
    """

    def __init__(self, config: LegConfig | None = None) -> None:
        self.cfg = config or LegConfig()

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def forward_kinematics(self, angles: JointAngles) -> np.ndarray:
        """
        Compute foot tip position in the leg's local frame.

        Returns
        -------
        np.ndarray shape (3,) — [x, y, z] foot position in metres
        """
        q1, q2, q3 = angles.coxa, angles.femur, angles.tibia
        L1 = self.cfg.coxa_length
        L2 = self.cfg.femur_length
        L3 = self.cfg.tibia_length

        # Project into the vertical plane defined by q1
        r = L1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)
        z = L2 * math.sin(q2) + L3 * math.sin(q2 + q3)

        x = r * math.cos(q1)
        y = r * math.sin(q1)
        return np.array([x, y, z])

    # ------------------------------------------------------------------
    # Inverse kinematics  (analytic, closed-form)
    # ------------------------------------------------------------------

    def inverse_kinematics(
        self,
        target: np.ndarray,
        elbow_up: bool = False,
    ) -> tuple[JointAngles, bool]:
        """
        Compute joint angles to reach a target foot position.

        Parameters
        ----------
        target : array-like [x, y, z] in the leg's local frame (metres)
        elbow_up : if True, prefer the elbow-up solution

        Returns
        -------
        angles : JointAngles (clamped to limits)
        reachable : bool — False if the target is outside the workspace
        """
        x, y, z = float(target[0]), float(target[1]), float(target[2])
        L1 = self.cfg.coxa_length
        L2 = self.cfg.femur_length
        L3 = self.cfg.tibia_length

        # --- Coxa angle (q1): rotate into the leg's vertical plane ---
        q1 = math.atan2(y, x)

        # Horizontal distance from coxa pivot to foot, minus coxa length
        r = math.hypot(x, y) - L1

        # --- Distance from femur pivot to foot ---
        D2 = r * r + z * z
        D = math.sqrt(D2)

        if D > L2 + L3 or D < abs(L2 - L3):
            # Target unreachable
            angles = JointAngles(q1, 0.0, 0.0).clamp(self.cfg)
            return angles, False

        # --- Tibia angle (q3) via cosine rule ---
        cos_q3 = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
        cos_q3 = float(np.clip(cos_q3, -1.0, 1.0))
        q3_mag = math.acos(cos_q3)
        q3 = -q3_mag if elbow_up else q3_mag  # sign convention: tibia bends inward

        # --- Femur angle (q2) ---
        alpha = math.atan2(z, r)
        beta = math.atan2(L3 * math.sin(abs(q3)), L2 + L3 * math.cos(abs(q3)))
        q2 = alpha - beta if not elbow_up else alpha + beta

        angles = JointAngles(q1, q2, q3).clamp(self.cfg)
        return angles, True

    # ------------------------------------------------------------------
    # Workspace helpers
    # ------------------------------------------------------------------

    def is_reachable(self, target: np.ndarray) -> bool:
        _, ok = self.inverse_kinematics(target)
        return ok

    def workspace_radius(self) -> tuple[float, float]:
        """Return (min_reach, max_reach) horizontal distances from the coxa."""
        cfg = self.cfg
        min_reach = cfg.coxa_length + abs(cfg.femur_length - cfg.tibia_length)
        max_reach = cfg.coxa_length + cfg.femur_length + cfg.tibia_length
        return min_reach, max_reach


# ------------------------------------------------------------------
# Quick smoke-test
# ------------------------------------------------------------------

if __name__ == "__main__":
    cfg = LegConfig()
    leg = Leg(cfg)

    target = np.array([0.12, 0.02, -0.04])
    angles, ok = leg.inverse_kinematics(target)
    if ok:
        recovered = leg.forward_kinematics(angles)
        err = np.linalg.norm(recovered - target) * 1000  # mm
        print(f"Target:    {target}")
        print(f"Recovered: {recovered}")
        print(f"IK error:  {err:.3f} mm")
        print(f"Angles (deg): coxa={angles.to_degrees()[0]:.1f}  "
              f"femur={angles.to_degrees()[1]:.1f}  "
              f"tibia={angles.to_degrees()[2]:.1f}")
    else:
        print("Target is outside the workspace.")
