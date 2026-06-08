"""
software/kinematics/body.py

Maps a desired body pose (position + orientation) to foot-tip targets
for all six legs, then solves IK on each leg.

Leg mounting positions follow the standard hexapod convention:
    Legs 0–2: right side (positive Y in body frame), front to rear
    Legs 3–5: left side  (negative Y in body frame), front to rear

         Front
     [2]       [3]
     [1]       [4]
     [0]       [5]
         Rear
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from software.kinematics.leg import Leg, LegConfig, JointAngles


# ------------------------------------------------------------------
# Body configuration
# ------------------------------------------------------------------

@dataclass
class BodyConfig:
    """
    Physical layout of the hexapod body.
    All dimensions in metres.
    """
    # Lateral distance from body centre to coxa mounting point
    body_width: float = 0.080

    # Longitudinal spacing between leg pairs
    body_length_front: float = 0.060   # centre → front pair
    body_length_mid:   float = 0.000   # centre → middle pair (0 = centred)
    body_length_rear:  float = -0.060  # centre → rear pair

    # Default mounting angle of each coxa (radians, measured from +X body axis)
    # Right side: 0, 30, 60 deg outward; Left side: mirrored
    mount_angles_right: list[float] = field(default_factory=lambda: [
        math.radians(a) for a in [30, 0, -30]
    ])

    leg_config: LegConfig = field(default_factory=LegConfig)

    # Default standing foot position in each leg's local frame [x, y, z]
    default_foot_local: np.ndarray = field(
        default_factory=lambda: np.array([0.10, 0.0, -0.06])
    )


# ------------------------------------------------------------------
# Body kinematics solver
# ------------------------------------------------------------------

class Body:
    """
    Full-body kinematics for a hexapod.

    Coordinate frames
    -----------------
    world  — fixed inertial frame (Z up)
    body   — centred on the robot body; follows body pose
    leg[i] — centred on the coxa mount of leg i; fixed relative to body

    Usage
    -----
    body = Body(BodyConfig())

    # Get default joint angles (standing pose)
    angles = body.stand()

    # Move body, get new joint angles
    pose = BodyPose(position=np.array([0, 0, 0.05]), roll=0.1)
    angles = body.solve(pose, foot_targets_world)
    """

    def __init__(self, config: BodyConfig | None = None) -> None:
        self.cfg = config or BodyConfig()
        self.legs = [Leg(self.cfg.leg_config) for _ in range(6)]
        self._mount_transforms = self._build_mount_transforms()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _build_mount_transforms(self) -> list[np.ndarray]:
        """
        Build 4×4 homogeneous transforms: body frame → each leg's local frame.
        """
        cfg = self.cfg
        transforms = []

        long_positions = [
            cfg.body_length_front,
            cfg.body_length_mid,
            cfg.body_length_rear,
        ]

        for side, y_sign, angles in [
            ("right", +1, cfg.mount_angles_right),
            ("left",  -1, [-a for a in cfg.mount_angles_right]),
        ]:
            for i, (long, angle) in enumerate(zip(long_positions, angles)):
                T = np.eye(4)
                T[0, 3] = long
                T[1, 3] = y_sign * cfg.body_width
                # Rotation about Z by mount angle
                c, s = math.cos(angle), math.sin(angle)
                T[0, 0], T[0, 1] = c, -s
                T[1, 0], T[1, 1] = s,  c
                transforms.append(T)

        return transforms  # 6 transforms, right legs first

    # ------------------------------------------------------------------
    # Default stance
    # ------------------------------------------------------------------

    def default_foot_positions_world(
        self,
        body_pose: "BodyPose | None" = None,
    ) -> list[np.ndarray]:
        """
        Compute foot positions in world frame for the default standing pose.
        """
        if body_pose is None:
            body_pose = BodyPose()

        T_world_body = body_pose.transform()
        positions = []

        for i, T_body_leg in enumerate(self._mount_transforms):
            T_world_leg = T_world_body @ T_body_leg
            default_local = self.cfg.default_foot_local.copy()
            foot_world = T_world_leg[:3, :3] @ default_local + T_world_leg[:3, 3]
            positions.append(foot_world)

        return positions

    # ------------------------------------------------------------------
    # Full body IK solve
    # ------------------------------------------------------------------

    def solve(
        self,
        body_pose: "BodyPose",
        foot_targets_world: list[np.ndarray],
    ) -> list[JointAngles | None]:
        """
        Given a body pose and desired foot positions in world frame,
        solve IK for all six legs.

        Returns a list of JointAngles (or None if a leg is unreachable).
        """
        T_world_body = body_pose.transform()
        T_body_world = np.linalg.inv(T_world_body)
        results = []

        for i, (T_body_leg, foot_world) in enumerate(
            zip(self._mount_transforms, foot_targets_world)
        ):
            # Transform foot target into leg-local frame
            T_leg_body = np.linalg.inv(T_body_leg)
            T_leg_world = T_leg_body @ T_body_world

            foot_local = T_leg_world[:3, :3] @ foot_world + T_leg_world[:3, 3]
            angles, ok = self.legs[i].inverse_kinematics(foot_local)
            results.append(angles if ok else None)

        return results

    def stand(self) -> list[JointAngles | None]:
        """Return joint angles for the default upright standing pose."""
        pose = BodyPose()
        targets = self.default_foot_positions_world(pose)
        return self.solve(pose, targets)


# ------------------------------------------------------------------
# Body pose descriptor
# ------------------------------------------------------------------

@dataclass
class BodyPose:
    """
    6-DOF body pose in the world frame.

    position : [x, y, z] body centre in metres
    roll     : rotation about X (radians)
    pitch    : rotation about Y (radians)
    yaw      : rotation about Z (radians)
    """
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    roll:  float = 0.0
    pitch: float = 0.0
    yaw:   float = 0.0

    def transform(self) -> np.ndarray:
        """Return a 4×4 homogeneous world→body transform."""
        cr, sr = math.cos(self.roll),  math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw),   math.sin(self.yaw)

        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp],  [0, 1, 0],  [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0],    [0, cr, -sr], [0, sr, cr]])
        R = Rz @ Ry @ Rx

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = self.position
        return T
