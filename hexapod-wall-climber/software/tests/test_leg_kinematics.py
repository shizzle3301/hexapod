"""
software/tests/test_leg_kinematics.py

Unit tests for the leg IK/FK solver.
Run with: pytest software/tests/test_leg_kinematics.py -v
"""

import math
import pytest
import numpy as np

from software.kinematics.leg import Leg, LegConfig, JointAngles


@pytest.fixture
def leg():
    return Leg(LegConfig(
        coxa_length=0.030,
        femur_length=0.060,
        tibia_length=0.080,
    ))


class TestForwardKinematics:
    def test_zero_angles_points_along_x(self, leg):
        """At zero angles the foot should be directly along +X."""
        pos = leg.forward_kinematics(JointAngles(0.0, 0.0, 0.0))
        expected_x = leg.cfg.coxa_length + leg.cfg.femur_length + leg.cfg.tibia_length
        assert abs(pos[0] - expected_x) < 1e-6
        assert abs(pos[1]) < 1e-6
        assert abs(pos[2]) < 1e-6

    def test_coxa_rotates_in_xy(self, leg):
        """Rotating only the coxa should keep Z unchanged."""
        a = JointAngles(math.pi / 4, 0.0, 0.0)
        pos = leg.forward_kinematics(a)
        assert abs(pos[2]) < 1e-6
        # X and Y should both be positive
        assert pos[0] > 0
        assert pos[1] > 0

    def test_femur_raises_z(self, leg):
        """A positive femur angle should lift the foot (positive Z)."""
        a = JointAngles(0.0, math.pi / 4, 0.0)
        pos = leg.forward_kinematics(a)
        assert pos[2] > 0


class TestInverseKinematics:
    def test_round_trip_accuracy(self, leg):
        """IK followed by FK should recover the original target within 0.1 mm."""
        targets = [
            np.array([0.12, 0.0, -0.04]),
            np.array([0.10, 0.03, -0.05]),
            np.array([0.08, -0.02, 0.01]),
        ]
        for target in targets:
            angles, ok = leg.inverse_kinematics(target)
            assert ok, f"Target {target} should be reachable"
            recovered = leg.forward_kinematics(angles)
            err_mm = np.linalg.norm(recovered - target) * 1000
            assert err_mm < 0.1, f"IK error {err_mm:.3f} mm for target {target}"

    def test_unreachable_target_too_far(self, leg):
        """A target beyond the arm's full extension should return reachable=False."""
        too_far = np.array([0.50, 0.0, 0.0])
        _, ok = leg.inverse_kinematics(too_far)
        assert not ok

    def test_unreachable_target_too_close(self, leg):
        """A target closer than the minimum reach should return reachable=False."""
        too_close = np.array([0.001, 0.0, 0.0])
        _, ok = leg.inverse_kinematics(too_close)
        assert not ok

    def test_angles_within_limits(self, leg):
        """Returned angles should always respect joint limits."""
        target = np.array([0.10, 0.02, -0.03])
        angles, ok = leg.inverse_kinematics(target)
        assert ok
        cfg = leg.cfg
        assert cfg.coxa_min  <= angles.coxa  <= cfg.coxa_max
        assert cfg.femur_min <= angles.femur <= cfg.femur_max
        assert cfg.tibia_min <= angles.tibia <= cfg.tibia_max

    def test_symmetry_left_right(self, leg):
        """Mirroring Y should mirror the coxa angle only."""
        t_right = np.array([0.10,  0.02, -0.04])
        t_left  = np.array([0.10, -0.02, -0.04])
        a_right, ok_r = leg.inverse_kinematics(t_right)
        a_left,  ok_l = leg.inverse_kinematics(t_left)
        assert ok_r and ok_l
        assert abs(a_right.coxa + a_left.coxa) < 1e-4, "Coxa angles should be equal and opposite"
        assert abs(a_right.femur - a_left.femur) < 1e-4
        assert abs(a_right.tibia - a_left.tibia) < 1e-4


class TestWorkspace:
    def test_workspace_radius_positive(self, leg):
        min_r, max_r = leg.workspace_radius()
        assert min_r > 0
        assert max_r > min_r

    def test_is_reachable(self, leg):
        assert leg.is_reachable(np.array([0.10, 0.0, -0.05]))
        assert not leg.is_reachable(np.array([1.00, 0.0, 0.0]))


class TestJointAngles:
    def test_to_degrees(self):
        a = JointAngles(math.radians(30), math.radians(-45), math.radians(90))
        d = a.to_degrees()
        assert abs(d[0] - 30)  < 1e-4
        assert abs(d[1] - -45) < 1e-4
        assert abs(d[2] - 90)  < 1e-4

    def test_clamp(self):
        cfg = LegConfig(coxa_min=-1.0, coxa_max=1.0)
        a = JointAngles(coxa=5.0, femur=0.0, tibia=0.0)
        clamped = a.clamp(cfg)
        assert clamped.coxa == pytest.approx(1.0)
