"""
software/utils/servo_controller.py

Drives the PCA9685 16-channel PWM servo HAT over I2C.

Channel mapping (configurable in config/robot.yaml):
    Leg 0 (FR): channels 0, 1, 2   (coxa, femur, tibia)
    Leg 1 (MR): channels 3, 4, 5
    Leg 2 (RR): channels 6, 7, 8
    Leg 3 (FL): channels 9, 10, 11
    Leg 4 (ML): channels 12, 13, 14
    Leg 5 (RL): channels 15, — (overflow to second HAT if needed)

Each servo's neutral PWM pulse and range can be calibrated individually
via scripts/calibrate_servos.py and saved to config/robot.yaml.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field

try:
    from adafruit_pca9685 import PCA9685         # type: ignore
    from adafruit_motor import servo as af_servo  # type: ignore
    import board                                  # type: ignore
    import busio                                  # type: ignore
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False

from software.kinematics.leg import JointAngles

log = logging.getLogger(__name__)

# ------------------------------------------------------------------
# Per-servo calibration record
# ------------------------------------------------------------------

@dataclass
class ServoCalibration:
    """Maps a joint angle (radians) to a PWM pulse width (microseconds)."""
    neutral_us: int   = 1500   # µs — pulse for 0 rad
    min_us:     int   = 500    # µs — pulse for most negative joint limit
    max_us:     int   = 2500   # µs — pulse for most positive joint limit
    min_angle:  float = -math.pi / 2
    max_angle:  float =  math.pi / 2
    reversed:   bool  = False  # set True if servo is physically mirrored

    def angle_to_us(self, angle_rad: float) -> int:
        """Convert a joint angle to a PWM pulse width in microseconds."""
        ratio = (angle_rad - self.min_angle) / (self.max_angle - self.min_angle)
        ratio = max(0.0, min(1.0, ratio))
        if self.reversed:
            ratio = 1.0 - ratio
        us = self.min_us + ratio * (self.max_us - self.min_us)
        return int(round(us))


# ------------------------------------------------------------------
# Controller
# ------------------------------------------------------------------

@dataclass
class ServoControllerConfig:
    i2c_address: int = 0x40
    pwm_freq_hz: int = 50
    simulate:    bool = not _HW_AVAILABLE

    # 6 legs × 3 joints = 18 servo channels
    # List of (channel, calibration) per joint, indexed [leg][joint]
    calibrations: list[list[ServoCalibration]] = field(
        default_factory=lambda: [
            [ServoCalibration() for _ in range(3)] for _ in range(6)
        ]
    )

    @property
    def channel_map(self) -> list[list[int]]:
        """Default channel assignment: leg i occupies channels 3i, 3i+1, 3i+2."""
        return [[3 * i, 3 * i + 1, 3 * i + 2] for i in range(6)]


class ServoController:
    """
    Thin wrapper around the PCA9685 HAT.

    Usage
    -----
    ctrl = ServoController(ServoControllerConfig())
    ctrl.start()
    ctrl.set_leg_angles(leg_idx=0, angles=JointAngles(0.1, -0.4, 0.7))
    ctrl.set_all(angles_per_leg)   # list[JointAngles | None], length 6
    ctrl.stop()
    """

    def __init__(self, config: ServoControllerConfig | None = None) -> None:
        self.cfg = config or ServoControllerConfig()
        self._pca: object = None
        self._servos: list[list[object]] = []

    def start(self) -> None:
        if self.cfg.simulate:
            log.info("[SIMULATED] ServoController started (no PCA9685 available).")
            return

        i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(i2c, address=self.cfg.i2c_address)
        self._pca.frequency = self.cfg.pwm_freq_hz
        log.info("PCA9685 initialised at 0x%02X, %d Hz.", self.cfg.i2c_address, self.cfg.pwm_freq_hz)

    def stop(self) -> None:
        if self._pca is not None:
            self._pca.deinit()
            log.info("PCA9685 released.")

    def __enter__(self) -> "ServoController":
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Angle → PWM
    # ------------------------------------------------------------------

    def set_leg_angles(self, leg_idx: int, angles: JointAngles) -> None:
        """Drive the three servos of one leg to the given joint angles."""
        joints = [angles.coxa, angles.femur, angles.tibia]
        channels = self.cfg.channel_map[leg_idx]
        cals = self.cfg.calibrations[leg_idx]

        for joint_angle, ch, cal in zip(joints, channels, cals):
            pulse_us = cal.angle_to_us(joint_angle)
            self._set_channel_us(ch, pulse_us)

    def set_all(self, angles_list: list[JointAngles | None]) -> None:
        """Set all 6 legs at once. Pass None for a leg to leave it unchanged."""
        for i, angles in enumerate(angles_list):
            if angles is not None:
                self.set_leg_angles(i, angles)

    def centre_all(self) -> None:
        """Move all servos to their neutral (0 rad) position."""
        for i in range(6):
            self.set_leg_angles(i, JointAngles(0.0, 0.0, 0.0))

    # ------------------------------------------------------------------
    # Low-level
    # ------------------------------------------------------------------

    def _set_channel_us(self, channel: int, pulse_us: int) -> None:
        if self.cfg.simulate:
            log.debug("[SIM] ch%02d → %d µs", channel, pulse_us)
            return

        # PCA9685 tick resolution at 50 Hz: 1 tick = 1e6 / (50 * 4096) µs
        ticks_per_us = (self.cfg.pwm_freq_hz * 4096) / 1_000_000
        on_ticks = int(round(pulse_us * ticks_per_us))
        on_ticks = max(0, min(4095, on_ticks))
        self._pca.channels[channel].duty_cycle = int(on_ticks / 4095 * 0xFFFF)
