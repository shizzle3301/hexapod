"""
software/gait/adhesion_sync.py

Coordinates suction valve timing with the gait cycle.

For wall climbing, suction must be active whenever a foot is in the
stance phase. The valve timing includes small lead/lag margins to ensure
the cup is firmly attached before the foot bears weight, and fully
released before the leg begins its swing.

Hardware model
--------------
Each foot has one silicone suction cup connected to a shared vacuum
pump via an individual solenoid valve (normally-open, so it defaults
to suction when the Pi loses power — a safe fail-state on a wall).

GPIO mapping (BCM numbering, configurable via config/robot.yaml):
    Leg 0 (front-right)  → GPIO 17
    Leg 1 (mid-right)    → GPIO 18
    Leg 2 (rear-right)   → GPIO 27
    Leg 3 (front-left)   → GPIO 22
    Leg 4 (mid-left)     → GPIO 23
    Leg 5 (rear-left)    → GPIO 24
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

try:
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False

from software.gait.planner import GaitFrame, LegPhase

log = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

DEFAULT_VALVE_PINS = [17, 18, 27, 22, 23, 24]

# Safety: maximum time any single foot can be in STANCE without
# suction confirmed (if you add pressure sensors, use this).
MAX_UNSUPPORTED_MS = 500


@dataclass
class AdhesionConfig:
    valve_pins: list[int] = None        # BCM GPIO pin per leg
    active_low: bool = True             # True = GPIO LOW opens valve (common for relay boards)
    pump_pin: int = 25                  # GPIO to control the pump enable
    pump_pwm_freq: int = 1000           # Hz
    pump_duty_idle: float = 60.0        # % PWM when all feet down
    pump_duty_swing: float = 100.0      # % PWM during swing (build up vacuum faster)
    simulate: bool = not _GPIO_AVAILABLE

    def __post_init__(self):
        if self.valve_pins is None:
            self.valve_pins = DEFAULT_VALVE_PINS


# ------------------------------------------------------------------
# Valve controller
# ------------------------------------------------------------------

class AdhesionController:
    """
    Controls the suction pump and per-leg solenoid valves.

    Usage
    -----
    ctrl = AdhesionController(AdhesionConfig())
    ctrl.start()

    for frame in gait_planner.frames():
        ctrl.apply(frame)
        # ... send joint angles to servos ...

    ctrl.stop()
    """

    def __init__(self, config: AdhesionConfig | None = None) -> None:
        self.cfg = config or AdhesionConfig()
        self._valve_state: list[bool] = [False] * 6
        self._pump: object = None
        self._initialized = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Initialise GPIO and start the pump."""
        if self.cfg.simulate:
            log.info("[SIMULATED] AdhesionController started (no GPIO available).")
            self._initialized = True
            return

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in self.cfg.valve_pins:
            GPIO.setup(pin, GPIO.OUT)
            self._set_valve_raw(pin, False)  # all valves closed at startup

        GPIO.setup(self.cfg.pump_pin, GPIO.OUT)
        self._pump = GPIO.PWM(self.cfg.pump_pin, self.cfg.pump_pwm_freq)
        self._pump.start(0)

        self._initialized = True
        log.info("AdhesionController started. Pump on GPIO %d.", self.cfg.pump_pin)

    def stop(self) -> None:
        """Safely shut down: open all valves (release suction), stop pump."""
        if not self._initialized:
            return

        if not self.cfg.simulate:
            for pin in self.cfg.valve_pins:
                self._set_valve_raw(pin, False)   # release all cups
            if self._pump:
                self._pump.stop()
            GPIO.cleanup()

        self._initialized = False
        log.info("AdhesionController stopped.")

    def __enter__(self) -> "AdhesionController":
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Apply a gait frame
    # ------------------------------------------------------------------

    def apply(self, frame: GaitFrame) -> None:
        """
        Open/close valves according to the gait frame's valve_open mask,
        and adjust pump duty based on how many legs are in swing.
        """
        if not self._initialized:
            raise RuntimeError("Call start() before apply().")

        swing_count = sum(1 for v in frame.valve_open if not v)

        for i, (open_valve, pin) in enumerate(
            zip(frame.valve_open, self.cfg.valve_pins)
        ):
            if open_valve != self._valve_state[i]:
                self._set_valve(i, pin, open_valve)

        # Ramp up pump during swing to pre-charge the cups
        if swing_count > 0:
            self._set_pump_duty(self.cfg.pump_duty_swing)
        else:
            self._set_pump_duty(self.cfg.pump_duty_idle)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _set_valve(self, leg_idx: int, pin: int, open_: bool) -> None:
        self._valve_state[leg_idx] = open_
        if self.cfg.simulate:
            log.debug("[SIM] Leg %d valve → %s", leg_idx, "OPEN" if open_ else "CLOSED")
        else:
            self._set_valve_raw(pin, open_)

    def _set_valve_raw(self, pin: int, open_: bool) -> None:
        """Drive GPIO. active_low=True means LOW = open (suction on)."""
        level = GPIO.LOW if (open_ == self.cfg.active_low) else GPIO.HIGH
        GPIO.output(pin, level)

    def _set_pump_duty(self, duty: float) -> None:
        if not self.cfg.simulate and self._pump:
            self._pump.ChangeDutyCycle(float(duty))

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    @property
    def active_legs(self) -> list[int]:
        """Return indices of legs with suction currently active."""
        return [i for i, v in enumerate(self._valve_state) if v]

    def status_str(self) -> str:
        symbols = ["●" if v else "○" for v in self._valve_state]
        return f"Valves: {' '.join(symbols)}  Active: {len(self.active_legs)}/6"
