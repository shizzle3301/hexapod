"""
software/gait/planner.py

Gait planners for the hexapod wall climber.

Supported gaits
---------------
TripodGait  — groups legs into two alternating tripods (fast, less stable)
WaveGait    — moves one leg at a time (slow, maximum stability — default for walls)
RippleGait  — moves two legs at a time in a ripple pattern (compromise)

Each planner is a generator that yields GaitFrame objects: one per control tick,
containing the desired foot positions and adhesion valve states for all six legs.

Leg indexing (matches body.py convention)
-----------------------------------------
    Right: 0=front-right  1=mid-right  2=rear-right
    Left:  3=front-left   4=mid-left   5=rear-left

Tripod groups
    Group A (swing together): 0, 4, 2  (RF, ML, RR)
    Group B (swing together): 3, 1, 5  (LF, MR, LR)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Generator

import numpy as np


class LegPhase(Enum):
    STANCE = auto()   # foot on ground / wall, bearing load
    SWING  = auto()   # foot lifted, moving to next position


@dataclass
class GaitFrame:
    """Output produced by the gait planner for one control tick."""
    foot_targets: list[np.ndarray]    # [6] desired foot positions (world frame)
    leg_phases:   list[LegPhase]      # [6] STANCE or SWING
    valve_open:   list[bool]          # [6] True = suction valve open
    t: float = 0.0                    # Elapsed time (seconds)


@dataclass
class GaitParams:
    """Shared parameters for all gait planners."""
    step_length:   float = 0.04    # m — how far each step moves the foot forward
    step_height:   float = 0.025   # m — how high the foot lifts during swing
    step_duration: float = 0.6     # s — time to complete one step
    duty_factor:   float = 0.5     # fraction of cycle foot is in stance
    control_freq:  float = 50.0    # Hz — ticks per second

    # Adhesion timing offsets within the swing phase
    valve_close_lead: float = 0.05  # s — close valve this early before lift
    valve_open_lag:   float = 0.05  # s — open valve this long after plant


# ------------------------------------------------------------------
# Base class
# ------------------------------------------------------------------

class GaitPlanner:
    """Abstract base — subclass and implement _cycle_offsets()."""

    def __init__(
        self,
        default_feet: list[np.ndarray],
        params: GaitParams | None = None,
    ) -> None:
        self.default_feet = [f.copy() for f in default_feet]
        self.p = params or GaitParams()
        self._offsets = self._cycle_offsets()  # phase offset [0,1) per leg

    def _cycle_offsets(self) -> list[float]:
        raise NotImplementedError

    def _swing_trajectory(
        self,
        start: np.ndarray,
        end: np.ndarray,
        phase: float,          # [0, 1] within swing phase
    ) -> np.ndarray:
        """Smooth raised arc: cubic XY, half-sine Z lift."""
        t = max(0.0, min(1.0, phase))
        # Smooth cubic interpolation for horizontal motion
        s = 3 * t * t - 2 * t * t * t
        pos = start + s * (end - start)
        # Half-sine for vertical lift
        pos = pos.copy()
        pos[2] += self.p.step_height * math.sin(math.pi * t)
        return pos

    def frames(
        self,
        direction: np.ndarray | None = None,
        n_steps: int | None = None,
    ) -> Generator[GaitFrame, None, None]:
        """
        Generator — yields GaitFrame on each control tick.

        direction : unit vector [dx, dy] giving the desired travel direction
                    in the horizontal plane. Defaults to +X (forward).
        n_steps   : stop after this many complete step cycles. None = infinite.
        """
        if direction is None:
            direction = np.array([1.0, 0.0, 0.0])
        direction = np.array(direction, dtype=float)
        if np.linalg.norm(direction[:2]) > 1e-6:
            direction = direction / np.linalg.norm(direction)

        dt = 1.0 / self.p.control_freq
        cycle_time = self.p.step_duration
        swing_frac = 1.0 - self.p.duty_factor

        foot_pos = [f.copy() for f in self.default_feet]
        foot_start = [f.copy() for f in self.default_feet]
        foot_end   = [f.copy() for f in self.default_feet]
        in_swing   = [False] * 6

        t = 0.0
        step_count = [0] * 6

        while True:
            phases = [(t / cycle_time + self._offsets[i]) % 1.0 for i in range(6)]
            leg_phases = []
            valve_open = []

            for i in range(6):
                ph = phases[i]

                if ph >= self.p.duty_factor:
                    # Swing phase
                    swing_ph = (ph - self.p.duty_factor) / swing_frac
                    if not in_swing[i]:
                        # Transition: start of swing
                        in_swing[i] = True
                        foot_start[i] = foot_pos[i].copy()
                        # Target = default position shifted in travel direction
                        step = direction * self.p.step_length
                        foot_end[i] = self.default_feet[i].copy()
                        foot_end[i][:2] += step[:2]
                        step_count[i] += 1

                    foot_pos[i] = self._swing_trajectory(
                        foot_start[i], foot_end[i], swing_ph
                    )
                    leg_phases.append(LegPhase.SWING)
                    # Valve closed during swing (cup is in the air)
                    valve_open.append(False)

                else:
                    # Stance phase
                    if in_swing[i]:
                        in_swing[i] = False
                        foot_pos[i] = foot_end[i].copy()
                    leg_phases.append(LegPhase.STANCE)
                    valve_open.append(True)

            yield GaitFrame(
                foot_targets=[p.copy() for p in foot_pos],
                leg_phases=leg_phases,
                valve_open=valve_open,
                t=t,
            )

            t += dt

            if n_steps is not None:
                if all(c >= n_steps for c in step_count):
                    break


# ------------------------------------------------------------------
# Tripod gait  (2 groups, 3 legs each)
# ------------------------------------------------------------------

class TripodGait(GaitPlanner):
    """
    Fastest gait. Alternates between two groups of three legs.
    Group A: legs 0, 2, 4  (RF, RR, ML)
    Group B: legs 1, 3, 5  (MR, LF, LR)
    """

    def _cycle_offsets(self) -> list[float]:
        # Group A at 0, Group B at 0.5 (half cycle offset)
        return [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]


# ------------------------------------------------------------------
# Wave gait  (one leg at a time)
# ------------------------------------------------------------------

class WaveGait(GaitPlanner):
    """
    Most stable gait. Only one leg is in swing at any time.
    Legs move in sequence: 0 → 1 → 2 → 3 → 4 → 5.
    Duty factor should be ≥ 5/6 ≈ 0.833 to ensure only one leg swings.
    Best choice for vertical wall climbing.
    """

    def _cycle_offsets(self) -> list[float]:
        # Evenly space all 6 legs through the cycle
        return [i / 6.0 for i in range(6)]

    def __init__(
        self,
        default_feet: list[np.ndarray],
        params: GaitParams | None = None,
    ) -> None:
        if params is None:
            params = GaitParams(duty_factor=5 / 6)
        elif params.duty_factor < 5 / 6:
            import warnings
            warnings.warn(
                "WaveGait duty_factor < 5/6: multiple legs may swing simultaneously.",
                stacklevel=2,
            )
        super().__init__(default_feet, params)


# ------------------------------------------------------------------
# Ripple gait  (two legs at a time)
# ------------------------------------------------------------------

class RippleGait(GaitPlanner):
    """
    Two legs swing at once (one from each side), giving a ripple effect.
    Pairs: (0,5), (1,3), (2,4)
    Duty factor should be ≥ 2/3 ≈ 0.667.
    """

    def _cycle_offsets(self) -> list[float]:
        return [0.0, 1/3, 2/3, 1/2, 5/6, 1/6]

    def __init__(
        self,
        default_feet: list[np.ndarray],
        params: GaitParams | None = None,
    ) -> None:
        if params is None:
            params = GaitParams(duty_factor=2 / 3)
        super().__init__(default_feet, params)


# ------------------------------------------------------------------
# Factory
# ------------------------------------------------------------------

GAIT_REGISTRY: dict[str, type[GaitPlanner]] = {
    "tripod": TripodGait,
    "wave":   WaveGait,
    "ripple": RippleGait,
}


def make_gait(
    name: str,
    default_feet: list[np.ndarray],
    params: GaitParams | None = None,
) -> GaitPlanner:
    """Instantiate a gait planner by name."""
    name = name.lower()
    if name not in GAIT_REGISTRY:
        raise ValueError(f"Unknown gait '{name}'. Choose from: {list(GAIT_REGISTRY)}")
    return GAIT_REGISTRY[name](default_feet, params)
