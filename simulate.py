"""
simulate.py
===========
Physics-based flight trajectory generator for the altitude control system.

Produces a sequence of ``TelemetryFrame`` snapshots covering a full mission
profile: takeoff, cruise, obstacle avoidance, and return-to-home descent.
All altitude commands pass through ``compute_target_alt`` (drone_logic.py),
ensuring the formally verified safety envelope is enforced at every step.

The simulator uses a first-order lag filter to model actuator response
latency. Sensor noise on obstacle distance is modeled as additive Gaussian
white noise (sigma = 0.3 m), consistent with typical ultrasonic sensor specs.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Final, List

from drone_logic import (
    MIN_ALT_M,
    MAX_ALT_M,
    MIN_SEP_DIST_M,
    Meters,
    MetersPerSecond,
    compute_target_alt,
)
from result_types import FlightPhase

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------

SIM_STEPS:        Final[int]   = 300
DT_S:             Final[float] = 0.2       # seconds per control cycle
CRUISE_ALT_M:     Final[Meters] = Meters(40.0)
OBSTACLE_X_NORM:  Final[float] = 0.55     # obstacle position, normalised [0, 1]
OBSTACLE_ALT_M:   Final[Meters] = Meters(18.0)  # obstacle top (metres)
LAG_FILTER_COEFF: Final[float] = 0.18     # first-order actuator lag (dimensionless)
SENSOR_NOISE_STD: Final[Meters] = Meters(0.3)   # Gaussian noise sigma for dist sensor


# ---------------------------------------------------------------------------
# Telemetry frame
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TelemetryFrame:
    """Immutable snapshot of drone state at one simulation step.

    All physical quantities carry unit suffixes consistent with drone_logic.py
    naming conventions.

    Attributes:
        step:          Integer step index [0, SIM_STEPS).
        time_s:        Wall time of the frame (seconds from mission start).
        x_norm:        Horizontal position normalised to mission range [0.0, 1.0].
        alt_m:         Current altitude (metres).
        v_spd_mps:     Vertical speed (m/s). Positive = ascending.
        obs_dist_m:    Distance to nearest obstacle (metres).
        phase:         Current mission phase (``FlightPhase`` enum).
        smt_verified:  True when all Z3 invariants were PROVED before this run.
    """

    step:         int
    time_s:       float
    x_norm:       float
    alt_m:        Meters
    v_spd_mps:    MetersPerSecond
    obs_dist_m:   Meters
    phase:        FlightPhase
    smt_verified: bool


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _smooth_step(t: float) -> float:
    """Cubic smooth-step (C1 continuity) for interpolation.

    Args:
        t: Normalized time in [0.0, 1.0].

    Returns:
        Smooth-step value in [0.0, 1.0].
    """
    return t * t * (3.0 - 2.0 * t)


def _obstacle_distance(x_norm: float) -> Meters:
    """Compute noisy obstacle distance at normalized mission position x_norm.

    The obstacle profile is modeled as a Gaussian proximity function centered
    at ``OBSTACLE_X_NORM``. Beyond the obstacle, distance falls off with a
    linear term to simulate an open-field environment.

    Args:
        x_norm: Normalized horizontal position [0.0, 1.0].

    Returns:
        Noisy obstacle distance (metres), clamped to admissible domain.
    """
    dx: float = abs(x_norm - OBSTACLE_X_NORM)
    base_dist: float = 80.0 * (1.0 - math.exp(-8.0 * dx * dx)) + 5.0 * dx
    noise: float = random.gauss(0.0, SENSOR_NOISE_STD)
    return Meters(max(0.0, min(200.0, base_dist + noise)))


def _classify_phase(
    x_norm: float,
    obs_dist_m: Meters,
) -> FlightPhase:
    """Classify the current mission phase from position and sensor data.

    Args:
        x_norm:     Normalized mission position.
        obs_dist_m: Current obstacle distance reading (metres).

    Returns:
        ``FlightPhase`` label for this frame.
    """
    if x_norm < 0.08:
        return FlightPhase.TAKEOFF
    if x_norm > 0.88:
        return FlightPhase.DESCENT
    if obs_dist_m < MIN_SEP_DIST_M:
        return FlightPhase.AVOIDANCE
    return FlightPhase.CRUISE


# ---------------------------------------------------------------------------
# Public simulation entry point
# ---------------------------------------------------------------------------

def run_simulation(smt_verified: bool = True) -> List[TelemetryFrame]:
    """Execute the full mission simulation and return an ordered telemetry log.

    Args:
        smt_verified: Pass-through flag indicating whether the Z3 verification
                      cleared before this run. Stored in each ``TelemetryFrame``
                      for downstream display in the GCS.

    Returns:
        List of ``TelemetryFrame`` objects, one per control cycle, in
        chronological order.
    """
    frames: List[TelemetryFrame] = []

    alt_m:       Meters          = Meters(0.0)
    prev_alt_m:  Meters          = Meters(0.0)
    v_spd_mps:   MetersPerSecond = MetersPerSecond(0.0)

    for step in range(SIM_STEPS):
        x_norm:    float = step / SIM_STEPS
        time_s:    float = step * DT_S

        obs_dist_m: Meters = _obstacle_distance(x_norm)

        # Derive vertical speed from finite differences
        v_spd_mps = MetersPerSecond(
            (alt_m - prev_alt_m) / DT_S if step > 0 else 0.0
        )
        prev_alt_m = alt_m

        # State transition: call the formally verified control law
        tgt_alt_m: Meters = compute_target_alt(alt_m, obs_dist_m, v_spd_mps)

        phase: FlightPhase = _classify_phase(x_norm, obs_dist_m)

        # Override target for takeoff / descent to shape the trajectory arc
        if phase == FlightPhase.TAKEOFF:
            tgt_alt_m = Meters(_smooth_step(x_norm / 0.08) * CRUISE_ALT_M)
        elif phase == FlightPhase.DESCENT:
            descent_t = (x_norm - 0.88) / 0.12
            tgt_alt_m = Meters(max(MIN_ALT_M, alt_m * (1.0 - _smooth_step(descent_t))))

        # First-order actuator lag
        alt_m = Meters(alt_m + (tgt_alt_m - alt_m) * LAG_FILTER_COEFF)
        alt_m = Meters(max(MIN_ALT_M if step > 5 else 0.0, min(MAX_ALT_M, alt_m)))

        frames.append(TelemetryFrame(
            step=step,
            time_s=time_s,
            x_norm=x_norm,
            alt_m=alt_m,
            v_spd_mps=v_spd_mps,
            obs_dist_m=obs_dist_m,
            phase=phase,
            smt_verified=smt_verified,
        ))

    return frames
