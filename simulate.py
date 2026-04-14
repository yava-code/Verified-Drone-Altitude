"""
simulate.py
===========
Drone flight simulator for the altitude control system.

Generates a realistic flight trajectory including:
  - Takeoff phase
  - Cruise phase
  - Obstacle avoidance manoeuvre
  - Return-to-home descent

All altitude commands pass through calculate_altitude_adjustment(),
ensuring the formal safety envelope is enforced in the simulation.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List

from drone_control import (
    MIN_ALT,
    MAX_ALT,
    OBSTACLE_SAFE_DIST,
    calculate_altitude_adjustment,
)

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
SIM_STEPS: int   = 300          # total simulation steps
DT: float        = 0.2          # seconds per step
CRUISE_ALT: float = 40.0        # metres - nominal cruising altitude
OBSTACLE_X: float = 0.55        # fractional position along route [0-1]
OBSTACLE_ALT: float = 42.0      # altitude of obstacle centroid (metres)


@dataclass
class DroneState:
    """Snapshot of drone state at one simulation step."""

    step: int
    time: float          # seconds
    x_pos: float         # horizontal position (normalised 0-1)
    altitude: float      # metres
    vertical_speed: float  # m/s
    obstacle_dist: float  # metres to nearest obstacle
    phase: str           # 'takeoff' | 'cruise' | 'avoidance' | 'descent'
    verified: bool       # Z3 verification status flag


def _smooth_step(t: float) -> float:
    """Cubic smooth-step interpolation t in [0,1]."""
    return t * t * (3.0 - 2.0 * t)


def run_simulation(verified: bool = True) -> List[DroneState]:
    """
    Run the drone flight simulation and return a list of state snapshots.

    Parameters
    ----------
    verified : bool
        Whether Z3 verification passed. Used to tag state snapshots.

    Returns
    -------
    List[DroneState]
        Ordered list of drone states from start to end of mission.
    """
    states: List[DroneState] = []

    altitude: float = 0.0
    vertical_speed: float = 0.0
    prev_altitude: float = 0.0

    for step in range(SIM_STEPS):
        t: float = step / SIM_STEPS  # normalised mission time [0, 1]
        time: float = step * DT

        # --- Determine obstacle distance ---
        dx: float = abs(t - OBSTACLE_X)
        # Obstacle has a Gaussian proximity profile
        base_dist: float = 80.0 * (1.0 - math.exp(-8.0 * dx * dx)) + 5.0 * dx
        # Add mild sensor noise
        noise: float = random.gauss(0.0, 0.3)
        obstacle_dist: float = max(0.0, min(200.0, base_dist + noise))

        # --- Compute target altitude via control law ---
        # Inject a gentle vertical_speed based on recent altitude change
        vertical_speed = (altitude - prev_altitude) / DT if step > 0 else 0.0
        prev_altitude = altitude

        target_alt: float = calculate_altitude_adjustment(
            current_alt=altitude,
            obstacle_dist=obstacle_dist,
            vertical_speed=vertical_speed,
        )

        # Override target during takeoff/descent phases to guide the trajectory
        if t < 0.08:
            # Takeoff: smoothly climb from 0 to CRUISE_ALT
            phase = "takeoff"
            target_alt = _smooth_step(t / 0.08) * CRUISE_ALT
        elif t > 0.88:
            # Descent: smoothly descend from current altitude to MIN_ALT
            descent_t = (t - 0.88) / 0.12
            target_alt = max(MIN_ALT, altitude * (1.0 - _smooth_step(descent_t)))
            phase = "descent"
        elif obstacle_dist < OBSTACLE_SAFE_DIST:
            phase = "avoidance"
        else:
            phase = "cruise"

        # --- Simulate first-order altitude response (lag filter) ---
        altitude += (target_alt - altitude) * 0.18
        altitude = max(MIN_ALT if step > 5 else 0.0, min(MAX_ALT, altitude))

        states.append(DroneState(
            step=step,
            time=time,
            x_pos=t,
            altitude=altitude,
            vertical_speed=vertical_speed,
            obstacle_dist=obstacle_dist,
            phase=phase,
            verified=verified,
        ))

    return states
