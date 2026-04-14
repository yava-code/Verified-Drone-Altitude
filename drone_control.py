"""
drone_control.py
================
Core altitude control logic for an autonomous drone.

This module implements the calculate_altitude_adjustment() function,
which serves as the plant model for the formal verification layer.
All safety constraints are enforced here; the Z3 verifier in
verify_safety.py provides mathematical proof that they hold for
every possible input in the defined domain.

Safety envelope:
    MIN_ALT = 2.0  m  (ground clearance)
    MAX_ALT = 120.0 m  (regulatory ceiling, FAA Part 107 / EU drone reg)
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Safety constants  (single source of truth - importable by verifier)
# ---------------------------------------------------------------------------
MIN_ALT: float = 2.0    # metres - minimum safe altitude (ground clearance)
MAX_ALT: float = 120.0  # metres - maximum safe altitude (regulatory ceiling)

# Control parameters
MAX_STEP_UP: float  = 5.0   # metres - max altitude gain per control cycle
MAX_STEP_DOWN: float = 3.0  # metres - max altitude loss per control cycle
OBSTACLE_SAFE_DIST: float = 15.0  # metres - trigger distance for climb manoeuvre


def calculate_altitude_adjustment(
    current_alt: float,
    obstacle_dist: float,
    vertical_speed: float,
) -> float:
    """
    Compute a new target altitude from sensor inputs.

    The function implements a conservative, safety-first control law:
      1. Obstacle avoidance: if an obstacle is within OBSTACLE_SAFE_DIST,
         command a climb.
      2. Descend when no obstacle is present and altitude is above the
         cruising band.
      3. All outputs are hard-clamped to [MIN_ALT, MAX_ALT] BEFORE return -
         this is the invariant that Z3 formally proves.

    Parameters
    ----------
    current_alt : float
        Current altitude of the drone in metres. Valid domain: [0, 200].
    obstacle_dist : float
        Distance to the nearest detected obstacle in metres. Valid domain: [0, 200].
    vertical_speed : float
        Current vertical velocity in m/s. Positive = ascending. Valid domain: [-10, 10].

    Returns
    -------
    float
        New target altitude in metres, guaranteed to lie within [MIN_ALT, MAX_ALT].
    """
    # --- Phase 1: determine raw desired altitude ---
    if obstacle_dist < OBSTACLE_SAFE_DIST:
        # Obstacle avoidance: climb away from the obstacle.
        # The step size scales with proximity (closer = larger step).
        proximity_factor: float = 1.0 - (obstacle_dist / OBSTACLE_SAFE_DIST)
        raw_target: float = current_alt + MAX_STEP_UP * proximity_factor
    else:
        # No immediate obstacle: gentle descent / hold to conserve battery.
        # Blend the vertical speed to smooth the trajectory.
        descent: float = 0.5 + 0.1 * max(0.0, vertical_speed)
        raw_target = current_alt - descent

    # --- Phase 2: hard safety clamp (formally verified by Z3) ---
    safe_target: float = max(MIN_ALT, min(MAX_ALT, raw_target))

    return safe_target
