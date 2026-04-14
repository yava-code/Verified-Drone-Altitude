"""
drone_logic.py
==============
Altitude arbitration module for an autonomous rotorcraft vehicle.

This module is the plant model for the formal verification pipeline.
The function ``compute_target_alt`` encodes one control cycle of the
altitude hold / obstacle-avoidance law. Its output is consumed by the
flight controller to set the next waypoint altitude.

Safety invariant (module-level):
    For all (alt_m, obs_dist_m, v_spd_mps) in the admissible domain::

        MIN_ALT_M <= compute_target_alt(alt_m, obs_dist_m, v_spd_mps) <= MAX_ALT_M

    This invariant is formally discharged by ``verify_z3.py`` using the
    Z3 SMT solver (Microsoft Research). No simulation or execution may
    proceed until the verifier returns PROVED for all three safety
    assertions (INV-001, INV-002, INV-003).

Naming conventions:
    *_m    : value in metres
    *_mps  : value in metres per second
    *_s    : value in seconds
    *_pct  : dimensionless ratio [0.0, 1.0]

References:
    - FAA 14 CFR Part 107.51 - operating limitations (400 ft AGL / ~120 m)
    - RTCA DO-178C - software considerations in airborne systems
    - NASA/TM-2019-220054 - formal methods in aerospace
"""

from __future__ import annotations

from typing import Final, NewType

# ---------------------------------------------------------------------------
# Domain-specific types
# ---------------------------------------------------------------------------

Meters: type = NewType("Meters", float)
MetersPerSecond: type = NewType("MetersPerSecond", float)
DimensionlessRatio: type = NewType("DimensionlessRatio", float)


# ---------------------------------------------------------------------------
# Safety envelope constants - single source of truth, imported by verify_z3.py
# ---------------------------------------------------------------------------

MIN_ALT_M: Final[Meters] = Meters(2.0)
"""Minimum admissible altitude (m) - ground clearance per airspace regulation."""

MAX_ALT_M: Final[Meters] = Meters(120.0)
"""Maximum admissible altitude (m) - regulatory ceiling, FAA Part 107 / EU STS-02."""

MAX_ASCENT_STEP_M: Final[Meters] = Meters(5.0)
"""Maximum altitude gain (m) per one control cycle - rate limiter."""

MAX_DESCENT_STEP_M: Final[Meters] = Meters(3.0)
"""Maximum altitude loss (m) per one control cycle - rate limiter."""

MIN_SEP_DIST_M: Final[Meters] = Meters(15.0)
"""Minimum separation distance (m) to obstacle before avoidance climb is triggered."""


# ---------------------------------------------------------------------------
# Admissible input domain - must match symbolic constraints in verify_z3.py
# ---------------------------------------------------------------------------

DOMAIN_ALT_LO_M: Final[Meters] = Meters(0.0)
DOMAIN_ALT_HI_M: Final[Meters] = Meters(200.0)
DOMAIN_DIST_LO_M: Final[Meters] = Meters(0.0)
DOMAIN_DIST_HI_M: Final[Meters] = Meters(200.0)
DOMAIN_VSPD_LO_MPS: Final[MetersPerSecond] = MetersPerSecond(-10.0)
DOMAIN_VSPD_HI_MPS: Final[MetersPerSecond] = MetersPerSecond(10.0)


# ---------------------------------------------------------------------------
# Control law
# ---------------------------------------------------------------------------

def compute_target_alt(
    alt_m: Meters,
    obs_dist_m: Meters,
    v_spd_mps: MetersPerSecond,
) -> Meters:
    """Compute the target altitude for the next control cycle.

    Implements a two-phase, safety-first arbitration law:

    Phase 1 - Intent arbitration:
        If ``obs_dist_m < MIN_SEP_DIST_M``, an obstacle avoidance climb is
        commanded. The climb magnitude scales linearly with proximity ratio
        so that a closer obstacle produces a larger step (proximity-weighted
        ascent). Otherwise, a nominal descent of 0.5 m plus a damping term
        proportional to positive vertical speed is applied to bleed off
        accumulated altitude error.

    Phase 2 - Safety clamp (invariant enforcement point):
        The raw arbitrated target is hard-clamped to [MIN_ALT_M, MAX_ALT_M]
        before return. This single statement is the mechanically checked
        enforcement point for invariants INV-001 and INV-002. Z3 proves
        that the clamp is both necessary and sufficient.

    Args:
        alt_m: Current measured altitude (m).
            Precondition: DOMAIN_ALT_LO_M <= alt_m <= DOMAIN_ALT_HI_M
        obs_dist_m: Distance to nearest detected obstacle (m).
            Precondition: DOMAIN_DIST_LO_M <= obs_dist_m <= DOMAIN_DIST_HI_M
        v_spd_mps: Current vertical speed (m/s). Positive = ascending.
            Precondition: DOMAIN_VSPD_LO_MPS <= v_spd_mps <= DOMAIN_VSPD_HI_MPS

    Returns:
        Target altitude (m) for the next waypoint.
        Postcondition: MIN_ALT_M <= return_value <= MAX_ALT_M (PROVED by Z3).

    Invariants:
        INV-001: return_value >= MIN_ALT_M  [Z3 assertion ID: P1]
        INV-002: return_value <= MAX_ALT_M  [Z3 assertion ID: P2]
        INV-003: climb commands never exceed MAX_ALT_M  [Z3 assertion ID: P3]
    """
    # -- Phase 1: intent arbitration ------------------------------------------

    if obs_dist_m < MIN_SEP_DIST_M:
        prox_ratio: DimensionlessRatio = DimensionlessRatio(
            1.0 - (obs_dist_m / MIN_SEP_DIST_M)
        )
        raw_tgt_m: Meters = Meters(alt_m + MAX_ASCENT_STEP_M * prox_ratio)
    else:
        vs_damping: MetersPerSecond = MetersPerSecond(max(0.0, v_spd_mps))
        descent_m: Meters = Meters(0.5 + 0.1 * vs_damping)
        raw_tgt_m = Meters(alt_m - descent_m)

    # -- Phase 2: safety clamp (INV-001, INV-002 enforcement point) -----------
    tgt_alt_m: Meters = Meters(max(MIN_ALT_M, min(MAX_ALT_M, raw_tgt_m)))

    return tgt_alt_m
