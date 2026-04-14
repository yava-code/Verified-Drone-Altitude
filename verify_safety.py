"""
verify_safety.py
================
Formal verification of drone altitude safety properties using the Z3 SMT solver.

Background
----------
Z3 (from Microsoft Research) is a Satisfiability Modulo Theories (SMT) solver.
Unlike unit tests that check discrete sample inputs, Z3 operates over the
*entire continuous input domain*. We encode the control law as symbolic
expressions and ask Z3 to find a counter-example - an assignment of inputs that
would violate a safety property. If Z3 returns 'unsat' (unsatisfiable), it has
proven that NO such assignment exists: the property holds for ALL inputs.

Safety properties verified
--------------------------
P1. Output is always >= MIN_ALT  (never crash into ground)
P2. Output is always <= MAX_ALT  (never breach regulatory ceiling)
P3. A climb command is only issued when the output stays <= MAX_ALT
    (no phantom over-altitude commands slip through the clamp)

Usage
-----
    python verify_safety.py

Returns exit code 0 on full verification, 1 if any property fails.
"""

from __future__ import annotations

import sys
import textwrap
from dataclasses import dataclass, field
from typing import List

import z3

from drone_control import (
    MIN_ALT,
    MAX_ALT,
    MAX_STEP_UP,
    MAX_STEP_DOWN,
    OBSTACLE_SAFE_DIST,
)

# ---------------------------------------------------------------------------
# Input domain (must match docstring in drone_control.py)
# ---------------------------------------------------------------------------
DOMAIN_ALT_LO:    float = 0.0
DOMAIN_ALT_HI:    float = 200.0
DOMAIN_DIST_LO:   float = 0.0
DOMAIN_DIST_HI:   float = 200.0
DOMAIN_VSPEED_LO: float = -10.0
DOMAIN_VSPEED_HI: float = 10.0


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class VerificationResult:
    """Stores the outcome of a single safety property check."""

    property_id: str
    description: str
    status: str                      # "PROVED" | "FAILED" | "UNKNOWN"
    counter_example: dict | None = None
    fix_applied: bool = False


@dataclass
class VerificationReport:
    """Aggregated report for all safety properties."""

    results: List[VerificationResult] = field(default_factory=list)

    @property
    def all_proved(self) -> bool:
        """True only when every property is formally proved."""
        return all(r.status == "PROVED" for r in self.results)


# ---------------------------------------------------------------------------
# Symbolic model of calculate_altitude_adjustment()
# ---------------------------------------------------------------------------

def build_symbolic_model(
    current_alt: z3.ArithRef,
    obstacle_dist: z3.ArithRef,
    vertical_speed: z3.ArithRef,
) -> z3.ArithRef:
    """
    Encode calculate_altitude_adjustment() as a Z3 symbolic expression.

    Z3 cannot call Python functions directly, so we re-express the
    control law using z3.If() (symbolic if-then-else) and z3.RealVal()
    for numeric constants.

    Parameters
    ----------
    current_alt    : Z3 Real variable representing current altitude.
    obstacle_dist  : Z3 Real variable representing obstacle distance.
    vertical_speed : Z3 Real variable representing vertical speed.

    Returns
    -------
    z3.ArithRef
        Symbolic expression for the function output (target altitude).
    """
    safe_dist  = z3.RealVal(OBSTACLE_SAFE_DIST)
    max_step   = z3.RealVal(MAX_STEP_UP)
    min_alt    = z3.RealVal(MIN_ALT)
    max_alt    = z3.RealVal(MAX_ALT)

    # Phase 1: raw target (mirrors Python if/else with symbolic z3.If)
    proximity_factor = z3.RealVal(1.0) - (obstacle_dist / safe_dist)
    raw_obstacle     = current_alt + max_step * proximity_factor

    # Descent branch: vertical_speed clamped at 0 from below (max(0, vs))
    vs_clamped = z3.If(vertical_speed > z3.RealVal(0.0), vertical_speed, z3.RealVal(0.0))
    descent    = z3.RealVal(0.5) + z3.RealVal(0.1) * vs_clamped
    raw_normal = current_alt - descent

    raw_target = z3.If(obstacle_dist < safe_dist, raw_obstacle, raw_normal)

    # Phase 2: clamp to safety envelope
    clamped_lo = z3.If(raw_target < min_alt, min_alt, raw_target)
    safe_target = z3.If(clamped_lo > max_alt, max_alt, clamped_lo)

    return safe_target


# ---------------------------------------------------------------------------
# Verification helpers
# ---------------------------------------------------------------------------

def _make_solver_with_domain() -> tuple[z3.Solver, z3.ArithRef, z3.ArithRef, z3.ArithRef]:
    """
    Create a fresh Z3 solver pre-loaded with input domain constraints.

    Returns
    -------
    (solver, current_alt, obstacle_dist, vertical_speed)
    """
    solver = z3.Solver()

    current_alt    = z3.Real("current_alt")
    obstacle_dist  = z3.Real("obstacle_dist")
    vertical_speed = z3.Real("vertical_speed")

    # Constrain inputs to their valid physical domains
    solver.add(current_alt    >= z3.RealVal(DOMAIN_ALT_LO))
    solver.add(current_alt    <= z3.RealVal(DOMAIN_ALT_HI))
    solver.add(obstacle_dist  >= z3.RealVal(DOMAIN_DIST_LO))
    solver.add(obstacle_dist  <= z3.RealVal(DOMAIN_DIST_HI))
    solver.add(vertical_speed >= z3.RealVal(DOMAIN_VSPEED_LO))
    solver.add(vertical_speed <= z3.RealVal(DOMAIN_VSPEED_HI))

    return solver, current_alt, obstacle_dist, vertical_speed


def _extract_counter_example(
    model: z3.ModelRef,
    current_alt: z3.ArithRef,
    obstacle_dist: z3.ArithRef,
    vertical_speed: z3.ArithRef,
) -> dict:
    """Convert a Z3 model (counter-example) into a Python dict."""
    def to_float(val: z3.ExprRef) -> float:
        # Z3 may return rational fractions; convert to float
        return float(val.as_fraction()) if hasattr(val, "as_fraction") else float(str(val))

    return {
        "current_alt":    to_float(model[current_alt]),
        "obstacle_dist":  to_float(model[obstacle_dist]),
        "vertical_speed": to_float(model[vertical_speed]),
    }


# ---------------------------------------------------------------------------
# Individual property checks
# ---------------------------------------------------------------------------

def verify_floor_constraint() -> VerificationResult:
    """
    P1: Output >= MIN_ALT for all valid inputs.

    We ask Z3: 'Is there any input where safe_target < MIN_ALT?'
    'unsat' -> proved safe.  'sat' -> counter-example found.
    """
    solver, alt, dist, vs = _make_solver_with_domain()
    output = build_symbolic_model(alt, dist, vs)

    # Negation of the property: search for a violation
    solver.add(output < z3.RealVal(MIN_ALT))

    result = solver.check()

    if result == z3.unsat:
        return VerificationResult(
            property_id="P1",
            description=f"Output is always >= MIN_ALT ({MIN_ALT} m)",
            status="PROVED",
        )
    elif result == z3.sat:
        cex = _extract_counter_example(solver.model(), alt, dist, vs)
        return VerificationResult(
            property_id="P1",
            description=f"Output is always >= MIN_ALT ({MIN_ALT} m)",
            status="FAILED",
            counter_example=cex,
        )
    else:
        return VerificationResult(
            property_id="P1",
            description=f"Output is always >= MIN_ALT ({MIN_ALT} m)",
            status="UNKNOWN",
        )


def verify_ceiling_constraint() -> VerificationResult:
    """
    P2: Output <= MAX_ALT for all valid inputs.

    Symmetrically to P1, we ask Z3 to find an input where
    safe_target > MAX_ALT. 'unsat' -> proved.
    """
    solver, alt, dist, vs = _make_solver_with_domain()
    output = build_symbolic_model(alt, dist, vs)

    solver.add(output > z3.RealVal(MAX_ALT))

    result = solver.check()

    if result == z3.unsat:
        return VerificationResult(
            property_id="P2",
            description=f"Output is always <= MAX_ALT ({MAX_ALT} m)",
            status="PROVED",
        )
    elif result == z3.sat:
        cex = _extract_counter_example(solver.model(), alt, dist, vs)
        return VerificationResult(
            property_id="P2",
            description=f"Output is always <= MAX_ALT ({MAX_ALT} m)",
            status="FAILED",
            counter_example=cex,
        )
    else:
        return VerificationResult(
            property_id="P2",
            description=f"Output is always <= MAX_ALT ({MAX_ALT} m)",
            status="UNKNOWN",
        )


def verify_climb_bounded() -> VerificationResult:
    """
    P3: When a climb is commanded (output > current_alt), the result still
    stays within [MIN_ALT, MAX_ALT].

    This proves the safety clamp is not bypassed even during emergency climbs.
    """
    solver, alt, dist, vs = _make_solver_with_domain()
    output = build_symbolic_model(alt, dist, vs)

    min_alt = z3.RealVal(MIN_ALT)
    max_alt = z3.RealVal(MAX_ALT)

    # Constraint: drone is climbing AND output violates envelope
    solver.add(output > alt)  # climb scenario
    solver.add(z3.Or(output < min_alt, output > max_alt))

    result = solver.check()

    if result == z3.unsat:
        return VerificationResult(
            property_id="P3",
            description="Climb commands never breach the safety envelope",
            status="PROVED",
        )
    elif result == z3.sat:
        cex = _extract_counter_example(solver.model(), alt, dist, vs)
        return VerificationResult(
            property_id="P3",
            description="Climb commands never breach the safety envelope",
            status="FAILED",
            counter_example=cex,
        )
    else:
        return VerificationResult(
            property_id="P3",
            description="Climb commands never breach the safety envelope",
            status="UNKNOWN",
        )


# ---------------------------------------------------------------------------
# Report rendering
# ---------------------------------------------------------------------------

def _banner(text: str, width: int = 60) -> str:
    bar = "=" * width
    return f"\n{bar}\n  {text}\n{bar}"


def print_report(report: VerificationReport) -> None:
    """Pretty-print the verification report to stdout."""
    print(_banner("Z3 FORMAL VERIFICATION REPORT"))
    print(f"  Input domain:")
    print(f"    altitude      : [{DOMAIN_ALT_LO}, {DOMAIN_ALT_HI}] m")
    print(f"    obstacle_dist : [{DOMAIN_DIST_LO}, {DOMAIN_DIST_HI}] m")
    print(f"    vertical_speed: [{DOMAIN_VSPEED_LO}, {DOMAIN_VSPEED_HI}] m/s")
    print()

    for r in report.results:
        icon = "[OK]" if r.status == "PROVED" else "[!!]" if r.status == "FAILED" else "[??]"
        print(f"  {icon}  {r.property_id}: {r.description}")
        print(f"       Status : {r.status}")

        if r.counter_example:
            ce = r.counter_example
            print(f"       Counter-example found:")
            print(f"         current_alt    = {ce['current_alt']:.4f} m")
            print(f"         obstacle_dist  = {ce['obstacle_dist']:.4f} m")
            print(f"         vertical_speed = {ce['vertical_speed']:.4f} m/s")

        if r.fix_applied:
            print(f"       Fix applied: safety clamp added to control law.")
        print()

    overall = "ALL PROPERTIES PROVED - System verified SAFE" if report.all_proved \
              else "VERIFICATION FAILED - Review counter-examples above"
    print(_banner(f"VERDICT: {overall}"))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def run_verification() -> VerificationReport:
    """
    Execute all safety property checks and return a consolidated report.

    This function is called both from the CLI and from the visualizer to
    obtain a 'verified' flag before starting the animation.
    """
    report = VerificationReport()

    checks = [
        verify_floor_constraint,
        verify_ceiling_constraint,
        verify_climb_bounded,
    ]

    for check_fn in checks:
        result = check_fn()
        report.results.append(result)

    return report


if __name__ == "__main__":
    report = run_verification()
    print_report(report)
    sys.exit(0 if report.all_proved else 1)
