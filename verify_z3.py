"""
verify_z3.py
============
Formal verification of altitude safety invariants via the Z3 SMT solver.

Methodology
-----------
Safety properties are encoded as SMT assertions over the *symbolic* image
of ``compute_target_alt`` (from ``drone_logic.py``). Z3 operates over the
complete, infinite admissible input domain rather than a finite sample set.

For each invariant, the verifier asserts the *negation* of the property and
invokes ``Solver.check()``:

  - ``z3.unsat``  - The negation is unsatisfiable; the property holds for
                    ALL inputs in the domain. Result: PROVED.
  - ``z3.sat``    - Z3 found a concrete input assignment (counterexample)
                    that falsifies the property. Result: FALSIFIED.
  - ``z3.unknown``- The solver could not decide (e.g., resource exhaustion
                    or undecidable arithmetic fragment). Result: UNKNOWN.

No exception handling is used. All outcomes are represented as typed
``AssertionResult`` objects (see ``result_types.py``).

Safety assertions verified
--------------------------
  INV-001 (P1): compute_target_alt(...) >= MIN_ALT_M  for all admissible inputs
  INV-002 (P2): compute_target_alt(...) <= MAX_ALT_M  for all admissible inputs
  INV-003 (P3): Avoidance climb output stays within [MIN_ALT_M, MAX_ALT_M]

Counterexample demonstration
-----------------------------
  ``check_unsafe_variant()`` deliberately verifies a version of the control
  law with the safety clamp removed. Z3 immediately returns a counterexample
  demonstrating the ceiling violation. This is the pedagogical artifact
  animated by ``manim_counterexample.py``.

Usage
-----
    python verify_z3.py

Exit codes:
    0  - all invariants PROVED
    1  - one or more invariants FALSIFIED or UNKNOWN
"""

from __future__ import annotations

import sys
import time
from typing import Final, List, Tuple

import z3

from drone_logic import (
    MIN_ALT_M,
    MAX_ALT_M,
    MAX_ASCENT_STEP_M,
    MIN_SEP_DIST_M,
    DOMAIN_ALT_LO_M,
    DOMAIN_ALT_HI_M,
    DOMAIN_DIST_LO_M,
    DOMAIN_DIST_HI_M,
    DOMAIN_VSPD_LO_MPS,
    DOMAIN_VSPD_HI_MPS,
)
from result_types import (
    AssertionResult,
    Counterexample,
    VerificationReport,
    VerificationStatus,
)

# ---------------------------------------------------------------------------
# Z3 symbolic variable identifiers
# ---------------------------------------------------------------------------

_SYM_ALT:   Final[str] = "alt_m"
_SYM_DIST:  Final[str] = "obs_dist_m"
_SYM_VSPD:  Final[str] = "v_spd_mps"


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _make_bounded_solver() -> Tuple[z3.Solver, z3.ArithRef, z3.ArithRef, z3.ArithRef]:
    """Instantiate a Z3 Solver pre-loaded with the admissible input domain.

    Returns:
        (solver, alt_sym, obs_dist_sym, v_spd_sym) - the solver and its
        three free symbolic variables, all constrained to their physical
        domains. Callers add assertion-specific constraints on top.
    """
    slv = z3.Solver()

    alt_sym:      z3.ArithRef = z3.Real(_SYM_ALT)
    obs_dist_sym: z3.ArithRef = z3.Real(_SYM_DIST)
    v_spd_sym:    z3.ArithRef = z3.Real(_SYM_VSPD)

    slv.add(alt_sym      >= z3.RealVal(DOMAIN_ALT_LO_M))
    slv.add(alt_sym      <= z3.RealVal(DOMAIN_ALT_HI_M))
    slv.add(obs_dist_sym >= z3.RealVal(DOMAIN_DIST_LO_M))
    slv.add(obs_dist_sym <= z3.RealVal(DOMAIN_DIST_HI_M))
    slv.add(v_spd_sym    >= z3.RealVal(DOMAIN_VSPD_LO_MPS))
    slv.add(v_spd_sym    <= z3.RealVal(DOMAIN_VSPD_HI_MPS))

    return slv, alt_sym, obs_dist_sym, v_spd_sym


def _symbolic_compute_target_alt(
    alt_sym:      z3.ArithRef,
    obs_dist_sym: z3.ArithRef,
    v_spd_sym:    z3.ArithRef,
    include_clamp: bool = True,
) -> z3.ArithRef:
    """Encode ``compute_target_alt`` as a Z3 symbolic expression tree.

    Z3 cannot introspect Python bytecode; the control law must be
    re-expressed using Z3 IR. ``z3.If`` encodes piecewise-linear branches;
    ``z3.RealVal`` encodes numeric constants as exact rationals (no
    floating-point rounding in the symbolic model).

    Args:
        alt_sym:       Symbolic variable for current altitude (m).
        obs_dist_sym:  Symbolic variable for obstacle distance (m).
        v_spd_sym:     Symbolic variable for vertical speed (m/s).
        include_clamp: If False, the Phase-2 safety clamp is omitted.
                       This produces the unsafe variant used in the
                       counterexample demonstration.

    Returns:
        Z3 expression representing the function output. The expression
        is a closed-form real arithmetic formula with no uninterpreted
        functions, making it decidable by Z3's linear arithmetic tactic.
    """
    c_sep  = z3.RealVal(MIN_SEP_DIST_M)
    c_step = z3.RealVal(MAX_ASCENT_STEP_M)
    c_min  = z3.RealVal(MIN_ALT_M)
    c_max  = z3.RealVal(MAX_ALT_M)

    # -- Phase 1: intent arbitration (symbolic) --------------------------------
    prox_ratio    = z3.RealVal(1.0) - (obs_dist_sym / c_sep)
    raw_climb_m   = alt_sym + c_step * prox_ratio

    vs_clamped    = z3.If(v_spd_sym > z3.RealVal(0.0), v_spd_sym, z3.RealVal(0.0))
    descent_m     = z3.RealVal(0.5) + z3.RealVal(0.1) * vs_clamped
    raw_descend_m = alt_sym - descent_m

    raw_tgt_m     = z3.If(obs_dist_sym < c_sep, raw_climb_m, raw_descend_m)

    if not include_clamp:
        # Unsafe variant: no safety clamp applied.
        return raw_tgt_m

    # -- Phase 2: safety clamp ------------------------------------------------
    clamped_lo  = z3.If(raw_tgt_m < c_min, c_min, raw_tgt_m)
    tgt_alt_sym = z3.If(clamped_lo > c_max, c_max, clamped_lo)

    return tgt_alt_sym


def _extract_counterexample(
    model:        z3.ModelRef,
    alt_sym:      z3.ArithRef,
    obs_dist_sym: z3.ArithRef,
    v_spd_sym:    z3.ArithRef,
    output_sym:   z3.ArithRef,
) -> Counterexample:
    """Extract a concrete ``Counterexample`` from a Z3 satisfying model.

    Z3 models return values as exact rational numbers (``z3.RatNumRef``).
    This function converts them to Python floats for display and downstream
    use by the Manim animation script.

    Args:
        model:        The satisfying model returned by ``Solver.model()``.
        alt_sym:      Symbolic altitude variable.
        obs_dist_sym: Symbolic obstacle distance variable.
        v_spd_sym:    Symbolic vertical speed variable.
        output_sym:   Symbolic output expression (pre-evaluated by solver).

    Returns:
        ``Counterexample`` with concrete field values.
    """
    def _rat_to_float(expr: z3.ExprRef) -> float:
        val = model.eval(expr, model_completion=True)
        if hasattr(val, "as_fraction"):
            frac = val.as_fraction()
            # z3 >= 4.13 returns a Python Fraction directly
            if hasattr(frac, "numerator"):
                return float(frac)
            # older z3 returned a (numerator, denominator) tuple
            num, den = frac
            return float(num) / float(den)
        return float(str(val))

    return Counterexample(
        alt_m      = _rat_to_float(alt_sym),
        obs_dist_m = _rat_to_float(obs_dist_sym),
        v_spd_mps  = _rat_to_float(v_spd_sym),
        output_m   = _rat_to_float(output_sym),
    )


def _run_smt_query(
    assertion_id: str,
    description:  str,
    violation_constraint_fn,
    include_clamp: bool = True,
) -> AssertionResult:
    """Generic SMT query executor used by all three invariant checks.

    Args:
        assertion_id:            Short ID string (e.g. "INV-001").
        description:             Human-readable invariant description.
        violation_constraint_fn: Callable ``(solver, output_sym) -> None``.
                                 Adds the negation of the safety property
                                 to the solver. Z3 searches for a model
                                 satisfying this constraint (a counterexample).
        include_clamp:           Passed through to ``_symbolic_compute_target_alt``.

    Returns:
        ``AssertionResult`` with status set to PROVED, FALSIFIED, or UNKNOWN.
    """
    slv, alt_sym, obs_dist_sym, v_spd_sym = _make_bounded_solver()
    output_sym = _symbolic_compute_target_alt(
        alt_sym, obs_dist_sym, v_spd_sym, include_clamp=include_clamp
    )

    violation_constraint_fn(slv, output_sym)

    t0_ms     = time.perf_counter() * 1000.0
    smt_result = slv.check()
    elapsed_ms = time.perf_counter() * 1000.0 - t0_ms

    if smt_result == z3.unsat:
        return AssertionResult(
            assertion_id=assertion_id,
            description=description,
            status=VerificationStatus.PROVED,
            solver_ms=elapsed_ms,
        )

    if smt_result == z3.sat:
        cex = _extract_counterexample(
            slv.model(), alt_sym, obs_dist_sym, v_spd_sym, output_sym
        )
        return AssertionResult(
            assertion_id=assertion_id,
            description=description,
            status=VerificationStatus.FALSIFIED,
            counterexample=cex,
            solver_ms=elapsed_ms,
        )

    # z3.unknown
    return AssertionResult(
        assertion_id=assertion_id,
        description=description,
        status=VerificationStatus.UNKNOWN,
        solver_ms=elapsed_ms,
    )


# ---------------------------------------------------------------------------
# Public assertion checks
# ---------------------------------------------------------------------------

def check_inv001_floor() -> AssertionResult:
    """Verify INV-001: output >= MIN_ALT_M for all admissible inputs.

    SMT encoding: assert (output < MIN_ALT_M). An ``unsat`` result from Z3
    proves that no input can drive the altitude command below the ground
    clearance floor.

    Returns:
        ``AssertionResult`` with status PROVED, FALSIFIED, or UNKNOWN.
    """
    def _constraint(slv: z3.Solver, out: z3.ArithRef) -> None:
        slv.add(out < z3.RealVal(MIN_ALT_M))

    return _run_smt_query(
        assertion_id="INV-001",
        description=(
            f"compute_target_alt() >= MIN_ALT_M ({MIN_ALT_M} m) "
            f"for all inputs in the admissible domain"
        ),
        violation_constraint_fn=_constraint,
    )


def check_inv002_ceiling() -> AssertionResult:
    """Verify INV-002: output <= MAX_ALT_M for all admissible inputs.

    SMT encoding: assert (output > MAX_ALT_M). An ``unsat`` result proves
    the control law never issues an altitude command above the regulatory
    ceiling, even under maximum proximity obstacle avoidance.

    Returns:
        ``AssertionResult`` with status PROVED, FALSIFIED, or UNKNOWN.
    """
    def _constraint(slv: z3.Solver, out: z3.ArithRef) -> None:
        slv.add(out > z3.RealVal(MAX_ALT_M))

    return _run_smt_query(
        assertion_id="INV-002",
        description=(
            f"compute_target_alt() <= MAX_ALT_M ({MAX_ALT_M} m) "
            f"for all inputs in the admissible domain"
        ),
        violation_constraint_fn=_constraint,
    )


def check_inv003_climb_bounded() -> AssertionResult:
    """Verify INV-003: avoidance climb commands stay within the safety envelope.

    SMT encoding: assert (output > current_alt) AND (output violates envelope).
    This specifically targets the obstacle avoidance branch and proves that
    even at maximum proximity, the climb command cannot bypass the clamp.

    Returns:
        ``AssertionResult`` with status PROVED, FALSIFIED, or UNKNOWN.
    """
    def _constraint(slv: z3.Solver, out: z3.ArithRef) -> None:
        alt_sym = z3.Real(_SYM_ALT)
        slv.add(out > alt_sym)
        slv.add(
            z3.Or(out < z3.RealVal(MIN_ALT_M), out > z3.RealVal(MAX_ALT_M))
        )

    return _run_smt_query(
        assertion_id="INV-003",
        description=(
            "Avoidance climb output is bounded within "
            f"[{MIN_ALT_M}, {MAX_ALT_M}] m for all admissible inputs"
        ),
        violation_constraint_fn=_constraint,
    )


# ---------------------------------------------------------------------------
# Counterexample demonstration (unsafe variant)
# ---------------------------------------------------------------------------

def check_unsafe_variant() -> AssertionResult:
    """Attempt to verify the ceiling invariant on the CLAMP-REMOVED variant.

    This function is the counterexample demonstration. It runs INV-002 against
    a symbolic model of ``compute_target_alt`` with the Phase-2 safety clamp
    deliberately omitted. Z3 is expected to return FALSIFIED with a concrete
    input assignment proving the ceiling breach.

    The returned ``AssertionResult.counterexample`` is the payload consumed
    by ``manim_counterexample.py`` to animate the failure scenario.

    Returns:
        ``AssertionResult`` - expected status: FALSIFIED.
    """
    def _constraint(slv: z3.Solver, out: z3.ArithRef) -> None:
        slv.add(out > z3.RealVal(MAX_ALT_M))

    return _run_smt_query(
        assertion_id="INV-002-UNSAFE",
        description=(
            f"[UNSAFE VARIANT] Ceiling invariant on clamp-removed model "
            f"(expected: FALSIFIED)"
        ),
        violation_constraint_fn=_constraint,
        include_clamp=False,
    )


# ---------------------------------------------------------------------------
# Report rendering
# ---------------------------------------------------------------------------

_COL_W: Final[int] = 64


def _rule(char: str = "=") -> str:
    return char * _COL_W


def _status_tag(status: VerificationStatus) -> str:
    tags = {
        VerificationStatus.PROVED:    "PROVED   ",
        VerificationStatus.FALSIFIED: "FALSIFIED",
        VerificationStatus.TIMEOUT:   "TIMEOUT  ",
        VerificationStatus.UNKNOWN:   "UNKNOWN  ",
    }
    return tags.get(status, "?????????")


def print_verification_report(report: VerificationReport) -> None:
    """Render the verification report to stdout in machine-readable plain text.

    Format is structured for easy grep/awk extraction in CI pipelines.
    No ANSI color codes are used; color is the responsibility of the
    terminal or log viewer.

    Args:
        report: Completed ``VerificationReport`` from ``run_verification()``.
    """
    print()
    print(_rule("="))
    print("  Z3 SMT SOLVER - FORMAL VERIFICATION REPORT")
    print(f"  Module under verification : {report.module_name}")
    print(f"  SMT solver engine         : Z3 {z3.get_version_string()}")
    print(_rule("-"))
    print("  Admissible input domain:")
    print(f"    alt_m      in [{DOMAIN_ALT_LO_M:.1f}, {DOMAIN_ALT_HI_M:.1f}] m")
    print(f"    obs_dist_m in [{DOMAIN_DIST_LO_M:.1f}, {DOMAIN_DIST_HI_M:.1f}] m")
    print(f"    v_spd_mps  in [{DOMAIN_VSPD_LO_MPS:.1f}, {DOMAIN_VSPD_HI_MPS:.1f}] m/s")
    print(_rule("-"))

    for ar in report.assertions:
        tag = _status_tag(ar.status)
        print(f"  [{tag}]  {ar.assertion_id}")
        print(f"             {ar.description}")
        print(f"             Solver time : {ar.solver_ms:.2f} ms")

        if ar.counterexample is not None:
            cex = ar.counterexample
            print(f"             Counterexample:")
            print(f"               alt_m      = {cex.alt_m:.6f} m")
            print(f"               obs_dist_m = {cex.obs_dist_m:.6f} m")
            print(f"               v_spd_mps  = {cex.v_spd_mps:.6f} m/s")
            print(f"               output_m   = {cex.output_m:.6f} m  "
                  f"[violates [{MIN_ALT_M}, {MAX_ALT_M}] m]")
        print()

    print(_rule("-"))
    if report.all_proved:
        verdict = "VERDICT : ALL INVARIANTS PROVED - System cleared for simulation"
    else:
        verdict = (
            f"VERDICT : {report.falsified_count} INVARIANT(S) FALSIFIED "
            f"- DO NOT DEPLOY"
        )
    print(f"  {verdict}")
    print(_rule("="))
    print()


# ---------------------------------------------------------------------------
# Pipeline entry point
# ---------------------------------------------------------------------------

def run_verification() -> VerificationReport:
    """Execute all three safety invariant checks and return a consolidated report.

    This function is the public API for the verification pipeline. It is
    called by ``main.py`` before any simulation or GCS display is started.
    The returned ``VerificationReport.all_proved`` flag gates all downstream
    stages; simulation is blocked if any invariant is FALSIFIED.

    Returns:
        ``VerificationReport`` containing one ``AssertionResult`` per invariant.
    """
    report = VerificationReport(module_name="drone_logic.compute_target_alt")

    checks = [
        check_inv001_floor,
        check_inv002_ceiling,
        check_inv003_climb_bounded,
    ]

    for check_fn in checks:
        report.assertions.append(check_fn())

    return report


if __name__ == "__main__":
    # -- standard verification run -------------------------------------------
    rpt = run_verification()
    print_verification_report(rpt)

    # -- counterexample demonstration run ------------------------------------
    print(_rule("="))
    print("  COUNTEREXAMPLE DEMONSTRATION  -  Unsafe Variant (clamp removed)")
    print(_rule("="))
    unsafe_result = check_unsafe_variant()
    temp_report   = VerificationReport(module_name="drone_logic [UNSAFE VARIANT - NO CLAMP]")
    temp_report.assertions.append(unsafe_result)
    print_verification_report(temp_report)

    sys.exit(0 if rpt.all_proved else 1)
