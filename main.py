"""
main.py
=======
Pipeline entry point for the Drone Altitude Control formal verification system.

Execution stages:
    1. Z3 SMT formal verification (always runs first)
    2. Counterexample demonstration on the unsafe variant
    3. Mission simulation
    4. GCS display (Pygame) or Matplotlib fallback

Usage:
    python main.py                     # full pipeline: verify -> GCS
    python main.py --mode verify       # verification report only
    python main.py --mode matplotlib   # verify -> Matplotlib animation
    python main.py --mode manim        # render Manim counterexample video

Exit codes:
    0  - all invariants PROVED, pipeline completed
    1  - invariant FALSIFIED or UNKNOWN; downstream stages blocked
    2  - runtime argument error
"""

from __future__ import annotations

import argparse
import sys
from typing import Final, Literal

from verify_z3 import (
    run_verification,
    check_unsafe_variant,
    print_verification_report,
)
from result_types import VerificationReport, VerificationStatus
from simulate import run_simulation

# ---------------------------------------------------------------------------
# CLI argument parsing
# ---------------------------------------------------------------------------

_VALID_MODES: Final = ("verify", "pygame", "matplotlib", "manim")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Drone Altitude Control - Formal Verification Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=_VALID_MODES,
        default="pygame",
        help="Display mode to launch after verification (default: pygame)",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Stage runners
# ---------------------------------------------------------------------------

def _stage_verify() -> VerificationReport:
    """Run the three invariant checks and print the full report."""
    print("\n[STAGE 1/3]  Z3 SMT formal verification")
    print("            Encoding symbolic model of compute_target_alt() ...")

    rpt = run_verification()
    print_verification_report(rpt)

    print("[STAGE 1/3]  Counterexample demonstration (unsafe variant)")
    unsafe_rpt = VerificationReport(module_name="drone_logic [UNSAFE - NO CLAMP]")
    unsafe_rpt.assertions.append(check_unsafe_variant())
    print_verification_report(unsafe_rpt)

    return rpt


def _stage_simulate(rpt: VerificationReport):
    """Run the physics simulation and return telemetry frames."""
    print("[STAGE 2/3]  Running physics simulation ...")
    frames = run_simulation(smt_verified=rpt.all_proved)
    print(f"             Generated {len(frames)} telemetry frames.")
    return frames


def _stage_pygame(frames, smt_verified: bool) -> None:
    """Launch the Pygame GCS display."""
    print("[STAGE 3/3]  Launching GCS display (ESC or Q to quit) ...")
    from gcs_pygame import run_gcs
    run_gcs(frames, smt_verified)


def _stage_matplotlib(frames, smt_verified: bool) -> None:
    """Launch the Matplotlib animated visualization (fallback)."""
    print("[STAGE 3/3]  Launching Matplotlib animation (close window to exit) ...")
    from visualize import show
    show(frames, verified=smt_verified)


def _stage_manim() -> None:
    """Render the Manim counterexample video."""
    print("[STAGE 3/3]  Rendering Manim counterexample animation ...")
    import subprocess
    result = subprocess.run(
        [
            sys.executable, "-m", "manim",
            "-pqh",
            "manim_counterexample.py",
            "CounterexampleScene",
        ],
        check=False,
    )
    if result.returncode != 0:
        print("[ERROR]  Manim render failed. Ensure manim is installed:")
        print("         pip install manim")
        print("         See: https://docs.manim.community/en/stable/installation")
        sys.exit(1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    """Orchestrate the verification pipeline.

    Returns:
        int: Exit code (0 = success, 1 = verification failure).
    """
    args = _parse_args()

    # Stage 1: always run verification first
    rpt = _stage_verify()

    if not rpt.all_proved:
        print(
            "[BLOCKED]  Downstream stages require all invariants to be PROVED.\n"
            "           Fix the control law counter-examples before proceeding."
        )
        return 1

    # Mode: verify-only
    if args.mode == "verify":
        return 0

    # Mode: manim
    if args.mode == "manim":
        _stage_manim()
        return 0

    # Stages 2 + 3: simulate then display
    frames = _stage_simulate(rpt)

    if args.mode == "matplotlib":
        _stage_matplotlib(frames, rpt.all_proved)
    else:  # default: pygame
        _stage_pygame(frames, rpt.all_proved)

    return 0


if __name__ == "__main__":
    sys.exit(main())
