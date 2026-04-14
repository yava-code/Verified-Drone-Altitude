"""
main.py
=======
Entry point for the Drone Altitude Control formal verification demo.

Execution flow:
    1. Run Z3 formal verification and print report.
    2. Simulate a full drone mission.
    3. Launch the animated visualization.

Usage:
    python main.py

Exit codes:
    0  - verification passed, visualization launched.
    1  - verification failed (counter-examples printed).
"""

from __future__ import annotations

import sys

from verify_safety import run_verification, print_report
from simulate import run_simulation
from visualize import show


def main() -> int:
    """
    Orchestrate the three pipeline stages.

    Returns
    -------
    int
        Exit code (0 = success, 1 = verification failure).
    """
    # ------------------------------------------------------------------ #
    # Stage 1: Formal Verification                                        #
    # ------------------------------------------------------------------ #
    print("\n[1/3] Running Z3 formal verification ...")
    report = run_verification()
    print_report(report)

    if not report.all_proved:
        print("\n[ERROR] Verification failed. Fix counter-examples before flight.")
        return 1

    print("[1/3] Verification complete - all safety properties PROVED.\n")

    # ------------------------------------------------------------------ #
    # Stage 2: Simulate flight                                            #
    # ------------------------------------------------------------------ #
    print("[2/3] Simulating drone mission ...")
    states = run_simulation(verified=report.all_proved)
    print(f"      Generated {len(states)} simulation frames.\n")

    # ------------------------------------------------------------------ #
    # Stage 3: Visualize                                                  #
    # ------------------------------------------------------------------ #
    print("[3/3] Launching visualization (close window to exit) ...")
    show(states, verified=report.all_proved)

    return 0


if __name__ == "__main__":
    sys.exit(main())
