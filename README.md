<img width="1282" height="721" alt="Screenshot_25" src="https://github.com/user-attachments/assets/8284495d-1826-45d9-93cf-9de1f3f30185" />

# Verified Drone Altitude Control


https://github.com/user-attachments/assets/386bf0d7-75e1-4259-a9d8-3c243f8d652c

(video made by me)
Formal verification of a UAV altitude control law using the Z3 SMT solver.
The core function `compute_target_alt` is mathematically proved - not tested -
to respect its safety envelope for every input in the admissible domain.

---

## Technical Stack

| Component | Version | Role |
|-----------|---------|------|
| Z3 SMT solver (`z3-solver`) | >= 4.12 | Safety invariant proof engine |
| Manim Community | >= 0.18 | Counterexample visualization render |
| Pygame | >= 2.5 | Ground Control Station (GCS) display |
| Matplotlib | >= 3.7 | Fallback animated flight visualization |
| Python | 3.10+ | Implementation language |

---

## Safety Invariants

The following invariants are formally discharged by Z3 over the complete
continuous input domain (not sampled - all of it):

```
Domain:
  alt_m      in [0.0, 200.0]  m
  obs_dist_m in [0.0, 200.0]  m
  v_spd_mps  in [-10.0, 10.0] m/s
```

| ID | Invariant | Z3 Tactic |
|----|-----------|-----------|
| INV-001 | `compute_target_alt(...) >= MIN_ALT_M (2.0 m)` | Negation: assert `output < 2.0` |
| INV-002 | `compute_target_alt(...) <= MAX_ALT_M (120.0 m)` | Negation: assert `output > 120.0` |
| INV-003 | Avoidance climbs stay within `[2.0, 120.0] m` | Negation: climb AND envelope violation |

---

## Verification Results

Output of `python main.py --mode verify` (Z3 4.16.0, Windows x64):

```
================================================================
  Z3 SMT SOLVER - FORMAL VERIFICATION REPORT
  Module under verification : drone_logic.compute_target_alt
  SMT solver engine         : Z3 4.16.0
----------------------------------------------------------------
  Admissible input domain:
    alt_m      in [0.0, 200.0] m
    obs_dist_m in [0.0, 200.0] m
    v_spd_mps  in [-10.0, 10.0] m/s
----------------------------------------------------------------
  [PROVED   ]  INV-001
               compute_target_alt() >= MIN_ALT_M (2.0 m) for all inputs
               Solver time : 0.92 ms

  [PROVED   ]  INV-002
               compute_target_alt() <= MAX_ALT_M (120.0 m) for all inputs
               Solver time : 0.43 ms

  [PROVED   ]  INV-003
               Avoidance climb output bounded within [2.0, 120.0] m
               Solver time : 0.86 ms
----------------------------------------------------------------
  VERDICT : ALL INVARIANTS PROVED - System cleared for simulation
================================================================
```

### Counterexample demonstration (unsafe variant, clamp removed)

```
  [FALSIFIED]  INV-002-UNSAFE
               [UNSAFE VARIANT] Ceiling invariant on clamp-removed model
               Solver time : 1.09 ms
               Counterexample:
                 alt_m      = 121.111111 m
                 obs_dist_m = 15.000000  m
                 v_spd_mps  = 0.555556   m/s
                 output_m   = 120.555556 m  [violates [2.0, 120.0] m]
  VERDICT : 1 INVARIANT(S) FALSIFIED - DO NOT DEPLOY
```

Z3 found a specific input assignment that breaks the ceiling invariant in
under 2 ms. No fuzzing, no sampling - this is a proof-by-counterexample
over the entire input domain.

---

## Formal Verification vs. Unit Testing

| Criterion | Unit Tests | Z3 Formal Verification |
|-----------|-----------|------------------------|
| Input coverage | Finite, hand-picked samples | Complete continuous domain |
| Bug discovery | Only tested cases | Any input - no gaps |
| Guarantee | Evidence | Mathematical proof |
| Counterexample | None | Exact violating assignment |
| Standard | Universal | NASA FM, DO-333, AWS IAM |

The key distinction: unit tests provide *inductive evidence*. Z3 provides
a *deductive proof*. For `compute_target_alt`, the property
`MIN_ALT_M <= output <= MAX_ALT_M` is now a theorem, not a conjecture.

---

## Architecture

```
drone_logic.py          - Control law (plant model under verification)
verify_z3.py            - SMT assertions, counterexample extraction
result_types.py         - Typed result objects (no exceptions used)
simulate.py             - Physics trajectory generator (TelemetryFrame log)
gcs_pygame.py           - Amber-on-black GCS display (Pygame)
manim_counterexample.py - 3-act counterexample animation (Manim)
visualize.py            - Matplotlib animation (fallback)
main.py                 - Pipeline entry point with --mode selector
```

---

## Installation

```bash
pip install z3-solver matplotlib numpy pygame
# For Manim (requires FFmpeg and Cairo):
pip install manim
```

See the [Manim installation guide](https://docs.manim.community/en/stable/installation)
for platform-specific system dependencies (FFmpeg, Cairo, Pango).

---

## Usage

```bash
# Verification report only
python main.py --mode verify

# Full pipeline: verify -> GCS display (Pygame)
python main.py

# Verify -> Matplotlib animation (no system dependencies)
python main.py --mode matplotlib

# Render Manim counterexample video (requires manim install)
python main.py --mode manim
# or directly:
python -m manim -pqh manim_counterexample.py CounterexampleScene
```

---

## Coding Standards

- No `try/except` blocks. All outcomes are explicit `VerificationStatus` enum values.
- All physical quantities carry unit suffixes: `_m`, `_mps`, `_s`.
- `typing.Final` used for all safety constants; `NewType` for `Meters`, `MetersPerSecond`.
- Docstrings follow Google style with explicit `Precondition`, `Postcondition`,
  and `Invariants` fields.
- Z3 symbolic model uses only linear real arithmetic - decidable by the  
  LA(Q) tactic without requiring nonlinear extensions.

---

## References

- Z3 Theorem Prover - https://github.com/Z3Prover/z3 (Microsoft Research)
- RTCA DO-178C - Software Considerations in Airborne Systems and Equipment Certification
- NASA/TM-2019-220054 - Formal Methods in Aerospace
- FAA 14 CFR Part 107.51 - Small Unmanned Aircraft Operating Limitations
- SMTLIB2 Standard - https://smtlib.cs.uiowa.edu/
