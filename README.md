# Drone Altitude Control - Formal Verification with Z3

[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![Z3 SMT Solver](https://img.shields.io/badge/verified-Z3%20SMT-green.svg)](https://github.com/Z3Prover/z3)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A portfolio project demonstrating **formal verification** of a safety-critical
control system. The drone altitude controller is not just tested - it is
**mathematically proved** to be safe for every possible input.

---

## What Is Formal Verification?

Traditional unit tests verify behaviour for a *finite set of chosen inputs*.
No matter how comprehensive the test suite is, it leaves gaps: the combinations
of real-valued sensor readings a drone might encounter are infinite.

**Formal verification** closes that gap. Instead of sampling the input space,
we encode the system's logic as a mathematical formula and ask an automated
theorem prover: *"Does there exist ANY input that violates this property?"*

If the answer is *"no"* (the formula is **unsatisfiable**), the property is
**proved for all inputs simultaneously** - not just the ones we thought to test.

---

## Why Z3?

[Z3](https://github.com/Z3Prover/z3) is an industrial-grade SMT (Satisfiability
Modulo Theories) solver developed by Microsoft Research. It is used in
production at Amazon (AWS IAM policy verification), Microsoft (Windows driver
verification), and NASA (flight controller analysis). Its Python API makes
it accessible while retaining the full power of the underlying solver.

Key properties that make Z3 suitable here:
- Handles **real arithmetic** (not just integers), matching floating-point
  control laws.
- Returns **counter-examples**: if a bug exists, Z3 shows the exact sensor
  values that trigger it - not just a failed assert.
- Scales to complex state spaces when properties are linear or piecewise-linear.

---

## Safety Properties Verified

| ID | Property | Verdict |
|----|----------|---------|
| P1 | `output >= MIN_ALT` (2 m) for all valid inputs | **PROVED** |
| P2 | `output <= MAX_ALT` (120 m) for all valid inputs | **PROVED** |
| P3 | Climb commands never breach the envelope | **PROVED** |

The input domain is the full physical sensor range:
- `current_alt`: [0, 200] m
- `obstacle_dist`: [0, 200] m
- `vertical_speed`: [-10, 10] m/s

Z3 searched this **infinite continuous domain** and found no violations.

---

## Project Structure

```
simulation/
    drone_control.py    - Control law (the "plant" being verified)
    verify_safety.py    - Z3 formal verification, counter-example finder
    simulate.py         - Physics-based flight simulator
    visualize.py        - Matplotlib real-time animated visualization
    main.py             - Single entry point
    requirements.txt    - All dependencies
    README.md           - This file
```

---

## How It Works

```
Sensor Inputs            Z3 Symbolic Model
(current_alt,    ---->   (Real variables +   ---->  PROVED / Counter-example
 obstacle_dist,           z3.If expressions)
 vertical_speed)

If PROVED: run simulation --> animated flight visualization
```

### Control Law (`drone_control.py`)

```python
def calculate_altitude_adjustment(
    current_alt: float,
    obstacle_dist: float,
    vertical_speed: float,
) -> float:
    # Phase 1: obstacle avoidance or gentle descent
    if obstacle_dist < OBSTACLE_SAFE_DIST:
        raw_target = current_alt + MAX_STEP_UP * proximity_factor
    else:
        raw_target = current_alt - descent

    # Phase 2: HARD safety clamp (formally verified by Z3)
    return max(MIN_ALT, min(MAX_ALT, raw_target))
```

### Verification (`verify_safety.py`)

The Python control law is re-expressed using Z3 symbolic variables
(`z3.Real()`) and `z3.If()` for branches. The solver is then asked
to **find a counter-example** (negate the property). `unsat` = proved.

```python
solver.add(output < z3.RealVal(MIN_ALT))   # "find a violation"
result = solver.check()                    # z3.unsat -> PROVED
```

### Why This Beats Unit Tests

| Criterion | Unit Tests | Formal Verification (Z3) |
|-----------|-----------|--------------------------|
| Input coverage | Finite samples | Complete continuous domain |
| Bug discovery | Only tested cases | Counter-examples for ALL inputs |
| Proof strength | Inductive evidence | Mathematical guarantee |
| Scalability | O(n) with test count | Polynomial in property complexity |
| Industry use | Universal | NASA, AWS, Microsoft |

---

## Installation and Usage

```bash
# 1. Install dependencies (Python 3.10+)
pip install -r requirements.txt

# 2. Run verification only
python verify_safety.py

# 3. Run full demo (verification + simulation + visualization)
python main.py
```

Expected output of `verify_safety.py`:

```
============================================================
  Z3 FORMAL VERIFICATION REPORT
============================================================
  [OK]  P1: Output is always >= MIN_ALT (2.0 m)
         Status : PROVED
  [OK]  P2: Output is always <= MAX_ALT (120.0 m)
         Status : PROVED
  [OK]  P3: Climb commands never breach the safety envelope
         Status : PROVED

  VERDICT: ALL PROPERTIES PROVED - System verified SAFE
============================================================
```

---

## Simulating a Bug (Counter-Example Demo)

To see Z3 in action finding a real bug, temporarily comment out the
safety clamp in `drone_control.py`:

```python
# safe_target = max(MIN_ALT, min(MAX_ALT, raw_target))  # BUG: clamp removed
safe_target = raw_target                                 # UNSAFE
```

Then re-run `python verify_safety.py`. Z3 will immediately print
the exact input values that cause the ceiling breach - no fuzzing,
no hours of testing.

---

## References

- Z3 theorem prover: https://github.com/Z3Prover/z3
- "Satisfiability Modulo Theories" (SMT): https://smtlib.cs.uiowa.edu/
- NASA Formal Methods: https://shemesh.larc.nasa.gov/fm/
- AWS Automated Reasoning: https://aws.amazon.com/security/provable-security/
- FAA Part 107 altitude limits: https://www.faa.gov/uas/commercial_operators/part_107_waivers

---

*Built with Python 3.10+ - no simulation engine required, runs with a single `pip install`.*
# Verified-Drone-Altitude
