"""
result_types.py
===============
Shared result types and status codes for the verification pipeline.

In safety-critical systems, exceptions are an unstructured control flow
mechanism that cannot be statically reasoned about. This module replaces
all try/except patterns with explicit, typed result objects whose state
transitions are fully enumerable at compile time.

Design note:
    All public functions in this project return one of the result types
    defined here instead of raising exceptions. Callers must explicitly
    branch on ``VerificationStatus`` before accessing payload fields.
    This pattern mirrors the ``Result<T, E>`` idiom in Rust and Ada's
    checked exceptions, both common in DO-178C / DO-333 workflows.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Final, List, Optional


# ---------------------------------------------------------------------------
# Status codes
# ---------------------------------------------------------------------------

class VerificationStatus(Enum):
    """Outcome of a single SMT solver assertion check.

    Values:
        PROVED:     The negation of the safety property is ``unsat``.
                    The property holds for the entire admissible input domain.
        FALSIFIED:  The solver returned ``sat``; a counterexample exists.
                    The property is violated for the returned input assignment.
        TIMEOUT:    The solver exceeded its resource bound without a decision.
        UNKNOWN:    The solver returned ``unknown`` (e.g., nonlinear arithmetic
                    that Z3's default tactic cannot decide).
    """

    PROVED    = auto()
    FALSIFIED = auto()
    TIMEOUT   = auto()
    UNKNOWN   = auto()


class FlightPhase(Enum):
    """Discrete mission phase labels used in telemetry and simulation state."""

    PREFLIGHT = auto()
    TAKEOFF   = auto()
    CRUISE    = auto()
    AVOIDANCE = auto()
    DESCENT   = auto()
    LANDED    = auto()


# ---------------------------------------------------------------------------
# Counterexample payload
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Counterexample:
    """Concrete input assignment that falsifies a safety property.

    All fields carry the exact rational values reported by the Z3 model.
    The Z3 solver guarantees that substituting these values into the
    symbolic model produces an output that violates the checked property.

    Attributes:
        alt_m:       Altitude at which the violation occurs (m).
        obs_dist_m:  Obstacle distance at which the violation occurs (m).
        v_spd_mps:   Vertical speed at which the violation occurs (m/s).
        output_m:    Actual computed output that violated the bound (m).
    """

    alt_m:      float
    obs_dist_m: float
    v_spd_mps:  float
    output_m:   float


# ---------------------------------------------------------------------------
# Per-assertion result
# ---------------------------------------------------------------------------

@dataclass
class AssertionResult:
    """Outcome of a single SMT solver assertion (one safety invariant check).

    Attributes:
        assertion_id:  Short identifier, e.g. ``"INV-001"``.
        description:   Human-readable description of the safety property.
        status:        ``VerificationStatus`` - the solver decision.
        counterexample: Populated iff ``status == FALSIFIED``.
        solver_ms:     Wall-clock time the solver consumed (milliseconds).
    """

    assertion_id:   str
    description:    str
    status:         VerificationStatus
    counterexample: Optional[Counterexample] = None
    solver_ms:      float = 0.0

    @property
    def proved(self) -> bool:
        """True iff the assertion is formally proved."""
        return self.status == VerificationStatus.PROVED


# ---------------------------------------------------------------------------
# Consolidated verification report
# ---------------------------------------------------------------------------

@dataclass
class VerificationReport:
    """Aggregated results for all safety assertions in a verification run.

    Attributes:
        assertions:  Ordered list - one entry per checked invariant.
        module_name: Name of the Python module under verification.
    """

    assertions:  List[AssertionResult] = field(default_factory=list)
    module_name: str = "drone_logic"

    @property
    def all_proved(self) -> bool:
        """True iff every assertion in the report is formally proved."""
        return all(a.proved for a in self.assertions)

    @property
    def falsified_count(self) -> int:
        """Number of assertions for which the solver found a counterexample."""
        return sum(1 for a in self.assertions if a.status == VerificationStatus.FALSIFIED)
