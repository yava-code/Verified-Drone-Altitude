"""
manim_counterexample.py
=======================
Manim animation: Z3 counterexample discovery and proof of fix.

This script produces a high-quality MP4 animation demonstrating the
formal verification workflow in three acts:

  Act 1 - Unsafe Variant:
    The control law without the Phase-2 safety clamp is represented
    symbolically. Z3's SMT query is shown, and the counterexample
    (alt_m=120.0, obs_dist_m=0.0 => output=125.0 > MAX_ALT=120.0)
    is revealed on a number line.

  Act 2 - Counterexample Anatomy:
    The exact arithmetic of the violation is traced step by step.
    The output value is highlighted in red at its position on the
    altitude domain, clearly above the MAX_ALT ceiling.

  Act 3 - Applied Fix and Proof:
    The clamp expression is introduced. The number line updates to
    show the clamped output landing exactly at MAX_ALT. Z3's "unsat"
    response is displayed, confirming the invariant is now proved.

Render:
    python -m manim -pqh manim_counterexample.py CounterexampleScene

Dependencies:
    pip install manim
    # Also requires FFmpeg and Cairo; see: https://docs.manim.community/en/stable/installation
"""

from __future__ import annotations

from manim import (
    Scene,
    Text,
    MathTex,
    NumberLine,
    Dot,
    Arrow,
    VGroup,
    Rectangle,
    Line,
    AnimationGroup,
    Write,
    FadeIn,
    FadeOut,
    Create,
    Flash,
    Transform,
    MoveToTarget,
    Indicate,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    ORIGIN,
    RED,
    GREEN,
    YELLOW,
    WHITE,
    GRAY,
    ORANGE,
    config,
)

# ---------------------------------------------------------------------------
# Hardcoded counterexample (matches Z3 output from check_unsafe_variant())
# Known values for unsafe ceiling violation:
#   Input:  alt_m=120.0, obs_dist_m=0.0, v_spd_mps=0.0
#   Output: 120.0 + 5.0 * (1 - 0/15) = 125.0
# ---------------------------------------------------------------------------

CEX_ALT_M:     float = 120.0
CEX_OBS_M:     float = 0.0
CEX_RAW_OUT_M: float = 125.0
CEX_CLAMPED_M: float = 120.0
MAX_ALT_M:     float = 120.0
MIN_ALT_M:     float = 2.0


class CounterexampleScene(Scene):
    """Three-act animation of the counterexample discovery and proof-of-fix."""

    def construct(self) -> None:
        self._act1_unsafe_variant()
        self._act2_counterexample_anatomy()
        self._act3_fix_and_proof()

    # ------------------------------------------------------------------
    def _act1_unsafe_variant(self) -> None:
        """Act 1: present the unsafe variant and the SMT query."""

        title = Text(
            "Act 1 - Unsafe Variant: No Safety Clamp",
            font_size=32,
            color=YELLOW,
        ).to_edge(UP)
        self.play(Write(title))
        self.wait(0.5)

        # SMT query pseudocode
        smt_query = VGroup(
            Text("SMT Query (Z3 assertion):", font_size=22, color=GRAY),
            MathTex(
                r"\exists\; (alt, obs, vs) \in \mathcal{D}",
                font_size=28,
            ),
            MathTex(
                r"\text{s.t.}\quad \text{unsafe\_output}(alt, obs, vs) "
                r"> MAX\_ALT",
                font_size=26,
                color=RED,
            ),
        ).arrange(DOWN, buff=0.35).move_to(ORIGIN + UP * 0.8)

        self.play(FadeIn(smt_query, shift=UP * 0.2))
        self.wait(1.0)

        # Z3 response
        z3_result = Text(
            'Z3 response: "sat" - Counterexample found.',
            font_size=28,
            color=RED,
        ).next_to(smt_query, DOWN, buff=0.6)

        self.play(Write(z3_result))
        self.play(Flash(z3_result, color=RED, flash_radius=0.5))
        self.wait(1.5)
        self.play(FadeOut(smt_query), FadeOut(z3_result), FadeOut(title))

    # ------------------------------------------------------------------
    def _act2_counterexample_anatomy(self) -> None:
        """Act 2: trace the arithmetic of the ceiling violation."""

        title = Text(
            "Act 2 - Counterexample Anatomy",
            font_size=32,
            color=YELLOW,
        ).to_edge(UP)
        self.play(Write(title))
        self.wait(0.4)

        # Input assignment
        input_block = VGroup(
            Text("Counterexample input assignment:", font_size=22, color=GRAY),
            MathTex(r"alt\_m = 120.0 \; m", font_size=30),
            MathTex(r"obs\_dist\_m = 0.0 \; m", font_size=30),
            MathTex(r"v\_spd\_mps = 0.0 \; m/s", font_size=30),
        ).arrange(DOWN, buff=0.25).move_to(UP * 1.6 + LEFT * 2.5)

        self.play(FadeIn(input_block, shift=RIGHT * 0.3))
        self.wait(0.5)

        # Arithmetic trace
        trace_block = VGroup(
            Text("Control law evaluation (no clamp):", font_size=22, color=GRAY),
            MathTex(
                r"\text{prox\_ratio} = 1 - \frac{0.0}{15.0} = 1.0",
                font_size=26,
            ),
            MathTex(
                r"\text{raw\_tgt} = 120.0 + 5.0 \times 1.0 = \mathbf{125.0}",
                font_size=26,
                color=RED,
            ),
            MathTex(
                r"125.0 > MAX\_ALT\;(120.0) \quad \Rightarrow \quad \text{VIOLATION}",
                font_size=26,
                color=RED,
            ),
        ).arrange(DOWN, buff=0.28).move_to(UP * 1.6 + RIGHT * 2.2)

        self.play(FadeIn(trace_block, shift=LEFT * 0.3))
        self.wait(0.5)

        # Number line visualization
        num_line = NumberLine(
            x_range=[0, 140, 20],
            length=10,
            include_numbers=True,
            label_direction=DOWN,
            numbers_to_include=[0, 20, 40, 60, 80, 100, 120, 140],
            font_size=22,
        ).to_edge(DOWN, buff=1.2)

        line_label = Text("Altitude domain (m)", font_size=20, color=GRAY).next_to(
            num_line, DOWN, buff=0.35
        )

        self.play(Create(num_line), FadeIn(line_label))

        # MAX_ALT marker
        max_pt   = num_line.n2p(MAX_ALT_M)
        max_dot  = Dot(max_pt, color=GREEN, radius=0.1)
        max_lbl  = Text(f"MAX_ALT\n{MAX_ALT_M:.0f} m", font_size=18, color=GREEN).next_to(
            max_dot, UP, buff=0.1
        )
        self.play(Create(max_dot), FadeIn(max_lbl))

        # Unsafe output marker
        raw_pt   = num_line.n2p(CEX_RAW_OUT_M)
        raw_dot  = Dot(raw_pt, color=RED, radius=0.12)
        raw_lbl  = Text(f"unsafe output\n{CEX_RAW_OUT_M:.1f} m", font_size=18, color=RED).next_to(
            raw_dot, UP, buff=0.1
        )
        violation_arrow = Arrow(
            max_dot.get_center(),
            raw_dot.get_center(),
            color=RED,
            buff=0.08,
            stroke_width=3,
        )

        self.play(Create(raw_dot), FadeIn(raw_lbl))
        self.play(Create(violation_arrow))
        self.play(Flash(raw_dot, color=RED, flash_radius=0.4))
        self.wait(1.5)

        self.play(
            FadeOut(input_block),
            FadeOut(trace_block),
            FadeOut(raw_dot),
            FadeOut(raw_lbl),
            FadeOut(violation_arrow),
            FadeOut(max_dot),
            FadeOut(max_lbl),
            FadeOut(num_line),
            FadeOut(line_label),
            FadeOut(title),
        )

    # ------------------------------------------------------------------
    def _act3_fix_and_proof(self) -> None:
        """Act 3: apply the safety clamp and verify the fixed variant."""

        title = Text(
            "Act 3 - Applied Fix and Formal Proof",
            font_size=32,
            color=YELLOW,
        ).to_edge(UP)
        self.play(Write(title))
        self.wait(0.4)

        # Clamp expression
        fix_block = VGroup(
            Text("Phase-2 safety clamp (applied):", font_size=22, color=GRAY),
            MathTex(
                r"\text{tgt\_alt\_m} = \max(MIN\_ALT,\; \min(MAX\_ALT,\; raw\_tgt))",
                font_size=28,
                color=GREEN,
            ),
            MathTex(
                r"= \max(2.0,\; \min(120.0,\; 125.0)) = \mathbf{120.0 \; m}",
                font_size=28,
                color=GREEN,
            ),
        ).arrange(DOWN, buff=0.35).move_to(UP * 1.4)

        self.play(FadeIn(fix_block, shift=UP * 0.2))
        self.wait(0.8)

        # Number line
        num_line = NumberLine(
            x_range=[0, 140, 20],
            length=10,
            include_numbers=True,
            label_direction=DOWN,
            numbers_to_include=[0, 20, 40, 60, 80, 100, 120, 140],
            font_size=22,
        ).to_edge(DOWN, buff=1.2)

        line_label = Text("Altitude domain (m)", font_size=20, color=GRAY).next_to(
            num_line, DOWN, buff=0.35
        )
        self.play(Create(num_line), FadeIn(line_label))

        # MAX_ALT marker
        max_pt  = num_line.n2p(MAX_ALT_M)
        max_dot = Dot(max_pt, color=GREEN, radius=0.1)
        max_lbl = Text(f"MAX_ALT\n{MAX_ALT_M:.0f} m", font_size=18, color=GREEN).next_to(
            max_dot, UP, buff=0.1
        )
        self.play(Create(max_dot), FadeIn(max_lbl))

        # Clamped output = MAX_ALT
        clamped_dot = Dot(max_pt, color=GREEN, radius=0.14)
        clamped_lbl = Text(
            f"clamped output = {CEX_CLAMPED_M:.1f} m", font_size=18, color=GREEN
        ).next_to(clamped_dot, DOWN, buff=0.5)
        self.play(Create(clamped_dot), FadeIn(clamped_lbl))
        self.play(Flash(clamped_dot, color=GREEN, flash_radius=0.5))
        self.wait(0.6)

        # Z3 proof banner
        proof_banner = Text(
            'Z3 response: "unsat" - INV-002 PROVED for all inputs.',
            font_size=26,
            color=GREEN,
        ).next_to(fix_block, DOWN, buff=0.5)

        self.play(Write(proof_banner))
        self.play(Flash(proof_banner, color=GREEN, flash_radius=0.8))
        self.wait(1.0)

        # Final verdict
        verdict = Text(
            "Safety invariant holds for the ENTIRE continuous input domain.",
            font_size=24,
            color=WHITE,
        ).to_edge(DOWN, buff=0.2)
        self.play(FadeIn(verdict, shift=UP * 0.2))
        self.wait(2.5)

        self.play(
            FadeOut(title),
            FadeOut(fix_block),
            FadeOut(num_line),
            FadeOut(line_label),
            FadeOut(max_dot),
            FadeOut(max_lbl),
            FadeOut(clamped_dot),
            FadeOut(clamped_lbl),
            FadeOut(proof_banner),
            FadeOut(verdict),
        )
