"""
visualize.py
============
Real-time animated visualization of the drone altitude control system.

Layout
------
Left panel  - top-down/side-view of the drone flying past an obstacle.
Right panel - live altitude chart with safety envelope highlighted.
Status bar  - Z3 verification badge and current flight phase.

Requires: matplotlib >= 3.7, numpy >= 1.24
"""

from __future__ import annotations

import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyArrowPatch
from typing import List

from drone_control import MIN_ALT, MAX_ALT, OBSTACLE_SAFE_DIST
from simulate import DroneState, OBSTACLE_X, OBSTACLE_ALT

# ---------------------------------------------------------------------------
# Colour palette  (dark theme)
# ---------------------------------------------------------------------------
BG_DARK      = "#0D1117"
BG_PANEL     = "#161B22"
BG_PANEL2    = "#1C2128"
ACCENT_GREEN = "#39D353"
ACCENT_RED   = "#F85149"
ACCENT_BLUE  = "#58A6FF"
ACCENT_AMBER = "#E3B341"
ACCENT_CYAN  = "#56D364"
TEXT_PRIMARY = "#E6EDF3"
TEXT_MUTED   = "#8B949E"
GRID_COLOR   = "#21262D"
SAFE_BAND    = "#1A3A2A"     # faint green band for safe altitude zone
DRONE_COLOR  = "#58A6FF"
OBSTACLE_COL = "#F85149"


def _configure_axes(ax: plt.Axes, title: str = "") -> None:
    """Apply dark-theme styling to an axes object."""
    ax.set_facecolor(BG_PANEL)
    for spine in ax.spines.values():
        spine.set_edgecolor(GRID_COLOR)
    ax.tick_params(colors=TEXT_MUTED, labelsize=8)
    ax.xaxis.label.set_color(TEXT_MUTED)
    ax.yaxis.label.set_color(TEXT_MUTED)
    ax.grid(color=GRID_COLOR, linewidth=0.5, linestyle="--", alpha=0.6)
    if title:
        ax.set_title(title, color=TEXT_PRIMARY, fontsize=10, fontweight="bold", pad=8)


# ---------------------------------------------------------------------------
# Drone body drawing
# ---------------------------------------------------------------------------

def _draw_drone(ax: plt.Axes, x: float, y: float, size: float = 1.8) -> List:
    """
    Draw a schematic drone (quadcopter top-view) centred at (x, y).
    Returns a list of matplotlib artists for later removal.
    """
    artists = []
    arm_len = size * 0.9
    rotor_r = size * 0.35

    arms = [(-arm_len, -arm_len), (arm_len, -arm_len),
            (-arm_len,  arm_len), (arm_len,  arm_len)]

    # Arms
    for ax_off, ay_off in arms:
        line, = ax.plot([x, x + ax_off * 0.5], [y, y + ay_off * 0.5],
                        color=DRONE_COLOR, linewidth=2.5, solid_capstyle="round", zorder=5)
        artists.append(line)
        # Rotor disc
        circle = plt.Circle((x + ax_off * 0.5, y + ay_off * 0.5),
                             rotor_r, color=DRONE_COLOR, alpha=0.35, zorder=5)
        ax.add_patch(circle)
        artists.append(circle)

    # Body
    body = plt.Circle((x, y), size * 0.22, color=DRONE_COLOR, zorder=6)
    ax.add_patch(body)
    artists.append(body)

    return artists


# ---------------------------------------------------------------------------
# Main animation builder
# ---------------------------------------------------------------------------

def build_animation(states: List[DroneState], verified: bool) -> animation.FuncAnimation:
    """
    Build and return the matplotlib FuncAnimation object.

    Parameters
    ----------
    states   : Simulation state list from simulate.run_simulation().
    verified : True if Z3 proved all safety properties.

    Returns
    -------
    animation.FuncAnimation
    """
    matplotlib.rcParams["font.family"] = "DejaVu Sans"

    fig = plt.figure(figsize=(14, 7), facecolor=BG_DARK)
    fig.canvas.manager.set_window_title("Drone Altitude Control - Formal Verification Demo")

    gs = GridSpec(2, 2, figure=fig,
                  left=0.06, right=0.97, top=0.88, bottom=0.10,
                  hspace=0.45, wspace=0.32)

    ax_scene = fig.add_subplot(gs[:, 0])   # left: flight scene (tall)
    ax_alt   = fig.add_subplot(gs[0, 1])   # right-top: altitude chart
    ax_dist  = fig.add_subplot(gs[1, 1])   # right-bottom: obstacle distance

    # --- Scene axes setup ---
    SCENE_W = 1.0
    SCENE_H = MAX_ALT + 20
    ax_scene.set_xlim(-0.05, SCENE_W + 0.05)
    ax_scene.set_ylim(-5, SCENE_H)
    _configure_axes(ax_scene, "Flight Scene")
    ax_scene.set_xlabel("Mission progress")
    ax_scene.set_ylabel("Altitude (m)")

    # Ground
    ax_scene.axhline(0, color="#4A3728", linewidth=3, zorder=1)
    ax_scene.fill_between([0, 1], 0, -5, color="#2A1E15", zorder=0)

    # Safe altitude band (green shading)
    safe_rect = mpatches.Rectangle((0, MIN_ALT), SCENE_W,
                                    MAX_ALT - MIN_ALT,
                                    color=SAFE_BAND, alpha=0.3, zorder=0)
    ax_scene.add_patch(safe_rect)

    # Ceiling / floor lines
    ax_scene.axhline(MAX_ALT, color=ACCENT_RED,   linewidth=1, linestyle="--", alpha=0.7, zorder=1)
    ax_scene.axhline(MIN_ALT, color=ACCENT_AMBER, linewidth=1, linestyle="--", alpha=0.7, zorder=1)
    ax_scene.text(0.01, MAX_ALT + 1.5, f"MAX {MAX_ALT} m", color=ACCENT_RED,   fontsize=7)
    ax_scene.text(0.01, MIN_ALT + 1.5, f"MIN {MIN_ALT} m", color=ACCENT_AMBER, fontsize=7)

    # Obstacle block
    obstacle_w, obstacle_h = 0.04, 18.0
    obstacle_patch = mpatches.Rectangle(
        (OBSTACLE_X - obstacle_w / 2, 0),
        obstacle_w, obstacle_h,
        color=OBSTACLE_COL, alpha=0.85, zorder=3
    )
    ax_scene.add_patch(obstacle_patch)

    # Obstacle danger zone circle
    danger_circle = plt.Circle((OBSTACLE_X, obstacle_h / 2),
                                OBSTACLE_SAFE_DIST / MAX_ALT * 0.35,
                                color=OBSTACLE_COL, alpha=0.12, zorder=2)
    ax_scene.add_patch(danger_circle)
    ax_scene.text(OBSTACLE_X + 0.022, obstacle_h + 2, "Obstacle",
                  color=OBSTACLE_COL, fontsize=7)

    # Drone trail (line)
    trail_x: List[float] = []
    trail_y: List[float] = []
    trail_line, = ax_scene.plot([], [], color=DRONE_COLOR, linewidth=1.2,
                                alpha=0.5, zorder=4)

    drone_artists: List = []

    # --- Altitude chart ---
    _configure_axes(ax_alt, "Altitude over Time")
    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.set_xlabel("Time (s)")
    total_time = states[-1].time
    ax_alt.set_xlim(0, total_time)
    ax_alt.set_ylim(-2, MAX_ALT + 10)

    # Safe band shading on chart
    ax_alt.fill_between([0, total_time], MIN_ALT, MAX_ALT,
                        color=SAFE_BAND, alpha=0.4, label="Safe band")
    ax_alt.axhline(MAX_ALT, color=ACCENT_RED,   ls="--", lw=0.8)
    ax_alt.axhline(MIN_ALT, color=ACCENT_AMBER, ls="--", lw=0.8)

    alt_times: List[float] = []
    alt_vals:  List[float] = []
    alt_line, = ax_alt.plot([], [], color=ACCENT_GREEN, linewidth=1.8, zorder=5)
    alt_dot,  = ax_alt.plot([], [], "o", color=ACCENT_GREEN, markersize=5, zorder=6)

    # --- Obstacle distance chart ---
    _configure_axes(ax_dist, "Obstacle Distance")
    ax_dist.set_ylabel("Distance (m)")
    ax_dist.set_xlabel("Time (s)")
    ax_dist.set_xlim(0, total_time)
    ax_dist.set_ylim(0, 100)
    ax_dist.axhline(OBSTACLE_SAFE_DIST, color=OBSTACLE_COL,
                    ls="--", lw=1, label=f"Safe dist {OBSTACLE_SAFE_DIST} m")
    ax_dist.legend(fontsize=7, facecolor=BG_PANEL, labelcolor=TEXT_MUTED,
                   framealpha=0.8, loc="upper right")

    dist_times: List[float] = []
    dist_vals:  List[float] = []
    dist_line, = ax_dist.plot([], [], color=ACCENT_AMBER, linewidth=1.5)

    # --- Title / status bar ---
    verify_color = ACCENT_GREEN if verified else ACCENT_RED
    verify_label = "Verified by Z3" if verified else "Verification FAILED"
    verify_icon  = chr(0x25CF)  # filled circle

    fig.text(0.5, 0.95,
             "Drone Altitude Control System",
             ha="center", va="center",
             color=TEXT_PRIMARY, fontsize=15, fontweight="bold")

    fig.text(0.5, 0.91,
             f"{verify_icon}  {verify_label}  |  SMT Solver: Z3 (Microsoft Research)",
             ha="center", va="center",
             color=verify_color, fontsize=9)

    phase_text = fig.text(0.5, 0.01, "", ha="center", va="bottom",
                          color=TEXT_MUTED, fontsize=8)

    # Phase colour map
    PHASE_COLORS = {
        "takeoff":   ACCENT_BLUE,
        "cruise":    ACCENT_GREEN,
        "avoidance": ACCENT_RED,
        "descent":   ACCENT_AMBER,
    }

    # --- Animation update function ---
    def _update(frame: int):
        nonlocal drone_artists

        state = states[frame]
        x = state.x_pos
        y = state.altitude

        # Update trail
        trail_x.append(x)
        trail_y.append(y)
        trail_line.set_data(trail_x, trail_y)

        # Redraw drone
        for artist in drone_artists:
            artist.remove()
        drone_artists = _draw_drone(ax_scene, x, y)

        # Altitude chart
        alt_times.append(state.time)
        alt_vals.append(state.altitude)
        alt_line.set_data(alt_times, alt_vals)
        alt_dot.set_data([state.time], [state.altitude])

        # Distance chart
        dist_times.append(state.time)
        dist_vals.append(state.obstacle_dist)
        dist_line.set_data(dist_times, dist_vals)

        # Colour distance line by proximity
        if state.obstacle_dist < OBSTACLE_SAFE_DIST:
            dist_line.set_color(ACCENT_RED)
        else:
            dist_line.set_color(ACCENT_AMBER)

        # Phase status
        phase_color = PHASE_COLORS.get(state.phase, TEXT_MUTED)
        phase_text.set_text(
            f"Phase: {state.phase.upper()}   |   Alt: {state.altitude:.1f} m   "
            f"|   Obstacle dist: {state.obstacle_dist:.1f} m   "
            f"|   t = {state.time:.1f}s"
        )
        phase_text.set_color(phase_color)

        return (trail_line, alt_line, alt_dot, dist_line, phase_text, *drone_artists)

    anim = animation.FuncAnimation(
        fig,
        _update,
        frames=len(states),
        interval=30,       # ms between frames
        blit=False,        # blit=False for cross-platform compatibility
        repeat=False,
    )

    return anim


def show(states: List[DroneState], verified: bool) -> None:
    """Build the animation and display it in an interactive window."""
    anim = build_animation(states, verified)
    plt.show()
