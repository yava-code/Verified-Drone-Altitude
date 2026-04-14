"""
gcs_pygame.py
=============
Ground Control Station (GCS) display for the drone altitude control system.

Renders a CRT-style terminal interface using Pygame. The layout mimics
minimal avionics displays used in UAV ground stations: monochromatic
phosphor text on a near-black background, no decorative graphics.

Display layout:
    +--header bar (system status / verification badge)---+
    | Telemetry panel | Altitude gauge | Mission map      |
    +--telemetry log scrolling strip--------------------------+

Key design decisions:
    - Amber-on-black palette replicates P3 phosphor CRT terminals.
    - All text is rendered with a monospace font (Courier New / fallback).
    - The drone is rendered as a minimal + symbol with rotor arcs.
    - Scanline overlay provides CRT depth without obscuring data.
    - No images, sprites, or decorative assets are used.

Controls:
    ESC / Q  - quit
    SPACE    - pause / resume playback
"""

from __future__ import annotations

import math
import sys
from typing import Final, List, Tuple

import pygame

from drone_logic import MIN_ALT_M, MAX_ALT_M, MIN_SEP_DIST_M
from simulate import TelemetryFrame, CRUISE_ALT_M, OBSTACLE_X_NORM, OBSTACLE_ALT_M
from result_types import FlightPhase, VerificationStatus

# ---------------------------------------------------------------------------
# Display constants
# ---------------------------------------------------------------------------

WIN_W: Final[int] = 1280
WIN_H: Final[int] = 720
FPS:   Final[int] = 60

# Phosphor palette - amber CRT
C_BG:         Final[Tuple] = (5,   5,   5)
C_PANEL:      Final[Tuple] = (10,  10,  10)
C_BORDER:     Final[Tuple] = (60,  45,  0)
C_AMBER:      Final[Tuple] = (255, 176, 0)
C_AMBER_DIM:  Final[Tuple] = (140, 90,  0)
C_AMBER_DARK: Final[Tuple] = (40,  27,  0)
C_GREEN:      Final[Tuple] = (0,   255, 80)
C_GREEN_DIM:  Final[Tuple] = (0,   120, 40)
C_RED:        Final[Tuple] = (255, 50,  50)
C_RED_DIM:    Final[Tuple] = (120, 20,  20)
C_WHITE:      Final[Tuple] = (220, 220, 220)
C_GRID:       Final[Tuple] = (25,  18,  0)

# Phase label -> color
PHASE_COL = {
    FlightPhase.PREFLIGHT: C_AMBER_DIM,
    FlightPhase.TAKEOFF:   C_AMBER,
    FlightPhase.CRUISE:    C_GREEN,
    FlightPhase.AVOIDANCE: C_RED,
    FlightPhase.DESCENT:   C_AMBER,
    FlightPhase.LANDED:    C_AMBER_DIM,
}


# ---------------------------------------------------------------------------
# Pygame font helpers
# ---------------------------------------------------------------------------

def _load_mono_font(size: int) -> pygame.font.Font:
    """Load Courier New or fall back to the Pygame default monospace font."""
    for name in ("Courier New", "Courier", "monospace"):
        font = pygame.font.SysFont(name, size)
        if font is not None:
            return font
    return pygame.font.Font(None, size)


# ---------------------------------------------------------------------------
# Drawing primitives
# ---------------------------------------------------------------------------

def _text(
    surf: pygame.Surface,
    txt: str,
    x: int,
    y: int,
    font: pygame.font.Font,
    color: Tuple = C_AMBER,
    align: str = "left",
) -> None:
    """Blit a text string onto surf at (x, y) with optional alignment."""
    rendered = font.render(txt, True, color)
    if align == "center":
        x -= rendered.get_width() // 2
    elif align == "right":
        x -= rendered.get_width()
    surf.blit(rendered, (x, y))


def _hline(surf: pygame.Surface, x0: int, x1: int, y: int, color: Tuple) -> None:
    pygame.draw.line(surf, color, (x0, y), (x1, y))


def _vline(surf: pygame.Surface, x: int, y0: int, y1: int, color: Tuple) -> None:
    pygame.draw.line(surf, color, (x, y0), (x, y1))


def _rect_border(surf: pygame.Surface, rect: pygame.Rect, color: Tuple, w: int = 1) -> None:
    pygame.draw.rect(surf, color, rect, w)


def _draw_drone_icon(
    surf:    pygame.Surface,
    cx:      int,
    cy:      int,
    size:    int,
    color:   Tuple,
) -> None:
    """Draw a minimal quadcopter symbol: crossed arms + four rotor circles."""
    arm = size
    # Arms
    pygame.draw.line(surf, color, (cx - arm, cy - arm), (cx + arm, cy + arm), 2)
    pygame.draw.line(surf, color, (cx + arm, cy - arm), (cx - arm, cy + arm), 2)
    # Body dot
    pygame.draw.circle(surf, color, (cx, cy), max(2, size // 4))
    # Rotor arcs
    rotor_r = max(3, size // 2)
    for dx, dy in ((-arm, -arm), (arm, -arm), (-arm, arm), (arm, arm)):
        pygame.draw.circle(surf, color, (cx + dx, cy + dy), rotor_r, 1)


def _draw_scanlines(surf: pygame.Surface, alpha: int = 18) -> None:
    """Overlay horizontal scanlines to emulate a CRT raster."""
    scanline = pygame.Surface((WIN_W, 1), pygame.SRCALPHA)
    scanline.fill((0, 0, 0, alpha))
    for y in range(0, WIN_H, 2):
        surf.blit(scanline, (0, y))


# ---------------------------------------------------------------------------
# Panel drawers
# ---------------------------------------------------------------------------

def _draw_header(
    surf:     pygame.Surface,
    frame:    TelemetryFrame,
    fonts:    dict,
    verified: bool,
) -> None:
    """Render the top status bar."""
    pygame.draw.rect(surf, C_PANEL, pygame.Rect(0, 0, WIN_W, 40))
    _hline(surf, 0, WIN_W, 40, C_BORDER)

    title = "DRONE ALTITUDE CONTROL SYSTEM  |  GROUND CONTROL STATION"
    _text(surf, title, WIN_W // 2, 10, fonts["md"], C_AMBER, align="center")

    badge_color = C_GREEN if verified else C_RED
    badge_txt   = "[SMT-VERIFIED]" if verified else "[UNVERIFIED]"
    _text(surf, badge_txt, WIN_W - 20, 10, fonts["md"], badge_color, align="right")

    _text(surf, f"T+{frame.time_s:07.2f}s", 20, 10, fonts["md"], C_AMBER_DIM)


def _draw_telemetry_panel(
    surf:   pygame.Surface,
    frame:  TelemetryFrame,
    fonts:  dict,
    rect:   pygame.Rect,
) -> None:
    """Render the left-side numerical telemetry panel."""
    _rect_border(surf, rect, C_BORDER)
    x, y = rect.x + 10, rect.y + 10

    _text(surf, "-- TELEMETRY --", x, y, fonts["sm"], C_AMBER_DIM)
    y += 28

    rows = [
        ("ALT    ", f"{frame.alt_m:7.2f} m"),
        ("V_SPD  ", f"{frame.v_spd_mps:+7.2f} m/s"),
        ("OBS    ", f"{frame.obs_dist_m:7.2f} m"),
        ("X_NORM ", f"{frame.x_norm:7.4f}"),
        ("STEP   ", f"{frame.step:7d}"),
    ]

    for label, value in rows:
        _text(surf, label, x, y, fonts["sm"], C_AMBER_DIM)
        _text(surf, value, x + 80, y, fonts["sm"], C_AMBER)
        y += 22

    y += 12
    _hline(surf, rect.x + 8, rect.right - 8, y, C_BORDER)
    y += 12

    _text(surf, "-- SAFETY LIMITS --", x, y, fonts["sm"], C_AMBER_DIM)
    y += 28

    limit_rows = [
        ("MIN_ALT  ", f"{MIN_ALT_M:.1f} m"),
        ("MAX_ALT  ", f"{MAX_ALT_M:.1f} m"),
        ("MIN_SEP  ", f"{MIN_SEP_DIST_M:.1f} m"),
    ]
    for label, value in limit_rows:
        _text(surf, label, x, y, fonts["sm"], C_AMBER_DIM)
        _text(surf, value, x + 80, y, fonts["sm"], C_AMBER_DIM)
        y += 22

    y += 12
    _hline(surf, rect.x + 8, rect.right - 8, y, C_BORDER)
    y += 12

    phase_name  = frame.phase.name
    phase_color = PHASE_COL.get(frame.phase, C_AMBER)
    _text(surf, "PHASE  :", x, y, fonts["sm"], C_AMBER_DIM)
    _text(surf, phase_name, x + 80, y, fonts["sm"], phase_color)
    y += 22

    sep_color = C_RED if frame.obs_dist_m < MIN_SEP_DIST_M else C_GREEN
    sep_label = "WARN" if frame.obs_dist_m < MIN_SEP_DIST_M else "CLEAR"
    _text(surf, "SEP    :", x, y, fonts["sm"], C_AMBER_DIM)
    _text(surf, sep_label, x + 80, y, fonts["sm"], sep_color)


def _draw_altitude_gauge(
    surf:  pygame.Surface,
    frame: TelemetryFrame,
    fonts: dict,
    rect:  pygame.Rect,
) -> None:
    """Render a vertical altitude bar gauge with envelope markings."""
    _rect_border(surf, rect, C_BORDER)
    cx = rect.centerx
    bar_x    = cx - 18
    bar_w    = 36
    bar_top  = rect.y + 40
    bar_bot  = rect.bottom - 40
    bar_h    = bar_bot - bar_top

    _text(surf, "ALTITUDE", cx, rect.y + 12, fonts["sm"], C_AMBER_DIM, align="center")

    # Background bar
    pygame.draw.rect(surf, C_GRID, pygame.Rect(bar_x, bar_top, bar_w, bar_h))

    # Safe zone fill
    safe_top_px = int(bar_top + bar_h * (1.0 - MAX_ALT_M / 200.0))
    safe_bot_px = int(bar_top + bar_h * (1.0 - MIN_ALT_M / 200.0))
    pygame.draw.rect(
        surf, C_AMBER_DARK,
        pygame.Rect(bar_x, safe_top_px, bar_w, safe_bot_px - safe_top_px)
    )

    # Current altitude fill
    alt_frac = max(0.0, min(1.0, frame.alt_m / 200.0))
    fill_h   = int(bar_h * alt_frac)
    fill_col = C_RED if (frame.alt_m >= MAX_ALT_M or frame.alt_m <= MIN_ALT_M) else C_AMBER
    pygame.draw.rect(
        surf, fill_col,
        pygame.Rect(bar_x, bar_bot - fill_h, bar_w, fill_h)
    )

    # MAX / MIN markers
    _hline(surf, bar_x - 6, bar_x + bar_w + 6, safe_top_px, C_RED_DIM)
    _hline(surf, bar_x - 6, bar_x + bar_w + 6, safe_bot_px, C_GREEN_DIM)
    _text(surf, f"MAX {MAX_ALT_M:.0f}m", bar_x + bar_w + 8, safe_top_px - 6, fonts["xs"], C_RED_DIM)
    _text(surf, f"MIN {MIN_ALT_M:.0f}m", bar_x + bar_w + 8, safe_bot_px - 6, fonts["xs"], C_GREEN_DIM)

    # Numeric readout
    _text(surf, f"{frame.alt_m:6.1f} m", cx, bar_bot + 8, fonts["sm"], fill_col, align="center")

    # Altitude tick marks
    for tick_m in range(0, 210, 20):
        tick_frac = tick_m / 200.0
        tick_y    = int(bar_bot - bar_h * tick_frac)
        tick_len  = 8 if tick_m % 40 == 0 else 4
        _hline(surf, bar_x - tick_len, bar_x, tick_y, C_BORDER)
        if tick_m % 40 == 0:
            _text(surf, str(tick_m), bar_x - tick_len - 24, tick_y - 6, fonts["xs"], C_AMBER_DIM)


def _draw_mission_map(
    surf:   pygame.Surface,
    frames: List[TelemetryFrame],
    idx:    int,
    fonts:  dict,
    rect:   pygame.Rect,
) -> None:
    """Render a top-down mission progress strip with trail and obstacle marker."""
    _rect_border(surf, rect, C_BORDER)
    _text(surf, "MISSION PROFILE", rect.centerx, rect.y + 10, fonts["sm"], C_AMBER_DIM, align="center")

    strip_y  = rect.centery
    strip_x0 = rect.x + 20
    strip_x1 = rect.right - 20
    strip_w  = strip_x1 - strip_x0

    # Ground line
    _hline(surf, strip_x0, strip_x1, strip_y, C_AMBER_DIM)

    # Altitude trace (up to current frame)
    alt_scale = (rect.height // 2 - 30) / MAX_ALT_M
    pts = []
    step = max(1, idx // 200)
    for f in frames[::step]:
        if f.step > idx:
            break
        px = int(strip_x0 + f.x_norm * strip_w)
        py = int(strip_y - f.alt_m * alt_scale)
        pts.append((px, py))
    if len(pts) >= 2:
        pygame.draw.lines(surf, C_AMBER_DIM, False, pts, 1)

    # Obstacle marker
    obs_px = int(strip_x0 + OBSTACLE_X_NORM * strip_w)
    obs_h  = int(OBSTACLE_ALT_M * alt_scale)
    pygame.draw.rect(surf, C_RED_DIM, pygame.Rect(obs_px - 3, strip_y - obs_h, 6, obs_h))
    _text(surf, "OBS", obs_px, strip_y - obs_h - 16, fonts["xs"], C_RED_DIM, align="center")

    # MIN / MAX envelope lines
    _hline(surf, strip_x0, strip_x1, int(strip_y - MIN_ALT_M * alt_scale), C_GREEN_DIM)
    _hline(surf, strip_x0, strip_x1, int(strip_y - MAX_ALT_M * alt_scale), C_RED_DIM)

    # Current drone position
    frame   = frames[idx]
    drone_x = int(strip_x0 + frame.x_norm * strip_w)
    drone_y = int(strip_y - frame.alt_m * alt_scale)
    drone_col = PHASE_COL.get(frame.phase, C_AMBER)
    _draw_drone_icon(surf, drone_x, drone_y, 8, drone_col)


def _draw_log_strip(
    surf:   pygame.Surface,
    frames: List[TelemetryFrame],
    idx:    int,
    fonts:  dict,
    rect:   pygame.Rect,
    log:    List[str],
) -> None:
    """Render a scrolling telemetry log at the bottom of the screen."""
    _rect_border(surf, rect, C_BORDER)
    max_lines = rect.height // 16

    # Build log string for current frame
    f = frames[idx]
    entry = (
        f"[{f.time_s:07.2f}s]  "
        f"PHASE={f.phase.name:<9}  "
        f"ALT={f.alt_m:6.2f}m  "
        f"OBS={f.obs_dist_m:6.2f}m  "
        f"VSPD={f.v_spd_mps:+5.2f}m/s"
    )
    if not log or log[-1] != entry:
        log.append(entry)

    visible = log[-max_lines:]
    y = rect.y + 4
    for i, line in enumerate(visible):
        alpha = 0.4 + 0.6 * (i / max(1, len(visible) - 1))
        color = (
            int(C_AMBER[0] * alpha),
            int(C_AMBER[1] * alpha),
            int(C_AMBER[2] * alpha),
        )
        _text(surf, line, rect.x + 8, y, fonts["xs"], color)
        y += 16


# ---------------------------------------------------------------------------
# Main GCS entry point
# ---------------------------------------------------------------------------

def run_gcs(frames: List[TelemetryFrame], smt_verified: bool) -> None:
    """Launch the Pygame GCS display and animate the telemetry frames.

    Args:
        frames:       Ordered telemetry frames from ``simulate.run_simulation()``.
        smt_verified: Overall Z3 verification status flag.
    """
    pygame.init()
    pygame.display.set_caption("Drone ACS - Ground Control Station")
    surf = pygame.display.set_mode((WIN_W, WIN_H))
    clock = pygame.time.Clock()

    fonts = {
        "lg": _load_mono_font(22),
        "md": _load_mono_font(15),
        "sm": _load_mono_font(12),
        "xs": _load_mono_font(10),
    }

    # Panel geometry
    HEADER_H  = 42
    LOG_H     = 80
    BODY_TOP  = HEADER_H + 2
    BODY_BOT  = WIN_H - LOG_H - 2
    BODY_H    = BODY_BOT - BODY_TOP

    TEL_W     = 220
    GAUGE_W   = 120
    MAP_X     = TEL_W + GAUGE_W + 6
    MAP_W     = WIN_W - MAP_X

    rect_tel   = pygame.Rect(0,     BODY_TOP, TEL_W,  BODY_H)
    rect_gauge = pygame.Rect(TEL_W, BODY_TOP, GAUGE_W, BODY_H)
    rect_map   = pygame.Rect(MAP_X, BODY_TOP, MAP_W,  BODY_H)
    rect_log   = pygame.Rect(0,     BODY_BOT, WIN_W,  LOG_H)

    idx:   int        = 0
    paused: bool      = False
    log:   List[str]  = []

    # Playback speed: advance N frames per render tick
    SPEED: int = 1

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused

        surf.fill(C_BG)

        _draw_header(surf, frames[idx], fonts, smt_verified)
        _draw_telemetry_panel(surf, frames[idx], fonts, rect_tel)
        _draw_altitude_gauge(surf, frames[idx], fonts, rect_gauge)
        _draw_mission_map(surf, frames, idx, fonts, rect_map)
        _draw_log_strip(surf, frames, idx, fonts, rect_log, log)
        _draw_scanlines(surf)

        _draw_scanlines(surf)

        pygame.display.flip()
        clock.tick(FPS)

        if not paused:
            idx = min(idx + SPEED, len(frames) - 1)
            if idx == len(frames) - 1:
                # Hold on last frame
                pass

    pygame.quit()
