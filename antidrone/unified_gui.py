#!/usr/bin/env python3
"""
unified_gui.py  -  Anti-Drone Unified Display

Layout:
    Left Top    : Camera feed from ptcamera_tracker (UDP 9998, JPEG)
    Left Bottom : Radar PPI from ptcamera_tracker (UDP 9999, [RADAR] text)
    Right Top   : Information / Control / Telemetry panel
    Right Bottom: 3D spatial plot

Control:
    display-only.
    ptcamera_tracker owns the hardware port and relays data over UDP.

Usage:
    python unified_gui.py
"""

import io
import json
import math
import os
import re
import socket
import sys
import time

import numpy as np
import pygame

# ============================================================
#  Layout
# ============================================================
BASE_LEFT_W = 800
BASE_SIDE_W = 400
BASE_CAM_W, BASE_CAM_H = 800, 450  # 16:9 camera panel
BASE_PPI_W, BASE_PPI_H = 800, 450  # radar PPI panel
BASE_WIN_W = BASE_LEFT_W + BASE_SIDE_W
BASE_WIN_H = BASE_CAM_H + BASE_PPI_H

LEFT_W = BASE_LEFT_W
SIDE_W = BASE_SIDE_W
CAM_W, CAM_H = BASE_CAM_W, BASE_CAM_H
PPI_W, PPI_H = BASE_PPI_W, BASE_PPI_H
WIN_W, WIN_H = BASE_WIN_W, BASE_WIN_H
LAYOUT_SCALE = 1.0

# Radar geometry
MAX_RANGE = 8000
FOV = 120
CX = PPI_W // 2
CY = PPI_H - 40
MAX_PX = min(PPI_W // 2 - 50, PPI_H - 80)

MAX_STARS = 3
STAR_RADIUS = 30

UDP_CAM_PORT = 9998
UDP_PPI_PORT = 9999
UDP_TELEM_PORT = 10000

Z_MAX_RANGE = 4000


def configure_layout(display_w, display_h):
    """Fit the fixed display layout inside the current laptop screen."""
    global LEFT_W, SIDE_W, CAM_W, CAM_H, PPI_W, PPI_H, WIN_W, WIN_H
    global CX, CY, MAX_PX, LAYOUT_SCALE

    # Leave room for the window title bar and taskbar on small laptop screens.
    usable_w = max(900, display_w - 40)
    usable_h = max(600, display_h - 80)
    LAYOUT_SCALE = min(1.0, usable_w / BASE_WIN_W, usable_h / BASE_WIN_H)

    CAM_W = max(560, int(BASE_CAM_W * LAYOUT_SCALE))
    CAM_H = max(315, int(BASE_CAM_H * LAYOUT_SCALE))
    PPI_W = CAM_W
    PPI_H = CAM_H
    LEFT_W = CAM_W
    SIDE_W = max(300, int(BASE_SIDE_W * LAYOUT_SCALE))
    WIN_W = LEFT_W + SIDE_W
    WIN_H = CAM_H + PPI_H

    CX = PPI_W // 2
    CY = PPI_H - max(28, int(40 * LAYOUT_SCALE))
    MAX_PX = min(
        PPI_W // 2 - max(35, int(50 * LAYOUT_SCALE)),
        PPI_H - max(60, int(80 * LAYOUT_SCALE)),
    )


# ============================================================
#  Slider
# ============================================================
class Slider:
    def __init__(self, x, y, w, label, lo, hi, val, fmt="{:.3f}"):
        self.rect = pygame.Rect(x, y, w, 8)
        self.label = label
        self.lo, self.hi, self.value = lo, hi, val
        self.fmt = fmt
        self.drag = False

    def draw(self, surf, font):
        pygame.draw.rect(surf, (48, 58, 66), self.rect, border_radius=4)

        r = (self.value - self.lo) / (self.hi - self.lo)
        r = max(0.0, min(1.0, r))

        filled = pygame.Rect(
            self.rect.x,
            self.rect.y,
            int(self.rect.w * r),
            self.rect.h,
        )
        pygame.draw.rect(surf, (0, 190, 120), filled, border_radius=4)

        hx = self.rect.x + int(self.rect.w * r)
        pygame.draw.circle(surf, (210, 255, 230), (hx, self.rect.centery), 8)
        pygame.draw.circle(surf, (0, 130, 85), (hx, self.rect.centery), 8, 1)

        surf.blit(
            font.render(
                f"{self.label}: {self.fmt.format(self.value)}",
                True,
                (178, 196, 202),
            ),
            (self.rect.x, self.rect.y - 18),
        )

    def handle_event(self, event, x_off=0):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            lx = event.pos[0] - x_off
            hx = self.rect.x + int(
                self.rect.w * (self.value - self.lo) / (self.hi - self.lo)
            )

            if math.hypot(lx - hx, event.pos[1] - self.rect.centery) < 12:
                self.drag = True

        if event.type == pygame.MOUSEBUTTONUP:
            self.drag = False

        if event.type == pygame.MOUSEMOTION and self.drag:
            r = (event.pos[0] - x_off - self.rect.x) / self.rect.w
            r = max(0.0, min(1.0, r))
            self.value = self.lo + r * (self.hi - self.lo)


# ============================================================
#  Heatmap kernel
# ============================================================
def _make_gauss(size=11, sigma=2.5):
    k = size // 2
    y, x = np.mgrid[-k : k + 1, -k : k + 1]
    g = np.exp(-(x**2 + y**2) / (2 * sigma**2))
    return (g / g.max() * 255).astype(np.float32)


GAUSS = _make_gauss()
GAUSS_R = len(GAUSS) // 2


# ============================================================
#  Helpers
# ============================================================
def to_screen(x_mm, y_mm):
    return (
        int(CX + (x_mm / MAX_RANGE) * MAX_PX),
        int(CY - (y_mm / MAX_RANGE) * MAX_PX),
    )


def heat_centroid(heat, cx, cy, radius=40):
    x0, x1 = max(0, cx - radius), min(PPI_W, cx + radius)
    y0, y1 = max(0, cy - radius), min(PPI_H, cy + radius)

    region = heat[x0:x1, y0:y1]

    if region.size == 0 or region.max() < 15:
        return None, None

    total = region.sum()
    if total <= 0:
        return None, None

    xs = np.arange(x0, x1)
    ys = np.arange(y0, y1)

    return (
        int((region.sum(axis=1) @ xs) / total),
        int((region.sum(axis=0) @ ys) / total),
    )


def _first_number(info, keys):
    for key in keys:
        if key not in info:
            continue
        try:
            return float(info[key])
        except (TypeError, ValueError):
            continue
    return None


def telemetry_z_mm(track_info):
    return _first_number(
        track_info,
        (
            "z_mm",
            "altitude_mm",
            "height_mm",
            "world_z_mm",
            "target_z_mm",
        ),
    )


def target_z_mm(t, track_info=None, tid=None, target_count=0):
    z = _first_number(
        t,
        (
            "z",
            "z_mm",
            "altitude_mm",
            "height_mm",
            "world_z_mm",
            "target_z_mm",
        ),
    )
    if z is not None:
        return z, True

    track_info = track_info or {}
    tracked_id = track_info.get("target_id")
    try:
        tracked_id = int(tracked_id) if tracked_id is not None else None
    except (TypeError, ValueError):
        pass
    use_tracker_z = tracked_id == tid or (tracked_id is None and target_count <= 1)
    if use_tracker_z:
        z = telemetry_z_mm(track_info)
        if z is not None:
            return z, True

    return 0.0, False


def parse_optional_z_mm(line):
    m = re.search(r"\[RADAR\]\s*T\d+:\(-?\d+,-?\d+,(-?\d+)\)mm", line)
    if m:
        return float(m.group(1))

    m = re.search(
        r"\b(?:z|alt|altitude|height)=(-?\d+(?:\.\d+)?)\s*(mm|m)?\b",
        line,
        re.IGNORECASE,
    )
    if not m:
        return None

    z = float(m.group(1))
    unit = (m.group(2) or "mm").lower()
    return z * 1000.0 if unit == "m" else z


def project_space_point(x_mm, y_mm, z_mm, rect):
    scale = min(rect.w * 0.34 / MAX_RANGE, rect.h * 0.30 / MAX_RANGE)
    z_scale = rect.h * 0.24 / Z_MAX_RANGE

    cx = rect.centerx
    cy = rect.y + int(rect.h * 0.68)
    px = cx + (x_mm - y_mm * 0.36) * scale
    py = cy - y_mm * scale * 0.52 - z_mm * z_scale
    return int(px), int(py)


def load_ui_font(candidates, size, bold=False):
    for name in candidates:
        path = pygame.font.match_font(name, bold=bold)
        if path:
            return pygame.font.Font(path, size)
    return pygame.font.SysFont(candidates[-1], size, bold=bold)


def build_ui_fonts():
    base = max(10, int(12 * LAYOUT_SCALE))
    return {
        "title": load_ui_font(("consolas",), max(17, int(22 * LAYOUT_SCALE)), True),
        "section": load_ui_font(("consolas",), max(11, int(13 * LAYOUT_SCALE)), True),
        "body": load_ui_font(("consolas",), max(10, int(12 * LAYOUT_SCALE))),
        "small": load_ui_font(("consolas",), max(9, int(10 * LAYOUT_SCALE))),
        "mono": load_ui_font(("consolas",), base),
        "mono_bold": load_ui_font(("consolas",), base, True),
    }


def draw_text(surf, font, text, color, pos, max_w=None):
    if max_w is None:
        surf.blit(font.render(text, True, color), pos)
        return

    clipped = text
    while clipped and font.size(clipped)[0] > max_w:
        clipped = clipped[:-1]
    if clipped != text and len(clipped) > 1:
        clipped = clipped[:-1] + "."
    surf.blit(font.render(clipped, True, color), pos)


def draw_panel_box(surf, rect, fill, border=(40, 52, 61), radius=6):
    pygame.draw.rect(surf, fill, rect, border_radius=radius)
    pygame.draw.rect(surf, border, rect, 1, border_radius=radius)


def draw_pill(surf, font, rect, label, color, fill=(14, 24, 28)):
    pygame.draw.rect(surf, fill, rect, border_radius=6)
    pygame.draw.rect(surf, color, rect, 1, border_radius=6)
    txt = font.render(label, True, color)
    surf.blit(txt, (rect.centerx - txt.get_width() // 2, rect.centery - txt.get_height() // 2))


def draw_metric_tile(surf, fonts, rect, label, value, color):
    draw_panel_box(surf, rect, (15, 23, 28), (43, 57, 66), 6)
    draw_text(surf, fonts["small"], label, (126, 146, 154), (rect.x + 8, rect.y + 6), rect.w - 16)
    draw_text(surf, fonts["mono_bold"], value, color, (rect.x + 8, rect.y + 22), rect.w - 16)


def draw_section_label(surf, fonts, text, x, y, w):
    pygame.draw.line(surf, (42, 56, 64), (x, y + 9), (x + w, y + 9), 1)
    label = fonts["section"].render(text, True, (0, 220, 140))
    pygame.draw.rect(surf, (8, 12, 16), (x, y, label.get_width() + 10, label.get_height()))
    surf.blit(label, (x, y))


def draw_star(surf, cx, cy, color, size=14):
    pts = [
        (
            cx
            + (size if i % 2 == 0 else size * 0.45)
            * math.cos(math.radians(i * 36 - 90)),
            cy
            + (size if i % 2 == 0 else size * 0.45)
            * math.sin(math.radians(i * 36 - 90)),
        )
        for i in range(10)
    ]

    pygame.draw.polygon(surf, color, pts)
    pygame.draw.polygon(surf, (255, 255, 0), pts, 2)


# ============================================================
#  Camera panel renderer
# ============================================================
def render_camera_panel(surf, font, font_s, cam_surf):
    surf.fill((8, 10, 12))

    if cam_surf is not None:
        surf.blit(cam_surf, (0, 0))
        sig_txt, sig_col = "LIVE", (0, 230, 120)
    else:
        pygame.draw.rect(surf, (16, 20, 24), (0, 0, CAM_W, CAM_H))
        pygame.draw.rect(surf, (54, 65, 72), (0, 0, CAM_W, CAM_H), 2)

        ns = font.render("NO SIGNAL", True, (91, 104, 112))
        surf.blit(
            ns,
            (
                CAM_W // 2 - ns.get_width() // 2,
                CAM_H // 2 - ns.get_height() // 2,
            ),
        )

        sig_txt, sig_col = "WAITING...", (230, 130, 55)

    # Header overlay
    pygame.draw.rect(surf, (0, 0, 0), (0, 0, CAM_W, 30))
    pygame.draw.line(surf, (42, 54, 62), (0, 29), (CAM_W, 29), 1)
    surf.blit(font.render("CAMERA FEED", True, (0, 220, 140)), (10, 7))
    surf.blit(font_s.render(sig_txt, True, sig_col), (130, 9))

    # Border
    pygame.draw.rect(surf, (42, 54, 62), (0, 0, CAM_W, CAM_H), 1)


# ============================================================
#  PPI panel renderer
# ============================================================
def render_ppi_panel(surf, font, font_s, stars, targets, heat, half_fov):
    surf.fill((0, 4, 6))

    hu8 = heat.astype(np.uint8)
    heat_rgb = np.stack([hu8 // 2, hu8, hu8], axis=-1)
    surf.blit(pygame.surfarray.make_surface(heat_rgb), (0, 0))

    # Range arcs
    for r_mm in [2000, 4000, 6000, 8000]:
        r_px = int((r_mm / MAX_RANGE) * MAX_PX)
        rect = pygame.Rect(CX - r_px, CY - r_px, r_px * 2, r_px * 2)

        pygame.draw.arc(
            surf,
            (38, 74, 62),
            rect,
            math.radians(210),
            math.radians(330),
            1,
        )

        lx = int(CX + r_px * math.sin(half_fov)) + 5
        ly = int(CY - r_px * math.cos(half_fov))
        surf.blit(
            font_s.render(f"{r_mm // 1000}m", True, (80, 180, 135)),
            (lx, ly),
        )

    # FOV boundary lines
    for sign in [-1, 1]:
        rad = sign * half_fov
        ex = int(CX + MAX_PX * math.sin(rad))
        ey = int(CY - MAX_PX * math.cos(rad))
        pygame.draw.line(surf, (0, 125, 75), (CX, CY), (ex, ey), 1)

    # Angle guide lines
    for deg in [-60, -30, 0, 30, 60]:
        rad = math.radians(deg)
        ex = int(CX + MAX_PX * math.sin(rad))
        ey = int(CY - MAX_PX * math.cos(rad))

        pygame.draw.line(
            surf,
            (0, 210, 120) if deg == 0 else (0, 82, 52),
            (CX, CY),
            (ex, ey),
            1,
        )

        surf.blit(
            font_s.render(f"{deg}deg", True, (82, 170, 122)),
            (ex - 18, ey - 22),
        )

    # Radar origin
    pygame.draw.circle(surf, (0, 255, 130), (CX, CY), 5)

    pygame.draw.rect(surf, (0, 0, 0), (0, 0, PPI_W, 30))
    pygame.draw.line(surf, (42, 54, 62), (0, 29), (PPI_W, 29), 1)
    surf.blit(font.render("RADAR PPI", True, (0, 220, 140)), (10, 7))

    # Assigned stars
    for star in stars:
        sx, sy = star["pos"]
        draw_star(surf, sx, sy, (255, 60, 60))
        surf.blit(
            font_s.render(f"T{star['id']}", True, (255, 190, 40)),
            (sx + 16, sy - 8),
        )

    # Raw targets
    for tid, t in targets.items():
        rx, ry = to_screen(t["x"], t["y"])

        if not (0 <= rx < PPI_W and 0 <= ry < PPI_H):
            continue

        pygame.draw.circle(surf, (0, 220, 255), (rx, ry), 5)
        surf.blit(
            font_s.render(f"RAW_{tid}", True, (0, 220, 255)),
            (rx + 8, ry - 8),
        )

    # Border
    pygame.draw.rect(surf, (42, 54, 62), (0, 0, PPI_W, PPI_H), 1)


# ============================================================
#  3D spatial panel renderer
# ============================================================
def render_space_panel(surf, font, font_s, targets, track_info):
    surf.fill((8, 10, 12))
    panel_w, panel_h = surf.get_size()
    rect = pygame.Rect(12, 42, panel_w - 24, panel_h - 78)

    pygame.draw.rect(surf, (0, 0, 0), (0, 0, panel_w, 30))
    pygame.draw.line(surf, (42, 54, 62), (0, 29), (panel_w, 29), 1)
    surf.blit(font.render("3D SPACE", True, (0, 220, 140)), (10, 7))

    any_z_live = any(
        target_z_mm(t, track_info, tid, len(targets))[1]
        for tid, t in targets.items()
    )

    if not any_z_live:
        surf.blit(
            font_s.render("Z: CALIB PENDING", True, (180, 140, 60)),
            (panel_w - 145, 9),
        )
    else:
        surf.blit(
            font_s.render("Z: LIVE", True, (0, 230, 120)),
            (panel_w - 78, 9),
        )

    grid_col = (28, 70, 74)
    axis_col = (0, 180, 120)
    z_col = (100, 120, 220)
    label_col = (100, 150, 150)

    # Ground grid in radar/world coordinates: x is lateral, y is forward.
    xs = [-8000, -4000, 0, 4000, 8000]
    ys = [0, 2000, 4000, 6000, 8000]

    for x_mm in xs:
        p0 = project_space_point(x_mm, 0, 0, rect)
        p1 = project_space_point(x_mm, MAX_RANGE, 0, rect)
        pygame.draw.line(surf, grid_col, p0, p1, 1)

    for y_mm in ys:
        p0 = project_space_point(-MAX_RANGE, y_mm, 0, rect)
        p1 = project_space_point(MAX_RANGE, y_mm, 0, rect)
        pygame.draw.line(surf, grid_col, p0, p1, 1)
        label = f"{y_mm // 1000}m"
        lp = project_space_point(-MAX_RANGE, y_mm, 0, rect)
        surf.blit(font_s.render(label, True, label_col), (lp[0] + 4, lp[1] - 10))

    origin = project_space_point(0, 0, 0, rect)
    x_axis = project_space_point(5000, 0, 0, rect)
    y_axis = project_space_point(0, 5000, 0, rect)
    z_axis = project_space_point(0, 0, Z_MAX_RANGE, rect)
    pygame.draw.line(surf, axis_col, origin, x_axis, 2)
    pygame.draw.line(surf, axis_col, origin, y_axis, 2)
    pygame.draw.line(surf, z_col, origin, z_axis, 2)
    surf.blit(font_s.render("X", True, axis_col), (x_axis[0] + 5, x_axis[1] - 8))
    surf.blit(font_s.render("Y", True, axis_col), (y_axis[0] + 5, y_axis[1] - 8))
    surf.blit(font_s.render("Z", True, z_col), (z_axis[0] + 5, z_axis[1] - 8))

    if not targets:
        msg = font.render("NO TARGET", True, (70, 82, 88))
        surf.blit(msg, (panel_w // 2 - msg.get_width() // 2, rect.centery))
    else:
        target_count = len(targets)
        for i, (tid, t) in enumerate(targets.items()):
            z_mm, z_valid = target_z_mm(t, track_info, tid, target_count)
            ground = project_space_point(t["x"], t["y"], 0, rect)
            point = project_space_point(t["x"], t["y"], z_mm, rect)

            pygame.draw.line(surf, (64, 70, 96), ground, point, 1)
            pygame.draw.circle(surf, (82, 100, 112), ground, 4, 1)
            dot_col = (0, 225, 255) if z_valid else (255, 185, 70)
            pygame.draw.circle(surf, dot_col, point, 6)
            pygame.draw.circle(surf, (220, 255, 255), point, 9, 1)

            dist_m = t["dist"] / 1000.0
            z_m = z_mm / 1000.0
            label = f"T{tid}  R{dist_m:.1f}m"
            sublabel = f"Z {z_m:.1f}m"
            label_x = max(10, min(panel_w - 116, point[0] + 10))
            label_y = max(34, min(panel_h - 74, point[1] - 13 + i * 18))
            label_rect = pygame.Rect(label_x, label_y, 104, 36)
            pygame.draw.rect(surf, (12, 20, 24), label_rect, border_radius=5)
            pygame.draw.rect(surf, (48, 66, 76), label_rect, 1, border_radius=5)
            surf.blit(font_s.render(label, True, (190, 230, 220)), (label_x + 6, label_y + 4))
            surf.blit(font_s.render(sublabel, True, dot_col), (label_x + 6, label_y + 19))

    footer_y = panel_h - 52
    pygame.draw.line(surf, (36, 48, 56), (12, footer_y), (panel_w - 12, footer_y), 1)
    footer = "x/y: radar mm   z: telemetry z_mm/altitude_mm"
    surf.blit(font_s.render(footer, True, (105, 126, 132)), (14, footer_y + 10))
    surf.blit(
        font_s.render("calibration can replace skeleton z later", True, (105, 126, 132)),
        (14, footer_y + 28),
    )

    pygame.draw.rect(surf, (42, 54, 62), (0, 0, panel_w, panel_h), 1)


# ============================================================
#  Right side information / control panel renderer
# ============================================================
def render_side_panel(
    surf,
    font,
    font_s,
    sliders,
    targets,
    link_status,
    track_info,
    ui_fonts=None,
):
    fonts = ui_fonts or {
        "title": font,
        "section": font,
        "body": font_s,
        "small": font_s,
        "mono": font_s,
        "mono_bold": font_s,
    }
    panel_h = surf.get_height()
    panel_w = surf.get_width()
    surf.fill((8, 12, 16))

    x = 15
    y = 13
    content_w = panel_w - 30
    accent = (0, 220, 140)
    muted = (126, 148, 156)
    text = (215, 238, 235)

    pygame.draw.rect(surf, (6, 22, 24), (0, 0, panel_w, 64))
    pygame.draw.line(surf, (0, 95, 78), (0, 63), (panel_w, 63), 1)
    pygame.draw.line(surf, (42, 54, 62), (0, 0), (0, panel_h), 1)

    draw_text(surf, fonts["title"], "ANTI-DRONE", (225, 255, 246), (x, y), content_w - 110)
    draw_text(surf, fonts["small"], "Unified display", (126, 154, 160), (x + 2, y + 27), content_w)

    link_col = (0, 230, 120) if link_status == "UDP" else (255, 80, 80)
    draw_pill(
        surf,
        fonts["small"],
        pygame.Rect(panel_w - 86, y + 7, 70, 24),
        link_status,
        link_col,
        (12, 28, 30),
    )

    y = 78
    if track_info:
        enabled = track_info.get("motor_enabled", False)
        found = track_info.get("target_found", False)
        fps = track_info.get("fps", 0.0)

        bbox_ex = track_info.get("bbox_ex", 0)
        bbox_ey = track_info.get("bbox_ey", 0)
        cx = track_info.get("center_x", 0)
        cy = track_info.get("center_y", 0)
        conf = track_info.get("confidence", 0.0)
        motor_val = "ON" if enabled else "OFF"
        target_val = "LOCK" if found else "NONE"
        motor_col = (0, 230, 120) if enabled else (230, 160, 70)
        target_col = (0, 230, 120) if found else (150, 160, 110)
    else:
        fps = 0.0
        bbox_ex = 0
        bbox_ey = 0
        cx = 0
        cy = 0
        conf = 0.0
        motor_val = "--"
        target_val = "WAIT"
        motor_col = (130, 140, 146)
        target_col = (230, 160, 70)

    gap = 7
    tile_w = (content_w - gap * 2) // 3
    tile_h = 48
    draw_metric_tile(
        surf,
        fonts,
        pygame.Rect(x, y, tile_w, tile_h),
        "MOTOR",
        motor_val,
        motor_col,
    )
    draw_metric_tile(
        surf,
        fonts,
        pygame.Rect(x + tile_w + gap, y, tile_w, tile_h),
        "TARGET",
        target_val,
        target_col,
    )
    draw_metric_tile(
        surf,
        fonts,
        pygame.Rect(x + (tile_w + gap) * 2, y, content_w - (tile_w + gap) * 2, tile_h),
        "FPS",
        f"{fps:.1f}",
        (120, 200, 255),
    )
    y += tile_h + 18

    draw_section_label(surf, fonts, "TRACKER", x, y, content_w)
    y += 22

    tracker_rect = pygame.Rect(x, y, content_w, 72)
    draw_panel_box(surf, tracker_rect, (12, 18, 23), (43, 57, 66), 6)
    left_x = tracker_rect.x + 10
    right_x = tracker_rect.centerx + 4
    draw_text(
        surf,
        fonts["small"],
        "BBOX ERROR",
        muted,
        (left_x, tracker_rect.y + 9),
        tracker_rect.w // 2 - 20,
    )
    draw_text(
        surf,
        fonts["mono_bold"],
        f"{bbox_ex:+d}, {bbox_ey:+d}",
        text,
        (left_x, tracker_rect.y + 28),
        tracker_rect.w // 2 - 20,
    )
    draw_text(
        surf,
        fonts["small"],
        "CENTER",
        muted,
        (right_x, tracker_rect.y + 9),
        tracker_rect.w // 2 - 18,
    )
    draw_text(
        surf,
        fonts["mono_bold"],
        f"{cx:.0f}, {cy:.0f}",
        text,
        (right_x, tracker_rect.y + 28),
        tracker_rect.w // 2 - 18,
    )
    conf_y = tracker_rect.y + 55
    conf_w = tracker_rect.w - 88
    draw_text(surf, fonts["small"], "CONF", muted, (left_x, conf_y - 3), 42)
    pygame.draw.rect(surf, (36, 46, 52), (left_x + 43, conf_y, conf_w, 6), border_radius=3)
    pygame.draw.rect(
        surf,
        accent,
        (left_x + 43, conf_y, int(conf_w * max(0.0, min(1.0, conf))), 6),
        border_radius=3,
    )
    draw_text(
        surf,
        fonts["mono"],
        f"{conf:.2f}",
        (190, 230, 218),
        (left_x + 50 + conf_w, conf_y - 7),
        45,
    )
    y += tracker_rect.h + 16

    draw_section_label(surf, fonts, "CONTROL", x, y, content_w)
    y += 26

    for s in sliders:
        s.rect.x = x
        s.rect.y = y + 17
        s.rect.w = content_w
        s.draw(surf, fonts["small"])
        y += 44

    draw_section_label(surf, fonts, "RADAR", x, y, content_w)
    y += 22

    if not targets:
        empty_rect = pygame.Rect(x, y, content_w, 38)
        draw_panel_box(surf, empty_rect, (12, 18, 23), (43, 57, 66), 6)
        draw_text(
            surf,
            fonts["small"],
            "No targets detected",
            (110, 126, 132),
            (empty_rect.x + 10, empty_rect.y + 11),
            empty_rect.w - 20,
        )
    else:
        row_h = 56
        max_rows = max(1, min(3, (panel_h - y - 10) // (row_h + 6)))
        for i, (tid, t) in enumerate(targets.items()):
            if i >= max_rows:
                draw_text(surf, fonts["small"], "... more targets", muted, (x + 4, y + 4), content_w)
                break

            row = pygame.Rect(x, y, content_w, row_h)
            draw_panel_box(surf, row, (12, 18, 23), (43, 57, 66), 6)
            dist_m = t["dist"] / 1000.0
            angle_deg = math.degrees(math.atan2(t["x"], t["y"]))
            z_mm, z_valid = target_z_mm(t, track_info, tid, len(targets))
            z_col = (120, 200, 255) if z_valid else (230, 160, 70)
            z_label = f"{z_mm / 1000.0:.1f}m" if z_valid else "pending"

            draw_text(surf, fonts["mono_bold"], f"T{tid}", (245, 250, 250), (row.x + 9, row.y + 9), 42)
            draw_text(
                surf,
                fonts["mono"],
                f"R {dist_m:.1f}m   A {angle_deg:+.1f}",
                (194, 232, 220),
                (row.x + 50, row.y + 9),
                row.w - 60,
            )
            draw_text(
                surf,
                fonts["mono"],
                f"V {t.get('speed', 0):.1f}m/s",
                (244, 172, 172),
                (row.x + 50, row.y + 30),
                row.w // 2,
            )
            draw_text(
                surf,
                fonts["mono"],
                f"Z {z_label}",
                z_col,
                (row.centerx + 20, row.y + 30),
                row.w // 2 - 28,
            )
            y += row_h + 6

    pygame.draw.rect(surf, (42, 54, 62), (0, 0, panel_w, panel_h), 1)


# ============================================================
#  Main
# ============================================================
def main():
    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
    pygame.init()

    display_info = pygame.display.Info()
    configure_layout(display_info.current_w, display_info.current_h)

    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Anti-Drone Unified Display")

    clock = pygame.time.Clock()
    ui_fonts = build_ui_fonts()
    font = ui_fonts["mono_bold"]
    font_s = ui_fonts["mono"]

    print(
        "[OK] Display-only GUI. "
        "ptcamera_tracker owns the hardware port; GUI listens on UDP only."
    )

    # ── Surfaces ───────────────────────────────────────────
    cam_panel = pygame.Surface((CAM_W, CAM_H))
    ppi_surf = pygame.Surface((PPI_W, PPI_H))
    info_surf = pygame.Surface((SIDE_W, CAM_H))
    space_surf = pygame.Surface((SIDE_W, PPI_H))

    # ── PPI state ──────────────────────────────────────────
    heat = np.zeros((PPI_W, PPI_H), dtype=np.float32)
    stars = []
    targets = {}

    link_status = "OFFLINE"
    last_radar_udp_time = 0.0

    half_fov = math.radians(FOV / 2)
    pattern = r"\[RADAR\]\s*T(\d+):\((-?\d+),(-?\d+)(?:,(-?\d+))?\)mm"

    # Slider coordinates are local to right side panel.
    sliders = [
        Slider(
            15,
            190,
            SIDE_W - 30,
            "Afterglow Decay",
            0.80,
            0.999,
            0.985,
        )
    ]
    decay_s = sliders[0]

    # ── Camera state ───────────────────────────────────────
    cam_surf = None

    # ── Telemetry state ────────────────────────────────────
    track_info = {}

    # ── UDP sockets ────────────────────────────────────────
    ppi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ppi_sock.bind(("127.0.0.1", UDP_PPI_PORT))
    ppi_sock.setblocking(False)

    cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cam_sock.bind(("127.0.0.1", UDP_CAM_PORT))
    cam_sock.setblocking(False)

    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telem_sock.bind(("127.0.0.1", UDP_TELEM_PORT))
    telem_sock.setblocking(False)

    print(
        f"[OK] UDP camera:{UDP_CAM_PORT}  radar:{UDP_PPI_PORT}  "
        f"telemetry:{UDP_TELEM_PORT}  window:{WIN_W}x{WIN_H}"
    )

    def shutdown():
        ppi_sock.close()
        cam_sock.close()
        telem_sock.close()
        pygame.quit()
        sys.exit()

    def process_radar(line, now):
        m = re.search(pattern, line)

        if not m:
            return

        tid = int(m.group(1))
        x_mm = int(m.group(2))
        y_mm = int(m.group(3))
        z_mm = parse_optional_z_mm(line)

        if x_mm == 0 and y_mm == 0:
            return

        if not (0 <= y_mm <= MAX_RANGE):
            return

        dist = math.hypot(x_mm, y_mm)
        speed = 0.0

        if tid in targets:
            prev = targets[tid]
            dt = now - prev["time"]

            if dt > 0.05:
                dz_m = 0.0
                if z_mm is not None and "z" in prev:
                    dz_m = (z_mm - prev["z"]) / 1000
                speed = (
                    math.hypot(
                        (x_mm - prev["x"]) / 1000,
                        (y_mm - prev["y"]) / 1000,
                        dz_m,
                    )
                    / dt
                )

        target = {
            "x": x_mm,
            "y": y_mm,
            "dist": dist,
            "speed": speed,
            "time": now,
        }
        if z_mm is not None:
            target["z"] = z_mm
        elif tid in targets and "z" in targets[tid]:
            target["z"] = targets[tid]["z"]

        targets[tid] = target

    # ── Main loop ──────────────────────────────────────────
    while True:
        now = time.time()

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_q
            ):
                shutdown()

            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                stars.clear()
                heat[:] = 0

            # Sliders are inside the right side panel.
            for s in sliders:
                s.handle_event(event, x_off=LEFT_W)

            # PPI click area: left-bottom panel.
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos

                if 0 <= mx < PPI_W and CAM_H <= my < CAM_H + PPI_H:
                    ppi_x = mx
                    ppi_y = my - CAM_H

                    removed = any(
                        stars.remove(star) or True
                        for star in stars[:]
                        if math.hypot(
                            ppi_x - star["pos"][0],
                            ppi_y - star["pos"][1],
                        )
                        < STAR_RADIUS
                    )

                    if not removed and len(stars) < MAX_STARS:
                        stars.append(
                            {
                                "pos": (ppi_x, ppi_y),
                                "id": len(stars) + 1,
                            }
                        )

        # Camera frame
        try:
            while True:
                data, _ = cam_sock.recvfrom(70000)

                try:
                    raw = pygame.image.load(io.BytesIO(data))
                    cam_surf = pygame.transform.scale(raw, (CAM_W, CAM_H))
                except Exception:
                    pass

        except BlockingIOError:
            pass

        # Radar data
        udp_got = False

        try:
            while True:
                data, _ = ppi_sock.recvfrom(4096)

                for line in data.decode(errors="ignore").splitlines():
                    process_radar(line.strip(), now)

                udp_got = True
                last_radar_udp_time = now

        except BlockingIOError:
            pass

        if udp_got or (now - last_radar_udp_time) <= 3.0:
            link_status = "UDP"
        else:
            link_status = "OFFLINE"

        # Remove stale targets
        for tid in [k for k, t in targets.items() if now - t["time"] > 0.5]:
            del targets[tid]

        # Heatmap decay
        heat *= decay_s.value
        heat -= 2.0
        np.maximum(heat, 0, out=heat)

        # Heatmap update
        for t in targets.values():
            angle_deg = math.degrees(math.atan2(t["x"], t["y"]))

            if abs(angle_deg) > FOV / 2:
                continue

            sx, sy = to_screen(t["x"], t["y"])

            if not (0 <= sx < PPI_W and 0 <= sy < PPI_H):
                continue

            x0, x1 = max(0, sx - GAUSS_R), min(PPI_W, sx + GAUSS_R + 1)
            y0, y1 = max(0, sy - GAUSS_R), min(PPI_H, sy + GAUSS_R + 1)

            kx0 = x0 - (sx - GAUSS_R)
            kx1 = kx0 + (x1 - x0)
            ky0 = y0 - (sy - GAUSS_R)
            ky1 = ky0 + (y1 - y0)

            if x1 > x0 and y1 > y0:
                heat[x0:x1, y0:y1] = np.minimum(
                    255,
                    heat[x0:x1, y0:y1] + GAUSS[kx0:kx1, ky0:ky1],
                )

        # Telemetry from ptcamera_tracker
        latest_telem = None

        try:
            while True:
                data, _ = telem_sock.recvfrom(4096)

                try:
                    latest_telem = json.loads(data.decode("utf-8", errors="ignore"))
                except Exception:
                    pass

        except BlockingIOError:
            pass

        if latest_telem:
            track_info = latest_telem

        # Star tracking
        surviving = []

        for star in stars:
            nx, ny = heat_centroid(
                heat,
                star["pos"][0],
                star["pos"][1],
            )

            if nx is not None:
                star["pos"] = (nx, ny)
                surviving.append(star)

        stars = surviving

        # Render
        render_camera_panel(
            cam_panel,
            font,
            font_s,
            cam_surf,
        )

        render_ppi_panel(
            ppi_surf,
            font,
            font_s,
            stars,
            targets,
            heat,
            half_fov,
        )

        render_side_panel(
            info_surf,
            font,
            font_s,
            sliders,
            targets,
            link_status,
            track_info,
            ui_fonts,
        )

        render_space_panel(
            space_surf,
            font,
            font_s,
            targets,
            track_info,
        )

        screen.blit(cam_panel, (0, 0))
        screen.blit(ppi_surf, (0, CAM_H))
        screen.blit(info_surf, (LEFT_W, 0))
        screen.blit(space_surf, (LEFT_W, CAM_H))

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
