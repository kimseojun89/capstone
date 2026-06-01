#!/usr/bin/env python3
"""
unified_gui.py  -  Anti-Drone Unified Display

Layout:
    Left Top    : Camera feed from ptcamera_tracker (UDP 9998, JPEG)
    Left Bottom : Radar PPI from ptcamera_tracker (UDP 9999, [RADAR] text)
    Right       : Information / Control / Telemetry panel

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
        pygame.draw.rect(surf, (60, 60, 60), self.rect, border_radius=4)

        r = (self.value - self.lo) / (self.hi - self.lo)
        r = max(0.0, min(1.0, r))

        filled = pygame.Rect(
            self.rect.x,
            self.rect.y,
            int(self.rect.w * r),
            self.rect.h,
        )
        pygame.draw.rect(surf, (0, 180, 80), filled, border_radius=4)

        hx = self.rect.x + int(self.rect.w * r)
        pygame.draw.circle(surf, (200, 255, 200), (hx, self.rect.centery), 8)

        surf.blit(
            font.render(
                f"{self.label}: {self.fmt.format(self.value)}",
                True,
                (180, 180, 180),
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
    surf.fill((10, 10, 10))

    if cam_surf is not None:
        surf.blit(cam_surf, (0, 0))
        sig_txt, sig_col = "LIVE", (0, 220, 0)
    else:
        pygame.draw.rect(surf, (20, 20, 20), (0, 0, CAM_W, CAM_H))
        pygame.draw.rect(surf, (60, 60, 60), (0, 0, CAM_W, CAM_H), 2)

        ns = font.render("NO SIGNAL", True, (90, 90, 90))
        surf.blit(
            ns,
            (
                CAM_W // 2 - ns.get_width() // 2,
                CAM_H // 2 - ns.get_height() // 2,
            ),
        )

        sig_txt, sig_col = "WAITING...", (180, 60, 60)

    # Header overlay
    pygame.draw.rect(surf, (0, 0, 0), (0, 0, CAM_W, 30))
    surf.blit(font.render("CAMERA FEED", True, (0, 200, 80)), (10, 7))
    surf.blit(font_s.render(sig_txt, True, sig_col), (130, 9))

    # Border
    pygame.draw.rect(surf, (40, 40, 40), (0, 0, CAM_W, CAM_H), 1)


# ============================================================
#  PPI panel renderer
# ============================================================
def render_ppi_panel(surf, font, font_s, stars, targets, heat, half_fov):
    surf.fill((0, 0, 0))

    hu8 = heat.astype(np.uint8)
    heat_rgb = np.stack([hu8 // 2, hu8, hu8], axis=-1)
    surf.blit(pygame.surfarray.make_surface(heat_rgb), (0, 0))

    # Range arcs
    for r_mm in [2000, 4000, 6000, 8000]:
        r_px = int((r_mm / MAX_RANGE) * MAX_PX)
        rect = pygame.Rect(CX - r_px, CY - r_px, r_px * 2, r_px * 2)

        pygame.draw.arc(
            surf,
            (50, 80, 50),
            rect,
            math.radians(210),
            math.radians(330),
            1,
        )

        lx = int(CX + r_px * math.sin(half_fov)) + 5
        ly = int(CY - r_px * math.cos(half_fov))
        surf.blit(
            font_s.render(f"{r_mm // 1000}m", True, (80, 180, 80)),
            (lx, ly),
        )

    # FOV boundary lines
    for sign in [-1, 1]:
        rad = sign * half_fov
        ex = int(CX + MAX_PX * math.sin(rad))
        ey = int(CY - MAX_PX * math.cos(rad))
        pygame.draw.line(surf, (0, 120, 0), (CX, CY), (ex, ey), 1)

    # Angle guide lines
    for deg in [-60, -30, 0, 30, 60]:
        rad = math.radians(deg)
        ex = int(CX + MAX_PX * math.sin(rad))
        ey = int(CY - MAX_PX * math.cos(rad))

        pygame.draw.line(
            surf,
            (0, 180, 0) if deg == 0 else (0, 80, 0),
            (CX, CY),
            (ex, ey),
            1,
        )

        surf.blit(
            font_s.render(f"{deg}deg", True, (60, 150, 60)),
            (ex - 18, ey - 22),
        )

    # Radar origin
    pygame.draw.circle(surf, (0, 255, 0), (CX, CY), 5)

    pygame.draw.rect(surf, (0, 0, 0), (0, 0, PPI_W, 30))
    surf.blit(font.render("RADAR PPI", True, (100, 200, 100)), (10, 7))

    # Assigned stars
    for star in stars:
        sx, sy = star["pos"]
        draw_star(surf, sx, sy, (255, 50, 50))
        surf.blit(
            font_s.render(f"T{star['id']}", True, (255, 200, 0)),
            (sx + 16, sy - 8),
        )

    # Raw targets
    for tid, t in targets.items():
        rx, ry = to_screen(t["x"], t["y"])

        if not (0 <= rx < PPI_W and 0 <= ry < PPI_H):
            continue

        pygame.draw.circle(surf, (0, 255, 255), (rx, ry), 5)
        surf.blit(
            font_s.render(f"RAW_{tid}", True, (0, 255, 255)),
            (rx + 8, ry - 8),
        )

    # Border
    pygame.draw.rect(surf, (40, 40, 40), (0, 0, PPI_W, PPI_H), 1)


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
):
    surf.fill((15, 15, 15))

    x = 15
    y = 15

    pygame.draw.line(surf, (50, 50, 50), (0, 0), (0, WIN_H), 1)

    # System title
    surf.blit(font.render("ANTI-DRONE DISPLAY", True, (0, 220, 100)), (x, y))
    y += 35

    # Link status
    link_col = {
        "UDP": (0, 255, 0),
        "OFFLINE": (255, 0, 0),
    }.get(link_status, (255, 0, 0))

    surf.blit(font.render(f"RADAR LINK: {link_status}", True, link_col), (x, y))
    y += 35

    pygame.draw.line(surf, (40, 40, 40), (x, y), (SIDE_W - 15, y), 1)
    y += 18

    # Tracker telemetry
    surf.blit(font.render("TRACKER / MOTOR", True, (0, 200, 80)), (x, y))
    y += 28

    if track_info:
        enabled = track_info.get("motor_enabled", False)
        found = track_info.get("target_found", False)
        fps = track_info.get("fps", 0.0)

        motor_col = (0, 220, 80) if enabled else (180, 120, 40)
        target_col = (0, 220, 80) if found else (120, 120, 80)

        surf.blit(
            font_s.render(f'motor={"ON" if enabled else "OFF"}', True, motor_col),
            (x, y),
        )
        y += 18

        surf.blit(
            font_s.render(f'target={"LOCK" if found else "NONE"}', True, target_col),
            (x, y),
        )
        y += 18

        surf.blit(
            font_s.render(f"fps={fps:.1f}", True, (180, 180, 180)),
            (x, y),
        )
        y += 24

        bbox_ex = track_info.get("bbox_ex", 0)
        bbox_ey = track_info.get("bbox_ey", 0)
        cx = track_info.get("center_x", 0)
        cy = track_info.get("center_y", 0)
        conf = track_info.get("confidence", 0.0)

        surf.blit(
            font_s.render(
                f"BBOX ex={bbox_ex:+d}  ey={bbox_ey:+d}",
                True,
                (200, 255, 200),
            ),
            (x, y),
        )
        y += 18

        surf.blit(
            font_s.render(
                f"center x={cx:.0f}  y={cy:.0f}",
                True,
                (200, 255, 200),
            ),
            (x, y),
        )
        y += 18

        surf.blit(
            font_s.render(
                f"confidence={conf:.2f}",
                True,
                (200, 255, 200),
            ),
            (x, y),
        )
        y += 30

    else:
        surf.blit(
            font_s.render("TRACKER UDP WAITING", True, (200, 100, 0)),
            (x, y),
        )
        y += 36

    pygame.draw.line(surf, (40, 40, 40), (x, y), (SIDE_W - 15, y), 1)
    y += 20

    # Control
    surf.blit(font.render("CONTROL", True, (0, 200, 80)), (x, y))
    y += 35

    for s in sliders:
        s.draw(surf, font_s)

    y = 240
    pygame.draw.line(surf, (40, 40, 40), (x, y), (SIDE_W - 15, y), 1)
    y += 18

    # Radar data
    surf.blit(font.render("RADAR DATA", True, (0, 255, 255)), (x, y))
    y += 30

    if not targets:
        surf.blit(
            font_s.render("No Targets Detected.", True, (100, 100, 100)),
            (x, y),
        )
        y += 28

    for tid, t in targets.items():
        if y > WIN_H - 150:
            surf.blit(
                font_s.render("... more targets omitted", True, (120, 120, 120)),
                (x, y),
            )
            break

        dist_m = t["dist"] / 1000.0
        angle_deg = math.degrees(math.atan2(t["x"], t["y"]))

        surf.blit(
            font_s.render(f"ID {tid}:", True, (255, 255, 255)),
            (x, y),
        )
        y += 18

        surf.blit(
            font_s.render(
                f"R:{dist_m:.1f}m  A:{angle_deg:+.1f}deg",
                True,
                (200, 255, 200),
            ),
            (x, y),
        )
        y += 18

        surf.blit(
            font_s.render(
                f'V:{t.get("speed", 0):.1f}m/s',
                True,
                (255, 150, 150),
            ),
            (x, y),
        )
        y += 26

        pygame.draw.line(surf, (30, 30, 30), (x, y), (SIDE_W - 15, y), 1)
        y += 10

    # Guide
    guide_y = WIN_H - 110
    pygame.draw.line(surf, (40, 40, 40), (x, guide_y), (SIDE_W - 15, guide_y), 1)
    guide_y += 14

    surf.blit(font.render("GUIDE", True, (0, 200, 80)), (x, guide_y))
    guide_y += 25

    guides = [
        "Click PPI : assign star",
        "R : reset radar",
        "Q : quit",
    ]

    for g in guides:
        surf.blit(
            font_s.render(g, True, (120, 120, 120)),
            (x, guide_y),
        )
        guide_y += 20


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
    font = pygame.font.SysFont(
        "consolas",
        max(12, int(15 * LAYOUT_SCALE)),
        bold=True,
    )
    font_s = pygame.font.SysFont("consolas", max(10, int(12 * LAYOUT_SCALE)))

    print(
        "[OK] Display-only GUI. "
        "ptcamera_tracker owns the hardware port; GUI listens on UDP only."
    )

    # ── Surfaces ───────────────────────────────────────────
    cam_panel = pygame.Surface((CAM_W, CAM_H))
    ppi_surf = pygame.Surface((PPI_W, PPI_H))
    side_surf = pygame.Surface((SIDE_W, WIN_H))

    # ── PPI state ──────────────────────────────────────────
    heat = np.zeros((PPI_W, PPI_H), dtype=np.float32)
    stars = []
    targets = {}

    link_status = "OFFLINE"
    last_radar_udp_time = 0.0

    half_fov = math.radians(FOV / 2)
    pattern = r"\[RADAR\]\s*T(\d+):\((-?\d+),(-?\d+)\)mm"

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
                speed = (
                    math.hypot(
                        (x_mm - prev["x"]) / 1000,
                        (y_mm - prev["y"]) / 1000,
                    )
                    / dt
                )

        targets[tid] = {
            "x": x_mm,
            "y": y_mm,
            "dist": dist,
            "speed": speed,
            "time": now,
        }

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
            side_surf,
            font,
            font_s,
            sliders,
            targets,
            link_status,
            track_info,
        )

        screen.blit(cam_panel, (0, 0))
        screen.blit(ppi_surf, (0, CAM_H))
        screen.blit(side_surf, (LEFT_W, 0))

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
