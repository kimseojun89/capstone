#!/usr/bin/env python3
"""
unified_gui.py  -  Anti-Drone Unified Display
Left  : Camera feed from ptcamera_tracker (UDP 9998, JPEG)
Right : Radar PPI   from ptcamera_tracker (UDP 9999, [RADAR] text)
"""
import io, math, re, socket, sys, time
import numpy as np
import pygame

# ============================================================
#  Layout
# ============================================================
CAM_PANEL_W  = 700
CAM_W, CAM_H = 700, 394        # 16:9 display size inside left panel

PPI_PANEL_W  = 1000
PPI_W        = 750              # radar circle area inside PPI panel
WIN_W        = CAM_PANEL_W + PPI_PANEL_W
WIN_H        = 700

# Radar geometry (coordinates relative to PPI panel surface)
MAX_RANGE    = 8000
FOV          = 120
CX           = PPI_W // 2
CY           = WIN_H - 60
MAX_PX       = 560
MAX_STARS    = 3
STAR_RADIUS  = 30

UDP_CAM_PORT = 9998
UDP_PPI_PORT = 9999

# ============================================================
#  Slider
# ============================================================
class Slider:
    def __init__(self, x, y, w, label, lo, hi, val, fmt='{:.3f}'):
        self.rect  = pygame.Rect(x, y, w, 8)
        self.label = label
        self.lo, self.hi, self.value = lo, hi, val
        self.fmt   = fmt
        self.drag  = False

    def draw(self, surf, font):
        pygame.draw.rect(surf, (60, 60, 60), self.rect, border_radius=4)
        r = (self.value - self.lo) / (self.hi - self.lo)
        filled = pygame.Rect(self.rect.x, self.rect.y, int(self.rect.w * r), self.rect.h)
        pygame.draw.rect(surf, (0, 180, 80), filled, border_radius=4)
        hx = self.rect.x + int(self.rect.w * r)
        pygame.draw.circle(surf, (200, 255, 200), (hx, self.rect.centery), 8)
        surf.blit(font.render(f'{self.label}: {self.fmt.format(self.value)}',
                              True, (180, 180, 180)),
                  (self.rect.x, self.rect.y - 18))

    def handle_event(self, event, x_off=0):
        """x_off = screen x offset of the surface this slider lives on."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            lx = event.pos[0] - x_off
            hx = self.rect.x + int(self.rect.w * (self.value - self.lo) / (self.hi - self.lo))
            if math.hypot(lx - hx, event.pos[1] - self.rect.centery) < 12:
                self.drag = True
        if event.type == pygame.MOUSEBUTTONUP:
            self.drag = False
        if event.type == pygame.MOUSEMOTION and self.drag:
            r = (event.pos[0] - x_off - self.rect.x) / self.rect.w
            self.value = self.lo + max(0.0, min(1.0, r)) * (self.hi - self.lo)

# ============================================================
#  Heatmap kernel
# ============================================================
def _make_gauss(size=11, sigma=2.5):
    k = size // 2
    y, x = np.mgrid[-k:k+1, -k:k+1]
    g = np.exp(-(x**2 + y**2) / (2 * sigma**2))
    return (g / g.max() * 255).astype(np.float32)

GAUSS   = _make_gauss()
GAUSS_R = len(GAUSS) // 2

# ============================================================
#  Helpers
# ============================================================
def to_screen(x_mm, y_mm):
    return (int(CX + (x_mm / MAX_RANGE) * MAX_PX),
            int(CY - (y_mm / MAX_RANGE) * MAX_PX))

def heat_centroid(heat, cx, cy, radius=40):
    x0, x1 = max(0, cx-radius), min(PPI_W, cx+radius)
    y0, y1 = max(0, cy-radius), min(WIN_H, cy+radius)
    region  = heat[x0:x1, y0:y1]
    if region.size == 0 or region.max() < 15:
        return None, None
    total = region.sum()
    xs, ys = np.arange(x0, x1), np.arange(y0, y1)
    return (int((region.sum(axis=1) @ xs) / total),
            int((region.sum(axis=0) @ ys) / total))

def draw_star(surf, cx, cy, color, size=14):
    pts = [(cx + (size if i%2==0 else size*0.45) * math.cos(math.radians(i*36-90)),
            cy + (size if i%2==0 else size*0.45) * math.sin(math.radians(i*36-90)))
           for i in range(10)]
    pygame.draw.polygon(surf, color, pts)
    pygame.draw.polygon(surf, (255, 255, 0), pts, 2)

# ============================================================
#  PPI panel renderer  (draws onto a 1000x700 surface)
# ============================================================
def render_ppi(surf, font, font_s, sliders, stars,
               targets, link_status, heat, half_fov):
    surf.fill((0, 0, 0))

    # Heatmap
    hu8 = heat.astype(np.uint8)
    surf.blit(pygame.surfarray.make_surface(
        np.stack([hu8//2, hu8, hu8], axis=-1)), (0, 0))

    # Distance rings & labels
    for r_mm in [2000, 4000, 6000, 8000]:
        r_px = int((r_mm / MAX_RANGE) * MAX_PX)
        rect = pygame.Rect(CX-r_px, CY-r_px, r_px*2, r_px*2)
        pygame.draw.arc(surf, (50, 80, 50), rect,
                        math.radians(210), math.radians(330), 1)
        lx = int(CX + r_px * math.sin(half_fov)) + 5
        ly = int(CY - r_px * math.cos(half_fov))
        surf.blit(font_s.render(f'{r_mm//1000}m', True, (80,180,80)), (lx, ly))

    # FOV boundary lines
    for sign in [-1, 1]:
        ex = int(CX + MAX_PX * math.sin(sign * half_fov))
        ey = int(CY - MAX_PX * math.cos(half_fov))
        pygame.draw.line(surf, (0,120,0), (CX, CY), (ex, ey), 1)

    # Azimuth lines
    for deg in [-60, -30, 0, 30, 60]:
        rad = math.radians(deg)
        ex = int(CX + MAX_PX * math.sin(rad))
        ey = int(CY - MAX_PX * math.cos(rad))
        pygame.draw.line(surf, (0,180,0) if deg==0 else (0,80,0),
                         (CX, CY), (ex, ey), 1)
        surf.blit(font_s.render(f'{deg}deg', True, (60,150,60)), (ex-14, ey-22))

    pygame.draw.circle(surf, (0,255,0), (CX, CY), 5)
    surf.blit(font.render('RADAR', True, (100,200,100)), (CX+8, CY-12))

    # Stars & raw targets
    for star in stars:
        sx, sy = star['pos']
        draw_star(surf, sx, sy, (255,50,50))
        surf.blit(font_s.render(f"T{star['id']}", True, (255,200,0)), (sx+16, sy-8))
    for tid, t in targets.items():
        rx, ry = to_screen(t['x'], t['y'])
        pygame.draw.circle(surf, (0,255,255), (rx, ry), 5)
        surf.blit(font_s.render(f'RAW_{tid}', True, (0,255,255)), (rx+8, ry-8))

    # Data panel (right 250px of ppi surface)
    px = PPI_W + 15
    pygame.draw.rect(surf, (15,15,15),
                     pygame.Rect(PPI_W, 0, PPI_PANEL_W-PPI_W, WIN_H))
    pygame.draw.line(surf, (50,50,50), (PPI_W,0), (PPI_W,WIN_H), 1)

    link_col = {
        'UDP': (0,255,0), 'OFFLINE': (255,0,0)
    }.get(link_status, (255,0,0))
    surf.blit(font.render(f'LINK: {link_status}', True, link_col), (px, 10))
    surf.blit(font.render('CONTROL', True, (0,200,80)), (px, 40))

    for s in sliders:
        s.draw(surf, font_s)

    pygame.draw.line(surf, (40,40,40), (px,120), (PPI_PANEL_W-10,120), 1)
    surf.blit(font.render('RADAR DATA', True, (0,255,255)), (px, 135))

    if not targets:
        surf.blit(font_s.render('No Targets Detected.', True, (100,100,100)), (px, 165))

    y_off = 165
    for tid, t in targets.items():
        dist_m    = t['dist'] / 1000.0
        angle_deg = math.degrees(math.atan2(t['x'], t['y']))
        surf.blit(font_s.render(f'ID {tid}:', True, (255,255,255)), (px, y_off))
        surf.blit(font_s.render(f'R:{dist_m:.1f}m  A:{angle_deg:+.1f}deg',
                                True, (200,255,200)), (px, y_off+18))
        surf.blit(font_s.render(f'V:{t.get("speed",0):.1f}m/s',
                                True, (255,150,150)), (px, y_off+36))
        y_off += 65
        pygame.draw.line(surf, (30,30,30),
                         (px, y_off-10), (PPI_PANEL_W-10, y_off-10), 1)

    pygame.draw.line(surf, (40,40,40), (px,580), (PPI_PANEL_W-10,580), 1)
    surf.blit(font.render('GUIDE', True, (0,200,80)), (px, 590))
    for i, g in enumerate(['Click : assign star', 'R : reset', 'Q : quit']):
        surf.blit(font_s.render(g, True, (120,120,120)), (px, 615+i*20))

# ============================================================
#  Camera panel renderer  (draws directly on screen, x=0..CAM_PANEL_W)
# ============================================================
def render_cam(screen, font, font_s, cam_surf):
    pygame.draw.rect(screen, (10,10,10), (0, 0, CAM_PANEL_W, WIN_H))
    pygame.draw.line(screen, (50,50,50),
                     (CAM_PANEL_W-1, 0), (CAM_PANEL_W-1, WIN_H), 1)

    cam_y = (WIN_H - CAM_H) // 2   # vertically center the frame

    if cam_surf is not None:
        screen.blit(cam_surf, (0, cam_y))
        sig_txt, sig_col = 'LIVE', (0, 220, 0)
    else:
        pygame.draw.rect(screen, (20,20,20), (0, cam_y, CAM_PANEL_W, CAM_H))
        pygame.draw.rect(screen, (60,60,60), (0, cam_y, CAM_PANEL_W, CAM_H), 2)
        ns = font.render('NO SIGNAL', True, (90,90,90))
        screen.blit(ns, (CAM_PANEL_W//2 - ns.get_width()//2,
                         cam_y + CAM_H//2 - ns.get_height()//2))
        sig_txt, sig_col = 'WAITING...', (180, 60, 60)

    # Status bar below frame
    bar_y = cam_y + CAM_H + 8
    screen.blit(font.render('CAMERA FEED', True, (0,200,80)), (10, bar_y))
    screen.blit(font_s.render(sig_txt, True, sig_col), (10, bar_y + 20))
    screen.blit(font.render('Anti-Drone Unified', True, (140,140,140)),
                (10, WIN_H - 25))

# ============================================================
#  Main
# ============================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption('Anti-Drone Unified Display')
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont('consolas', 15, bold=True)
    font_s = pygame.font.SysFont('consolas', 12)

    # PPI state
    heat         = np.zeros((PPI_W, WIN_H), dtype=np.float32)
    stars        = []
    targets      = {}
    link_status  = 'OFFLINE'
    half_fov     = math.radians(FOV / 2)
    ppi_surf     = pygame.Surface((PPI_PANEL_W, WIN_H))
    pattern      = r'\[RADAR\]\s*T(\d+):\((-?\d+),(-?\d+)\)mm'

    px_slider = PPI_W + 15   # x within ppi_surf
    sliders   = [Slider(px_slider, 90, 200,
                        'Afterglow Decay', 0.80, 0.999, 0.985)]
    decay_s   = sliders[0]

    # Camera state
    cam_surf     = None

    # UDP sockets
    ppi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ppi_sock.bind(('127.0.0.1', UDP_PPI_PORT))
    ppi_sock.setblocking(False)

    cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cam_sock.bind(('127.0.0.1', UDP_CAM_PORT))
    cam_sock.setblocking(False)

    print(f'[OK] UDP camera:{UDP_CAM_PORT}  radar:{UDP_PPI_PORT}  window:{WIN_W}x{WIN_H}')

    while True:
        now = time.time()

        # -- Events ------------------------------------------
        for event in pygame.event.get():
            if (event.type == pygame.QUIT or
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_q)):
                ppi_sock.close(); cam_sock.close()
                pygame.quit(); sys.exit()

            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                stars.clear(); heat[:] = 0

            # Sliders live on ppi_surf, offset by CAM_PANEL_W on screen
            for s in sliders:
                s.handle_event(event, x_off=CAM_PANEL_W)

            # Star click (inside PPI circle area)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                ppi_x = mx - CAM_PANEL_W
                if 0 <= ppi_x < PPI_W:
                    removed = any(
                        stars.remove(star) or True
                        for star in stars[:]
                        if math.hypot(ppi_x - star['pos'][0],
                                      my  - star['pos'][1]) < STAR_RADIUS
                    )
                    if not removed and len(stars) < MAX_STARS:
                        stars.append({'pos': (ppi_x, my), 'id': len(stars)+1})

        # -- Camera frame receive -----------------------------
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

        # -- Radar data receive -------------------------------
        def process_radar(line):
            m = re.search(pattern, line)
            if not m:
                return
            tid, x_mm, y_mm = int(m.group(1)), int(m.group(2)), int(m.group(3))
            if x_mm == 0 and y_mm == 0:
                return
            if not (0 <= y_mm <= MAX_RANGE):
                return
            dist  = math.hypot(x_mm, y_mm)
            speed = 0.0
            if tid in targets:
                prev = targets[tid]
                dt   = now - prev['time']
                if dt > 0.05:
                    speed = math.hypot((x_mm-prev['x'])/1000,
                                       (y_mm-prev['y'])/1000) / dt
            targets[tid] = {'x': x_mm, 'y': y_mm,
                            'dist': dist, 'speed': speed, 'time': now}

        udp_got = False
        try:
            while True:
                data, _ = ppi_sock.recvfrom(4096)
                for line in data.decode(errors='ignore').splitlines():
                    process_radar(line.strip())
                udp_got = True
        except BlockingIOError:
            pass

        if udp_got:
            link_status = 'UDP'
        elif (now - max((t['time'] for t in targets.values()), default=0)) > 3.0:
            link_status = 'OFFLINE'

        # Stale target cleanup
        for tid in [k for k, t in targets.items() if now - t['time'] > 0.5]:
            del targets[tid]

        # -- Heatmap -----------------------------------------
        heat *= decay_s.value
        heat -= 2.0
        np.maximum(heat, 0, out=heat)

        for t in targets.values():
            if abs(math.degrees(math.atan2(t['x'], t['y']))) > FOV / 2:
                continue
            sx, sy = to_screen(t['x'], t['y'])
            if not (0 <= sx < PPI_W and 0 <= sy < WIN_H):
                continue
            x0, x1 = max(0, sx-GAUSS_R), min(PPI_W, sx+GAUSS_R+1)
            y0, y1 = max(0, sy-GAUSS_R), min(WIN_H, sy+GAUSS_R+1)
            kx0 = x0-(sx-GAUSS_R); kx1 = kx0+(x1-x0)
            ky0 = y0-(sy-GAUSS_R); ky1 = ky0+(y1-y0)
            if x1 > x0 and y1 > y0:
                heat[x0:x1, y0:y1] = np.minimum(
                    255, heat[x0:x1,y0:y1] + GAUSS[kx0:kx1,ky0:ky1])

        # -- Star tracking ------------------------------------
        surviving = []
        for star in stars:
            nx, ny = heat_centroid(heat, star['pos'][0], star['pos'][1])
            if nx is not None:
                star['pos'] = (nx, ny)
                surviving.append(star)
        stars = surviving

        # -- Render ------------------------------------------
        render_ppi(ppi_surf, font, font_s, sliders, stars,
                   targets, link_status, heat, half_fov)
        render_cam(screen, font, font_s, cam_surf)
        screen.blit(ppi_surf, (CAM_PANEL_W, 0))
        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()
