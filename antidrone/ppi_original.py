#!/usr/bin/env python3
import serial
import pygame
import numpy as np
import struct
import sys
import math

# ===== 설정 =====
SERIAL_PORT  = '/dev/ttyAMA0'
BAUD_RATE    = 256000

WIN_W        = 1000   # 오른쪽에 컨트롤 패널 추가
WIN_H        = 700
PPI_W        = 800    # PPI 영역
MAX_RANGE    = 8000
FOV          = 120
CX           = PPI_W // 2
CY           = WIN_H - 60
MAX_PX       = 560

FRAME_HEADER = b'\xAA\xFF\x03\x00'
FRAME_TAIL   = b'\x55\xCC'
FRAME_LEN    = 30

X_OFFSET     = 290
MAX_STARS    = 3
STAR_RADIUS  = 30

# ===== 슬라이더 클래스 =====
class Slider:
    def __init__(self, x, y, w, label, min_val, max_val, init_val, fmt='{:.2f}'):
        self.rect     = pygame.Rect(x, y, w, 8)
        self.label    = label
        self.min_val  = min_val
        self.max_val  = max_val
        self.value    = init_val
        self.fmt      = fmt
        self.dragging = False

    def draw(self, surface, font):
        # 배경 트랙
        pygame.draw.rect(surface, (60, 60, 60), self.rect, border_radius=4)
        # 채워진 트랙
        ratio = (self.value - self.min_val) / (self.max_val - self.min_val)
        filled = pygame.Rect(self.rect.x, self.rect.y,
                             int(self.rect.w * ratio), self.rect.h)
        pygame.draw.rect(surface, (0, 180, 80), filled, border_radius=4)
        # 핸들
        hx = self.rect.x + int(self.rect.w * ratio)
        hy = self.rect.centery
        pygame.draw.circle(surface, (200, 255, 200), (hx, hy), 8)
        # 레이블 + 값
        label_surf = font.render(
            f'{self.label}: {self.fmt.format(self.value)}', True, (180, 180, 180))
        surface.blit(label_surf, (self.rect.x, self.rect.y - 18))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            hx = self.rect.x + int(self.rect.w *
                 (self.value - self.min_val) / (self.max_val - self.min_val))
            hy = self.rect.centery
            if math.sqrt((event.pos[0]-hx)**2 + (event.pos[1]-hy)**2) < 12:
                self.dragging = True
        if event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        if event.type == pygame.MOUSEMOTION and self.dragging:
            ratio = (event.pos[0] - self.rect.x) / self.rect.w
            ratio = max(0.0, min(1.0, ratio))
            self.value = self.min_val + ratio * (self.max_val - self.min_val)

def make_gaussian(size=11, sigma=2.5):
    k = size // 2
    y, x = np.mgrid[-k:k+1, -k:k+1]
    g = np.exp(-(x**2 + y**2) / (2 * sigma**2))
    return (g / g.max() * 200).astype(np.float32)

GAUSS   = make_gaussian(size=11, sigma=2.5)
GAUSS_R = len(GAUSS) // 2

def parse_targets(frame):
    targets = []
    for i in range(3):
        offset = 4 + i * 8
        x_raw, y_raw = struct.unpack_from('<HH', frame, offset)
        x = -(x_raw & 0x7FFF) if (x_raw & 0x8000) else x_raw
        y = -(y_raw & 0x7FFF) if (y_raw & 0x8000) else y_raw
        if x == 0 and y == 0:
            continue
        targets.append((x, y))
    return targets

def calibrated_angle(x_mm, y_mm):
    x_corrected = x_mm - X_OFFSET
    return math.degrees(math.atan2(x_corrected, -y_mm))

def to_screen(x_mm, y_mm):
    dist      = math.sqrt(x_mm**2 + y_mm**2)
    angle_deg = calibrated_angle(x_mm, y_mm)
    angle_rad = math.radians(angle_deg)
    r_px      = (dist / MAX_RANGE) * MAX_PX
    sx        = int(CX + r_px * math.sin(angle_rad))
    sy        = int(CY - r_px * math.cos(angle_rad))
    return sx, sy

def find_heat_centroid(heat, cx, cy, radius=40):
    x0 = max(0, cx - radius);  x1 = min(PPI_W, cx + radius)
    y0 = max(0, cy - radius);  y1 = min(WIN_H, cy + radius)
    region = heat[x0:x1, y0:y1]
    if region.max() < 10:
        return cx, cy
    total = region.sum()
    xs    = np.arange(x0, x1)
    ys    = np.arange(y0, y1)
    new_x = int((region.sum(axis=1) @ xs) / total)
    new_y = int((region.sum(axis=0) @ ys) / total)
    return new_x, new_y

def draw_star(surface, cx, cy, color, size=14):
    points = []
    for i in range(10):
        angle = math.radians(i * 36 - 90)
        r     = size if i % 2 == 0 else size * 0.45
        points.append((cx + r * math.cos(angle),
                        cy + r * math.sin(angle)))
    pygame.draw.polygon(surface, color, points)
    pygame.draw.polygon(surface, (255, 255, 0), points, 2)

def draw_panel(surface, font, font_s, sliders, stars):
    # 패널 배경
    panel = pygame.Rect(PPI_W, 0, WIN_W - PPI_W, WIN_H)
    pygame.draw.rect(surface, (15, 15, 15), panel)
    pygame.draw.line(surface, (50, 50, 50), (PPI_W, 0), (PPI_W, WIN_H), 1)

    # 제목
    surface.blit(font.render('CONTROL', True, (0, 200, 80)), (PPI_W + 10, 10))

    # 슬라이더 그리기
    for s in sliders:
        s.draw(surface, font_s)

    # 구분선
    pygame.draw.line(surface, (40, 40, 40),
                     (PPI_W + 10, 340), (WIN_W - 10, 340), 1)

    # 별표 상태
    surface.blit(font.render('TARGETS', True, (0, 200, 80)), (PPI_W + 10, 355))
    if not stars:
        surface.blit(font_s.render('없음', True, (100, 100, 100)), (PPI_W + 10, 385))
    for i, star in enumerate(stars):
        sx, sy = star['pos']
        txt = font_s.render(f"T{star['id']}  ({sx},{sy})", True, (255, 200, 0))
        surface.blit(txt, (PPI_W + 10, 385 + i * 22))

    # 구분선
    pygame.draw.line(surface, (40, 40, 40),
                     (PPI_W + 10, 460), (WIN_W - 10, 460), 1)

    # 조작 안내
    surface.blit(font.render('GUIDE', True, (0, 200, 80)), (PPI_W + 10, 470))
    guides = ['클릭  : ★ 부여',
              '재클릭: ★ 해제',
              'R     : 전체 초기화',
              'q     : 종료']
    for i, g in enumerate(guides):
        surface.blit(font_s.render(g, True, (120, 120, 120)),
                     (PPI_W + 10, 495 + i * 20))

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption('LD2450 PPI  |  q: 종료')
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont('monospace', 15, bold=True)
    font_s = pygame.font.SysFont('monospace', 12)

    heat     = np.zeros((PPI_W, WIN_H), dtype=np.float32)
    ftc_prev = np.zeros((PPI_W, WIN_H), dtype=np.float32)
    stars    = []

    # ===== 슬라이더 정의 =====
    PX = PPI_W + 15
    sliders = [
        Slider(PX, 60,  160, 'Decay',       0.80, 0.99, 0.92),
        Slider(PX, 120, 160, 'STC MinDist', 0,    3000, 500,  fmt='{:.0f}mm'),
        Slider(PX, 180, 160, 'STC MaxDist', 1000, 8000, 3000, fmt='{:.0f}mm'),
        Slider(PX, 240, 160, 'FTC Alpha',   0.0,  0.99, 0.85),
        Slider(PX, 300, 160, 'Gauss Gain',  50,   255,  200,  fmt='{:.0f}'),
    ]
    decay_s, stc_min_s, stc_max_s, ftc_s, gain_s = sliders

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f'[OK] {SERIAL_PORT} 연결됨')
    except serial.SerialException as e:
        print(f'[에러] {e}')
        sys.exit(1)

    buf       = b''
    distances = [2000, 4000, 6000, 8000]
    half_fov  = math.radians(FOV / 2)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                ser.close(); pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    ser.close(); pygame.quit(); sys.exit()
                if event.key == pygame.K_r:
                    stars.clear()
                    heat[:] = 0
                    ftc_prev[:] = 0

            # 슬라이더 이벤트
            for s in sliders:
                s.handle_event(event)

            # 클릭 → ★ 부여/해제 (PPI 영역만)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if mx < PPI_W:
                    removed = False
                    for star in stars:
                        sx, sy = star['pos']
                        if math.sqrt((mx-sx)**2 + (my-sy)**2) < STAR_RADIUS:
                            stars.remove(별)
                            removed = True
                            break
                    if not removed and len(stars) < MAX_STARS:
                        stars.append({'pos': (mx, my), 'id': len(stars)+1})

        # ===== 시리얼 =====
        buf += ser.read(max(1, ser.in_waiting))
        current_targets = []
        while True:
            idx = buf.find(FRAME_HEADER)
            if idx == -1:
                buf = b''
                break
            if idx + FRAME_LEN > len(buf):
                buf = buf[idx:]
                break
            frame = buf[idx:idx+FRAME_LEN]
            if frame[-2:] == FRAME_TAIL:
                current_targets = parse_targets(frame)
            buf = buf[idx+FRAME_LEN:]

        # ===== heat 업데이트 =====
        heat *= decay_s.value

        for x_mm, y_mm in current_targets:
            dist      = math.sqrt(x_mm**2 + y_mm**2)
            angle_deg = calibrated_angle(x_mm, y_mm)
            if dist > MAX_RANGE or abs(angle_deg) > FOV / 2:
                continue
            sx, sy = to_screen(x_mm, y_mm)
            if not (0 <= sx < PPI_W and 0 <= sy < WIN_H):
                continue

            # STC 적용
            stc_gain = np.clip(
                (dist - stc_min_s.value) / (stc_max_s.value - stc_min_s.value),
                0.1, 1.0
            )

            x0 = max(0, sx-GAUSS_R);  x1 = min(PPI_W, sx+GAUSS_R+1)
            y0 = max(0, sy-GAUSS_R);  y1 = min(WIN_H, sy+GAUSS_R+1)
            kx0 = x0-(sx-GAUSS_R);    kx1 = kx0+(x1-x0)
            ky0 = y0-(sy-GAUSS_R);    ky1 = ky0+(y1-y0)
            heat[x0:x1, y0:y1] = np.minimum(
                255,
                heat[x0:x1, y0:y1] + GAUSS[kx0:kx1, ky0:ky1] * stc_gain * (gain_s.value/200)
            )

        # FTC 적용
        ftc_diff = heat - ftc_prev * ftc_s.value
        heat_ftc = np.maximum(0, ftc_diff).astype(np.float32)
        ftc_prev[:] = heat

        # ===== ★ 위치 업데이트 =====
        for star in stars:
            sx, sy = star['pos']
            nx, ny = find_heat_centroid(heat_ftc, sx, sy, radius=40)
            star['pos'] = (nx, ny)

        # ===== 렌더링 =====
        screen.fill((0, 0, 0))

        rgb       = np.stack([heat_ftc.astype(np.uint8)] * 3, axis=-1)
        heat_surf = pygame.surfarray.make_surface(rgb)
        screen.blit(heat_surf, (0, 0))

        # 거리 링
        for r_mm in distances:
            r_px = int((r_mm / MAX_RANGE) * MAX_PX)
            rect = pygame.Rect(CX-r_px, CY-r_px, r_px*2, r_px*2)
            pygame.draw.arc(screen, (50,80,50), rect,
                            math.radians(210), math.radians(330), 1)
            lx = int(CX + r_px * math.sin(half_fov)) + 5
            ly = int(CY - r_px * math.cos(half_fov))
            screen.blit(font_s.render(f'{r_mm//1000}m', True, (80,180,80)), (lx, ly))

        # 부채꼴 경계선
        for sign in [-1, 1]:
            ex = int(CX + MAX_PX * math.sin(sign * half_fov))
            ey = int(CY - MAX_PX * math.cos(half_fov))
            pygame.draw.line(screen, (0,120,0), (CX,CY), (ex,ey), 1)

        # 각도선
        for deg in [-60, -30, 0, 30, 60]:
            rad   = math.radians(deg)
            ex    = int(CX + MAX_PX * math.sin(rad))
            ey    = int(CY - MAX_PX * math.cos(rad))
            color = (0,180,0) if deg == 0 else (0,80,0)
            pygame.draw.line(screen, color, (CX,CY), (ex,ey), 1)
            screen.blit(font_s.render(f'{deg}°', True, (60,150,60)), (ex-12, ey-22))

        # 안테나
        pygame.draw.circle(screen, (0,255,0), (CX,CY), 5)
        screen.blit(font.render('안테나', True, (100,200,100)), (CX+8, CY-12))

        # ★ 그리기
        for star in stars:
            sx, sy = star['pos']
            draw_star(screen, sx, sy, (255, 50, 50))
            screen.blit(font_s.render(f"T{star['id']}", True, (255,200,0)),
                        (sx+16, sy-8))

        # 컨트롤 패널
        draw_panel(screen, font, font_s, sliders, stars)

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()