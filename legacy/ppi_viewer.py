#!/usr/bin/env python3
import serial
import socket
import pygame
import numpy as np
import sys
import math
import re
import time

# ===== 시스템 설정 =====
COM_PORT     = 'COM4'     # PYNQ-Z2 할당 포트 (ptcamera_tracker 미실행 시 직접 연결)
BAUD_RATE    = 256000     # PS UART0 baud rate
UDP_PORT     = 9999       # ptcamera_tracker.exe가 릴레이하는 [RADAR] 라인 수신 포트

# ===== UI & 렌더링 설정 =====
WIN_W        = 1000       # 전체 윈도우 가로
WIN_H        = 700        # 전체 윈도우 세로
PPI_W        = 750        # 레이더 표시 영역 (데이터 패널 확보를 위해 약간 축소)
MAX_RANGE    = 8000       # 최대 탐지 거리 (mm)
FOV          = 120        # 탐지 시야각 (도)
CX           = PPI_W // 2 # 레이더 원점 X
CY           = WIN_H - 60 # 레이더 원점 Y
MAX_PX       = 560        # 렌더링 픽셀 스케일

MAX_STARS    = 3
STAR_RADIUS  = 30

# ===== 슬라이더 클래스 (Decay만 남김) =====
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
        pygame.draw.rect(surface, (60, 60, 60), self.rect, border_radius=4)
        ratio = (self.value - self.min_val) / (self.max_val - self.min_val)
        filled = pygame.Rect(self.rect.x, self.rect.y, int(self.rect.w * ratio), self.rect.h)
        pygame.draw.rect(surface, (0, 180, 80), filled, border_radius=4)
        hx = self.rect.x + int(self.rect.w * ratio)
        hy = self.rect.centery
        pygame.draw.circle(surface, (200, 255, 200), (hx, hy), 8)
        label_surf = font.render(f'{self.label}: {self.fmt.format(self.value)}', True, (180, 180, 180))
        surface.blit(label_surf, (self.rect.x, self.rect.y - 18))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            hx = self.rect.x + int(self.rect.w * (self.value - self.min_val) / (self.max_val - self.min_val))
            hy = self.rect.centery
            if math.sqrt((event.pos[0]-hx)**2 + (event.pos[1]-hy)**2) < 12:
                self.dragging = True
        if event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        if event.type == pygame.MOUSEMOTION and self.dragging:
            ratio = (event.pos[0] - self.rect.x) / self.rect.w
            ratio = max(0.0, min(1.0, ratio))
            self.value = self.min_val + ratio * (self.max_val - self.min_val)

# ===== 가우시안 커널 (Heatmap용) =====
def make_gaussian(size=11, sigma=2.5):
    k = size // 2
    y, x = np.mgrid[-k:k+1, -k:k+1]
    g = np.exp(-(x**2 + y**2) / (2 * sigma**2))
    return (g / g.max() * 255).astype(np.float32) # Gain 고정

GAUSS   = make_gaussian(size=11, sigma=2.5)
GAUSS_R = len(GAUSS) // 2

# ===== 좌표 변환 알고리즘 =====
def to_screen(x_mm, y_mm):
    sx = int(CX + (x_mm / MAX_RANGE) * MAX_PX)
    sy = int(CY - (y_mm / MAX_RANGE) * MAX_PX)
    return sx, sy

def find_heat_centroid(heat, cx, cy, radius=40):
    x0 = max(0, cx - radius);  x1 = min(PPI_W, cx + radius)
    y0 = max(0, cy - radius);  y1 = min(WIN_H, cy + radius)
    region = heat[x0:x1, y0:y1]
    
    # 수정: 열기가 15 미만으로 식어버리면 None을 반환하여 타겟 소멸 판정
    if region.size == 0 or region.max() < 15:
        return None, None 
        
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
        points.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    pygame.draw.polygon(surface, color, points)
    pygame.draw.polygon(surface, (255, 255, 0), points, 2)

# ===== 데이터 패널 렌더링 =====
def draw_panel(surface, font, font_s, sliders, stars, active_targets, link_status):
    panel = pygame.Rect(PPI_W, 0, WIN_W - PPI_W, WIN_H)
    pygame.draw.rect(surface, (15, 15, 15), panel)
    pygame.draw.line(surface, (50, 50, 50), (PPI_W, 0), (PPI_W, WIN_H), 1)

    # 통신 상태
    colors = {'UDP': (0, 255, 0), 'UART': (0, 200, 255), 'OFFLINE': (255, 0, 0)}
    status_color = colors.get(link_status, (255, 0, 0))
    status_txt = link_status
    surface.blit(font.render(f'LINK: {status_txt}', True, status_color), (PPI_W + 10, 10))

    # 컨트롤 레이블 (슬라이더 1개만 남음)
    surface.blit(font.render('CONTROL', True, (0, 200, 80)), (PPI_W + 10, 40))
    for s in sliders:
        s.draw(surface, font_s)

    pygame.draw.line(surface, (40, 40, 40), (PPI_W + 10, 120), (WIN_W - 10, 120), 1)

    # ===== 핵심 추가: 타겟 정보 표시 (거리/각도/속도) =====
    surface.blit(font.render('RADAR DATA', True, (0, 255, 255)), (PPI_W + 10, 135))
    
    if not active_targets:
        surface.blit(font_s.render('No Targets Detected.', True, (100, 100, 100)), (PPI_W + 10, 165))
    
    y_offset = 165
    for tid, t in active_targets.items():
        # 데이터 계산
        dist_m = t['dist'] / 1000.0
        angle_deg = math.degrees(math.atan2(t['x'], t['y']))
        speed = t.get('speed', 0.0)

        # UI 텍스트 렌더링
        txt_id = font_s.render(f"ID {tid}:", True, (255, 255, 255))
        txt_pos = font_s.render(f"R: {dist_m:.1f}m | A: {angle_deg:+.1f}°", True, (200, 255, 200))
        txt_spd = font_s.render(f"Vel: {speed:.1f} m/s", True, (255, 150, 150))
        
        surface.blit(txt_id, (PPI_W + 10, y_offset))
        surface.blit(txt_pos, (PPI_W + 10, y_offset + 18))
        surface.blit(txt_spd, (PPI_W + 10, y_offset + 36))
        
        y_offset += 65
        pygame.draw.line(surface, (30, 30, 30), (PPI_W + 10, y_offset-10), (WIN_W - 10, y_offset-10), 1)

    # 하단 가이드
    pygame.draw.line(surface, (40, 40, 40), (PPI_W + 10, 580), (WIN_W - 10, 580), 1)
    surface.blit(font.render('GUIDE', True, (0, 200, 80)), (PPI_W + 10, 590))
    guides = ['클릭  : ★ 부여/해제', 'R     : 화면 초기화', 'q     : 프로그램 종료']
    for i, g in enumerate(guides):
        surface.blit(font_s.render(g, True, (120, 120, 120)), (PPI_W + 10, 615 + i * 20))

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption(f'Sensor Fusion Core - Phase 2 PPI | {COM_PORT}')
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont('consolas', 15, bold=True)
    font_s = pygame.font.SysFont('consolas', 12)

    heat     = np.zeros((PPI_W, WIN_H), dtype=np.float32)
    stars    = []
    
    # 활성 타겟 메모리 딕셔너리
    active_targets = {}

    PX = PPI_W + 15
    # 슬라이더 대폭 축소 (잔상 감쇠율만 남김)
    sliders = [
        Slider(PX, 90, 200, 'Afterglow Decay', 0.80, 0.999, 0.985, fmt='{:.3f}'),
    ]
    decay_s = sliders[0]

    # UDP socket (primary) — ptcamera_tracker.exe가 [RADAR] 라인을 여기로 relay
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(('127.0.0.1', UDP_PORT))
    udp_sock.setblocking(False)
    print(f'[OK] UDP 소켓 열림: 127.0.0.1:{UDP_PORT}')

    # Serial fallback — ptcamera_tracker 없이 단독 실행 시
    ser = None
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
        print(f'[OK] {COM_PORT} 직접 연결됨 (fallback)')
    except serial.SerialException as e:
        print(f'[정보] {COM_PORT} 직접 연결 없음 ({e}) — UDP 릴레이 대기 중')

    link_status = 'OFFLINE'

    distances = [2000, 4000, 6000, 8000]
    half_fov  = math.radians(FOV / 2)
    pattern   = r"\[RADAR\]\s*T(\d+):\((-?\d+),(-?\d+)\)mm"

    while True:
        # ===== 1. 이벤트 처리 =====
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                if ser: ser.close()
                udp_sock.close()
                pygame.quit(); sys.exit()
            
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                stars.clear()
                heat[:] = 0

            for s in sliders:
                s.handle_event(event)

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if mx < PPI_W:
                    removed = False
                    for star in stars:
                        sx, sy = star['pos']
                        if math.sqrt((mx-sx)**2 + (my-sy)**2) < STAR_RADIUS:
                            stars.remove(star)
                            removed = True
                            break
                    if not removed and len(stars) < MAX_STARS:
                        stars.append({'pos': (mx, my), 'id': len(stars)+1})

        # ===== 2. 레이더 데이터 수신 (UDP primary / serial fallback) =====
        now = time.time()

        def process_line(line):
            m = re.search(pattern, line)
            if not m:
                return
            tid, x_mm, y_mm = int(m.group(1)), int(m.group(2)), int(m.group(3))
            if x_mm == 0 and y_mm == 0:
                return
            if y_mm < 0 or y_mm > MAX_RANGE:
                return
            dist = math.sqrt(x_mm**2 + y_mm**2)
            speed = 0.0
            if tid in active_targets:
                prev = active_targets[tid]
                dt = now - prev['time']
                if dt > 0.05:
                    dx_m = (x_mm - prev['x']) / 1000.0
                    dy_m = (y_mm - prev['y']) / 1000.0
                    speed = math.sqrt(dx_m**2 + dy_m**2) / dt
            active_targets[tid] = {"x": x_mm, "y": y_mm, "dist": dist,
                                   "speed": speed, "time": now}

        # UDP receive (non-blocking)
        udp_got = False
        try:
            while True:
                data, _ = udp_sock.recvfrom(4096)
                for line in data.decode(errors='ignore').splitlines():
                    process_line(line.strip())
                udp_got = True
        except BlockingIOError:
            pass

        # Serial fallback (only if no UDP data this frame)
        serial_got = False
        if not udp_got and ser is not None:
            try:
                while ser.in_waiting > 0:
                    raw = ser.readline()
                    process_line(raw.decode(errors='ignore').strip())
                    serial_got = True
            except serial.SerialException:
                ser = None

        if udp_got:
            link_status = 'UDP'
        elif serial_got:
            link_status = 'UART'
        elif (now - max((t['time'] for t in active_targets.values()), default=0)) > 3.0:
            link_status = 'OFFLINE'

        # 오래된 타겟 정보 삭제 (0.5초 이상 미수신 시)
        for tid in list(active_targets.keys()):
            if now - active_targets[tid]["time"] > 0.5:
                del active_targets[tid]

        # ===== 3. Heatmap 업데이트 =====
        heat *= decay_s.value # 기존의 비율 감쇠
        heat -= 2.0           # [추가된 핵심] 찌꺼기를 강제로 갉아먹는 절대 감쇠
        heat = np.maximum(0, heat) # 0 이하로 내려가면 0으로 고정

        for tid, t in active_targets.items():
            dist, x_mm, y_mm = t["dist"], t["x"], t["y"]
            
            angle_deg = math.degrees(math.atan2(x_mm, y_mm))
            if abs(angle_deg) > FOV / 2:
                continue
                
            sx, sy = to_screen(x_mm, y_mm)
            if not (0 <= sx < PPI_W and 0 <= sy < WIN_H):
                continue

            # 가우시안 덧셈
            x0 = max(0, sx-GAUSS_R);  x1 = min(PPI_W, sx+GAUSS_R+1)
            y0 = max(0, sy-GAUSS_R);  y1 = min(WIN_H, sy+GAUSS_R+1)
            kx0 = x0-(sx-GAUSS_R);    kx1 = kx0+(x1-x0)
            ky0 = y0-(sy-GAUSS_R);    ky1 = ky0+(y1-y0)
            
            if x1 > x0 and y1 > y0:
                heat[x0:x1, y0:y1] = np.minimum(255, heat[x0:x1, y0:y1] + GAUSS[kx0:kx1, ky0:ky1])

        # ===== 4. 타겟(★) 트래킹 로직 =====
        surviving_stars = []
        for star in stars:
            sx, sy = star['pos']
            nx, ny = find_heat_centroid(heat, sx, sy, radius=40)
            
            # nx, ny가 None이 아닐 때만 살아남음 (열기가 식으면 자동 삭제)
            if nx is not None and ny is not None:
                star['pos'] = (nx, ny)
                surviving_stars.append(star)
                
        stars = surviving_stars # 살아남은 타겟만 리스트에 덮어쓰기

        # ===== 5. 화면 렌더링 =====
        screen.fill((0, 0, 0))

        # 잔상을 하얗게/밝게 표현 (Cyan/White 색상 매핑)
        heat_u8 = heat.astype(np.uint8)
        # R 채널은 조금 낮게, G와 B 채널을 높여서 밝은 청백색(Cyan/White) 발광 효과 구현
        rgb = np.stack([heat_u8 // 2, heat_u8, heat_u8], axis=-1)
        heat_surf = pygame.surfarray.make_surface(rgb)
        screen.blit(heat_surf, (0, 0))

        # 거리 링 및 레이블
        for r_mm in distances:
            r_px = int((r_mm / MAX_RANGE) * MAX_PX)
            rect = pygame.Rect(CX-r_px, CY-r_px, r_px*2, r_px*2)
            pygame.draw.arc(screen, (50, 80, 50), rect, math.radians(210), math.radians(330), 1)
            lx = int(CX + r_px * math.sin(half_fov)) + 5
            ly = int(CY - r_px * math.cos(half_fov))
            screen.blit(font_s.render(f'{r_mm//1000}m', True, (80,180,80)), (lx, ly))

        # 부채꼴 경계선
        for sign in [-1, 1]:
            ex = int(CX + MAX_PX * math.sin(sign * half_fov))
            ey = int(CY - MAX_PX * math.cos(half_fov))
            pygame.draw.line(screen, (0, 120, 0), (CX, CY), (ex, ey), 1)

        # 각도선
        for deg in [-60, -30, 0, 30, 60]:
            rad = math.radians(deg)
            ex = int(CX + MAX_PX * math.sin(rad))
            ey = int(CY - MAX_PX * math.cos(rad))
            color = (0, 180, 0) if deg == 0 else (0, 80, 0)
            pygame.draw.line(screen, color, (CX, CY), (ex, ey), 1)
            screen.blit(font_s.render(f'{deg}°', True, (60,150,60)), (ex-12, ey-22))

        pygame.draw.circle(screen, (0, 255, 0), (CX, CY), 5)
        screen.blit(font.render('RADAR', True, (100, 200, 100)), (CX+8, CY-12))

        # 추적 마커 렌더링
        for star in stars:
            sx, sy = star['pos']
            draw_star(screen, sx, sy, (255, 50, 50))
            screen.blit(font_s.render(f"T{star['id']}", True, (255, 200, 0)), (sx+16, sy-8))

        # 수신된 Raw 타겟 위치 표출
        for tid, t in active_targets.items():
            rx, ry = to_screen(t["x"], t["y"])
            pygame.draw.circle(screen, (0, 255, 255), (rx, ry), 5)
            screen.blit(font_s.render(f"RAW_{tid}", True, (0, 255, 255)), (rx+8, ry-8))
            
        # 컨트롤 패널
        draw_panel(screen, font, font_s, sliders, stars, active_targets, link_status)

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()