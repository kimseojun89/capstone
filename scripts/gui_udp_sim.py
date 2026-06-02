#!/usr/bin/env python3
"""
Send synthetic camera, radar, and tracker telemetry packets to unified_gui.py.

Usage:
  python scripts/gui_udp_sim.py

Open antidrone/unified_gui.py first, then run this script from another shell.
It does not touch the FPGA, camera, serial port, or model runtime.
"""
import argparse
import json
import math
import socket
import struct
import time


CAM_PORT = 9998
RADAR_PORT = 9999
TELEM_PORT = 10000


def put_pixel(buf, w, h, x, y, color):
    if 0 <= x < w and 0 <= y < h:
        idx = (y * w + x) * 3
        buf[idx:idx + 3] = bytes(color)


def draw_rect(buf, w, h, x0, y0, x1, y1, color):
    for x in range(x0, x1 + 1):
        put_pixel(buf, w, h, x, y0, color)
        put_pixel(buf, w, h, x, y1, color)
    for y in range(y0, y1 + 1):
        put_pixel(buf, w, h, x0, y, color)
        put_pixel(buf, w, h, x1, y, color)


def make_bmp_frame(t, w=160, h=90):
    rgb = bytearray(w * h * 3)
    for y in range(h):
        for x in range(w):
            idx = (y * w + x) * 3
            rgb[idx] = 12 + (x * 35 // w)
            rgb[idx + 1] = 24 + (y * 45 // h)
            rgb[idx + 2] = 28

    cx = int(w * 0.5 + math.sin(t * 0.9) * w * 0.28)
    cy = int(h * 0.5 + math.cos(t * 0.7) * h * 0.20)
    draw_rect(rgb, w, h, cx - 14, cy - 9, cx + 14, cy + 9, (255, 210, 80))
    for dx in range(-3, 4):
        put_pixel(rgb, w, h, cx + dx, cy, (0, 255, 255))
        put_pixel(rgb, w, h, cx, cy + dx, (0, 255, 255))

    row_stride = (w * 3 + 3) & ~3
    pixel_bytes = bytearray(row_stride * h)
    for y in range(h):
        src_y = h - 1 - y
        for x in range(w):
            src = (src_y * w + x) * 3
            dst = y * row_stride + x * 3
            r, g, b = rgb[src], rgb[src + 1], rgb[src + 2]
            pixel_bytes[dst:dst + 3] = bytes((b, g, r))

    file_size = 54 + len(pixel_bytes)
    header = (
        b'BM'
        + struct.pack('<IHHI', file_size, 0, 0, 54)
        + struct.pack('<IIIHHIIIIII', 40, w, h, 1, 24, 0,
                      len(pixel_bytes), 2835, 2835, 0, 0)
    )
    return header + pixel_bytes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds; 0 means run until Ctrl+C')
    parser.add_argument('--send-z', action='store_true',
                        help='include synthetic z_mm/altitude_mm for 3D GUI testing')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cam_addr = (args.host, CAM_PORT)
    radar_addr = (args.host, RADAR_PORT)
    telem_addr = (args.host, TELEM_PORT)

    start = time.time()
    next_frame = start
    frame = 0
    print(f'[sim] sending camera:{CAM_PORT} radar:{RADAR_PORT} telemetry:{TELEM_PORT}')
    try:
        while True:
            now = time.time()
            if args.duration > 0 and now - start >= args.duration:
                break
            if now < next_frame:
                time.sleep(min(0.01, next_frame - now))
                continue

            t = now - start
            x_mm = int(math.sin(t * 0.8) * 2200)
            y_mm = int(4200 + math.cos(t * 0.55) * 1100)
            z_mm = int(900 + math.sin(t * 0.45) * 500)
            speed = int(120 + 35 * math.sin(t * 1.7))
            if args.send_z:
                line = f'[RADAR] T0:({x_mm},{y_mm},{z_mm})mm spd={speed}cm/s\n'
            else:
                line = f'[RADAR] T0:({x_mm},{y_mm})mm spd={speed}cm/s\n'
            telemetry = {
                'fps': args.fps,
                'target_found': True,
                'detections': 1,
                'tracks': 1,
                'target_id': 0,
                'confidence': round(0.65 + 0.25 * (0.5 + 0.5 * math.sin(t)), 3),
                'source': 'sim',
                'center_x': 320 + math.sin(t * 0.9) * 120,
                'center_y': 180 + math.cos(t * 0.7) * 60,
                'pan_steps': int(math.sin(t * 0.9) * 28),
                'tilt_steps': int(math.cos(t * 0.7) * 18),
                'motor_enabled': True,
                'serial_open': True,
                'device': 'SIM',
            }
            if args.send_z:
                telemetry['target_id'] = 0
                telemetry['z_mm'] = z_mm

            sock.sendto(make_bmp_frame(t), cam_addr)
            sock.sendto(line.encode('ascii'), radar_addr)
            sock.sendto(json.dumps(telemetry).encode('ascii'), telem_addr)

            frame += 1
            next_frame += 1.0 / max(args.fps, 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print(f'[sim] sent {frame} frames')


if __name__ == '__main__':
    main()
