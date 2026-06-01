# Anti-drone System Integration

카메라(PC) + FPGA(PYNQ-Z2) 현재 통합 구조 문서.

## 시스템 개요

| 모드 | 제어 소스 | 경로 |
|---|---|---|
| **AI 추적** | PC YOLO/ByteTrack bbox 중심 오차 | PC `B:ex,ey` → FPGA `motor_update_ai_bbox()` |
| **수동 점검** | PC 방향키/캘리브레이션 명령 | PC `M:1`, `P:/T:` → FPGA `motor_update_full_host()` |
| **레이더 단독** | 레이더 방위각 | UART1 → CORDIC/Kalman → FPGA `motor_update_hybrid()` |

상세 상태머신은 [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)와 [motor_control.md](motor_control.md)를 기준으로 한다.

```text
[Radar/Antenna] --UART1 256000bps--> [FPGA ps_main.cpp]
                                            |
[Windows PC]                                |
  Camera -> YOLOv8x -> ByteTrack            |
  sendBBox("B:ex,ey\n")                    |
  --UART0 256000bps COM4----------------> [FPGA ps_main.cpp]
                                            |
                                 motor_update_ai_bbox()
                                  Pan/Tilt: FPGA PID
                                            |
                                 [uln2003_controller HLS IP]
                                 [Pan Motor]   [Tilt Motor]
```

## 데이터 흐름

```text
[FPGA UART0] --COM4---> [ptcamera_tracker.exe]
                              |          |             |
                         bbox 명령 전송  [RADAR] 추출  카메라/AI 상태
                         "B:ex,ey\n"     |             |
                                    UDP:9999       UDP:9998 / 10000
                                         |             |
                                   [unified_gui.py]
                                   카메라 + PPI + 텔레메트리 표시
```

**COM4 독점 구조:** `ptcamera_tracker.exe`가 COM4를 단독으로 열고, FPGA 로그에서 `[RADAR]` 라인을 추출해 UDP `127.0.0.1:9999`로 중계한다. `unified_gui.py`는 UDP만 수신해야 하며, 시리얼 포트는 열지 않는다.

| UDP 포트 | 방향 | 내용 |
|---|---|---|
| 9998 | tracker → unified_gui | 오버레이가 그려진 카메라 JPEG 프레임 |
| 9999 | tracker → unified_gui | `[RADAR]` 텍스트 라인 |
| 10000 | tracker → unified_gui | JSON 텔레메트리(FPS, AI lock, confidence, bbox 오차, motor 상태) |

## PC → FPGA 프로토콜

| 명령 | 용도 |
|---|---|
| `B:ex,ey\n` | AI 추적. bbox 중심 오차를 320x240 기준으로 스케일해 전송 |
| `M:1\n` / `M:0\n` | 수동 모드 진입/해제 |
| `P:±N\n` | 수동 모드 Pan 상대 스텝 |
| `T:±N\n` | 수동 모드 Tilt 상대 스텝 |

## 실행 순서

```powershell
cd C:\Users\kimse\capstone\antidrone
.\run_system.ps1 -Preflight
.\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor
```

플래시가 필요 없으면 `-Flash`를 생략한다.

## 검증 순서

1. `.\scripts\build_win.bat`
2. `.\antidrone\run_system.ps1 -Preflight`
3. 보드 없이 `unified_gui.py`와 `scripts/gui_udp_sim.py`로 UDP 화면 점검
4. 보드 연결 후 `ptcamera_tracker.exe`가 COM4를 열고 `B:ex,ey`를 송신하는지 확인
5. GUI가 카메라, PPI, 텔레메트리를 UDP로 표시하는지 확인
