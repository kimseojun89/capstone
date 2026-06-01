# Anti-Drone System — 전체 구조 및 실행 흐름

---

## 1. 폴더 구조

> 2026-05-29 재구성: 문서는 `docs/`, 빌드 스크립트는 `scripts/`, 미사용은 `legacy/`로 분리.
> 런타임은 `antidrone/`에 유지. 전체 맵은 루트 [README.md](../README.md) 참고.

```
C:\Users\kimse\capstone\
│
├── README.md                        ← 프로젝트 진입점
├── docs/                            ← 설계·작업 문서 (이 문서 포함)
│   ├── SYSTEM_OVERVIEW.md           ← 이 문서
│   ├── INTEGRATION.md               ← 통합 설계 문서
│   ├── ONNX_CUDA_Migration.md
│   ├── FPGA_workflow.md
│   ├── motor_porting_guide.md
│   ├── motor_control_changes.md     ← 모터 제어 정본
│   └── OPEN_ISSUES.md               ← 미해결 항목 통합
├── scripts/                         ← 빌드 도구 (build_win.bat 등)
├── legacy/                          ← 미사용·레거시 격리
│
├── antidrone/                       ← ── 런타임 ──
│   ├── run_system.ps1               ← 통합 실행 스크립트 (진입점)
│   ├── flash.tcl                    ← XSCT용 FPGA 플래시 스크립트
│   ├── unified_gui.py               ← 통합 GUI (카메라 + PPI 한 창)
│   ├── requirements-ppi.txt         ← numpy, pygame, pyserial
│   ├── .venv/                       ← Python 가상환경
│   │
│   ├── cpp/
│   │   ├── CMakeLists.txt
│   │   ├── apps/ptcamera_tracker.cpp   ← 메인 PC 앱 소스
│   │   ├── src/  control.cpp(PID) · detector.cpp(YOLO ONNX) · pipeline.cpp
│   │   │         tracking.cpp(ByteTrack) · serial_port.cpp(T:/P:) · overlay · settings
│   │   ├── include/ptcamera/  settings.hpp(cameraIndex=1, 1920x1080) · serial_port.hpp
│   │   └── build_win/ptcamera_tracker.exe  ← 빌드 결과물
│   │
│   ├── models/drone_yolov8x/best.onnx  ← YOLOv8x 드론 감지 모델 (CUDA)
│   │
│   └── vitis_workspace/antidrone_app/
│       ├── src/ps_main.cpp          ← FPGA 베어메탈 소스
│       ├── build/antidrone_app.elf  ← 빌드 결과물
│       └── _ide/ bitstream/antidrone_wrapper.bit · psinit/ps7_init.tcl
│
├── Vitis/                           ← HLS IP 소스 (cordic/kalman/motor/mti)
├── vivado_project/                  ← Vivado 블록디자인·비트스트림
└── onnxruntime-gpu/                 ← ONNX Runtime GPU SDK
```

---

## 2. 하드웨어 연결 구조

```
[레이더 / 안테나]
       |
       | UART (256000 bps)
       v
[PYNQ-Z2 FPGA]  <---- USB JTAG (플래시 시에만)
       |
       |-- UART1 EMIO PMODA  <-- 레이더 패킷 수신
       |-- UART0 USB-Serial  <-> COM4 <-> [Windows PC]
       |-- HLS IP: CORDIC (방위각 계산)
       |-- HLS IP: Kalman  (표적 필터링)
       |-- HLS IP: ULN2003 (스테퍼 모터 드라이버)
       |-- HLS IP: MTI     (영상 모션 감지)
       |
       +--> Pan 모터 (수평 회전)
       +--> Tilt 모터 (수직 회전)

[Windows PC]
       |
       |-- USB 카메라: ABKO APC900 FHD (index 1, 1920x1080)
       |-- COM4: PYNQ-Z2 UART0 (256000 bps)
       |-- GPU: RTX 3050 4GB (YOLO CUDA 추론)
```

---

## 3. 실행 방법

### 명령어 하나로 전체 실행

```powershell
cd C:\Users\kimse\capstone\antidrone

# FPGA 플래시 포함 (코드 변경 후 or 첫 실행)
.\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor

# FPGA 이미 실행 중인 경우
.\run_system.ps1 -SerialPort COM4 -EnableMotor
```

### run_system.ps1 실행 순서

```
[1/3] xsct.bat flash.tcl
      → PYNQ-Z2에 antidrone_wrapper.bit + antidrone_app.elf 플래시
      → 완료 후 2초 대기

[2/3] .venv\Scripts\python.exe unified_gui.py
      → 통합 GUI 창 오픈 (1700x700)
      → UDP 9998, 9999, 10000 포트 바인딩

[3/3] ptcamera_tracker.exe --serial-port COM4 --baud 256000
                            --camera 1 --enable-motor
      → 카메라 오픈, YOLO 초기화, 시리얼 연결
      → 프레임 스트리밍 시작
```

---

## 4. 런타임 데이터 흐름

```
[레이더] --(UART1)--> [FPGA ps_main.cpp]
                              |
                   uart_parse() -> CORDIC -> Kalman
                   motor_update_hybrid / motor_update_full_host
                              |
                   xil_printf("[RADAR] T0:(x,y)mm ...")
                              |
                           UART0 COM4
                              |
                              v
              [ptcamera_tracker.exe]
                |                  |
      readAvailable()         Camera(idx1, 1920x1080)
      [RADAR] 라인 추출           |
                |           YOLOv8x ONNX (CUDA)
                |                 |
                |           ByteTracker
                |                 |
                |           ControlLoop PID
                |                 |
                |      motorEnabled 시:
                |        sendTiltCommand()  "T:±N\n" --> COM4 --> FPGA
                |        sendPanCommand()   "P:±N\n" --> COM4 --> FPGA
                |
         UDP 9999 [RADAR] 텍스트
         UDP 9998  JPEG 640x360 프레임
         UDP 10000 JSON 텔레메트리(FPS/AI/Motor/Serial)
                |
                v
        [unified_gui.py]
        +------------------+---------------------+
        |  카메라 패널      |    레이더 PPI 패널   |
        |  (700x700)       |    (1000x700)        |
        |                  |                      |
        |  JPEG 수신       |  히트맵 + 거리링      |
        |  YOLO 오버레이   |  trail + 선택 타겟   |
        |  AI/Motor 상태   |  RADAR 상태/HZ       |
        +------------------+---------------------+
                   1700 x 700 단일 창
```

---

## 5. FPGA 모터 제어 상태머신 (`ps_main.cpp:944~973`)

> 정본: [motor_control_changes.md](motor_control_changes.md). **Pan은 항상 안테나(레이더) 방위각 전용**,
> Tilt만 상황(카메라 퓨전 여부)에 따라 분기. **Pan/Tilt 모두 증분 PID**(`g_motor_abs_pan += dpan`).

| 모드 | 조건 | Pan 소스 | Tilt 소스 |
|---|---|---|---|
| **AI 트래킹** | `g_host_pan_steps != 0` & `cooldown==0` | PC `P:±N` (직접 스텝) | PC `T:±N` (직접 스텝) |
| **AI 잠금** | `cooldown>0` 또는 `ai_lock_frames>0` | — (현재 위치 유지, 레이더 차단) | — |
| **퓨전** | `fi >= 0` | 레이더 방위각 PID | 카메라 BBox `cy` PID |
| **레이더 단독** | `rvc > 0` & `fi < 0` | 레이더 방위각 PID | PC `T:±N` 명령 |
| **표적 없음** | `rvc == 0` | — (마지막 위치 유지) | — |

> **Pan은 절대 SET이 아니라 증분 PID.** 레이더 각도 `rang`(0.1도)을 LP필터(α=0.4)로 평활한 뒤
> `angle_to_px(rang)` 픽셀 오차를 `motor_update`/`motor_update_hybrid`의 PID에 입력한다(`KP=1.5, KD=0`).
> AI 명령 후에는 `cooldown`/`ai_lock_frames` 동안 레이더 오버라이드를 차단해 PC 추적 연속성을 보장.
> (구버전 문서의 `g_motor_abs_pan = -(rang/10×4096/360)` 절대 변환 서술은 현재 코드에 없음.)

---

## 6. 통신 프로토콜 정리

### PC → FPGA (UART0, COM4, 256000bps)

| 명령 | 형식 | 설명 |
|---|---|---|
| Tilt 명령 | `T:±N\n` | N = 스텝 수 (8~48), 음수=아래 |
| Pan 명령  | `P:±N\n` | N = 스텝 수 (8~48), 음수=왼쪽 |

### FPGA → PC (UART0, COM4, 256000bps)

| 로그 | 형식 | 설명 |
|---|---|---|
| 레이더 데이터 | `[RADAR] T0:(x,y)mm spd=Ncm/s` | 표적 좌표 (mm) |
| 상태 로그 | `[UART] [MTI] [FUSE] [KALM]` | 진단 정보 |

### PC 내부 UDP (localhost)

| 포트 | 방향 | 내용 |
|---|---|---|
| 9999 | tracker → unified_gui | `[RADAR]` 텍스트 라인 |
| 9998 | tracker → unified_gui | JPEG 640×360 카메라 프레임 |
| 10000 | tracker → unified_gui | JSON 텔레메트리(FPS, target lock, confidence, pan/tilt step, motor/serial 상태) |

### GUI 단독 점검

보드·카메라·COM4 없이 GUI 수신/렌더링만 확인할 때는 `scripts/gui_udp_sim.py`를 사용한다.

```powershell
# 터미널 1
.\antidrone\.venv\Scripts\python.exe .\antidrone\unified_gui.py

# 터미널 2
.\antidrone\.venv\Scripts\python.exe .\scripts\gui_udp_sim.py
```

---

## 7. 재빌드 기준

| 수정 파일 | 재빌드 방법 |
|---|---|
| `ps_main.cpp` | Vitis 2023.2 빌드 → `run_system.ps1 -Flash` |
| `ptcamera_tracker.cpp` 등 C++ | VS2022 BuildTools 환경에서 `nmake /nologo ptcamera_tracker` |
| `unified_gui.py` | 재빌드 없음 — 저장 즉시 반영 |
| `run_system.ps1` | 재빌드 없음 — 저장 즉시 반영 |

### C++ 빌드 명령 (전체)

```powershell
cd C:\Users\kimse\capstone\antidrone\cpp

# 최초 CMake 구성 (최초 1회)
cmake -B build_win -G "NMake Makefiles" `
    -DCMAKE_BUILD_TYPE=Release `
    -DOpenCV_DIR="C:/opencv/build"

# 이후 재빌드
cmake --build build_win --target ptcamera_tracker
```

---

## 8. 주요 설정값

| 항목 | 값 | 위치 |
|---|---|---|
| 카메라 인덱스 | 1 (ABKO APC900) | `settings.hpp` |
| 카메라 해상도 | 1920 × 1080 | `settings.hpp` |
| UART baud | 256000 bps | `settings.hpp`, `ps_main.cpp` |
| 시리얼 포트 | COM4 | `run_system.ps1` |
| YOLO 모델 | `models/drone_yolov8x/best.onnx` | `settings.cpp` |
| 추론 장치 | CUDA (RTX 3050) | `settings.hpp` |
| 모터 속도 딜레이 | 200 (154°/s; 탈조 시 300으로) | `ps_main.cpp:120` |
| 모터 PID KP / KD / MAX_STEP | KP=1.5 / KD=0.0 / 58 | `ps_main.cpp:101,102,108` |
| 레이더 rang LP필터 | α=0.4 (지터 억제) | `ps_main.cpp:922` |
| Pan deadband | 35 px | `ps_main.cpp:103`, `settings.hpp` |
| Tilt deadband | 35 px | `ps_main.cpp:104`, `settings.hpp` |
| 레이더 최대 거리 | 8000 mm | `unified_gui.py` |
| PPI FOV | 120 도 | `unified_gui.py` |

---

## 9. 문제 해결

| 증상 | 원인 | 해결 |
|---|---|---|
| 카메라가 노트북 웹캠으로 나옴 | `cameraIndex=0` | `settings.hpp`에서 `cameraIndex=1` 확인 |
| PPI 창이 안 뜸 | `.venv` Python 미사용 | `run_system.ps1`의 `$PYTHON` 경로 확인 |
| 모터가 떨리고 안 돌아감 (탈조) | `MOTOR_SPEED_DELAY` 너무 작음 | `ps_main.cpp`에서 200 → 300, MAX_STEP 58 → 38로 변경 후 재빌드 |
| `LINK: OFFLINE` 표시 | ptcamera_tracker 미실행 or COM4 미연결 | 시리얼 케이블 및 트래커 실행 확인 |
| CUDA 없이 추론 | GPU 드라이버 or ONNX Runtime 문제 | `--device CPU`로 fallback 가능 |
| COM4 포트 충돌 | Tera Term 등 다른 터미널이 COM4 점유 | 다른 시리얼 터미널 종료 후 재실행 |
