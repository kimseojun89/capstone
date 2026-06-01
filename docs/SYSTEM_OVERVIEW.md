# Anti-Drone System — 통합 구조 및 실행 흐름

이 문서는 **카메라 기반 AI 추적 PC 앱**, **PYNQ-Z2 FPGA 제어부**, **레이더 수신부**, **통합 GUI**의 현재 통합 구조와 실행 흐름을 정리한 문서이다.

상세 상태머신과 모터 제어 기준은 다음 문서를 정본으로 한다.

* [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)
* [motor_control.md](motor_control.md)
* [P0_PLAN.md](P0_PLAN.md)

---

## 1. 시스템 개요

### 1.1 동작 모드

| 모드         | 제어 소스                        | 경로                                                   |
| ---------- | ---------------------------- | ---------------------------------------------------- |
| **AI 추적**  | PC YOLO/ByteTrack bbox 중심 오차 | PC `B:ex,ey` → FPGA `motor_update_ai_bbox()`         |
| **수동 점검**  | PC 방향키/캘리브레이션 명령             | PC `M:1`, `P:/T:` → FPGA `motor_update_full_host()`  |
| **레이더 단독** | 레이더 방위각                      | UART1 → CORDIC/Kalman → FPGA `motor_update_hybrid()` |

AI 추적 모드에서는 PC가 bbox 중심 오차만 FPGA로 전송하고, FPGA가 Pan/Tilt PID 및 스텝 계산을 담당한다.

---

## 2. 폴더 구조

문서는 `docs/`, 빌드 스크립트는 `scripts/`, 미사용 코드는 `legacy/`로 분리한다.

```
C:\Users\kimse\capstone\
│
├── docs/                            ← 설계·작업 문서
│   ├── README.md                    ← 문서 진입점
│   ├── SYSTEM_OVERVIEW.md           ← 전체 구조 문서
│   ├── INTEGRATION.md               ← 통합 설계 문서
│   ├── ONNX_CUDA_Migration.md
│   ├── FPGA_workflow.md
│   ├── motor_control.md             ← 모터 제어 정본
│   ├── P0_PLAN.md                   ← P0 통합 계획
│   └── OPEN_ISSUES.md               ← 미해결 항목 통합
│
├── scripts/                         ← 빌드·점검 도구
│   ├── build_win.bat
│   └── gui_udp_sim.py
│
├── legacy/                          ← 미사용·레거시 코드 격리
│
├── antidrone/                       ← 런타임 디렉터리
│   ├── run_system.ps1               ← 통합 실행 스크립트
│   ├── flash.tcl                    ← XSCT용 FPGA 플래시 스크립트
│   ├── unified_gui.py               ← 통합 GUI
│   ├── requirements-ppi.txt         ← GUI 의존성
│   ├── .venv/                       ← Python 가상환경
│   │
│   ├── cpp/
│   │   ├── CMakeLists.txt
│   │   ├── apps/
│   │   │   └── ptcamera_tracker.cpp ← 메인 PC 앱 소스
│   │   ├── src/
│   │   │   ├── detector.cpp         ← YOLO ONNX 추론
│   │   │   ├── pipeline.cpp
│   │   │   ├── tracking.cpp         ← ByteTrack
│   │   │   ├── serial_port.cpp      ← B:/P:/T:/M: 송수신
│   │   │   ├── overlay.cpp
│   │   │   └── settings.cpp
│   │   ├── include/ptcamera/
│   │   │   ├── settings.hpp         ← cameraIndex=1, 1920x1080 등
│   │   │   └── serial_port.hpp
│   │   └── build_win/
│   │       └── ptcamera_tracker.exe ← 빌드 결과물
│   │
│   ├── models/
│   │   └── drone_yolov8x/
│   │       └── best.onnx            ← YOLOv8x 드론 감지 모델
│   │
│   └── vitis_workspace/antidrone_app/
│       ├── src/
│       │   └── ps_main.cpp          ← FPGA 베어메탈 소스
│       ├── build/
│       │   └── antidrone_app.elf    ← 빌드 결과물
│       └── _ide/
│           ├── bitstream/
│           │   └── antidrone_wrapper.bit
│           └── psinit/
│               └── ps7_init.tcl
│
├── Vitis/                           ← HLS IP 소스
├── vivado_project/                  ← Vivado 블록디자인·비트스트림
└── onnxruntime-gpu/                 ← ONNX Runtime GPU SDK
```

---

## 3. 하드웨어 연결 구조

```
[레이더 / 안테나]
       |
       | UART 256000 bps
       v
[PYNQ-Z2 FPGA]  <---- USB JTAG, 플래시 시에만 사용
       |
       |-- UART1 EMIO PMODA  <-- 레이더 패킷 수신
       |-- UART0 USB-Serial  <-> COM4 <-> [Windows PC]
       |-- HLS IP: CORDIC    ← 방위각 계산
       |-- HLS IP: Kalman    ← 표적 필터링
       |-- HLS IP: ULN2003   ← 스테퍼 모터 드라이버
       |
       +--> Pan 모터
       +--> Tilt 모터

[Windows PC]
       |
       |-- USB 카메라: ABKO APC900 FHD, index 1, 1920x1080
       |-- COM4: PYNQ-Z2 UART0, 256000 bps
       |-- GPU: RTX 3050 4GB, YOLO CUDA 추론
```

---

## 4. 전체 통합 구조

```
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

---

## 5. 런타임 데이터 흐름

### 5.1 FPGA → PC → GUI 흐름

```
[레이더] --UART1--> [FPGA ps_main.cpp]
                         |
              uart_parse() -> CORDIC -> Kalman
                         |
              motor_update_ai_bbox()
              motor_update_hybrid()
                         |
              xil_printf("[RADAR] T0:(x,y)mm spd=Ncm/s")
                         |
                      UART0 COM4
                         |
                         v
              [ptcamera_tracker.exe]
                |                  |
        readAvailable()       Camera index 1
        [RADAR] 라인 추출          |
                |          YOLOv8x ONNX CUDA
                |                  |
                |             ByteTrack
                |                  |
                |        bbox 중심 오차 계산
                |                  |
                |   motorEnabled 시 `B:ex,ey\n`
                |        COM4 --> FPGA
                |
        UDP 9999  [RADAR] 텍스트
        UDP 9998  JPEG 640x360 카메라 프레임
        UDP 10000 JSON 텔레메트리
                |
                v
        [unified_gui.py]
```

### 5.2 PC 내부 UDP 흐름

```
[FPGA UART0] --COM4---> [ptcamera_tracker.exe]
                              |          |             |
                         bbox 명령 전송  [RADAR] 추출  카메라/AI 상태
                         "B:ex,ey\n"     |             |
                                    UDP:9999       UDP:9998 / 10000
                                         |             |
                                   [unified_gui.py]
                                   카메라 + PPI + 텔레메트리 표시
```

---

## 6. COM4 독점 구조

`ptcamera_tracker.exe`가 COM4를 단독으로 연다.

`unified_gui.py`는 시리얼 포트를 열지 않는다. GUI는 반드시 UDP만 수신한다.

| 포트        | 방향                    | 내용                             |
| --------- | --------------------- | ------------------------------ |
| COM4      | PC ↔ FPGA             | `ptcamera_tracker.exe` 단독 사용   |
| UDP 9998  | tracker → unified_gui | 오버레이가 그려진 카메라 JPEG 프레임         |
| UDP 9999  | tracker → unified_gui | FPGA 로그에서 추출한 `[RADAR]` 텍스트 라인 |
| UDP 10000 | tracker → unified_gui | JSON 텔레메트리                     |

COM4를 Tera Term, PuTTY, 별도 Python 스크립트, `unified_gui.py` 등에서 동시에 열면 포트 충돌이 발생한다.

---

## 7. 통합 GUI 구성

`unified_gui.py`는 카메라 화면, 레이더 PPI, 텔레메트리를 하나의 창에 표시한다.

```
+------------------+---------------------+
|  카메라 패널      |    레이더 PPI 패널   |
|  700 x 700       |    1000 x 700       |
|                  |                     |
|  JPEG 수신       |  히트맵 + 거리링     |
|  YOLO 오버레이   |  trail + 선택 타겟   |
|  AI/Motor 상태   |  RADAR 상태/HZ       |
+------------------+---------------------+
           1700 x 700 단일 창
```

---

## 8. 통신 프로토콜

### 8.1 PC → FPGA, UART0 COM4 256000bps

| 명령         | 형식          | 설명                           |
| ---------- | ----------- | ---------------------------- |
| AI 추적      | `B:ex,ey\n` | bbox 중심 오차. FPGA가 PID와 스텝 계산 |
| 수동 모드 진입   | `M:1\n`     | 수동 점검/캘리브레이션 모드 진입           |
| 수동 모드 해제   | `M:0\n`     | 자동 추적 모드 복귀                  |
| Pan 상대 스텝  | `P:±N\n`    | 수동 모드 전용 Pan 이동              |
| Tilt 상대 스텝 | `T:±N\n`    | 수동 모드 전용 Tilt 이동             |

`P:/T:`는 `M:1` 수동 모드에서 캘리브레이션과 점검 용도로만 사용한다.

### 8.2 FPGA → PC, UART0 COM4 256000bps

| 로그      | 형식                             | 설명         |
| ------- | ------------------------------ | ---------- |
| 레이더 데이터 | `[RADAR] T0:(x,y)mm spd=Ncm/s` | 표적 좌표 및 속도 |
| 상태 로그   | `[UART]`, `[KALM]`             | 진단 정보      |

`[MTI]`, `[FUSE]` 계열 로그는 현재 구조에서는 레거시 항목으로 취급한다.

### 8.3 PC 내부 UDP

| UDP 포트 | 방향                    | 내용                   |
| ------ | --------------------- | -------------------- |
| 9998   | tracker → unified_gui | JPEG 640×360 카메라 프레임 |
| 9999   | tracker → unified_gui | `[RADAR]` 텍스트 라인     |
| 10000  | tracker → unified_gui | JSON 텔레메트리           |

텔레메트리에는 FPS, AI lock, confidence, bbox 오차, motor 상태 등이 포함된다.

---

## 9. FPGA 모터 제어 상태머신

정본은 [motor_control.md](motor_control.md)를 따른다.

| 모드         | 조건                                     | Pan 소스          | Tilt 소스         |
| ---------- | -------------------------------------- | --------------- | --------------- |
| **AI 추적**  | `B:ex,ey` 수신 + `cooldown == 0`         | bbox `ex` PID   | bbox `ey` PID   |
| **수동**     | `M:1` + `P:/T:` + `cooldown == 0`      | PC `P:±N` 직접 스텝 | PC `T:±N` 직접 스텝 |
| **AI 잠금**  | `cooldown > 0` 또는 `ai_lock_frames > 0` | 현재 위치 유지        | 현재 위치 유지        |
| **레이더 단독** | `rvc > 0`                              | 레이더 방위각 PID     | 정지              |
| **표적 없음**  | `rvc == 0`                             | 마지막 위치 유지       | 마지막 위치 유지       |

레이더 `rang`은 LP 필터로 평활한 뒤 `angle_to_px()`를 통해 픽셀 오차로 변환하고, 이를 `motor_update_hybrid()`에 입력한다.

AI 명령 후 `cooldown` 또는 `ai_lock_frames` 동안은 레이더 오버라이드를 차단한다.

---

## 10. 실행 방법

### 10.1 전체 실행

```powershell
cd C:\Users\kimse\capstone\antidrone

# FPGA 플래시 포함, 코드 변경 후 또는 첫 실행
.\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor

# FPGA가 이미 실행 중이면 플래시 생략
.\run_system.ps1 -SerialPort COM4 -EnableMotor
```

### 10.2 Preflight 점검

```powershell
cd C:\Users\kimse\capstone\antidrone

.\run_system.ps1 -Preflight
```

### 10.3 run_system.ps1 실행 순서

```
[1/3] xsct.bat flash.tcl
      → -Flash 옵션이 있을 때만 실행
      → PYNQ-Z2에 antidrone_wrapper.bit + antidrone_app.elf 플래시
      → 완료 후 2초 대기

[2/3] .venv\Scripts\python.exe unified_gui.py
      → 통합 GUI 창 오픈
      → UDP 9998, 9999, 10000 포트 바인딩

[3/3] ptcamera_tracker.exe --serial-port COM4 --baud 256000
                            --camera 1 --enable-motor
      → 카메라 오픈
      → YOLO 초기화
      → 시리얼 연결
      → 프레임 스트리밍 및 bbox 명령 송신 시작
```

---

## 11. 검증 순서

1. Windows C++ 앱 빌드

```powershell
.\scripts\build_win.bat
```

1. Preflight 실행

```powershell
.\antidrone\run_system.ps1 -Preflight
```

1. 보드 없이 GUI UDP 화면 점검

```powershell
# 터미널 1
.\antidrone\.venv\Scripts\python.exe .\antidrone\unified_gui.py

# 터미널 2
.\antidrone\.venv\Scripts\python.exe .\scripts\gui_udp_sim.py
```

1. 보드 연결 후 `ptcamera_tracker.exe`가 COM4를 여는지 확인

2. `ptcamera_tracker.exe`가 `B:ex,ey`를 FPGA로 송신하는지 확인

3. GUI가 다음 항목을 UDP로 표시하는지 확인

| 항목           | 확인 포트     |
| ------------ | --------- |
| 카메라 JPEG 프레임 | UDP 9998  |
| 레이더 PPI      | UDP 9999  |
| 텔레메트리        | UDP 10000 |

---

## 12. 재빌드 기준

| 수정 파일                        | 재빌드 방법                                    |
| ---------------------------- | ----------------------------------------- |
| `ps_main.cpp`                | Vitis 2023.2 빌드 → `run_system.ps1 -Flash` |
| `ptcamera_tracker.cpp` 등 C++ | VS2022 BuildTools 환경에서 CMake/NMake 빌드     |
| `unified_gui.py`             | 재빌드 없음. 저장 후 재실행                          |
| `run_system.ps1`             | 재빌드 없음. 저장 후 재실행                          |

### C++ 전체 빌드 명령

```powershell
cd C:\Users\kimse\capstone\antidrone\cpp

# 최초 CMake 구성
cmake -B build_win -G "NMake Makefiles" `
    -DCMAKE_BUILD_TYPE=Release `
    -DOpenCV_DIR="C:/opencv/build"

# 이후 재빌드
cmake --build build_win --target ptcamera_tracker
```

또는 통합 빌드 스크립트를 사용한다.

```powershell
.\scripts\build_win.bat
```

---

## 13. 주요 설정값

| 항목                        | 값                                        | 위치                            |
| ------------------------- | ---------------------------------------- | ----------------------------- |
| 카메라 인덱스                   | 1, ABKO APC900                           | `settings.hpp`                |
| 카메라 해상도                   | 1920 × 1080                              | `settings.hpp`                |
| UART baud                 | 256000 bps                               | `settings.hpp`, `ps_main.cpp` |
| 시리얼 포트                    | COM4                                     | `run_system.ps1`              |
| YOLO 모델                   | `models/drone_yolov8x/best.onnx`         | `settings.cpp`                |
| 추론 장치                     | CUDA, RTX 3050                           | `settings.hpp`                |
| 모터 속도 딜레이                 | 200, 탈조 시 300                            | `ps_main.cpp`                 |
| 모터 PID KP / KD / MAX_STEP | KP=1.5 / KD=0.0 / MAX_STEP=58            | `ps_main.cpp`                 |
| 레이더 rang LP 필터            | α=0.4                                    | `ps_main.cpp`                 |
| Pan deadband              | 35 px                                    | `ps_main.cpp`, `settings.hpp` |
| Tilt deadband             | 35 px                                    | `ps_main.cpp`, `settings.hpp` |
| 각도/스텝 변환                  | `STEPS_PER_DEGREE = 4096 / 360 ≈ 11.378` | `control.hpp`                 |
| 수동 모드 전역                  | `g_manual_mode`                          | `ps_main.cpp`                 |
| 레이더 최대 거리                 | 8000 mm                                  | `unified_gui.py`              |
| PPI FOV                   | 120°                                     | `unified_gui.py`              |

---

## 14. 문제 해결

| 증상                 | 원인                                       | 해결                                                      |
| ------------------ | ---------------------------------------- | ------------------------------------------------------- |
| 카메라가 노트북 웹캠으로 나옴   | `cameraIndex=0`                          | `settings.hpp`에서 `cameraIndex=1` 확인                     |
| GUI 또는 PPI가 안 뜸    | `.venv` Python 미사용                       | `run_system.ps1`의 `$PYTHON` 경로 확인                       |
| 모터가 떨리고 안 돌아감      | `MOTOR_SPEED_DELAY`가 너무 작음               | `ps_main.cpp`에서 200 → 300, `MAX_STEP` 58 → 38로 변경 후 재빌드 |
| `LINK: OFFLINE` 표시 | `ptcamera_tracker.exe` 미실행 또는 COM4 미연결   | 시리얼 케이블 및 트래커 실행 확인                                     |
| CUDA 없이 추론됨        | GPU 드라이버 또는 ONNX Runtime 문제              | `--device CPU`로 fallback 가능                             |
| COM4 포트 충돌         | Tera Term, PuTTY, Python 스크립트 등이 COM4 점유 | 다른 시리얼 프로그램 종료 후 재실행                                    |
| GUI에 레이더가 표시되지 않음  | tracker가 `[RADAR]` 라인을 UDP 9999로 중계하지 않음 | tracker 로그와 UDP 9999 수신 여부 확인                           |
| GUI에 카메라가 표시되지 않음  | UDP 9998 프레임 미수신 또는 카메라 오픈 실패            | 카메라 index, tracker 실행 상태 확인                             |
| 텔레메트리가 비어 있음       | UDP 10000 JSON 송신 실패                     | tracker telemetry 송신 로직 확인                              |

---

## 15. 운영 규칙 요약

1. COM4는 `ptcamera_tracker.exe`만 연다.
2. `unified_gui.py`는 UDP만 수신한다.
3. AI 추적에서는 PC가 bbox 오차만 보내고, FPGA가 PID를 수행한다.
4. 수동 `P:/T:` 명령은 `M:1` 수동 모드에서만 사용한다.
5. AI lock 또는 cooldown 중에는 레이더 오버라이드를 차단한다.
6. 보드 없이 GUI를 점검할 때는 `scripts/gui_udp_sim.py`를 사용한다.
7. FPGA 코드 변경 후에는 Vitis 빌드와 `run_system.ps1 -Flash`가 필요하다.
8. C++ 앱 변경 후에는 `ptcamera_tracker.exe`를 재빌드해야 한다.

---

## 16. 권장 실행 체크리스트

```powershell
cd C:\Users\kimse\capstone

# 1. C++ 빌드
.\scripts\build_win.bat

# 2. 사전 점검
.\antidrone\run_system.ps1 -Preflight

# 3. 첫 실행 또는 FPGA 코드 변경 후
.\antidrone\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor

# 4. 이후 일반 실행
.\antidrone\run_system.ps1 -SerialPort COM4 -EnableMotor
```
