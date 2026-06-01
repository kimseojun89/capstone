# Capstone — Anti-Drone Pan/Tilt Tracking System

레이더 + 카메라(YOLOv8x) 융합으로 드론을 탐지·추적하고 Pan/Tilt 스테퍼 모터를 구동하는 시스템.
PC(Windows)에서 영상 추론·관제, PYNQ-Z2 FPGA에서 신호처리·모터 제어를 담당한다.

```
[레이더] ─UART1→ [PYNQ-Z2 FPGA] ─UART0(COM4)→ [Windows PC] ─UDP→ [통합 GUI]
                  CORDIC/Kalman/모터          YOLOv8x(CUDA)+ByteTrack       카메라+PPI
                  (ps_main.cpp)               (ptcamera_tracker.exe)        (unified_gui.py)
```

자세한 구조·프로토콜은 **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)** 참고.

---

## 폴더 구조

```
capstone/
├── CLAUDE.md              ← 세션 기준 요약
├── docs/                  ← 모든 설계·작업 문서
│   ├── README.md                  이 문서 (문서 진입점)
│   ├── SYSTEM_OVERVIEW.md         전체 구조·실행·프로토콜 (기준 문서)
│   ├── INTEGRATION.md             PC↔FPGA 현재 통합 구조
│   ├── ONNX_CUDA_Migration.md     OpenVINO→ONNX CUDA 전환 기록
│   ├── FPGA_workflow.md           HLS→Vivado→Vitis 빌드 흐름
│   ├── motor_control.md           모터 제어 정본
│   ├── P0_PLAN.md                 P0 통합 계획
│   ├── PANTILT_CALIBRATION_WORKFLOW.md Pan/Tilt 웹캠 ChArUco 캘리브레이션 워크플로우
│   └── OPEN_ISSUES.md             미해결 항목 통합 ★
├── scripts/               ← 빌드·점검 도구
│   ├── build_win.bat          호스트 C++ 앱 빌드 (Windows)
│   ├── gui_udp_sim.py         통합 GUI용 가짜 UDP 송신기 (보드 없이 화면 점검)
│   ├── run_build.bat          FPGA 앱 빌드 (Vitis pyesw)
│   ├── build_app.{py,tcl}     FPGA 앱 빌드 (대체 방식)
│   └── recreate_platform.tcl  Vitis 플랫폼 XSA 재생성
├── legacy/                ← 미사용·레거시 격리 (legacy/README.md 참고)
├── antidrone/            ← 런타임
│   ├── run_system.ps1        ★ 단일 실행 진입점
│   ├── unified_gui.py        통합 GUI (카메라+PPI)
│   ├── flash.tcl             FPGA 플래시 스크립트
│   ├── cpp/                  호스트 C++ 소스 + build_win/(빌드 산출)
│   ├── models/drone_yolov8x/ best.onnx (YOLOv8x)
│   └── vitis_workspace/      FPGA 베어메탈 (ps_main.cpp)
├── Vitis/                ← HLS IP 소스 (cordic/kalman/motor/mti)
├── vivado_project/       ← Vivado 블록디자인·비트스트림
└── onnxruntime-gpu/      ← ONNX Runtime GPU SDK (다운로드)
```

---

## 실행 (단일 파이프라인)

```powershell
cd C:\Users\kimse\capstone\antidrone

# 0) 하드웨어 무접촉 사전점검만 (파일·환경 검증)
.\run_system.ps1 -Preflight

# 1) FPGA 플래시 포함 전체 실행 (코드 변경 후/첫 실행)
.\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor

# 2) FPGA 이미 실행 중이면 플래시 생략
.\run_system.ps1 -SerialPort COM4 -EnableMotor
```

실행 순서: FPGA 플래시 → `unified_gui.py`(UDP 9998/9999/10000) → `ptcamera_tracker.exe`(COM4 점유, 레이더 릴레이).

GUI만 보드 없이 점검하려면 터미널 2개에서 아래처럼 실행한다.

```powershell
# 1) GUI
.\antidrone\.venv\Scripts\python.exe .\antidrone\unified_gui.py

# 2) 가짜 카메라/레이더/텔레메트리 UDP
.\antidrone\.venv\Scripts\python.exe .\scripts\gui_udp_sim.py
```

---

## 빌드

```powershell
# 호스트 C++ 앱 (ptcamera_tracker.exe 등) — Windows
.\scripts\build_win.bat

# FPGA 베어메탈 앱 (antidrone_app.elf)
.\scripts\run_build.bat
```

HLS IP 또는 블록디자인 수정 시 전체 재빌드 흐름은 [FPGA_workflow.md](FPGA_workflow.md) §7 체크리스트 참고.

---

## 버전 관리 메모
- 이 저장소는 **소스·문서·스크립트만** git 추적. 빌드 산출물(`build_win/`, `*.gen/`),
  대용량 바이너리(`*.onnx`, `*.pt`, `*.bin`, `*.zip`), EDA 캐시는 `.gitignore`로 제외.
- 모델 가중치(`best.onnx` 261MB 등)는 git에 없음 → 별도 보관/재변환 필요.

## 미해결 항목
→ **[OPEN_ISSUES.md](OPEN_ISSUES.md)**
