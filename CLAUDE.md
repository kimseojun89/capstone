# CLAUDE.md

> 이 파일은 매 세션 자동 로드된다. **코드가 항상 기준(source of truth)**이며,
> 값이 의심되면 문서가 아니라 아래 명시된 코드 위치를 직접 확인할 것.
> 최종 코드 대조: 2026-06-01.

## 프로젝트 개요

레이더(안테나) + 카메라(YOLOv8x) 융합으로 드론을 탐지·추적하고 Pan/Tilt 스테퍼 모터를 구동하는 시스템.
PC(Windows)가 영상 추론·관제, PYNQ-Z2 FPGA가 신호처리·모터 제어를 담당한다.

```
[레이더] ─UART1→ [PYNQ-Z2 FPGA] ─UART0(COM4)→ [Windows PC] ─UDP→ [통합 GUI]
                  CORDIC/Kalman/모터          YOLOv8x(CUDA)+ByteTrack       카메라+PPI 한 창
                  (ps_main.cpp)               (ptcamera_tracker.exe)        (unified_gui.py)
```

## 핵심 컴포넌트 (코드 위치)

| 컴포넌트 | 경로 | 역할 |
|---|---|---|
| FPGA 베어메탈 | `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp` | 레이더 파싱·CORDIC·Kalman·퓨전·모터 제어 |
| PC 트래커 | `antidrone/cpp/apps/ptcamera_tracker.cpp` | 카메라·YOLO·ByteTrack·PID·시리얼·UDP 릴레이 |
| 트래커 설정 | `antidrone/cpp/include/ptcamera/settings.hpp` | 카메라/모터/YOLO 기본값 |
| 통합 GUI | `antidrone/unified_gui.py` | 카메라+PPI 표시 **전용** (모터 명령 송신 안 함) |
| 실행 진입점 | `antidrone/run_system.ps1` | 플래시→GUI→트래커 일괄 실행 |
| HLS IP 소스 | `Vitis/{cordic_polar,kalman_filter,uln2003_controller}.cpp` | PL 가속 IP |

## 빌드

```powershell
# PC 호스트 앱 (ptcamera_tracker.exe) — VS2022 BuildTools + OpenCV + ONNXRuntime-GPU 1.20.1
.\scripts\build_win.bat

# FPGA 베어메탈 앱 (antidrone_app.elf) — Vitis 2023.2
.\scripts\run_build.bat
```

- `build_win.bat`: vcvars64 + WinSDK 10.0.26100.0 자동 설정, OpenCV=`C:/opencv/build/x64/vc16/lib`,
  ONNX=`onnxruntime-gpu/onnxruntime-win-x64-gpu-1.20.1`, CMake NMake로 `antidrone/cpp/build_win/`에 산출.
- HLS IP/블록디자인 수정 시 전체 재빌드는 [docs/FPGA_workflow.md](docs/FPGA_workflow.md) §7 체크리스트.

## 실행

```powershell
cd C:\Users\kimse\capstone\antidrone
.\run_system.ps1 -Preflight                              # 하드웨어 무접촉 사전점검만
.\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor    # 플래시 포함 전체 실행 (코드 변경 후/첫 실행)
.\run_system.ps1 -SerialPort COM4 -EnableMotor           # 플래시 생략 (FPGA 이미 실행 중)
```

보드 없이 GUI만 점검: 터미널 2개에서 `unified_gui.py`와 `scripts/gui_udp_sim.py`(가짜 UDP 송신).

## 통신 프로토콜

**PC → FPGA (UART0, COM4, 256000bps)** — `ptcamera_tracker.exe`가 COM4 독점, GUI는 시리얼로 모터 송신 안 함:
- `P:±N\n` Pan 직접 스텝 (N=8~48, FPGA가 5~58로 클램프) → AI 추적 모드 진입
- `T:±N\n` Tilt 직접 스텝 → 퓨전/레이더 단독 모드의 Tilt
- FPGA는 매 프레임(33ms) 끝에 `g_host_pan/tilt_steps`를 0으로 리셋 → 30Hz 재전송 필요

**FPGA → PC (UART0)**: `[RADAR] T0:(x,y)mm spd=Ncm/s | dist=Nmm ang=N.Ndeg` + `[UART]/[MTI]/[FUSE]/[KALM]` 로그.

**PC 내부 UDP (localhost)** — tracker → unified_gui:
- `9998` 카메라 JPEG 640×360 · `9999` `[RADAR]` 텍스트 · `10000` JSON 텔레메트리(FPS/AI/Motor/Serial)

## FPGA 모터 상태머신 (`ps_main.cpp:944~973`)

우선순위 분기. **Pan은 항상 레이더 방위각, 증분 PID**(절대 SET 아님). Tilt만 모드별 분기.

| 모드 | 조건 | Pan | Tilt |
|---|---|---|---|
| AI 추적 | `g_host_pan_steps!=0 && cooldown==0` | PC `P:` 직접 | PC `T:` 직접 |
| AI 잠금 | `cooldown>0` or `ai_lock_frames>0` | 위치 유지(레이더 차단) | 유지 |
| 퓨전 | `fi>=0` | 레이더 `angle_to_px(rang)` PID | 카메라 BBox cy PID |
| 레이더 단독 | `rvc>0 && fi<0` | 레이더 PID | PC `T:` 직접 |
| 표적 없음 | `rvc==0` | 마지막 위치 유지 | — |

- 레이더 `rang`은 LP필터(α=0.4, `:922`)로 평활 후 PID 입력 → 좌우 지터 억제.
- AI 명령 후 `cooldown`/`ai_lock_frames` 동안 레이더 오버라이드 차단 → PC 추적 연속성 보장.
- 상세/튜닝: [docs/motor_control_changes.md](docs/motor_control_changes.md), [docs/motor_porting_guide.md](docs/motor_porting_guide.md).

## 주요 설정값 (코드 실측)

| 항목 | 값 | 위치 |
|---|---|---|
| 카메라 | index 1, 1920×1080 (ABKO APC900) | `settings.hpp:9~11` |
| 추론 | YOLOv8x ONNX, CUDA(RTX 3050) | `settings.hpp:16`, `models/drone_yolov8x/best.onnx` |
| 시리얼 | COM4 @ 256000 (tracker 독점) | `run_system.ps1`, `settings.hpp:59` |
| 모터 PID | KP=1.5, KD=0.0 | `ps_main.cpp:101,102` |
| 모터 스텝 | MIN=5, MAX=58, deadband 35px | `ps_main.cpp:103~110` |
| 모터 속도 | SPEED_DELAY=200 (154°/s; 탈조 시 300/MAX38) | `ps_main.cpp:120` |
| 슬루/주기 | CMD_RAMP=20.0, DT=0.033(30Hz) | `ps_main.cpp:112,122` |
| 컴파일 스위치 | `USE_MOCK_MTI=1`, `DISABLE_MTI=1`, `USE_HLS_CORDIC/KALMAN=1`, `LOG_LEVEL=1` | `ps_main.cpp:30~43` |

> ⚠️ `MOTOR_ACQUIRE_RAMP`는 `:121`에서 0.6f 정의 후 `:128` `#undef`→`:129` 1.0f 재정의 → **유효값 1.0f**.

## HLS IP 주소맵

| IP | 주소 | 비고 |
|---|---|---|
| CORDIC (극좌표) | `0x40000000` | angle 출력 0.1도 단위 |
| Kalman | `0x40010000` | ⚠️ 내부 DT=0.1, 루프=0.033 → vx/vy 약 3배 과대 (OPEN_ISSUES A2) |
| MTI | `0x40020000` | 현재 `DISABLE_MTI=1`, Mock 상태 |
| ULN2003 모터 | `0x40030000` | 0x10=target_pan, 0x18=target_tilt, 0x20=speed_delay(루프카운트) |

## 재빌드 기준

| 수정 파일 | 재빌드 |
|---|---|
| `ps_main.cpp` | Vitis 2023.2 빌드(`run_build.bat`) → `run_system.ps1 -Flash` |
| `ptcamera_tracker.cpp` 등 C++ | `scripts\build_win.bat` |
| HLS IP(`Vitis/*.cpp`) | HLS 재합성→Export RTL→Vivado bitstream→XSA ([FPGA_workflow.md](docs/FPGA_workflow.md) §7) |
| `unified_gui.py` / `run_system.ps1` | 재빌드 없음 (저장 즉시) |

## 미해결 항목

[docs/OPEN_ISSUES.md](docs/OPEN_ISSUES.md) — MTI 실카메라 미연결(Mock), Kalman DT 불일치, 모터 GPIO XDC 핀맵 미확정, UART 256000 어댑터 호환 등.

## 작업 규약

- 문서(`docs/`)는 한국어. **코드와 문서가 다르면 코드를 신뢰**하고 문서를 갱신할 것.
- 빌드 산출물(`build_win/`, `*.elf`, `*.bit`)·대용량 바이너리(`*.onnx`, `*.pt`)는 git 미추적(`.gitignore`).
- 미사용/레거시는 `legacy/`로 격리됨 (`ppi_viewer.py` 등).
