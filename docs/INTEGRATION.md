# Anti-drone System Integration

카메라(PC) + FPGA(PYNQ-Z2) 통합 제어 구현 문서

---

## 시스템 개요

| 축 | 제어 소스 | 경로 |
|---|---|---|
| **Pan** | 안테나/레이더 방위각 | UART1 → ps_main.cpp 증분 PID (`angle_to_px(rang)`) |
| **Tilt** | 퓨전: 카메라 BBox cy PID / 레이더단독: PC `T:` 명령 | UART0 (COM4) → ps_main.cpp |

> 전체 4-모드(AI추적·AI잠금·퓨전·레이더단독) 분기는 [SYSTEM_OVERVIEW.md §5](SYSTEM_OVERVIEW.md) /
> [motor_control_changes.md](motor_control_changes.md) 참고. 본 문서는 통합 작업의 변경 이력 위주.

```
[Radar/Antenna] --UART1 256000bps--> [FPGA ps_main.cpp]
                                            |
[Windows PC]                                |
  Camera -> YOLOv8x -> ControlLoop          |
  sendTiltCommand("T:±N\n")                |
  --UART0 256000bps COM4----------------> [FPGA ps_main.cpp]
                                            |
                                 motor_update_hybrid()
                                  Pan: 레이더 PID
                                  Tilt: 호스트 직접 스텝
                                            |
                                 [uln2003_controller HLS IP]
                                 [Pan Motor]   [Tilt Motor]
```

---

## 데이터 흐름 (UDP 릴레이 포함)

```
[FPGA UART0] --COM4---> [ptcamera_tracker.exe]
                              |          |             |
                         모터 명령 전송  [RADAR] 추출  카메라/AI 상태
                         "P:/T:±N\n"     |             |
                                    UDP:9999       UDP:9998 / 10000
                                         |             |
                                   [unified_gui.py]
                                   카메라 + PPI + 텔레메트리 표시
```

**COM4 독점 구조**: `ptcamera_tracker.exe`가 COM4를 단독으로 열고,  
FPGA 로그에서 `[RADAR]` 라인을 추출해 UDP 127.0.0.1:9999으로 중계한다.  
`unified_gui.py`는 시리얼을 열지 않고 UDP만 수신하므로 포트 충돌이 없다.

| UDP 포트 | 방향 | 내용 |
|---|---|---|
| 9998 | tracker → unified_gui | 오버레이가 그려진 카메라 프레임 |
| 9999 | tracker → unified_gui | `[RADAR]` 텍스트 라인 |
| 10000 | tracker → unified_gui | JSON 텔레메트리(FPS, AI lock, confidence, pan/tilt command, motor/serial 상태) |

---

## 변경 파일 목록

### 1. `vitis_workspace/antidrone_app/src/ps_main.cpp`

#### 1-1. UART0 baud rate 변경
```c
#define CONSOLE_UART_BAUD   256000u   // 호스트 tilt 명령 채널 겸용
```

#### 1-2. 호스트 링버퍼 전역 변수 추가
```c
#define HOST_RING_SZ  256
static uint8_t  g_host_ring[HOST_RING_SZ];
static uint32_t g_host_head = 0;
static uint32_t g_host_tail = 0;
static int      g_host_tilt_steps = 0;
```

#### 1-3. `host_accumulate()` — UART0 RX FIFO 드레인
```c
static void host_accumulate(void) {
    uint8_t tmp[64]; int n = 0;
    while ((n = XUartPs_Recv(&g_console_uart, tmp, sizeof(tmp))) > 0)
        for (int i = 0; i < n; ++i) host_ring_push(tmp[i]);
}
```

#### 1-4. `host_parse_tilt()` — `"T:±N\n"` 파싱
호스트에서 수신한 tilt 스텝 명령을 `g_host_tilt_steps`에 저장.

#### 1-5. `motor_update_hybrid()` — Pan/Tilt 분리 제어
```c
static void motor_update_hybrid(int pan_err_x, bool has_target, int direct_tilt_steps) {
    // Pan: 레이더 PID
    int dpan = motor_pid_step(...);
    g_motor_abs_pan += dpan;

    // Tilt: 호스트 직접 스텝
    int dtilt = (abs(direct_tilt_steps) >= MOTOR_TILT_MIN_STEP) ? direct_tilt_steps : 0;
    g_motor_abs_tilt += dtilt;

    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    if (MOTOR_RD(0x00) & AP_IDLE) MOTOR_WR(0x00, AP_START);
}
```

#### 1-6. 메인 루프 Step 5 교체
```c
host_accumulate();
host_parse_tilt();
if (fi >= 0) {
    motor_update_hybrid(cx - CAM_W_PX/2, true, g_host_tilt_steps);
    g_host_tilt_steps = 0;
} else if (rvc > 0) {
    motor_update_hybrid(angle_to_px(rang) - CAM_W_PX/2, true, g_host_tilt_steps);
    g_host_tilt_steps = 0;
} else {
    g_host_tilt_steps = 0;
    g_had_target = false;
}
```

#### 1-7. `responsive_sleep_us()` 내 `host_accumulate()` 추가
슬립 중에도 tilt 명령을 놓치지 않도록 1ms 마다 드레인.

---

### 2. `cpp/include/ptcamera/serial_port.hpp`

- Windows HANDLE 기반 private 멤버 추가 (`#ifdef _WIN32`)
- `isOpen()` 플랫폼별 조건부 구현
- `sendTiltCommand(int tiltSteps)` 선언
- `readAvailable(std::string& buf)` 선언 (논블로킹 RX 드레인)

---

### 3. `cpp/src/serial_port.cpp`

- **Windows**: no-op 스텁을 WinAPI 전체 구현으로 교체
  - `CreateFile` + DCB 설정 (256000 baud, 8N1, 흐름 제어 없음)
  - `COMMTIMEOUTS` (ReadIntervalTimeout=MAXDWORD → 논블로킹 읽기)
  - `WriteFile` 루프, `ReadFile` 폴링
  - `sendTiltCommand`, `readAvailable` 구현
- **Linux**: `sendTiltCommand`, `readAvailable` 추가

---

### 4. `cpp/include/ptcamera/settings.hpp`

```cpp
int serialBaud = 256000;  // 115200 → 256000
```

---

### 5. `cpp/apps/ptcamera_tracker.cpp`

- winsock2 / POSIX 소켓 include 추가
- **시리얼 항상 오픈** (모터 비활성 시에도 레이더 릴레이를 위해)
- **UDP 소켓 초기화** → 127.0.0.1:9999
- 메인 루프에서 `readAvailable` 호출 → `[RADAR]` 라인 추출 → UDP sendto
- 카메라 오버레이 프레임 → UDP 127.0.0.1:9998
- tracker 상태 JSON → UDP 127.0.0.1:10000
- tilt 전송: `sendTiltCommand(tiltSteps)` (`sendStepperCommand` 대체)
- 종료 시 소켓 정리 (`closesocket` / `WSACleanup`)

---

### 6. `cpp/CMakeLists.txt`

```cmake
if(WIN32)
    target_link_libraries(ptcamera_tracker PRIVATE ws2_32)
endif()
```

---

### 7. `ppi_viewer.py` → **현재 `unified_gui.py`로 대체**

> ⚠️ 2026-05-29 재구성: 런타임 뷰어는 `antidrone/unified_gui.py`(카메라+PPI 통합 창) 1개로 일원화.
> `ppi_viewer.py`(PPI 단독)와 `run_ppi_viewer.bat`은 [legacy/](../legacy/)로 격리됨.
> 아래 UDP 릴레이 동작은 `unified_gui.py`에 동일하게 구현되어 있다.

- `BAUD_RATE`: 115200 → 256000
- **UDP 소켓**: `9998`(카메라), `9999`(레이더), `10000`(tracker 텔레메트리), non-blocking
- **시리얼 직접 연결 없음**: COM4는 `ptcamera_tracker.exe`가 독점
- 상태 표시: 카메라/레이더/AI/모터를 `LIVE`, `STALE`, `OFFLINE`, `LOCK` 등으로 분리
- PPI 기능: heatmap, target trail, 타겟 클릭 선택, `D` 디버그 로그 토글

---

### 8. `flash.tcl` (신규)

XSCT 원라인 플래시 스크립트.

```tcl
connect
targets 1 / rst -system / after 3000
targets 2 / rst -processor
source ps7_init.tcl / ps7_init
fpga -f antidrone_wrapper.bit / ps7_post_config
catch {stop}
dow antidrone_app.elf
con
```

---

### 9. `run_system.ps1` (신규)

통합 실행 스크립트.

```powershell
# FPGA 플래시 포함
.\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor

# 플래시 없이 (FPGA 이미 실행 중)
.\run_system.ps1 -SerialPort COM4 -EnableMotor
```

순서: FPGA 플래시 → unified_gui.py 실행 → ptcamera_tracker.exe 실행

---

## 빌드 방법

### FPGA (Vitis 2023.2)
```powershell
cd C:\Users\kimse\capstone\antidrone\vitis_workspace\antidrone_app
cmake --build build -j4
```

### Windows 호스트 앱
```powershell
cd C:\Users\kimse\capstone\antidrone\cpp
# 최초 1회 configure (VS2022 BuildTools 필요)
cmake -B build_win -G "NMake Makefiles" `
    -DCMAKE_BUILD_TYPE=Release `
    -DOpenCV_DIR="C:/opencv/build"

cmake --build build_win --target ptcamera_tracker
```

---

## 검증 순서

1. Tera Term / PuTTY → COM4, **256000 bps** 확인
2. `.\run_system.ps1 -Flash -SerialPort COM4 -EnableMotor` 실행
3. 통합 GUI 상단 **CAM/RADAR/AI/MOTOR** 상태 표시 확인
4. 카메라 화면에서 드론이 아래로 이동 → tilt 모터 상향 추종 확인
5. 레이더 신호 방향 변화 → pan 모터 추종 확인

### 보드 없이 GUI 점검

```powershell
.\antidrone\.venv\Scripts\python.exe .\antidrone\unified_gui.py
.\antidrone\.venv\Scripts\python.exe .\scripts\gui_udp_sim.py
```

---

## 주의사항

| 항목 | 내용 |
|---|---|
| UART0 겸용 | `xil_printf` 출력도 UART0으로 나옴. 호스트는 `T:` 외 라인 무시 |
| 디버깅 시 | 터미널(Tera Term)과 ptcamera_tracker 동시 COM4 접근 불가 |
| 단발 소비 | 호스트 명령 전송 중단 시 FPGA tilt 자동 정지 (안전) |
| USB-UART | 256000bps 지원 어댑터 필요 (FTDI FT232, CP2102 권장; CH340 일부 미지원) |
