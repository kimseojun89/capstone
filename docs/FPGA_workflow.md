# Anti-Drone FPGA 작업 워크플로우

## 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 보드 | PYNQ-Z2 (Zynq XC7Z020) |
| Vivado 프로젝트 | `C:\Users\kimse\capstone\vivado_project\antidrone.xpr` |
| Vitis HLS 소스 | `C:\Users\kimse\capstone\Vitis\` |
| Vitis Unified | 2023.2 — 워크스페이스: `C:\Users\kimse\capstone\antidrone\vitis_workspace\` |
| 베어메탈 앱 | `antidrone_app\src\ps_main.cpp` |

---

## HLS IP 목록

| HLS 프로젝트 | 최상위 함수 | 인터페이스 | 역할 |
|---|---|---|---|
| `cordic_prj` | `cordic_polar` | AXI4-Lite | CORDIC 극좌표 변환 |
| `kalman_prj` | `kalman_filter` | AXI4-Lite | 칼만 필터 |
| `motor_prj` | `uln2003_controller` | AXI4-Lite (제어) + ap_none (GPIO) | 28BYJ-48 듀얼 스테퍼 모터 |
| `mti_process_prj` | `mti_process` | AXI4-Lite (제어) + AXI Master (DDR) | MTI 영상 처리 + BBox 추출 |

---

## 표준 빌드 흐름

```
[Vitis HLS]
  ↓ C/C++ 작성 → C Synthesis → Export RTL (IP)
  
[Vivado]
  ↓ IP Repository 등록 → Block Design에 IP 추가 → 배선 연결
  ↓ Validate Design → Generate Wrapper → Generate Bitstream
  ↓ File > Export Hardware (Include Bitstream) → antidrone_wrapper.xsa
  
[Vitis Unified 2023.2]
  ↓ antidrone_platform (XSA 기반) 빌드
  ↓ antidrone_app (ps_main.cpp) 빌드 및 보드 실행
```

---

## 파일 경로 정리

- XSA 출력: `C:\Users\kimse\capstone\vivado_project\antidrone_wrapper.xsa`
- Block Design: `vivado_project\antidrone.srcs\sources_1\bd\antidrone_bd\antidrone_bd.bd`
- 베어메탈 소스: `antidrone\vitis_workspace\antidrone_app\src\ps_main.cpp`
- BSP 플랫폼: `antidrone\vitis_workspace\antidrone_platform\`

---

## 부족하거나 확인이 필요한 사항

### 1. CORDIC / Kalman HLS IP — PL 하드웨어 호출로 전환 완료

두 IP 모두 `ps_main.cpp`에서 PL 하드웨어 직접 호출로 전환되었다.
컴파일 타임 스위치로 SW/HLS 선택 가능하다.

```cpp
#define USE_HLS_CORDIC  1   // 0으로 변경 시 ARM 소프트웨어 폴백
#define USE_HLS_KALMAN  1   // 0으로 변경 시 ARM 소프트웨어 폴백
```

| IP | 주소 | 입력 | 출력 |
|---|---|---|---|
| CORDIC | 0x40000000 | x_in(int16)@0x10, y_in(int16)@0x18 | distance(u16)@0x20, angle_deg(int16, **0.1도 단위**)@0x30 |
| Kalman | 0x40010000 | targets_in(3×64-bit)@0x20, reset(bit)@0x10 | states_out(3×160-bit)@0x80 |

**Kalman targets_in 패킹 (64-bit/target, little-endian):**
```
Word 0 (0x20+8n): (uint16)y<<16 | (uint16)x
Word 1 (0x24+8n): (uint32)valid<<16 | (uint16)speed
```

**Kalman states_out 패킹 (32-byte/state, 5 words 사용):**
```
0x80+32n:    x   (float32)
0x84+32n:    y   (float32)
0x88+32n:    vx  (float32)
0x8C+32n:    vy  (float32)
0x90+32n:    init (u32 bit0)
```

**⚠️ DT 불일치 주의:**
Kalman HLS IP 내부 `DT = 0.1f` (10Hz 기준)이나, 현재 메인 루프는 `usleep(33000)` (~30fps = 33ms).
속도 추정치(vx, vy)가 실제보다 약 3배 크게 나온다.
추후 HLS 소스의 `DT`를 `0.033f`로 수정 후 재합성하거나, 루프 주기를 100ms로 변경하여 맞춰야 한다.

---

### 2. Motor Control 코드 추가 완료

`antidrone_app\src\ps_main.cpp`에 다음 내용이 추가되었다.

- `motor_pid_step()` — C++ AxisController + PIDController 이식 (단일 축 PID + 슬루 리미팅 + 최소 스텝 보정)
- `motor_update(err_x, err_y)` — AP_IDLE 확인 후 target_pan/tilt 레지스터 기록 및 AP_START
- 메인 루프 스텝 5 — 퓨전 매칭 시 BBox 중심 기준, 미매칭 시 레이더 방위각 기준으로 Pan 보정

**Motor IP 주소:** `XPAR_ULN2003_CONTROLLER_0_BASEADDR = 0x40030000` (xparameters.h 확인)

**레지스터 맵:**
| 오프셋 | 내용 |
|--------|------|
| 0x00 | ap_ctrl (AP_START bit0, AP_DONE bit1, AP_IDLE bit2) |
| 0x10 | target_pan (절대 스텝) |
| 0x18 | target_tilt (절대 스텝) |
| 0x20 | speed_delay (hw_delay 루프 카운트 — µs 아님; ps_main 기본 200) |

---

### 3. Motor GPIO 핀 제약 (XDC) 미문서화

`uln2003_controller`의 `pan_out[3:0]`, `tilt_out[3:0]`은 `ap_none` 포트로,
Vivado 블록 디자인에서 **외부 포트(External Port)로 끌어내거나 EMIO GPIO에 연결** 해야 한다.
어느 PYNQ-Z2 핀(PMODA / PMODB)에 연결했는지 제약 파일(`.xdc`)에 기록이 없다면
보드 재조립 시 핀 연결을 확인하기 어렵다.

**확인 사항:** `vivado_project\antidrone.srcs\constrs_1\` 에 XDC 파일이 있는지,
pan/tilt GPIO가 명시적으로 할당되어 있는지 점검.

---

### 4. USE_MOCK_MTI = 1 상태 (실제 카메라 미연결)

`ps_main.cpp` 상단:
```cpp
#define USE_MOCK_MTI  1  // 현재 Mock 프레임 사용 중
```
실제 카메라 입력이 없으면 MTI 정밀도 검증이 불가하다.
카메라 프레임을 DDR `CURR_BUF`에 쓰는 방식(DMA 또는 PS 직접 쓰기)이 확정되어야 `0`으로 전환 가능하다.

---

### 5. MTI IP DDR 접근 방식 — 보류

MTI 파트는 추후 진행 예정이므로 현재 검토 제외.
현재 `USE_MOCK_MTI = 1` 상태로 MTI 관련 코드는 유지되어 있으며 실제 IP 호출은 발생하지 않는다.

---

### 6. IP Repository 경로 등록 (필수 — 매 설치/이전 시 재수행)

HLS Export 후 Vivado에서 IP를 인식하려면 IP Repository 경로 등록이 필요하다.
프로젝트를 다른 PC로 옮기거나 Vivado 재설치 시 **자동으로 유지되지 않는다.**

**등록 절차:**

```
Vivado → Tools → Settings → IP → Repository → "+" 버튼으로 아래 4개 경로 추가
  C:\Users\kimse\capstone\Vitis\cordic_prj\solution1\impl\ip
  C:\Users\kimse\capstone\Vitis\kalman_prj\solution1\impl\ip
  C:\Users\kimse\capstone\Vitis\motor_prj\solution1\impl\ip
  C:\Users\kimse\capstone\Vitis\mti_process_prj\solution1\impl\ip
→ OK → IP Catalog에서 4개 IP가 "User Repository"에 표시되는지 확인
```

**등록 후 블록 디자인에 IP가 없다고 뜨는 경우:**
```
Block Design 우클릭 → Report IP Status →
노란 경고 IP 선택 → Upgrade Selected
→ Validate Design 재실행
```

> 주의: IP 소스(.cpp)를 수정하면 HLS에서 C Synthesis + Export RTL을 다시 한 뒤,
> Vivado IP Catalog에서 "Refresh All" 또는 해당 IP 우클릭 → "Upgrade IP" 수행 필요.

---

### 7. HLS 수정 → Vivado 반영 재빌드 체크리스트

HLS 코드를 수정할 때마다 아래 전체 흐름을 반복해야 한다.
중간 단계를 생략하면 구 버전 IP로 bitstream이 생성된다.

```
[ ] 1. Vitis HLS: C Synthesis 재실행
[ ] 2. Vitis HLS: Export RTL (IP 덮어쓰기)
[ ] 3. Vivado: IP Catalog → 해당 IP 우클릭 → "Upgrade IP"
[ ] 4. Vivado: Block Design → Validate Design
[ ] 5. Vivado: Generate Bitstream
[ ] 6. Vivado: File → Export Hardware (Include Bitstream)
[ ] 7. Vitis: antidrone_platform → 우클릭 → "Update Hardware Specification"
[ ] 8. Vitis: antidrone_platform 빌드
[ ] 9. Vitis: antidrone_app 빌드 및 실행
```

---

### 8. C++ 호스트 앱과 베어메탈의 역할 분담 미명세

`C:\Users\kimse\capstone\antidrone\cpp\` 에는 OpenCV + ByteTracker 기반의 별도 C++ 앱이 있다.
`serial_port.cpp`가 있는 것으로 보아 이 앱이 UART로 PYNQ와 통신하는 것으로 추정되지만,
**PS 베어메탈(ps_main.cpp) ↔ 호스트 C++ 앱 간 프로토콜과 역할 분담**이 명확히 문서화되어 있지 않다.

---

### 9. UART 바우드레이트 불일치 위험

`ps_main.cpp`에서 UART1을 256000 bps로 설정한다.
레이더 모듈 및 호스트 앱의 시리얼 설정이 이와 일치하는지 확인이 필요하다.

---

### 10. MTI 폴링 루프가 메인 루프를 블로킹함

```cpp
while(!(MTI_RD(MTI_CTRL_OFF) & AP_DONE)) { ... }  // 동기 폴링
```
MTI IP 처리 중에는 UART 수신도 멈춘다.
레이더 샘플링 속도가 빠를 경우 링 버퍼 오버플로우 가능성이 있다.
인터럽트 기반(`ap_ctrl_hs` IRQ → GIC 등록)으로 전환하거나, MTI 처리 전후 UART flush를 추가하는 것을 고려해야 한다.

---

## 보드 핀 요약 (현재 파악된 것)

| 신호 | 연결 위치 | 비고 |
|------|----------|------|
| UART1 TX/RX | EMIO → PMODA | 레이더 / 호스트 통신 (256000 bps) |
| pan_out[3:0] | 미확인 (XDC 필요) | ULN2003 Pan 모터 IN1~IN4 |
| tilt_out[3:0] | 미확인 (XDC 필요) | ULN2003 Tilt 모터 IN1~IN4 |
| Camera 입력 | 미연결 (Mock 상태) | CURR_BUF DMA 경로 미결정 |
