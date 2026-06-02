# 모터 제어 통합 문서

> **대상 파일:**
> - `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp` — FPGA PS 메인
> - `Vitis/uln2003_controller.cpp` — HLS IP (PL 하드웨어)
> - `antidrone/cpp/apps/ptcamera_tracker.cpp` — PC 트래커
> - `antidrone/cpp/src/serial_port.cpp` — UART 통신


---

## 2. HLS IP — `uln2003_controller` (PL 하드웨어)

> `Vitis/uln2003_controller.cpp` — 85줄, HLS C 합성

### 2-1. 인터페이스

```cpp
void uln2003_controller(
    int target_pan,        // AXI4-Lite 오프셋 0x10 (PS Write)
    int target_tilt,       // AXI4-Lite 오프셋 0x18
    int speed_delay,       // AXI4-Lite 오프셋 0x20
    ap_uint<4> &pan_out,   // ap_none: 직접 GPIO 와이어 (4비트)
    ap_uint<4> &tilt_out   // ap_none: 직접 GPIO 와이어 (4비트)
)
```

- `target_pan`, `target_tilt`, `speed_delay`는 `CTRL` AXI4-Lite 번들에 매핑된다.
- `pan_out`, `tilt_out`은 `ap_none` 출력 포트이며 별도 핸드셰이크 없이 ULN2003 IN1~IN4 신호로 직접 연결한다.
- `return`도 `CTRL` AXI4-Lite에 포함되어 `ap_start`, `ap_done`, `ap_idle` 제어 레지스터를 만든다.

### 2-2. AXI 레지스터 맵

| 오프셋 | 레지스터 | 방향 | 용도 |
|:---:|---|:---:|---|
| `0x00` | ap_ctrl | R/W | bit0=AP_START, bit1=AP_DONE, bit2=AP_IDLE, bit3=AP_READY |
| `0x10` | target_pan | W | Pan 절대 스텝 목표 |
| `0x18` | target_tilt | W | Tilt 절대 스텝 목표 |
| `0x20` | speed_delay | W | `hw_delay()` 루프 카운트 (µs 아님) |

```c
#define MOTOR_BASEADDR  0x40030000
#define MOTOR_WR(off,v) Xil_Out32(MOTOR_BASEADDR+(off),(u32)(v))
#define MOTOR_RD(off)   Xil_In32 (MOTOR_BASEADDR+(off))
```

### 2-3. 내부 static 변수 (FPGA 레지스터로 유지)

```cpp
static int current_pan  = 0;  // 현재 Pan 위치
static int current_tilt = 0;  // 현재 Tilt 위치
static int pan_idx  = 0;      // 하프스텝 시퀀스 인덱스 (0~7)
static int tilt_idx = 0;
```

> IP가 다시 시작되어도 static 값은 하드웨어 상태로 유지된다. PS가 새 **절대 목표 스텝**을 쓰면 IP는 현재 위치와 목표의 차이만큼 이동한다.

- `current_pan/current_tilt`: IP가 알고 있는 현재 절대 스텝 위치
- `pan_idx/tilt_idx`: 다음 출력에 사용할 8상 하프스텝 시퀀스 인덱스

### 2-4. 28BYJ-48 하프스텝 시퀀스

```cpp
const ap_uint<4> step_seq[8] = {
    0b0001, 0b0011, 0b0010, 0b0110,
    0b0100, 0b1100, 0b1000, 0b1001
};
```

- 시퀀스는 ULN2003 입력 IN1~IN4에 그대로 출력되는 4비트 패턴이다.
- 정방향 이동 시 인덱스는 `(idx + 1) % 8`, 역방향 이동 시 `(idx - 1 + 8) % 8`로 갱신한다.
- 8상 시퀀스 1회전 = 5.625° → 기어비 1:64 → 출력축 **1스텝 ≈ 0.011°**
- 360° = **4096 하프스텝**

### 2-5. 핵심 구동 루프

```cpp
while (current_pan != target_pan || current_tilt != target_tilt) {
    bool moved = false;

    if (current_pan < target_pan)       { pan_idx = (pan_idx+1)%8; current_pan++; moved = true; }
    else if (current_pan > target_pan)  { pan_idx = (pan_idx-1+8)%8; current_pan--; moved = true; }

    if (current_tilt < target_tilt)      { tilt_idx = (tilt_idx+1)%8; current_tilt++; moved = true; }
    else if (current_tilt > target_tilt) { tilt_idx = (tilt_idx-1+8)%8; current_tilt--; moved = true; }

    pan_out  = step_seq[pan_idx];
    tilt_out = step_seq[tilt_idx];
    if (moved) hw_delay(speed_delay);
}
```

- Pan과 Tilt 중 목표에 도달하지 않은 축만 1스텝씩 이동한다.
- 두 축이 모두 이동해야 하는 경우 같은 루프 반복에서 각각 1스텝씩 갱신되므로 같은 `speed_delay` 간격으로 동시 구동된다.
- 목표에 도달한 축은 마지막 `step_seq[idx]` 출력을 유지한다. 즉 코일 출력은 0으로 풀리지 않고 현재 상을 계속 유지한다.
- 함수는 목표에 도달할 때까지 `while` 안에 머무르므로, PS는 다음 명령을 쓰기 전에 `ap_done/ap_idle` 확인 또는 충분한 대기 시간이 필요하다.

### 2-6. `hw_delay()` — 스텝 간 딜레이

```cpp
void hw_delay(int delay_count) {
    #pragma HLS INLINE off
    volatile int dummy = 0;
    for (int i = 0; i < delay_count; i++) dummy++;
}
```

- busy-wait 루프. `delay_count=200` 기준 실측 **≈ 0.57ms/step** (200MHz 클럭, 1루프≈2.825µs)
- ⚠️ HLS 합성 설정 변경 시 재실측 필요
- `speed_delay <= 0`이면 실질적인 대기 없이 루프가 진행되므로, 실제 모터 구동에서는 양수 값을 사용한다.

---

## 3. PID 알고리즘 — `motor_pid_step()` (ps_main.cpp)

```cpp
static int motor_pid_step(
    float err_px,    // 픽셀 단위 오차 (화면 중심 대비)
    float frame_half,// 프레임 절반 크기 (Pan=160, Tilt=120)
    int   deadband,  // 데드밴드 px (현재 35)
    float kp,        // 비례 게인 (현재 1.5)
    float kd,        // 미분 게인 (현재 0.0 — 비활성)
    int   min_step,  // 최소 스텝 (현재 5)
    int   max_step,  // 최대 스텝 (현재 58)
    float* prev_err, // [상태] 이전 정규화 오차
    float* smooth_cmd,// [상태] 슬루 리미터 출력
    float ramp_scale // 가속 배율 (1.0 평상시)
)
```

### 처리 단계 (수치 예시: 오차 100px)

| 단계 | 처리 | 예시 |
|---|---|---|
| **A. 데드밴드** | `|err| ≤ 35px` → 0 반환, smooth_cmd 서서히 감속 | 100px > 35 → 통과 |
| **B. 정규화+PID** | `norm = err/frame_half`, `target = KP×norm` | `0.625 → 0.938` (클램프→0.938) |
| **C. 슬루 리미터** | `ramp = CMD_RAMP×DT = 20×0.033 = 0.66` | 첫 프레임: `smooth_cmd = 0.66` |
| **D. 최소 스텝 보정** | `min_ratio = 5/(5+53) = 0.086` | `max(0.66, 0.086) = 0.66` |
| **E. 스텝 변환** | `step = 5 + int(53×0.66) = 39` | `→ +39 스텝` |

### 시간에 따른 응답 시뮬레이션 (100px 오차)

| 프레임 | 오차(px) | smooth_cmd | 스텝 |
|:---:|:---:|:---:|:---:|
| 1 | 100 | 0.66 | 39 |
| 2 | 65 | 0.609 | 37 |
| 3 | 30 | — | 0 (데드밴드) |

→ **약 2프레임(66ms)에 100px 오차 해소**

---

## 4. 모터 상태머신 (ps_main.cpp 메인 루프)

```
메인 루프 30Hz (33ms)
    │
    ├─ [1] g_host_bbox_valid && cooldown==0
    │       → motor_update_ai_bbox(ex, ey)
    │         Pan PID: motor_pid_step(ex, 160, ...)
    │         Tilt PID: motor_pid_step(ey, 120, ...)
    │         g_host_cmd_cooldown=2, g_ai_lock_frames=4
    │
    ├─ [2] g_manual_mode && (pan||tilt steps) && cooldown==0
    │       → motor_update_full_host(pan_steps, tilt_steps)
    │         minStep 제한 없음 (캘리브레이션 모드)
    │         g_host_cmd_cooldown=2
    │
    ├─ [3] cooldown > 0
    │       → 감소만 (모터 이동 완료 대기, 레이더 개입 없음)
    │
    ├─ [4] ai_lock_frames > 0
    │       → 감소만 (다음 bbox 도착 전 공백에 레이더 차단)
    │
    ├─ [5] rvc > 0  (AI 잠금 완전 해제 후)
    │       → motor_update_hybrid(angle_to_px(rang)-160, true, 0)
    │         Pan: 레이더 방위각 PID
    │         Tilt: 정지
    │
    └─ [6] 표적 없음 → g_had_target=false (마지막 위치 유지)

매 프레임 끝: g_host_bbox_valid=false, g_host_pan/tilt_steps=0
```

> **Pan은 항상 "증분 PID"** — `g_motor_abs_pan += dpan` 누적.
> 절대 위치 SET이 아님.

### AI 잠금 타이밍 구조

```
프레임:   N    N+1  N+2  N+3  N+4  N+5  N+6
bbox수신: ✓              ✓
cooldown: 2→1  1→0
ai_lock:       4→3  3→2  2→1  1→0
레이더:   ✗    ✗    ✗    ✗    ✗    ✓    ✓
```

- cooldown(2프레임) — 모터 물리 이동 완료 대기
- ai_lock(4프레임) — 공백 기간에 레이더가 끼어들지 못하도록 보호

---

## 5. 통신 프로토콜

### PC → FPGA (UART0, COM4, 256000bps)

| 명령 | 형식 | 용도 |
|---|---|---|
| **AI 추적** | `B:ex,ey\n` | bbox 중심 오차 → FPGA PID |
| 수동 Pan | `P:±N\n` | M:1 수동 모드 전용 |
| 수동 Tilt | `T:±N\n` | M:1 수동 모드 전용 |
| 수동 모드 진입 | `M:1\n` | minStep 해제 (캘리브레이션) |
| 수동 모드 해제 | `M:0\n` | AI 추적 복귀 |

**bbox 오차 스케일 변환 (PC측 `serial_port.cpp:sendBBox()`):**
```cpp
ex = (center_x - frame_w/2) / (frame_w/2) * 160;  // Pan: 320px 기준 스케일
ey = (center_y - frame_h/2) / (frame_h/2) * 120;  // Tilt: 240px 기준 스케일
```

**수동 모드 각도↔스텝 변환 (`serial_port.cpp`):**
```cpp
STEPS_PER_DEGREE = 4096 / 360 ≈ 11.378
sendPanDegrees(1.0)  → P:+11\n
sendTiltDegrees(1.0) → T:+11\n
```

### FPGA → PC (UART0)

```
[RADAR] T0:(x,y)mm spd=Ncm/s | dist=Nmm ang=N.Ndeg
[KALM ] T0 (x,y)mm v=(vx,vy)cm/s
[UART ] bytes=N packets=N ring_ovf=N hw_ovr=N
```

---

## 6. 최종 모터 파라미터 (ps_main.cpp)

```c
#define MOTOR_KP            1.5f     // 비례 게인 (0.25→1.5; 2.0은 진동 롤백)
#define MOTOR_KD            0.0f     // 미분 게인 0 (레이더 노이즈 증폭 방지)
#define MOTOR_PAN_DEADBAND  35       // px — 데드밴드
#define MOTOR_TILT_DEADBAND 35       // px
#define MOTOR_PAN_MIN_STEP  5        // 최소 스텝 (자동 모드)
#define MOTOR_PAN_MAX_STEP  58       // 33ms / 0.57ms ≈ 58 → 30Hz 연속 업데이트
#define MOTOR_TILT_MIN_STEP 5
#define MOTOR_TILT_MAX_STEP 58
#define MOTOR_CMD_RAMP      20.0f    // 슬루율 (2프레임에 최대 명령 도달)
#define MOTOR_ACQUIRE_RAMP  1.0f     // 첫 표적 가속 억제 없음
#define MOTOR_DT            0.033f   // 30Hz
#define MOTOR_SPEED_DELAY   200      // 0.57ms/step → 154°/s
```

### 속도 공식

```
각속도(°/s) = 360 / (4096 × SPEED_DELAY × 2.825µs)
            = 154°/s  (SPEED_DELAY=200 기준)

최적 MAX_STEP = 프레임주기(ms) / step당시간(ms)
              = 33ms / 0.57ms ≈ 58
```

---

## 8. 하드웨어 배선

### PMODA — Pan 모터 (오른쪽 열) + 레이더 UART1 (왼쪽 열)

```
        왼쪽 열 (레이더 UART1)   오른쪽 열 (Pan 모터)
       ┌──────────────────┬──────────────────┐
Row 1  │  JA1P  Y18       │  JA3P  U18       │  ← 레이더 RX / Pan IN1
Row 2  │  JA1N  Y19       │  JA3N  U19       │  ← 레이더 TX / Pan IN2
Row 3  │  JA2P  Y16       │  JA4P  W18       │  ←  미사용   / Pan IN3
Row 4  │  JA2N  Y17       │  JA4N  W19       │  ←  미사용   / Pan IN4
Row 5  │       GND        │       GND        │
Row 6  │       3V3        │       3V3        │
       └──────────────────┴──────────────────┘
```

### PMODB — Tilt 모터 (왼쪽 열)

```
        왼쪽 열 (Tilt 모터)
       ┌──────────────────┐
Row 1  │  JB1P  W14  IN1  │
Row 2  │  JB1N  Y14  IN2  │
Row 3  │  JB2P  T11  IN3  │
Row 4  │  JB2N  T10  IN4  │
Row 5  │       GND        │ 
       └──────────────────┘
```

### 전원

```
외부 5V (+) ──┬── ULN2003 Pan  VCC
              └── ULN2003 Tilt VCC
외부 5V (–) ──┬── ULN2003 Pan  GND
              ├── ULN2003 Tilt GND
              └── PMODA/B GND Row5  ← 공통 GND 필수
```

> ⚠️ 28BYJ-48은 **5V 전용**. PMOD 3V3으로는 구동 불가.  
> 신호선(IN1~4)은 FPGA 3.3V LVCMOS33 그대로 연결 OK (ULN2003 입력 문턱 ~1V).

---

## 9. 부팅 및 진단

### 부팅 시 정상 로그 (UART0 터미널, 256000bps)

```
============================================
  Anti-Drone Sensor Fusion Core Booting...
  Phase 2: FPGA-Only Control
============================================
[+] Radar UART1 OK (256000 bps, EMIO PMODA)
[+] Kalman IP (PL HLS) OK
[+] Motor IP OK (home position)    ← 이 시점에 모터 미세 진동 후 잠금
```

### 모터 반응 진단표

| 증상 | 원인 | 조치 |
|---|---|---|
| 미세 진동 후 정지 | 정상 홈포지션 | — |
| 아무 반응 없음 | 5V 미공급 또는 공통 GND 누락 | 전원/GND 확인 |
| 진동만, 회전 안 됨 | 탈조 — SPEED_DELAY 너무 낮음 | `SPEED_DELAY=300, MAX_STEP=38` |
| 반대 방향 회전 | IN 핀 순서 반전 | IN1↔IN4, IN2↔IN3 스왑 |
| 표적 추종 안 함 | tracker 미실행 또는 COM4 미점유 | `ptcamera_tracker.exe`가 `B:ex,ey` 전송 중인지 확인 |
| 표적 추종 끊김 | bbox 전송 30Hz 미달 | `settings.sendInterval` ≤ 0.033 확인 |

### 수동 점검 명령 (COM4 터미널에서 직접 입력)

```
M:1\n          수동 모드 진입
P:+58\n        Pan +58스텝 (우향 최대)
T:-58\n        Tilt -58스텝
M:0\n          수동 모드 해제

B:80,40\n      AI 추적 테스트 (Pan 우향, Tilt 아래)
```

### 튜닝 가이드

| 증상 | 조치 |
|---|---|
| Pan 좌우 진동 | LP 필터 α 낮춤 (0.4→0.25), 또는 deadband 확대 (35→50px) |
| Pan 반응 너무 느림 | LP 필터 α 높임 (0.4→0.6), 또는 KP 상향 (1.5→2.0, 진동 재확인) |
| 모터 탈조 | `SPEED_DELAY=300, MAX_STEP=38` 으로 복원 |
| 모터 반응 없음 | 5V 전원·GND 공통 연결 확인 |
| Tilt 감도 차이 | `MOTOR_TILT_DEADBAND` 또는 KP 개별 조정 (현재 Pan과 동일 파라미터) |

---

## 10. 미해결 항목

| ID | 항목 | 우선순위 |
|:---:|---|:---:|
| M1 | **Kalman HLS DT 불일치** — IP 내부 DT=0.1f, 루프=0.033s → vx/vy 약 3배 과대. SW 보정: `ks[i].vx *= 0.33f` 또는 HLS 재합성 | 🟡 중 |
| M2 | **LP 필터 고정 α=0.4** — 드론 속도에 따라 동적 조절하면 성능 향상 가능 | 🟢 낮음 |
| M3 | **절대 위치 범위 제한 없음** — `g_motor_abs_pan/tilt` 무한 누적. 소프트 리밋(`±MOTOR_PAN_LIMIT`) 및 홈 복귀 미구현 | 🟡 중 |
| M4 | **hw_delay 타이밍 불확실** — HLS 합성 최적화에 따라 실제 지연 달라질 수 있음. 재합성 후 반드시 재실측 | 🟡 중 |
| M5 | **AP_IDLE 시 프레임 드롭** — IP 실행 중 bbox 명령 폐기 → 연속 고속 이동 시 일부 명령 소실 가능 | 🟢 낮음 |
