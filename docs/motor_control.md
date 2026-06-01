# 모터 제어 통합 문서

> **대상 파일:**
> - `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp` — FPGA PS 메인
> - `Vitis/uln2003_controller.cpp` — HLS IP (PL 하드웨어)
> - `antidrone/cpp/apps/ptcamera_tracker.cpp` — PC 트래커
> - `antidrone/cpp/src/serial_port.cpp` — UART 통신
>
> **최종 갱신:** 2026-06-01 (Phase 2 — FPGA-Only Control 반영)
> 이 파일을 모터 제어 정본으로 사용한다.

---

## 1. 시스템 아키텍처 (Phase 2 현재)

```
┌─────────────────────────────────────────────────────────┐
│                    PC (Windows)                          │
│  ptcamera_tracker.exe                                    │
│  ┌──────────┐    ┌─────────────┐   ┌──────────────────┐ │
│  │ YOLO v8x │───→│ ByteTrack   │───→│ serial_port.cpp  │ │
│  │ 드론탐지 │    │ bbox 추출   │   │ "B:ex,ey\n" 전송 │ │
│  └──────────┘    └─────────────┘   └────────┬─────────┘ │
└───────────────────────────────────────────  │  ──────────┘
                                              │ UART0 (256Kbps, COM4)
┌─────────────────────────────────────────── │  ──────────┐
│                PYNQ-Z2 (Zynq)              ▼             │
│  ┌─────────────────────────────────────────────────┐    │
│  │              PS (ARM Cortex-A9) ps_main.cpp      │    │
│  │                                                  │    │
│  │  UART0 RX ──→ host_parse_commands()             │    │
│  │                "B:ex,ey" 파싱 → motor_pid_step() │    │
│  │                "P:/T:" 수동 모드 전용             │    │
│  │                                                  │    │
│  │  UART1 RX ──→ uart_parse() ──→ CORDIC ──→ rang  │    │
│  │  (레이더)     30byte 패킷      극좌표변환         │    │
│  │                                                  │    │
│  │         ┌──── 상태머신 (4단계) ────┐             │    │
│  │         │  1. AI(bbox_valid) PID   │             │    │
│  │         │  2. 수동(M:1+P/T)        │             │    │
│  │         │  3. 레이더 단독 PID      │             │    │
│  │         │  4. 정지                 │             │    │
│  │         └────────────┬────────────┘             │    │
│  │                      │ AXI4-Lite                 │    │
│  │  ┌───────────────────▼──────────────────────┐   │    │
│  │  │  PL — uln2003_controller HLS IP           │   │    │
│  │  │  target_pan/tilt → 하프스텝 시퀀스 구동   │   │    │
│  │  │  pan_out[3:0] → PMODA → ULN2003 → Pan모터 │   │    │
│  │  │  tilt_out[3:0]→ PMODB → ULN2003 → Tilt모터│  │    │
│  │  └───────────────────────────────────────────┘   │    │
│  └─────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

**Phase 2 핵심 구조:**
- PC는 bbox 중심 오차(ex, ey)만 전송 — `B:ex,ey\n`
- PID 계산(Pan+Tilt 모두)은 FPGA가 담당
- `P:/T:` 명령은 수동 캘리브레이션(`M:1` 모드) 전용으로 격하

---

## 2. HLS IP — `uln2003_controller` (PL 하드웨어)

> `Vitis/uln2003_controller.cpp` — 86줄, HLS C 합성

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

### 2-2. AXI 레지스터 맵

| 오프셋 | 레지스터 | 방향 | 용도 |
|:---:|---|:---:|---|
| `0x00` | ap_ctrl | R/W | bit0=AP_START, bit1=AP_DONE, bit2=AP_IDLE |
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

> AP_START로 재호출되어도 이전 값이 유지 → PS가 **절대 위치**를 쓰면 IP는 차이만큼만 이동

### 2-4. 28BYJ-48 하프스텝 시퀀스

```cpp
const ap_uint<4> step_seq[8] = {
    0b0001, 0b0011, 0b0010, 0b0110,
    0b0100, 0b1100, 0b1000, 0b1001
};
```

- 8상 시퀀스 1회전 = 5.625° → 기어비 1:64 → 출력축 **1스텝 ≈ 0.011°**
- 360° = **4096 하프스텝**

### 2-5. 핵심 구동 루프

```cpp
while (current_pan != target_pan || current_tilt != target_tilt) {
    if (current_pan < target_pan)       { pan_idx = (pan_idx+1)%8; current_pan++; }
    else if (current_pan > target_pan)  { pan_idx = (pan_idx-1+8)%8; current_pan--; }
    // Tilt 동일
    pan_out  = step_seq[pan_idx];
    tilt_out = step_seq[tilt_idx];
    if (moved) hw_delay(speed_delay);
}
```

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

### 속도 튜닝 전체 이력

| SPEED_DELAY | ms/step | MAX_STEP | 각속도 | 비고 |
|:---:|:---:|:---:|:---:|---|
| 80000 | 226ms | 48 | — | 초기, 35° 이동 90초 |
| 2000 | 5.6ms | 48 | ~13°/s | 35° ~2.2초 |
| 500 | 1.4ms | 48 | ~44°/s | |
| 300 | 0.85ms | 48 | ~67°/s | |
| 300 | 0.85ms | 400 | ~94°/s | 2.7Hz 업데이트 — 끊김 |
| 200 | 0.57ms | 400 | ~94°/s | MAX_STEP이 각속도와 무관함을 확인 |
| **200** | **0.57ms** | **58** | **154°/s** | **현재 — 30Hz 연속, 부드럽고 빠름** |

**탈조 발생 시 안전 복구:**
```c
#define MOTOR_SPEED_DELAY   300
#define MOTOR_PAN_MAX_STEP  38
#define MOTOR_TILT_MAX_STEP 38
// → 101°/s, 30Hz, 탈조 없음
```

---

## 7. 수정 이력 (튜닝 기록)

### 수정 1 — AP_IDLE 가드 위치 수정 (체감 속도 저하 해결)

abs 누적 코드 **이전**에 AP_IDLE 체크를 배치하여 IP 실행 중 명령 누적에 의한 연쇄 지연 폭주 방지.

```c
// 이전: abs 누적 후 IDLE 체크 → IP 바쁠 때 abs가 계속 쌓임
// 이후: IDLE 아니면 즉시 return → 최신 1프레임치만 적용
if (!(MOTOR_RD(0x00) & AP_IDLE)) return;
g_motor_abs_pan += dpan;
MOTOR_WR(0x00, AP_START);
```

### 수정 2 — MOTOR_KP 상향 (각속도 개선)

`KP=0.25` → `KP=1.5`. 오차 100px 기준 13스텝(26°/s) → 54스텝(111°/s).  
`KP=2.0` 시험 시 진동 증가 → 1.5로 롤백 확정.

### 수정 3 — rang LP 필터 추가 (Pan 좌우 진동 억제)

```c
rang_lp = 0.4f * (float)rang + 0.6f * rang_lp;  // α=0.4
rang = (int16_t)rang_lp;
```

- α=0.4: ~2.5프레임(82ms) 지연, 노이즈 억제+추적속도 절충
- 느린 표적엔 α 낮춤, 빠른 표적엔 높임으로 수동 조정 가능

### 수정 4 — MOTOR_KD 제거 (Pan 진동 해결)

레이더 노이즈를 미분항이 증폭해 매 프레임 방향반전 → `KD=0.0f`로 제거.  
슬루 리미터(`MOTOR_CMD_RAMP`)가 급격한 명령 변화를 이미 완충하므로 D항 불필요.

### 수정 5 — 수동 모드 `M:` 명령 추가

`M:1\n` → `g_manual_mode=true` → minStep(5) 제한 해제, 1스텝(≈0.011°) 정밀 이동 가능.  
캘리브레이션 및 점검 전용.

### 수정 6 — Tilt 단독 이동 허용

AI 추적 진입 조건: `pan_steps != 0` → `(pan||tilt) != 0`  
`T:+N\n` 단독 전송으로 Tilt 이동 가능.

### 수정 7 — Phase 2: B:ex,ey 프로토콜 도입 (2026-06-01)

PC PID(`control.cpp`) 제거, FPGA PID 통일.  
- PC: `sendBBox(ex, ey)` → `B:ex,ey\n` 전송
- FPGA: `motor_update_ai_bbox()` — Pan+Tilt 모두 `motor_pid_step()` 처리
- 이중 PID 구조(AI모드 PC PID vs 레이더모드 FPGA PID)로 인한 모드 간 응답 특성 차이 해소

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
