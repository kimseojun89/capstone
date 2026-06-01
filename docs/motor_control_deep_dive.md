# FPGA 모터 제어 코드 상세 동작 설명

> **대상 보드:** PYNQ-Z2 (Zynq XC7Z020)
> **모터:** 28BYJ-48 × 2 (Pan + Tilt) + ULN2003 드라이버
> **핵심 파일:**
> - [uln2003_controller.cpp](file:///c:/Users/kimse/capstone/Vitis/uln2003_controller.cpp) — HLS IP (PL 하드웨어)
> - [ps_main.cpp](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp) — PS ARM 소프트웨어
> - [motor_pins.xdc](file:///c:/Users/kimse/capstone/vivado_project/antidrone.srcs/constrs_1/new/motor_pins.xdc) — 핀 배치

---

## 전체 구조 개요

```
┌─────────────────────────────────────────────────────────┐
│                    PC (Windows)                          │
│  ptcamera_tracker.exe                                    │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────┐   │
│  │ YOLO v8x │───→│ control  │───→│ serial_port.cpp  │   │
│  │ 드론탐지 │    │ .cpp PID │    │ "P:+30\n" 전송   │   │
│  └──────────┘    └──────────┘    └────────┬─────────┘   │
└───────────────────────────────────────────┼─────────────┘
                                            │ UART0 (USB-Serial, 256Kbps)
┌───────────────────────────────────────────┼─────────────┐
│                PYNQ-Z2 (Zynq)             │             │
│                                           ▼             │
│  ┌─────────────────────────────────────────────────┐    │
│  │              PS (ARM Cortex-A9)                  │    │
│  │              ps_main.cpp                         │    │
│  │                                                  │    │
│  │  UART0 RX ──→ host_parse_commands()             │    │
│  │                "P:±N" / "T:±N" 파싱              │    │
│  │                     │                            │    │
│  │  UART1 RX ──→ uart_parse() ──→ CORDIC ──→ rang  │    │
│  │  (레이더)     30byte 패킷      극좌표변환        │    │
│  │                     │                            │    │
│  │              ┌──────▼──────┐                     │    │
│  │              │  상태 머신   │                     │    │
│  │              │  (5단계)     │                     │    │
│  │              └──────┬──────┘                     │    │
│  │                     │ motor_pid_step() / 직접전달 │    │
│  │                     ▼                            │    │
│  │     g_motor_abs_pan/tilt (절대 스텝 목표)         │    │
│  │              │                                   │    │
│  │              │ AXI4-Lite 레지스터 Write           │    │
│  │  └───────────┼───────────────────────────────────┘    │
│  │                 ▼                                        │
│  │  ┌─────────────────────────────────────────────────┐    │
│  │  │              PL (프로그래머블 로직)               │    │
│  │  │         uln2003_controller HLS IP                │    │
│  │  │                                                  │    │
│  │  │  target_pan/tilt (AXI 레지스터) ──→ while 루프   │    │
│  │  │  current_pan/tilt (static 변수)     1스텝씩 이동 │    │
│  │  │                                                  │    │
│  │  │  pan_out[3:0] ──→ PMODA 핀 ──→ ULN2003 ──→ 모터  │    │
│  │  │  tilt_out[3:0]──→ PMODB 핀 ──→ ULN2003 ──→ 모터  │    │
│  │  └─────────────────────────────────────────────────┘    │
│  └────────────────────────────────────────────────────────┘
```

---

## 계층 1: HLS IP — `uln2003_controller` (PL 하드웨어)

> [uln2003_controller.cpp](file:///c:/Users/kimse/capstone/Vitis/uln2003_controller.cpp) — 86줄, HLS C로 합성된 하드웨어 IP

### 1-1. 인터페이스 정의

```cpp
void uln2003_controller(
    int target_pan,        // AXI4-Lite 오프셋 0x10 (PS가 Write)
    int target_tilt,       // AXI4-Lite 오프셋 0x18
    int speed_delay,       // AXI4-Lite 오프셋 0x20
    ap_uint<4> &pan_out,   // ap_none: 직접 GPIO 와이어 출력 (4비트)
    ap_uint<4> &tilt_out   // ap_none: 직접 GPIO 와이어 출력 (4비트)
)
```

| 포트 | HLS pragma | 의미 |
|---|---|---|
| `target_pan`, `target_tilt`, `speed_delay` | `s_axilite bundle=CTRL` | PS가 AXI 버스로 값을 쓰는 레지스터 |
| `return` | `s_axilite bundle=CTRL` | AP_START/AP_DONE/AP_IDLE 제어 신호 |
| `pan_out`, `tilt_out` | `ap_none` | 프로토콜 없는 **순수 와이어** — FPGA 핀에 직결 |

### 1-2. 내부 상태 변수 (static)

```cpp
static int current_pan = 0;    // 현재 Pan 위치 (하드웨어 레지스터로 유지)
static int current_tilt = 0;   // 현재 Tilt 위치
static int pan_idx = 0;        // 하프스텝 시퀀스 인덱스 (0~7)
static int tilt_idx = 0;
```

> [!IMPORTANT]
> **`static` 변수**는 HLS 합성 시 FPGA 내부 레지스터(FF/LUTRAM)가 됩니다.
> AP_START로 함수가 재호출되어도 이전 값이 **유지**됩니다.
> 따라서 PS가 매번 **절대 위치**를 쓰면, IP는 현재 위치에서 목표까지 **차이만큼만** 이동합니다.

### 1-3. 28BYJ-48 하프스텝 시퀀스

```cpp
const ap_uint<4> step_seq[8] = {
    0b0001,  // Phase 0: 코일 A만
    0b0011,  // Phase 1: 코일 A+B
    0b0010,  // Phase 2: 코일 B만
    0b0110,  // Phase 3: 코일 B+C
    0b0100,  // Phase 4: 코일 C만
    0b1100,  // Phase 5: 코일 C+D
    0b1000,  // Phase 6: 코일 D만
    0b1001   // Phase 7: 코일 D+A
};
```

**하프스텝 (1-2상 여자) 방식:**
- 8상 시퀀스를 한 바퀴 돌면 모터 축이 **5.625°** 회전
- 28BYJ-48의 내부 기어비 = 1:64
- → 출력축 기준: **한 스텝 = 5.625° / 64 / 8 ≈ 0.011°**
- → 360° 회전 = **4096 하프스텝**

### 1-4. 핵심 구동 루프

```cpp
while (current_pan != target_pan || current_tilt != target_tilt) {
    bool moved = false;

    // Pan: 목표보다 작으면 +1 스텝, 크면 -1 스텝
    if (current_pan < target_pan) {
        pan_idx = (pan_idx + 1) % 8;   // 정방향 시퀀스 전진
        current_pan++;
        moved = true;
    } else if (current_pan > target_pan) {
        pan_idx = (pan_idx - 1 + 8) % 8;  // 역방향
        current_pan--;
        moved = true;
    }

    // Tilt: 동일 로직
    if (current_tilt < target_tilt) { ... }
    else if (current_tilt > target_tilt) { ... }

    // GPIO 핀에 코일 패턴 출력
    pan_out  = step_seq[pan_idx];
    tilt_out = step_seq[tilt_idx];

    // 물리적 딜레이 (모터 반응 대기)
    if (moved) hw_delay(speed_delay);
}
```

**동작 원리:**
1. PS가 `target_pan=58`을 쓰고 `AP_START` → IP 깨어남
2. `current_pan=0`이므로 **58회 반복**: 매 반복마다 `pan_idx++`, 코일 패턴 출력, 딜레이
3. `current_pan == target_pan` 되면 while 종료 → **AP_DONE** 신호 발생
4. 다음 호출 시 `current_pan=58`에서 시작 → PS가 `target_pan=100` 쓰면 **42스텝만** 이동

### 1-5. `hw_delay()` — 스텝 간 딜레이

```cpp
void hw_delay(int delay_count) {
    #pragma HLS INLINE off
    volatile int dummy = 0;
    for (int i = 0; i < delay_count; i++) {
        dummy++;
    }
}
```

- **busy-wait 루프**: `volatile`로 최적화 방지
- `delay_count = 200` 기준 실측 **≈ 0.57ms/step**
- 58스텝 × 0.57ms = **≈ 33ms** (= 1프레임 @ 30fps)

> [!NOTE]
> `hw_delay`의 실제 지연 시간은 HLS 합성 결과(파이프라이닝, 클럭)에 따라 달라집니다.
> 현재 200MHz 클럭 기준 1루프 ≈ 2.825µs로 실측되었습니다.

### 1-6. 핀 배치 (XDC)

```
# Pan Motor → PMODA 오른쪽 열
pan_out[0] → U18 (JA3P)  → ULN2003 IN1
pan_out[1] → U19 (JA3N)  → ULN2003 IN2
pan_out[2] → W18 (JA4P)  → ULN2003 IN3
pan_out[3] → W19 (JA4N)  → ULN2003 IN4

# Tilt Motor → PMODB 왼쪽 열
tilt_out[0] → W14 (JB1P)  → ULN2003 IN1
tilt_out[1] → Y14 (JB1N)  → ULN2003 IN2
tilt_out[2] → T11 (JB2P)  → ULN2003 IN3
tilt_out[3] → T10 (JB2N)  → ULN2003 IN4
```

신호 레벨: FPGA 3.3V LVCMOS33 → ULN2003 입력 (문턱 ~1V이므로 호환)
전원: 28BYJ-48은 **외부 5V 별도 공급** 필수 (PMOD 3.3V 불가)

---

## 계층 2: PS 소프트웨어 — PID 제어 알고리즘

> [ps_main.cpp:585-627](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp#L585-L627) — `motor_pid_step()` 함수

### 2-1. 함수 시그니처

```cpp
static int motor_pid_step(
    float err_px,        // 픽셀 단위 오차 (화면 중심 대비)
    float frame_half,    // 프레임 절반 너비 (160px)
    int   deadband,      // 데드밴드 (35px)
    float kp,            // 비례 게인 (1.5)
    float kd,            // 미분 게인 (0.0 — 비활성)
    int   min_step,      // 최소 스텝 (5)
    int   max_step,      // 최대 스텝 (58)
    float* prev_err,     // [상태] 이전 정규화 오차
    float* smooth_cmd,   // [상태] 슬루 리미터 출력
    float ramp_scale     // 가속 배율 (1.0 평상시)
)
```

### 2-2. 처리 단계 (수치 예시: 오차 100px)

#### Step A: 데드밴드 체크

```cpp
if (fabsf(err_px) <= (float)deadband) {   // |100| > 35 → 통과
    // 데드밴드 내: smooth_cmd를 0으로 서서히 감속
    return 0;
}
```

- 오차 ≤ 35px이면 **모터 정지** (미세 진동 방지)
- 정지 시에도 `smooth_cmd`를 급정지가 아닌 **서서히 0으로** 감속

#### Step B: 오차 정규화 + PID

```cpp
float norm  = err_px / frame_half;           // 100 / 160 = 0.625
float deriv = (norm - *prev_err) / MOTOR_DT; // (0.625 - 0) / 0.033 = 18.9 (KD=0이라 무시)
*prev_err   = norm;                          // 다음 프레임용 저장

float target = kp * norm + kd * deriv;       // 1.5 × 0.625 + 0 = 0.9375
// 클램프 [-1, +1]
if (target >  1.0f) target =  1.0f;          // 0.9375 → 그대로
```

- 오차를 `-1.0 ~ +1.0` 범위로 **정규화** (화면 절반 = 1.0)
- 현재 KD=0.0이므로 **순수 P 제어** 동작 중

#### Step C: 슬루 리미터 (급가속 방지)

```cpp
float ramp = MOTOR_CMD_RAMP * MOTOR_DT * ramp_scale;
           = 20.0 * 0.033 * 1.0 = 0.66

float diff = target - *smooth_cmd;   // 0.9375 - 0 = 0.9375
if (diff >  ramp) *smooth_cmd += ramp;   // 0.9375 > 0.66 → smooth_cmd = 0.66
```

- 한 프레임에 최대 `0.66`만큼만 명령이 변할 수 있음
- **2프레임(66ms)**이면 최대 명령에 도달: `0 → 0.66 → 1.0`

#### Step D: 최소 스텝 비율 보정

```cpp
float span      = (float)(58 - 5) = 53.0;
float min_ratio = 5.0 / (5.0 + 53.0) = 0.0862;
float abs_cmd   = max(|0.66|, 0.0862) = 0.66;
```

- 명령이 너무 작으면 `min_ratio`로 올려서 **최소 5스텝** 보장

#### Step E: 스텝 변환

```cpp
int step = min_step + (int)(span * abs_cmd);
         = 5 + (int)(53 * 0.66)
         = 5 + 34 = 39
return cmd > 0.0f ? 39 : -39;  // 방향 포함
```

**결과:** 오차 100px → 첫 프레임에 **Pan 39스텝** 이동 명령

### 2-3. 시간에 따른 PID 응답 시뮬레이션

| 프레임 | 오차(px) | norm | target(PID) | smooth_cmd(슬루) | 스텝 |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | 100 | 0.625 | 0.938 | 0.66 | 39 |
| 2 | 65 | 0.406 | 0.609 | 0.609 | 37 |
| 3 | 30 | 0.188 | < deadband | 감속 중 | 0 |

→ **약 2프레임(66ms)에 100px 오차 해소**, 데드밴드(35px) 진입 시 정지

---

## 계층 3: PS 메인 루프 — 5단계 우선순위 상태머신

> [ps_main.cpp:778-803](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp#L778-L803)

### 3-1. 매 프레임(33ms) 실행 순서

```
┌─────────────────────────────────────────────────────┐
│ 30Hz 메인 루프 (33ms 주기)                           │
│                                                      │
│  Step 1: uart_accumulate()        레이더 UART1 수신  │
│  Step 2: uart_parse()             30byte 패킷 파싱   │
│  Step 3: process_radar_target()   CORDIC 극좌표변환   │
│  Step 4: rang LP필터              노이즈 억제         │
│  Step 5: kalman_ip_run()          칼만 예측/보정      │
│  Step 6: host_accumulate()        PC UART0 수신       │
│  Step 7: host_parse_commands()    P:/T: 명령 파싱     │
│  Step 8: ★ 모터 상태머신 ★        아래 5단계 분기     │
│  Step 9: g_host_pan/tilt = 0      명령 리셋           │
│  Step 10: responsive_sleep_us()   33ms 대기 (폴링)    │
└─────────────────────────────────────────────────────┘
```

### 3-2. 모터 상태머신 — 5단계 우선순위 분기

```
if (g_host_pan_steps != 0 || g_host_tilt_steps != 0) && cooldown == 0)
    ├─ ① AI 추적 모드 진입 (Pan 또는 Tilt 어느 쪽이든)
    │
elif (g_host_cmd_cooldown > 0)
    ├─ ② 쿨다운 대기 (모터 이동 완료 대기)
    │
elif (g_ai_lock_frames > 0)
    ├─ ③ AI 잠금 유지 (레이더 차단)
    │
elif (rvc > 0)
    ├─ ④ 레이더 단독 모드
    │
else
    └─ ⑤ 정지 (표적 없음)
```

#### ① AI 추적 모드 (`motor_update_full_host`)

```cpp
if (g_host_pan_steps != 0 || g_host_tilt_steps != 0) && g_host_cmd_cooldown == 0) {
    motor_update_full_host(g_host_pan_steps, g_host_tilt_steps);
    g_host_cmd_cooldown = 2;   // 2프레임(66ms) 쿨다운
    g_ai_lock_frames    = 4;   // 4프레임(132ms) AI 잠금
}
```

**동작:**
- PC가 `P:+30\n` 또는 `T:+11\n`을 보내면 `g_host_pan/tilt_steps` 갱신
- `P:` 또는 `T:` 둘 중 하나만 있어도 진입 (Tilt 단독 이동 가능)
- PID 없이 **스텝값을 직접 사용** (클램프만 적용)
- 수동 모드(`M:1\n`) 시 minStep 제한 해제 → 1스텝(0.011°)부터 정밀 이동
- 스텝 절대 목표에 누적: `g_motor_abs_pan += dpan`
- AXI로 IP에 전달 → IP가 스텝 구동

**왜 PID 없이?** PC 측 `control.cpp`에서 이미 PID 계산 완료 → 스텝 델타로 변환해서 전송하기 때문

#### ② 쿨다운 대기

```cpp
} else if (g_host_cmd_cooldown > 0) {
    g_host_cmd_cooldown--;
    if (g_ai_lock_frames > 0) g_ai_lock_frames--;
}
```

- 2프레임(66ms) 동안 **새 명령 수신 거부** → 모터 IP가 이전 명령 실행 완료 대기
- 이 기간에 레이더가 들어와도 **무시**

#### ③ AI 잠금 유지

```cpp
} else if (g_ai_lock_frames > 0) {
    g_ai_lock_frames--;
}
```

- 쿨다운 끝났지만 PC의 **다음 명령 도착 전** 공백 기간
- 이 기간에 레이더가 들어와도 **모터를 현재 위치에 고정**
- 총 4프레임(132ms) 유지 → PC의 전송 주기(~66ms)보다 길어서 공백을 완전 커버

#### ④ 레이더 단독 모드 (`motor_update_hybrid`)

```cpp
} else if (rvc > 0) {
    motor_update_hybrid(angle_to_px(rang) - CAM_W_PX / 2,  // Pan: 레이더 PID
                        true,
                        g_host_tilt_steps);                 // Tilt: PC T: 직접
}
```

**Pan 경로:**
1. LF 필터 적용된 레이더 각도 `rang`
2. `angle_to_px(rang)` 편차 변환 ($e_x = px - 160$)
3. `motor_pid_step` PID로 누적 목표 스텝 구동

**Tilt 경로:**
- PC 호스트 `T:±N` 직접 구동.

#### ⑤ 정지

```cpp
} else {
    g_had_target = false;
}
```

- 표적 없음 → 정지.

### 3-3. 프레임 끝: 호스트 명령 리셋

```cpp
g_host_pan_steps  = 0;
g_host_tilt_steps = 0;
```

---

## 계층 4: PS ↔ HLS IP 통신 — AXI4-Lite

### 4-1. 레지스터 접근 매크로

```cpp
#define MOTOR_BASEADDR  0x40030000

#define MOTOR_WR(off, v)  Xil_Out32(MOTOR_BASEADDR + (off), (u32)(v))
#define MOTOR_RD(off)     Xil_In32 (MOTOR_BASEADDR + (off))
```

### 4-2. 레지스터 맵

| 오프셋 | 레지스터 | 방향 | 용도 |
|:---:|---|:---:|---|
| `0x00` | ap_ctrl | R/W | bit0=AP_START, bit1=AP_DONE, bit2=AP_IDLE |
| `0x10` | target_pan | W | Pan 절대 스텝 목표 |
| `0x18` | target_tilt | W | Tilt 절대 스텝 목표 |
| `0x20` | speed_delay | W | hw_delay 루프 카운트 (현재 200) |

### 4-3. IP 호출 시퀀스

```cpp
if (!(MOTOR_RD(0x00) & AP_IDLE)) return;
g_motor_abs_pan  += dpan;
g_motor_abs_tilt += dtilt;
MOTOR_WR(0x10, (u32)g_motor_abs_pan);
MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
MOTOR_WR(0x00, AP_START);
```

### 4-4. AP_IDLE 가드 — 왜 필요한가?
(AP_IDLE=0 일 때 쓰기 무시하여 딜레이 폭주 방지)

---

## 계층 5: PC → FPGA 명령 흐름

```
YOLO 탐지 → ByteTrack 추적 → control.cpp PID → panSteps/tiltSteps
                                                       │
                                     serial.sendPanCommand(30)
                                     → "P:+30\n" 전송 (UART0, 256Kbps)
```

**동적 쿨다운 계산:**
```cpp
const double moveSec = abs(panSteps) * STEP_SEC;
const double cooldown = max(moveSec + STABLE_SEC, sendInterval);
```

---

## 부팅 시 초기화 흐름
`MOTOR_WR(0x10, 0); MOTOR_WR(0x18, 0); MOTOR_WR(0x20, MOTOR_SPEED_DELAY); MOTOR_WR(0x00, AP_START);` 미세 진동 후 로킹.

---

## 속도 참고표
` 각속도 = 360 / (4096 * speed_delay * 2.825us) `
(SPEED_DELAY=200 -> 154°/s)

---

## 요약: 한 프레임의 전체 동작 트레이스
(Frame N 루틴 상세 분석)
```
