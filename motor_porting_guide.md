# FPGA 모터 포팅 & 튜닝 가이드

> **대상 파일:** `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp`  
> **보드:** PYNQ-Z2 (Zynq XC7Z020)  
> **모터:** 28BYJ-48 × 2 (Pan/Tilt) + ULN2003 드라이버 모듈  
> **HLS IP:** `uln2003_controller` (`Vitis/uln2003_controller.cpp`)

---

## 1. 아키텍처 개요: Arduino → FPGA 변환

### 기존 Arduino 방식
```
PC (control.cpp, PID 계산)
  ↓ panSteps, tiltSteps (델타 스텝) — 시리얼 전송
Arduino 펌웨어
  ↓ 코일 시퀀스 직접 구동
28BYJ-48
```

### FPGA 방식 (현재)
```
PS (ps_main.cpp, PID 계산)
  ↓ g_motor_abs_pan/tilt (절대 위치) — AXI4-Lite 레지스터 기록
PL ULN2003 HLS IP (uln2003_controller)
  ↓ static current_pan/tilt 내부 유지, 하프스텝 시퀀스 자동 구동
28BYJ-48
```

**핵심 차이:** HLS IP는 절대 위치로 동작 (내부 `static current_pan/tilt` 유지). PS는 델타 스텝을 매 프레임 누적해 절대 목표를 IP에 기록.

---

## 2. HLS IP 레지스터 맵

| 오프셋 | 역할 | 방향 |
|:---:|---|:---:|
| `0x00` | AP 제어 (AP_START/AP_DONE/AP_IDLE) | R/W |
| `0x10` | target_pan (절대 스텝) | W |
| `0x18` | target_tilt (절대 스텝) | W |
| `0x20` | speed_delay (hw_delay 루프 카운트) | W |

```c
#define AP_START  (1u << 0)
#define AP_DONE   (1u << 1)
#define AP_IDLE   (1u << 2)
```

> ⚠️ `speed_delay`는 **µs 단위가 아닌 hw_delay() 루프 카운트**. 실측 기준: 80000 = 226ms/step.

---

## 3. 핀 연결 가이드

### PYNQ-Z2 커넥터 배치 (보드 우측)

#### PMODA — Pan 모터 + 레이더

```
        왼쪽 열 (레이더)    오른쪽 열 (Pan 모터)
       ┌─────────────┬──────────────┐
Row 1  │  JA1P (Y18) │  JA3P (U18)  │  ← 레이더RX / Pan IN1
Row 2  │  JA1N (Y19) │  JA3N (U19)  │  ← 레이더TX / Pan IN2
Row 3  │  JA2P (Y16) │  JA4P (W18)  │  ←  미사용  / Pan IN3
Row 4  │  JA2N (Y17) │  JA4N (W19)  │  ←  미사용  / Pan IN4
Row 5  │     GND     │     GND      │
Row 6  │     3V3     │     3V3      │
       └─────────────┴──────────────┘
```

#### PMODB — Tilt 모터

```
        왼쪽 열 (Tilt 모터)  오른쪽 열
       ┌─────────────┬──────────────┐
Row 1  │  JB1P (W14) │  JB3P (V16)  │  ← Tilt IN1 / 미사용
Row 2  │  JB1N (Y14) │  JB3N (W16)  │  ← Tilt IN2 / 미사용
Row 3  │  JB2P (T11) │  JB4P (V12)  │  ← Tilt IN3 / 미사용
Row 4  │  JB2N (T10) │  JB4N (W13)  │  ← Tilt IN4 / 미사용
Row 5  │     GND     │     GND      │
Row 6  │     3V3     │     3V3      │
       └─────────────┴──────────────┘
```

### ULN2003 배선

| PYNQ-Z2 | FPGA 핀 | ULN2003 (Pan) |
|:---:|:---:|:---:|
| JA3P | U18 | IN1 |
| JA3N | U19 | IN2 |
| JA4P | W18 | IN3 |
| JA4N | W19 | IN4 |
| GND (Row 5) | — | GND |

| PYNQ-Z2 | FPGA 핀 | ULN2003 (Tilt) |
|:---:|:---:|:---:|
| JB1P | W14 | IN1 |
| JB1N | Y14 | IN2 |
| JB2P | T11 | IN3 |
| JB2N | T10 | IN4 |
| GND (Row 5) | — | GND |

### 전원

```
외부 5V (+) ──┬── ULN2003 Pan  VCC
              └── ULN2003 Tilt VCC
외부 5V (-) ──┬── ULN2003 Pan  GND
              ├── ULN2003 Tilt GND
              └── PMODA/B GND (공통 GND 필수)
```

> ⚠️ 28BYJ-48은 **5V 전용**. PMODA/B의 3V3으로는 구동 불가.  
> 신호선(IN1~4)은 FPGA 3.3V LVCMOS33 출력을 그대로 연결해도 ULN2003이 인식함.

---

## 4. ps_main.cpp 모터 파라미터

```c
// ── 속도 관련 ─────────────────────────────────────────
// hw_delay() 루프 카운트 실측 기준표:
//   80000 = 226ms/step  (35도: ~90초)
//     300 = 0.85ms/step → MAX_STEP=38 최적 (101°/s, 안전)
//     200 = 0.57ms/step → MAX_STEP=58 최적 (154°/s) ← 현재
//     150 = 0.42ms/step → MAX_STEP=78 최적 (186°/s, 탈조 위험)
#define MOTOR_SPEED_DELAY   200

// MAX_STEP = 33ms(프레임) / ms/step 으로 설정해야
// IP 실행시간 ≈ 프레임 간격 → 30Hz 연속 업데이트 달성
// MAX_STEP이 너무 크면 IP가 수백ms 점유 → 2~3Hz로 떨어져 오히려 느림
#define MOTOR_PAN_MAX_STEP  58
#define MOTOR_TILT_MAX_STEP 58
#define MOTOR_PAN_MIN_STEP  5
#define MOTOR_TILT_MIN_STEP 5

// ── PID 관련 ──────────────────────────────────────────
// MOTOR_CMD_RAMP: AccelStepper의 setAcceleration 역할
// 2.2 → 14프레임(462ms) 가속 (너무 느림)
// 20.0 → 2프레임(66ms) 가속 ← 현재
#define MOTOR_CMD_RAMP      20.0f
#define MOTOR_DT            0.033f   // 30fps 기준 루프 주기
#define MOTOR_KP            0.25f
#define MOTOR_KD            0.02f
#define MOTOR_PAN_DEADBAND  35       // 픽셀
#define MOTOR_TILT_DEADBAND 35       // 픽셀
#define MOTOR_ACQUIRE_RAMP  1.0f     // 첫 표적 획득 시 가속 비율 (1.0 = 억제 없음)
```

---

## 5. motor_update 핵심 로직

### 누적 방지 (오버슈트 방지)

```c
static void motor_update(int err_x, int err_y, bool has_target)
{
    float ramp_scale = (has_target && !g_had_target) ? MOTOR_ACQUIRE_RAMP : 1.0f;
    g_had_target = has_target;

    int dpan  = motor_pid_step(..., ramp_scale);
    int dtilt = motor_pid_step(..., ramp_scale);

    if (dpan == 0 && dtilt == 0) return;

    // ★ IP 실행 중이면 PID 상태만 유지하고 누적하지 않음
    //   → IP가 끝날 때 최신 PID 출력 1프레임치만 적용 (오버슈트 방지)
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;

    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
}
```

### 표적 없을 때 정지

```c
// main 루프 내
if (fi >= 0) {
    motor_update(cx - CAM_W_PX/2, CAM_H_PX/2 - cy, true);  // 카메라 매칭
} else if (rvc > 0) {
    motor_update(angle_to_px(rang) - CAM_W_PX/2, 0, true);  // 레이더 단독
} else {
    g_had_target = false;  // 모터 명령 없음 → 마지막 위치 유지
}
```

---

## 6. 속도 튜닝 이력

| 단계 | SPEED_DELAY | MAX_STEP | 결과 |
|:---:|:---:|:---:|---|
| 초기 | 80000 | 48 | 35도 이동 90초 (너무 느림) |
| 1차 | 2000 | 48 | 35도 이동 ~2.2초 |
| 2차 | 500 | 48 | 35도 이동 ~0.56초 |
| 3차 | 300 | 48 | 35도 이동 ~0.34초 |
| 4차 | 300 | 400 | 35.2°/트리거, 업데이트 2.7Hz (끊김) |
| **최종** | **200** | **58** | **154°/s, 30Hz 연속 업데이트** |

### 탈조 발생 시 안전 설정

```c
#define MOTOR_SPEED_DELAY   300
#define MOTOR_PAN_MAX_STEP  38
#define MOTOR_TILT_MAX_STEP 38
// → 101°/s, 30Hz 업데이트, 안전
```

---

## 7. 속도 공식 (MAX_STEP 최적값 계산)

```
최적 MAX_STEP = PS 프레임 주기(ms) / step당 시간(ms)
             = 33ms / (SPEED_DELAY × 2.825µs)

각속도(°/s)  = MAX_STEP × (360 / 4096) / (MAX_STEP × SPEED_DELAY × 2.825µs)
             = 360 / (4096 × SPEED_DELAY × 2.825µs)
             ※ 각속도는 MAX_STEP과 무관, SPEED_DELAY로만 결정됨
```

| SPEED_DELAY | ms/step | 최적 MAX_STEP | 각속도 | 비고 |
|:---:|:---:|:---:|:---:|---|
| 300 | 0.85ms | 38 | 101°/s | 안전 |
| 200 | 0.57ms | 58 | 154°/s | 현재 |
| 150 | 0.42ms | 78 | 186°/s | 탈조 위험 |

> **MAX_STEP이 너무 크면:** IP 1회 실행이 수백ms → 업데이트가 2~3Hz로 떨어져 체감상 느려짐.  
> **MAX_STEP이 너무 작으면:** 업데이트는 많지만 각도 분해능이 낮아 추적 정밀도 하락.

---

## 8. 부팅 확인 체크리스트

```
Vitis 터미널 (115200 baud, UART0) 에서 확인:

[+] UART1 OK (256000bps, EMIO PMODA)   ← 레이더 포트
[+] MTI IP OK
[+] Kalman IP (PL HLS) OK
[+] Motor IP OK (home position)        ← 이 순간 모터 미세 진동 후 정지
```

| 모터 반응 | 진단 |
|---|---|
| 미세 진동 후 정지 | ✅ 정상 |
| 아무 반응 없음 | 5V 전원 · 공통 GND 확인 |
| 진동만 (회전 안 됨) | 탈조 → SPEED_DELAY 2배 증가 |
| 반대 방향 회전 | IN1↔IN4, IN2↔IN3 스왑 |
