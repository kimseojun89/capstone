# FPGA 모터 포팅 & 튜닝 가이드 (최종)

> **작성일:** 2026-05-29  
> **대상 파일:**  
> - `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp`  
> - `antidrone/unified_gui.py`  
> **보드:** PYNQ-Z2 (Zynq XC7Z020)  
> **모터:** 28BYJ-48 × 2 (Pan/Tilt) + ULN2003 드라이버 모듈  
> **HLS IP:** `uln2003_controller` (`Vitis/uln2003_controller.cpp`)

---

## 1. 시스템 아키텍처

### Arduino 방식 (이전)
```
PC (control.cpp, PID)
  ↓ panSteps, tiltSteps (델타) — 시리얼 115200bps
Arduino → 코일 시퀀스 구동
  ↓
28BYJ-48
```

### FPGA 방식 (현재)
```
PC / GUI (unified_gui.py)
  ↓ P:±N\n  T:±N\n — UART0 256000bps
PS (ps_main.cpp, PID + 직접 제어)
  ↓ g_motor_abs_pan/tilt (절대위치) — AXI4-Lite
PL ULN2003 HLS IP  (static current_pan/tilt 내부 유지)
  ↓ 하프스텝 시퀀스 자동 구동
28BYJ-48
```

**핵심 차이:** HLS IP는 절대 위치로 동작. PS는 델타를 누적해 절대 목표를 기록.

---

## 2. HLS IP 인터페이스 (`uln2003_controller.cpp`)

### 레지스터 맵
| 오프셋 | 역할 | 방향 |
|:---:|---|:---:|
| `0x00` | AP 제어 (AP_START / AP_DONE / AP_IDLE) | R/W |
| `0x10` | `target_pan` — 절대 스텝 목표 | W |
| `0x18` | `target_tilt` — 절대 스텝 목표 | W |
| `0x20` | `speed_delay` — `hw_delay()` 루프 카운트 | W |

```c
#define AP_START  (1u << 0)
#define AP_DONE   (1u << 1)
#define AP_IDLE   (1u << 2)
```

### 동작 원리
```c
// HLS IP 내부 (uln2003_controller.cpp)
static int current_pan = 0, current_tilt = 0;  // AP_START마다 유지됨

while (current_pan != target_pan || current_tilt != target_tilt) {
    // 한 스텝씩 이동
    if (current_pan < target_pan) { pan_idx++; current_pan++; }
    ...
    hw_delay(speed_delay);   // ← speed_delay = 루프 카운트, µs 아님!
}
```

> ⚠️ `speed_delay`는 **µs 단위가 아님** — `hw_delay()` 바쁜 대기 루프 횟수.  
> 실측: `speed_delay = 80000` → 226ms/step.

---

## 3. 핀 연결

### PYNQ-Z2 커넥터 배치 (보드 우측, 핀 레이블 기준)

#### PMODA — Pan 모터 (오른쪽 열) + 레이더 UART1 (왼쪽 열)
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

#### PMODB — Tilt 모터 (왼쪽 열)
```
        왼쪽 열 (Tilt 모터)      오른쪽 열
       ┌──────────────────┬──────────────────┐
Row 1  │  JB1P  W14       │  JB3P  V16       │  ← Tilt IN1 / 미사용
Row 2  │  JB1N  Y14       │  JB3N  W16       │  ← Tilt IN2 / 미사용
Row 3  │  JB2P  T11       │  JB4P  V12       │  ← Tilt IN3 / 미사용
Row 4  │  JB2N  T10       │  JB4N  W13       │  ← Tilt IN4 / 미사용
Row 5  │       GND        │       GND        │
Row 6  │       3V3        │       3V3        │
       └──────────────────┴──────────────────┘
```

### 배선 요약표
| PYNQ-Z2 | FPGA 핀 | ULN2003 (Pan) |
|:---:|:---:|:---:|
| JA3P | U18 | IN1 |
| JA3N | U19 | IN2 |
| JA4P | W18 | IN3 |
| JA4N | W19 | IN4 |

| PYNQ-Z2 | FPGA 핀 | ULN2003 (Tilt) |
|:---:|:---:|:---:|
| JB1P | W14 | IN1 |
| JB1N | Y14 | IN2 |
| JB2P | T11 | IN3 |
| JB2N | T10 | IN4 |

### 전원
```
외부 5V (+) ──┬── ULN2003 Pan  VCC
              └── ULN2003 Tilt VCC
외부 5V (-) ──┬── ULN2003 Pan  GND
              ├── ULN2003 Tilt GND
              └── PMODA/B GND Row5  ← 공통 GND 필수
```
> ⚠️ 28BYJ-48은 **5V 전용**. PMODA/B 3V3으로는 구동 불가.  
> 신호선(IN1~4)은 FPGA 3.3V LVCMOS33 그대로 연결 OK (ULN2003 입력 문턱 ~1V).

---

## 4. ps_main.cpp — 최종 모터 파라미터

```c
// ── PID ─────────────────────────────────────────────────────
#define MOTOR_KP            1.5f     // 비례 게인 (0.25 → 1.5; 2.0은 진동 증가로 롤백)
#define MOTOR_KD            0.0f     // 미분 게인 제거 (레이더 노이즈 증폭 방지, P제어만)
#define MOTOR_PAN_DEADBAND  35       // 픽셀
#define MOTOR_TILT_DEADBAND 35       // 픽셀

// ── 스텝 범위 ────────────────────────────────────────────────
// MAX_STEP = 프레임(33ms) / ms/step 으로 설정 → IP 실행≈프레임 → 30Hz 연속
#define MOTOR_PAN_MIN_STEP  5
#define MOTOR_PAN_MAX_STEP  58    // 33ms / 0.57ms ≈ 58 스텝
#define MOTOR_TILT_MIN_STEP 5
#define MOTOR_TILT_MAX_STEP 58

// ── 속도 ─────────────────────────────────────────────────────
// hw_delay() 루프 카운트 실측 기준표:
//   80000 = 226ms/step  (최초 세팅, 35도 이동에 90초)
//    2000 =   5.6ms/step (35도 ~2.2초)
//     500 =   1.4ms/step (35도 ~0.56초)
//     300 =   0.85ms/step → MAX_STEP=38 최적 (101°/s, 안전값)
//     200 =   0.57ms/step → MAX_STEP=58 최적 (154°/s) ← 현재
//     150 =   0.42ms/step → 탈조 위험
#define MOTOR_SPEED_DELAY   200

// ── 슬루율 ───────────────────────────────────────────────────
// AccelStepper setAcceleration 역할
// 2.2 → 14프레임(462ms)에 최대 명령 도달 (너무 느림)
// 20.0 → 2프레임(66ms)에 최대 명령 도달  ← 현재
#define MOTOR_CMD_RAMP      20.0f
// ACQUIRE_RAMP: :121에서 0.6f로 정의되나 :128 #undef → :129 1.0f 재정의.
// 모터 함수가 보는 유효값은 1.0f (첫 표적 가속 억제 없음)
#define MOTOR_ACQUIRE_RAMP  1.0f
#define MOTOR_DT            0.033f   // 30fps 기준 루프 주기
```

---

## 5. ps_main.cpp — 모터 제어 분기 (`ps_main.cpp:944~973`)

```
우선순위:  AI 추적 → (cooldown 대기) → (AI lock 유지) → 퓨전 → 레이더 단독 → 정지
```

> ⚠️ **Pan은 항상 "증분 PID"** (`g_motor_abs_pan += dpan`). 구버전 문서의
> "절대 위치 SET(`g_motor_abs_pan = -(rang/10×4096/360)`)"은 **현재 코드에 없음**.
> 레이더 좌표(rang)는 LP필터(α=0.4)로 평활된 뒤 `angle_to_px()`로 픽셀 오차가 되어 PID에 들어간다.

### 모드 1 — AI 추적 (PC가 Pan+Tilt 모두 제어)
```c
// g_host_pan_steps != 0 && g_host_cmd_cooldown == 0 일 때 진입
// PC ptcamera_tracker가 P:±N\n / T:±N\n 전송 시 활성화
motor_update_full_host(g_host_pan_steps, g_host_tilt_steps);
g_host_cmd_cooldown = HOST_CMD_COOLDOWN_FRAMES;  // 이동 완료까지 레이더 개입 차단
g_ai_lock_frames    = AI_LOCK_FRAMES;            // 다음 PC 명령 전 공백을 레이더가 못 채우게 잠금
```

### (모드 1 부속) cooldown / AI lock — 레이더 오버라이드 차단
```c
} else if (g_host_cmd_cooldown > 0) { g_host_cmd_cooldown--; /* 이동 완료 대기 */ }
} else if (g_ai_lock_frames  > 0) { g_ai_lock_frames--;     /* 현재 위치 유지 */ }
```

### 모드 2 — 레이더+카메라 퓨전 (Pan: 레이더, Tilt: 카메라)
```c
// fi >= 0 (퓨전 매칭) 시. Pan/Tilt 모두 증분 PID
int cy = bbox[fi].y + bbox[fi].h / 2;
motor_update(angle_to_px(rang) - CAM_W_PX/2,   // Pan : 레이더 방위각 픽셀오차 PID
             CAM_H_PX/2 - cy,                   // Tilt: 카메라 BBox cy PID
             true);
```

### 모드 3 — 레이더 단독 (Pan: 레이더, Tilt: PC T: 명령)
```c
// rvc > 0 && fi < 0 시
motor_update_hybrid(angle_to_px(rang) - CAM_W_PX/2,  // Pan : 레이더 방위각 PID
                    true, g_host_tilt_steps);        // Tilt: 호스트 T: 직접 스텝
```

### 모드 4 — 표적 없음
```c
g_had_target = false;   // 마지막 위치 유지
// 매 프레임 끝: g_host_pan_steps = g_host_tilt_steps = 0 (PC가 30Hz 재전송 필요)
```

---

## 6. motor_update 핵심 로직

### AP_IDLE 게이트 — 누적 방지 (오버슈트 방지)

```c
static void motor_update(int err_x, int err_y, bool has_target)
{
    float ramp_scale = (has_target && !g_had_target) ? MOTOR_ACQUIRE_RAMP : 1.0f;
    g_had_target = has_target;

    int dpan  = motor_pid_step(..., ramp_scale);
    int dtilt = motor_pid_step(..., ramp_scale);
    if (dpan == 0 && dtilt == 0) return;

    // ★ IP 실행 중이면 PID 상태만 유지, g_motor_abs는 갱신하지 않음
    //   → IP 완료 시 최신 1프레임치만 적용 → 오버슈트 방지
    if (!(MOTOR_RD(0x00) & AP_IDLE)) return;

    g_motor_abs_pan  += dpan;
    g_motor_abs_tilt += dtilt;
    MOTOR_WR(0x10, (u32)g_motor_abs_pan);
    MOTOR_WR(0x18, (u32)g_motor_abs_tilt);
    MOTOR_WR(0x20, MOTOR_SPEED_DELAY);
    MOTOR_WR(0x00, AP_START);
}
```

### MAX_STEP 최적값 계산 공식

```
최적 MAX_STEP = PS 프레임 주기(ms) / step당 시간(ms)
             = 33ms / (SPEED_DELAY × 2.825µs)

각속도(°/s)  = 360 / (4096 × SPEED_DELAY × 2.825µs)
             ※ MAX_STEP과 무관 — SPEED_DELAY로만 결정됨
```

> **MAX_STEP이 너무 크면:** IP 1회 실행이 수백ms → 업데이트 2~3Hz → 끊기고 느린 느낌.  
> **MAX_STEP = 프레임 주기에 동기화:** 30Hz 연속 업데이트 → 부드럽고 빠름.

---

## 7. 속도 튜닝 전체 이력

| 단계 | SPEED_DELAY | MAX_STEP | 각속도 | 비고 |
|:---:|:---:|:---:|:---:|---|
| 초기 | 80000 | 48 | — | 35도 이동 90초 |
| 1차 | 2000 | 48 | ~13°/s | 35도 ~2.2초 |
| 2차 | 500 | 48 | ~44°/s | 35도 ~0.56초 |
| 3차 | 300 | 48 | ~67°/s | 35도 ~0.34초 |
| 4차 | 300 | 400 | ~94°/s | 2.7Hz 업데이트 (끊김) |
| 5차 | 200 | 400 | ~94°/s | 동일 — MAX_STEP 커도 속도 불변 |
| **최종** | **200** | **58** | **154°/s** | **30Hz 연속, 부드럽고 빠름** |

### 탈조 발생 시 안전 복구 설정
```c
#define MOTOR_SPEED_DELAY   300
#define MOTOR_PAN_MAX_STEP  38
#define MOTOR_TILT_MAX_STEP 38
// → 101°/s, 30Hz, 안전
```

---

## 8. UART0 모터 명령 프로토콜 (ptcamera_tracker → FPGA)

> ⚠️ **키보드 수동 제어는 제거됨.** 모터 명령은 PC `ptcamera_tracker.exe`가
> YOLO/PID 결과로 자동 생성·전송한다(`serial_port.cpp`의 `sendPanCommand`/`sendTiltCommand`).
> `unified_gui.py`는 **표시 전용**이며 시리얼로 모터 명령을 보내지 않는다
> (`unified_gui.py:486~489` — COM4는 tracker가 독점, GUI가 송신하면 포트 충돌).

### FPGA UART0 명령 형식
```
포트: UART0 (Vitis 콘솔 겸용, CP210x USB-Serial, COM4)
속도: 256000 bps

  P:+30\n  → g_host_pan_steps  = +30  → AI 추적 모드 진입 (Pan+Tilt PC 직접 제어)
  P:-30\n  → g_host_pan_steps  = -30
  T:+20\n  → g_host_tilt_steps = +20  → 퓨전/레이더 단독 모드의 Tilt PC 제어
  T:-20\n  → g_host_tilt_steps = -20

주의: FPGA는 매 프레임(33ms) 끝에서 g_host_pan/tilt_steps를 0으로 리셋
      → tracker가 30Hz로 지속 전송해야 모터 움직임이 유지됨
```

### 명령 흐름 (ptcamera_tracker → FPGA)
```
YOLOv8x 탐지 + ByteTrack + ControlLoop(PID)   (ptcamera_tracker.exe)
  ↓ telemetry.command.panSteps / tiltSteps
serial.sendPanCommand(N)  → "P:+N\n"   (ptcamera_tracker.cpp:254)
serial.sendTiltCommand(N) → "T:+N\n"   (ptcamera_tracker.cpp:257)
  ↓ UART0 256000bps (COM4)
FPGA host_parse_commands()  → g_host_pan_steps / g_host_tilt_steps
  ↓ 매 프레임 소비 (motor_update_full_host / *_hybrid)
g_motor_abs_pan/tilt 갱신 → AXI write → ULN2003 HLS IP → 모터 이동
```

### unified_gui.py 키 (모터와 무관, 표시/디버그용)
| 키 | 동작 |
|:---:|---|
| R | 레이더 히트맵 리셋 |
| D | 디버그 로그 토글 |
| Q | 종료 |

---

## 9. 부팅 확인 체크리스트

### Vitis 터미널 (UART0, 256000 baud)
```
============================================
  Anti-Drone Sensor Fusion Core Booting...
  Console: UART0 @ 256000 bps
  Radar  : UART1 EMIO @ 256000 bps
============================================

[+] Radar UART1 OK (256000 bps, EMIO PMODA)
[+] MTI IP OK
[+] Kalman IP (PL HLS) OK
[+] Motor IP OK (home position)   ← 이 순간 모터 미세 진동 후 정지
```

### 모터 반응 진단
| 증상 | 원인 | 조치 |
|---|---|---|
| 미세 진동 후 정지 | ✅ 정상 홈포지션 이동 | — |
| 아무 반응 없음 | 5V 미공급 또는 공통 GND 누락 | 전원/GND 재확인 |
| 진동만 (회전 안 됨) | 탈조 — SPEED_DELAY 너무 낮음 | `SPEED_DELAY = 300, MAX_STEP = 38` |
| 반대 방향 회전 | IN 핀 순서 반전 | IN1↔IN4, IN2↔IN3 스왑 |
| 표적 추종 안 함 | tracker 미실행/COM4 미점유 | `ptcamera_tracker.exe`가 COM4로 `P:`/`T:` 전송 중인지 확인 |

---

## 10. 다음 작업 시 참고 사항

- `USE_MOCK_MTI = 1` — 실제 카메라 미연결 상태, 가상 BBox 사용 중 (OPEN_ISSUES A1)
- `USE_HLS_CORDIC = 1`, `USE_HLS_KALMAN = 1` — PL IP 사용 모드
- 레이더 없이 모터 단독 점검 시: COM4로 `P:+30\n`을 30Hz 반복 전송하면 Pan 단독 동작 확인 가능
- `MOTOR_KP = 1.5f` — 기존 `control.cpp`의 0.25에서 상향(2.0은 진동으로 롤백)
- Pan 방향: 퓨전/레이더 모드는 `angle_to_px(rang)` 픽셀 오차를 PID에 입력(증분).
  rang은 LP필터(α=0.4)로 평활됨. 방향이 반대면 `angle_to_px()` 또는 PID 부호를 점검할 것.
