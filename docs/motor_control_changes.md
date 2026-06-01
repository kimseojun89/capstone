# ps_main.cpp 모터 제어 수정 이력

> **대상 파일:** `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp`
> **최종 작성:** 2026-06-01 (코드 대조 갱신)
> **검증 기준:** 본 문서의 모든 파라미터/분기는 `ps_main.cpp` 실제 코드와 1:1 대조됨.

---

## 현재 최종 파라미터 (`ps_main.cpp:101~122`)

```c
#define MOTOR_KP            1.5f
#define MOTOR_KD            0.0f      // D항 제거 (레이더 노이즈 증폭 방지, P제어만 사용)
#define MOTOR_PAN_DEADBAND  35        // 픽셀
#define MOTOR_TILT_DEADBAND 35        // 픽셀
#define MOTOR_PAN_MIN_STEP  5
#define MOTOR_PAN_MAX_STEP  58
#define MOTOR_TILT_MIN_STEP 5
#define MOTOR_TILT_MAX_STEP 58
#define MOTOR_CMD_RAMP      20.0f
#define MOTOR_SPEED_DELAY   200       // 0.57ms/step → 154°/s
#define MOTOR_ACQUIRE_RAMP  1.0f      // ★ :121에서 0.6f로 정의되나 :128에서 #undef → :129에서 1.0f 재정의.
                                      //   모터 함수가 사용하는 유효값은 1.0f (가속 억제 없음)
#define MOTOR_DT            0.033f    // 30Hz
#define MAIN_LOOP_SLEEP_US  33000u    // 33ms
```

> ⚠️ 구버전 문서가 기재했던 `MOTOR_KP=2.0`, `MOTOR_KD=0.02`는 **코드와 불일치(stale)**.
> 현재 코드는 `KP=1.5`, `KD=0.0`이며 진동 억제는 KD가 아니라 **rang LP필터(수정 5)**가 담당한다.

---

## 현재 최종 제어 구조 (`ps_main.cpp:944~973`)

메인 루프(30Hz)는 아래 우선순위로 분기한다. **3-모드 + AI 잠금(lock/cooldown) 계층**이 핵심.

```
메인 루프 (30Hz)
    │
    ├─ [1] g_host_pan_steps != 0 && cooldown==0
    │       → motor_update_full_host()   PC YOLO가 Pan+Tilt 직접 스텝
    │         이후 g_host_cmd_cooldown, g_ai_lock_frames 설정 (레이더 오버라이드 차단 시작)
    │
    ├─ [2] g_host_cmd_cooldown > 0
    │       → 모터 이동 완료 대기 (레이더/퓨전 개입 없음, cooldown 감소)
    │
    ├─ [3] g_ai_lock_frames > 0
    │       → AI 잠금 유지 (현재 위치 유지, 다음 PC 명령 도착 전 레이더가 끼어들지 못하게 차단)
    │
    ├─ [4] fi >= 0 (레이더+카메라 퓨전)
    │       → motor_update(angle_to_px(rang) - CAM_W_PX/2,  CAM_H_PX/2 - cy,  true)
    │         Pan : 레이더 방위각(rang) → 픽셀 오차 PID
    │         Tilt: 카메라 BBox cy PID
    │
    ├─ [5] rvc > 0 (레이더 단독)
    │       → motor_update_hybrid(angle_to_px(rang) - CAM_W_PX/2, true, g_host_tilt_steps)
    │         Pan : 레이더 방위각 PID
    │         Tilt: PC T: 명령 직접 스텝
    │
    └─ [6] 표적 없음 → 정지 (g_had_target=false, 마지막 위치 유지)

매 프레임 끝: g_host_pan_steps = g_host_tilt_steps = 0  (PC가 30Hz로 재전송해야 유지)
```

> **중요 — Pan은 "증분 PID"이지 "절대 위치 SET"이 아니다.**
> 퓨전/레이더 모드 모두 `angle_to_px(rang)` 픽셀 오차를 `motor_pid_step()`에 넣어
> `g_motor_abs_pan += dpan` 방식으로 누적한다(`motor_update`/`motor_update_hybrid`).
> 레이더 좌표가 카메라 중심(픽셀 160)에 들어오면 오차→0, deadband(35px) 내에서 정지한다.
> 구버전 문서의 "절대 위치 SET(`g_motor_abs_pan = -(rang/10×4096/360)`)" 서술은 **현재 코드에 없음**.

---

## 수정 이력

---

### 수정 1 — AP_IDLE 가드 위치 수정 (체감 속도 저하 해결)

**증상:** 모터가 체감상 점점 느려짐

**원인:** `motor_update*()` 함수들이 IP 실행 중에도 `g_motor_abs_pan/tilt`를 매 프레임 누적 → 연쇄 지연 폭주.

```
프레임 1: IDLE → abs=58  → IP 실행 (33ms, 58스텝)
프레임 2: 실행 중 → abs=116  (AP_START 없음)
프레임 3: IDLE → abs=174  → current=58 → 116스텝 실행 (66ms) ← 지연 폭주
```

**수정:** abs 누적 코드 **이전**에 AP_IDLE 체크 이동 (`ps_main.cpp:700, 742, 778`).

```c
// 변경 전
g_motor_abs_pan += dpan;
if (MOTOR_RD(0x00) & AP_IDLE) { MOTOR_WR(0x00, AP_START); }

// 변경 후
if (!(MOTOR_RD(0x00) & AP_IDLE)) return;   // ← 누적 전에 체크 (IP 실행 중이면 PID slew만 진행)
g_motor_abs_pan += dpan;
MOTOR_WR(0x00, AP_START);
```

---

### 수정 2 — MOTOR_KP 상향 (각속도 개선)

**증상:** 모터가 표적을 못 따라감 (각속도 부족)

**원인:** `KP=0.25`가 너무 작아 정상 오차에서 MAX_STEP의 20~25%만 사용.

```
오차 100px 기준:
  norm     = 100 / 160 = 0.625
  PID 목표 = 0.25 × 0.625 = 0.156
  스텝     = 5 + (53 × 0.156) = 13스텝 → 7.4ms 구동 / 43ms 주기
  각속도   = 154°/s × (7.4/43) ≈ 26°/s  ← 실질 각속도 17%만 활용
```

**수정 이력:**

| 단계 | KP | 오차 100px 스텝 | 평균 각속도 | 비고 |
|---|---|---|---|---|
| 초기 | 0.25 | 13스텝 | 26°/s | 너무 느림 |
| 1차 | 1.5 | 54스텝 | 111°/s | 빨라짐 확인 |
| 시험 | 2.0 | 58스텝(MAX) | 119°/s | 진동 증가로 롤백 |
| **최종** | **1.5** | **54스텝** | **111°/s** | **현재 적용 (`ps_main.cpp:101`)** |

> **발진 불가 이유:** 프레임당 최대 이동 ≈ 58스텝 × (360°/4096) ≈ 5.1° ≈ 27px < deadband(35px)

---

### 수정 3 — 퓨전 모드 Tilt 복구 (카메라 BBox 기반)

**증상:** 카메라 연결 상태에서 Tilt 자동 추적 불가

**원인:** 퓨전 모드에서 Tilt가 PC T: 명령에만 의존.

```c
// 변경 전
motor_update_hybrid(cx - CAM_W_PX/2, true, g_host_tilt_steps);  // Tilt: PC 명령

// 변경 후 (퓨전 초기)
motor_update(cx - CAM_W_PX/2, CAM_H_PX/2 - cy, true);           // Tilt: 카메라 PID
```

---

### 수정 4 — Pan 소스 통일 (카메라 BBox → 레이더 전용)

**증상:** 퓨전 모드에서 Pan이 카메라 BBox cx 기반으로 동작, 두 소스 혼용

**원인:** 퓨전 모드(`fi >= 0`)의 Pan이 카메라 BBox cx 사용 → 레이더와 카메라 혼용.

**수정:** Pan은 항상 레이더 방위각, Tilt만 소스 분기 (`ps_main.cpp:964`).

```c
// 변경 전 (퓨전 모드)
motor_update(cx - CAM_W_PX/2, CAM_H_PX/2 - cy, true);  // Pan: 카메라 BBox

// 변경 후
motor_update(angle_to_px(rang) - CAM_W_PX/2, CAM_H_PX/2 - cy, true);  // Pan: 레이더
```

**최종 Pan 소스 정책:**

| 모드 | Pan 소스 | Tilt 소스 |
|---|---|---|
| AI 트래킹 | PC P: 직접 스텝 | PC T: 직접 스텝 |
| 퓨전 (fi≥0) | **레이더 방위각 PID** | 카메라 BBox cy PID |
| 레이더 단독 | **레이더 방위각 PID** | PC T: 직접 스텝 |

---

### 수정 5 — rang LP 필터 추가 (지터 억제) — `ps_main.cpp:908, 922`

**증상:** Pan 모터 좌우 진동

**원인:** 레이더 좌표(rang)가 프레임 간 노이즈로 진동 → PID가 매 프레임 방향 반전 명령 생성.

**수정:** rang에 단순 저역통과(LP) 필터 적용.

```c
// main() 변수 선언부
static float rang_lp = 0.0f;   // ps_main.cpp:908

// process_radar_target() 호출 직후 (rvc>0일 때만)
rang_lp = 0.4f * (float)rang + 0.6f * rang_lp;  // α=0.4   (ps_main.cpp:922)
rang = (int16_t)rang_lp;
```

- **α=0.4** (현재): 빠른 추적 + 노이즈 억제 절충. 약 2.5프레임(82ms) 지연.
- α를 낮추면(0.2~0.3) 더 부드럽지만 반응 느려짐 / 높이면(0.6) 반응 빠르지만 노이즈 더 통과.

---

### 수정 6 — MOTOR_KD 제거 (좌우 진동 해결) — `ps_main.cpp:102`

**증상:** Pan 좌우 진동 증가

**원인:** `KD=0.02f`가 rang의 프레임 간 변화(노이즈)를 미분하여 방향 반전 유발.

```
프레임1: rang=+50 → KD항=+0.099 → dpan=+17 (우향)
프레임2: rang=-30 → KD항=-0.159 (방향반전!) → dpan=-22 (좌향)
                  └─ 미분항이 방향을 뒤집음
```

**수정:** `#define MOTOR_KD 0.0f` (D항 제거). P 제어만으로 충분한 이유: `MOTOR_CMD_RAMP`(슬루 리미터)가 급격한 명령 변화를 이미 완충.

---

## 속도 공식 참고

```
각속도(°/s) = 360 / (4096 × SPEED_DELAY × 2.825µs)
            ≈ 154°/s  (SPEED_DELAY=200 기준)

최적 MAX_STEP = 프레임 주기(ms) / step당 시간(ms)
             = 33ms / 0.57ms ≈ 58스텝  ← IP 실행 시간 ≈ 프레임 주기
```

| SPEED_DELAY | ms/step | MAX_STEP | 각속도 | 비고 |
|---|---|---|---|---|
| 300 | 0.85ms | 38 | 101°/s | 안전 |
| **200** | **0.57ms** | **58** | **154°/s** | **현재** |
| 170 | 0.48ms | — | — | 탈조 확인됨 |
| 150 | 0.42ms | 78 | 186°/s | 탈조 위험 |

> **탈조 발생 시:** `MOTOR_SPEED_DELAY=300`, `MOTOR_PAN_MAX_STEP=38`으로 복원

---

## 추가 튜닝 참고

| 증상 | 조치 |
|---|---|
| Pan 진동 지속 | LP 필터 α 낮춤 (0.4→0.25), 또는 DEADBAND 확대 (35→50px) |
| Pan 반응 너무 느림 | LP 필터 α 높임 (0.4→0.6), 또는 KP 상향 (1.5→2.0, 단 진동 재확인) |
| 모터 탈조 | SPEED_DELAY=300, MAX_STEP=38 으로 복원 |
| 모터 반응 없음 | 5V 전원 및 GND 공통 연결 확인 |
