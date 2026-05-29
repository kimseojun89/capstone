# ps_main.cpp 모터 제어 수정 이력

> **대상 파일:** `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp`  
> **작성일:** 2026-05-29

---

## 최종 파라미터 설정

```c
#define MOTOR_KP            2.0f      // 변경: 0.25 → 1.5 → 2.0
#define MOTOR_KD            0.02f
#define MOTOR_PAN_DEADBAND  35        // 픽셀
#define MOTOR_TILT_DEADBAND 35        // 픽셀
#define MOTOR_PAN_MIN_STEP  5
#define MOTOR_PAN_MAX_STEP  58
#define MOTOR_TILT_MIN_STEP 5
#define MOTOR_TILT_MAX_STEP 58
#define MOTOR_CMD_RAMP      20.0f
#define MOTOR_SPEED_DELAY   200       // 0.57ms/step → 154°/s
```

---

## 수정 1 — AP_IDLE 가드 위치 수정

### 문제
`motor_update_full_host()` / `motor_update_hybrid()` 두 함수에서  
IP가 실행 중일 때도 `g_motor_abs_pan/tilt`를 매 프레임 누적 → 연쇄 지연 발생.

**폭주 메커니즘:**
| 프레임 | 상태 | abs 누적 | IP 실행 스텝 |
|---|---|---|---|
| 1 | IDLE | 58 | 58스텝 → 33ms |
| 2 | 실행 중 | 116 | (AP_START 없음) |
| 3 | IDLE | 174 | current=58 → **116스텝 → 66ms** |
| 5 | IDLE | 290 | current=174 → 116스텝 → ... |

IP가 길어질수록 누적이 더 쌓여 더 느려지는 양성 피드백.

### 수정 내용
두 함수 모두: abs 누적 코드 **이전**에 AP_IDLE 체크 이동.

```c
// 변경 전 (motor_update_full_host / motor_update_hybrid 공통)
g_motor_abs_pan  += dpan;
g_motor_abs_tilt += dtilt;
MOTOR_WR(...);
if (MOTOR_RD(0x00) & AP_IDLE) { MOTOR_WR(0x00, AP_START); }

// 변경 후
if (!(MOTOR_RD(0x00) & AP_IDLE)) return;   // ← 누적 전에 체크
g_motor_abs_pan  += dpan;
g_motor_abs_tilt += dtilt;
MOTOR_WR(...);
MOTOR_WR(0x00, AP_START);
```

> **원칙:** IP 실행 중에는 PID slew 상태만 진행, abs 위치는 IP 완료 후 1프레임치만 적용.

---

## 수정 2 — MOTOR_KP 상향으로 각속도 개선

### 문제
`MOTOR_KP = 0.25f`가 너무 작아 오차 100px 기준 스텝 13개만 명령 → 평균 각속도 26°/s.

**계산 근거:**
```
norm     = 100px / 160px = 0.625
PID 목표 = KP × norm = 0.25 × 0.625 = 0.156
스텝     = 5 + (53 × 0.156) = 13스텝 → 7.4ms 구동 / 43ms 주기
각속도   = 154°/s × (7.4/43) ≈ 26°/s
```

**MAX_STEP 발진 불가 이유:** 프레임당 최대 이동 ≈ 58스텝 × (360°/4096) ≈ 5.1° = 카메라 기준 **~27px** < deadband(35px) → 오버슈트가 deadband 안에 수렴.

### 수정 이력

| 단계 | KP | 오차 100px 스텝 | 평균 각속도 |
|---|---|---|---|
| 초기 | 0.25 | 13스텝 | 26°/s |
| 1차 | 1.5 | 54스텝 | 111°/s |
| **최종** | **2.0** | **58스텝(MAX)** | **119°/s** |

`KP = 2.0`에서 오차 ≥ 80px이면 항상 MAX_STEP 58 풀파워.

---

## 수정 3 — 퓨전 모드 Tilt 복구 (카메라 BBox 기반)

### 문제
3-모드 시스템 도입 이후 퓨전 모드의 Tilt가 PC T: 명령에만 의존 →  
카메라가 연결되어 있어도 Tilt 자동 추적 불가.

### 수정 내용
퓨전 모드(`fi >= 0`)에서 `motor_update_hybrid()` 대신 `motor_update()` 사용,  
카메라 BBox `cy`로 Tilt PID 계산.

```c
// 변경 전
} else if (fi >= 0) {
    int cx = bbox[fi].x + bbox[fi].w / 2;
    motor_update_hybrid(cx - CAM_W_PX / 2, true, g_host_tilt_steps);  // Tilt: PC 명령

// 변경 후
} else if (fi >= 0) {
    int cx = bbox[fi].x + bbox[fi].w / 2;
    int cy = bbox[fi].y + bbox[fi].h / 2;
    motor_update(cx - CAM_W_PX / 2, CAM_H_PX / 2 - cy, true);        // Tilt: 카메라 PID
```

---

## 수정 4 — 레이더 Pan: 증분 → 절대 위치 명령 (CCW 무한 회전 수정)

### 문제
레이더가 고정된 상태에서 증분 명령(`g_motor_abs_pan += dpan`)을 사용하면  
카메라가 팬해도 레이더 각도(`rang`)가 변하지 않아 오차가 줄지 않음 → 무한 회전.

**발산 메커니즘:**
```
rang = 5° (고정)
→ error = angle_to_px(50) - 160 = 26px (매 프레임 동일)
→ dpan = +22 (매 프레임 동일)
→ g_motor_abs_pan += 22 매 프레임 → 무한 누적 → 무한 회전
```

추가로 부호 반전 문제: 레이더 양수 각도 = 좌향, 모터 양수 = CW = 우향 → 반대 방향.

### 수정 내용
레이더 각도를 절대 모터 위치로 직접 변환 (SET, 누적 아님).

```c
// 변경 전 (증분 PID — 무한 회전)
motor_update_hybrid(angle_to_px(rang) - CAM_W_PX / 2, true, g_host_tilt_steps);

// 변경 후 (절대 위치 SET — 목표 각도에서 자동 정지)
float target_deg = rang / 10.0f;
g_motor_abs_pan = -(int)(target_deg * (4096.0f / 360.0f));  // 부호 반전 포함
if (g_motor_abs_pan >  1024) g_motor_abs_pan =  1024;       // ±90도 제한
if (g_motor_abs_pan < -1024) g_motor_abs_pan = -1024;
```

**동작 원리:**
- `rang = 0` (정면) → `g_motor_abs_pan = 0` → 홈 위치 유지
- `rang = 300` (30°) → `g_motor_abs_pan = -341` → 341스텝 후 **자동 정지**
- 레이더 각도가 바뀌면 새 절대 목표로 이동 후 정지

---

## 수정 5 — Pan 제어 소스 통일 (안테나 전용)

### 문제
퓨전 모드(`fi >= 0`)에서 Pan이 카메라 BBox cx를 사용 → Pan 소스 혼용.

```
이전: fi >= 0  → Pan: 카메라 BBox cx
      rvc > 0  → Pan: 안테나 방위각
```

### 수정 내용
제어 분기를 `fi`(카메라 퓨전 여부)가 아닌 `rvc`(레이더 신호 여부)로 통일.  
Pan은 항상 안테나, Tilt만 상황에 따라 분기.

```
변경 후:
rvc > 0 (안테나 있음)
  ├─ Pan:  안테나 방위각 절대 위치 (항상)
  └─ Tilt: fi >= 0이면 카메라 BBox cy PID
           fi <  0이면 호스트 T: 명령
```

**최종 메인 루프 모터 제어 분기:**

```c
if (g_host_pan_steps != 0) {
    // AI 모드: PC YOLO가 Pan+Tilt 모두 직접 제어
    motor_update_full_host(g_host_pan_steps, g_host_tilt_steps);

} else if (rvc > 0) {
    // Pan: 안테나 절대 위치 (퓨전/단독 공통)
    float target_deg = rang / 10.0f;
    g_motor_abs_pan = -(int)(target_deg * (4096.0f / 360.0f));
    // clamp ±90도...

    if (fi >= 0) {
        // Tilt: 카메라 BBox cy PID
        int cy = bbox[fi].y + bbox[fi].h / 2;
        int dtilt = motor_pid_step(CAM_H_PX/2 - cy, ...);
        // AP_IDLE 후 g_motor_abs_tilt += dtilt, AP_START
    } else {
        // Tilt: 호스트 T: 명령
        g_motor_abs_tilt += (host_steps 클램프);
        // AP_IDLE 후 AP_START
    }

} else {
    g_had_target = false;  // 표적 없음
}
```

---

## 최종 제어 구조 요약

| 모드 | 조건 | Pan 소스 | Tilt 소스 |
|---|---|---|---|
| AI 트래킹 | `g_host_pan_steps != 0` | PC P: 명령 (직접 스텝) | PC T: 명령 (직접 스텝) |
| 퓨전 | `rvc > 0` & `fi >= 0` | **안테나 방위각 (절대)** | 카메라 BBox cy PID |
| 레이더 단독 | `rvc > 0` & `fi < 0` | **안테나 방위각 (절대)** | PC T: 명령 |
| 표적 없음 | `rvc == 0` | — (마지막 위치 유지) | — |

---

## 속도 공식 참고

```
각속도(°/s) = 360 / (4096 × SPEED_DELAY × 2.825µs)
            = 360 / (4096 × 200 × 2.825µs) ≈ 154°/s   (SPEED_DELAY=200 기준)

최적 MAX_STEP = 프레임 주기(ms) / step당 시간(ms)
             = 33ms / (200 × 2.825µs) = 33ms / 0.565ms ≈ 58스텝  ← 현재
```

| SPEED_DELAY | ms/step | MAX_STEP | 각속도 | 비고 |
|---|---|---|---|---|
| 300 | 0.85ms | 38 | 101°/s | 안전 (탈조 위험 낮음) |
| **200** | **0.57ms** | **58** | **154°/s** | **현재** |
| 150 | 0.42ms | 78 | 186°/s | 탈조 위험 |

> **탈조 발생 시:** `MOTOR_SPEED_DELAY=300`, `MOTOR_PAN_MAX_STEP=38`으로 되돌릴 것.
