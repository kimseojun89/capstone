> **상태: 구현 완료 (2026-06-01)**. 이 문서는 설계 단계의 계획서.
> 실제 구현 이력은 [motor_control_changes.md](motor_control_changes.md) §수정7~9 참고.

# 모터 제어 개선: 1도 정밀 제어 + 직관적 API

## 현재 문제 분석 (캘리브레이션 스크립트 참조)

[capture_pantilt_motor_dataset.py](file:///c:/Users/kimse/capstone/scripts/pantilt_calibration/capture_pantilt_motor_dataset.py)를 분석한 결과, **이미 각도 기반 제어가 Python 레벨에서 구현**되어 있었습니다:

```python
# MotorState 클래스 (L226-258)
self.steps_per_degree = 4096.0 / 360.0  # 11.378
def move_pan(self, delta_deg):
    self.pan_deg += delta_deg
    target = round(self.pan_deg * self.steps_per_degree)
    delta = int(target - self.pan_steps)  # 반올림 오차 보정
    return delta
```

그러나 **비직관적인 해킹**이 필수적으로 사용되고 있습니다:

```python
# L460-461: Tilt 단독 이동 시 → P:+1 트리거 필요!
motor.send_pan(+1)    # ← 이게 없으면 Tilt가 무시됨
motor.send_tilt(delta_steps)
```

> [!CAUTION]
> **근본 원인:** FPGA `ps_main.cpp`에서 `g_host_pan_steps != 0`일 때만 `motor_update_full_host()`에 진입합니다 ([L782](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp#L782)). Pan이 0이면 **Tilt 명령도 함께 무시**됩니다. 이것이 캘리브레이션 스크립트에서 `P:+1` 더미 명령을 보내는 이유입니다.

---

## "PID를 FPGA에 통합" — 잘못된 부분 분석

> [!WARNING]
> **PID를 FPGA(PS)에 통합하는 것은 문제가 있습니다.** 이유:

### 1. PC의 YOLO→PID 파이프라인이 깨짐
현재 PID가 PC(`control.cpp`)에 있는 이유:
- **카메라 프레임**에서 드론 bbox 중심점 → 화면 중심 오차(px) → PID → 스텝
- YOLO 탐지 결과(bbox)는 PC에만 존재
- FPGA로 PID를 옮기면 **bbox 좌표를 UART로 전송** → PID 계산 → 스텝 실행 순서가 되어, 현재보다 **지연이 증가**

### 2. 카메라 정보가 FPGA에 없음
PID 계산에 필요한 정보:
- `frameSize` (1920×1080) — 화면 중심 기준 오차 계산
- 카메라 FOV — 정규화
- 실시간 bbox center — YOLO 결과

이 모든 것이 **PC에만** 존재합니다.

### 3. 레이더 모드 PID는 이미 FPGA에 있음
[ps_main.cpp L585-627](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp#L585-L627)의 `motor_pid_step()`은 **레이더 방위각** 기반 PID입니다. 이것은 FPGA에 적절히 위치해 있습니다.

### 올바른 방향

| 모드 | PID 위치 | 이유 |
|---|---|---|
| AI 추적 (YOLO) | **PC** `control.cpp` | bbox가 PC에만 존재 |
| 레이더 단독 | **FPGA** `ps_main.cpp` | 레이더 데이터가 FPGA에만 존재 |
| 수동 조작 (캘리브레이션) | **PID 불필요** | 직접 각도 명령 |

**결론:** PID를 한쪽으로 통일하는 것은 불가. 대신 **프로토콜과 상태머신을 개선**하여 직관성을 높이는 것이 올바른 방향입니다.

---

## Proposed Changes

### 핵심 목표
1. **1도 정밀 제어** — `P:+1` 트리거 해킹 제거
2. **직관적 프로토콜** — 각도 단위 명령 추가
3. **수동 모드** — 캘리브레이션용 직접 이동 (PID 우회)

---

### 1. FPGA 상태머신 개선 — Tilt 단독 이동 허용

#### [MODIFY] [ps_main.cpp](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp)

**현재 문제:** `g_host_pan_steps != 0` 조건이 진입 게이트

```diff
-  if (g_host_pan_steps != 0 && g_host_cmd_cooldown == 0) {
+  if ((g_host_pan_steps != 0 || g_host_tilt_steps != 0) && g_host_cmd_cooldown == 0) {
```

이 한 줄로 **P:+1 트리거 해킹이 불필요**해지고, Tilt 단독 이동이 정상 작동합니다.

---

### 2. 수동 모드 명령 추가 (FPGA PS)

#### [MODIFY] [ps_main.cpp](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp)

새 프로토콜 명령 추가:
```
M:1\n    → 수동 모드 진입 (PID 우회, minStep 제한 없음)
M:0\n    → 자동 모드 복귀 (기존 동작)
```

```cpp
// host_parse_commands()에 추가
static bool g_manual_mode = false;

if (cmd == 'M' && host_ring_at(g_host_tail + 1) == ':') {
    // M:0 또는 M:1 파싱
    if (ok) g_manual_mode = (val != 0);
}
```

수동 모드에서의 `motor_update_full_host` 변경:
```cpp
static void motor_update_full_host(int direct_pan_steps, int direct_tilt_steps)
{
    int dpan = direct_pan_steps;
    int dtilt = direct_tilt_steps;

    if (!g_manual_mode) {
        // 자동 모드: 기존 minStep/maxStep 클램프 유지
        if (dpan >  MOTOR_PAN_MAX_STEP)  dpan =  MOTOR_PAN_MAX_STEP;
        if (dpan < -MOTOR_PAN_MAX_STEP)  dpan = -MOTOR_PAN_MAX_STEP;
        if (abs(dpan) < MOTOR_PAN_MIN_STEP) dpan = 0;
        // ... tilt 동일
    } else {
        // 수동 모드: maxStep만 클램프 (minStep 제한 없음)
        // → 1스텝(0.011°)부터 자유롭게 이동 가능
        if (dpan >  MOTOR_PAN_MAX_STEP)  dpan =  MOTOR_PAN_MAX_STEP;
        if (dpan < -MOTOR_PAN_MAX_STEP)  dpan = -MOTOR_PAN_MAX_STEP;
        if (dtilt >  MOTOR_TILT_MAX_STEP)  dtilt =  MOTOR_TILT_MAX_STEP;
        if (dtilt < -MOTOR_TILT_MAX_STEP)  dtilt = -MOTOR_TILT_MAX_STEP;
    }
    // ... 나머지 동일
}
```

---

### 3. C++ PC 측 각도 유틸리티 추가

#### [MODIFY] [control.hpp](file:///c:/Users/kimse/capstone/antidrone/cpp/include/ptcamera/control.hpp)

```cpp
// 각도 ↔ 스텝 변환 상수 (캘리브레이션 스크립트와 일치)
constexpr double STEPS_PER_DEGREE = 4096.0 / 360.0;  // 11.378

// 유틸리티 함수
static int degreesToSteps(double degrees);
static double stepsToDegrees(int steps);
```

#### [MODIFY] [control.cpp](file:///c:/Users/kimse/capstone/antidrone/cpp/src/control.cpp)

```cpp
int ControlLoop::degreesToSteps(double degrees) {
    return static_cast<int>(std::round(degrees * STEPS_PER_DEGREE));
}
double ControlLoop::stepsToDegrees(int steps) {
    return steps / STEPS_PER_DEGREE;
}
```

---

### 4. serial_port에 수동 모드 + 각도 API 추가

#### [MODIFY] [serial_port.hpp](file:///c:/Users/kimse/capstone/antidrone/cpp/include/ptcamera/serial_port.hpp)

```cpp
bool sendManualMode(bool enable, std::string* error = nullptr);
bool sendPanDegrees(double degrees, std::string* error = nullptr);
bool sendTiltDegrees(double degrees, std::string* error = nullptr);
```

#### [MODIFY] [serial_port.cpp](file:///c:/Users/kimse/capstone/antidrone/cpp/src/serial_port.cpp)

```cpp
bool SerialPort::sendManualMode(bool enable, std::string* error) {
    return writeLine(enable ? "M:1\n" : "M:0\n", error);
}

bool SerialPort::sendPanDegrees(double degrees, std::string* error) {
    int steps = static_cast<int>(std::round(degrees * 4096.0 / 360.0));
    return sendPanCommand(steps, error);
}
```

---

### 5. ptcamera_tracker에 키보드 수동 조작 추가

#### [MODIFY] [ptcamera_tracker.cpp](file:///c:/Users/kimse/capstone/antidrone/cpp/apps/ptcamera_tracker.cpp)

```cpp
// cv::waitKey 분기에 추가
if (key == 'm' || key == 'M') {
    manualMode = !manualMode;
    serial.sendManualMode(manualMode);
    if (manualMode) motorEnabled = false;  // 자동 추적 비활성화
    std::cout << "Manual mode " << (manualMode ? "ON" : "OFF") << '\n';
}

// 수동 모드에서만 동작
if (manualMode) {
    constexpr double DEG_STEP = 1.0;
    constexpr double FINE_STEP = 0.1;
    double step = (key & 0x10000) ? FINE_STEP : DEG_STEP;  // Shift 감지

    if (key == 0x250000 || key == 2424832)  // Left arrow
        serial.sendPanDegrees(-step);
    if (key == 0x270000 || key == 2555904)  // Right arrow
        serial.sendPanDegrees(+step);
    if (key == 0x260000 || key == 2490368)  // Up arrow
        serial.sendTiltDegrees(+step);
    if (key == 0x280000 || key == 2621440)  // Down arrow
        serial.sendTiltDegrees(-step);
}
```

---

### 6. 캘리브레이션 스크립트 개선

#### [MODIFY] [capture_pantilt_motor_dataset.py](file:///c:/Users/kimse/capstone/scripts/pantilt_calibration/capture_pantilt_motor_dataset.py)

`P:+1` 트리거 해킹 제거:
```diff
  elif key in KEY_UP or key in (ord("w"), ord("W")):
      delta_steps = state.move_tilt(+state.angle_step_deg)
-     # P:+1\n 먼저 → g_host_pan_steps=1 로 트리거 (1 step < MIN_STEP=5, pan 이동 없음)
-     motor.send_pan(+1)
      motor.send_tilt(delta_steps)
```

수동 모드 진입 추가 (startup):
```diff
+ # 수동 모드 진입 (minStep 제한 해제)
+ if serial_ok:
+     motor.send_manual_mode(True)
```

---

## 변경 요약

| 변경 | 파일 | 효과 |
|---|---|---|
| 상태머신 조건 수정 | ps_main.cpp L782 | Tilt 단독 이동 가능, `P:+1` 해킹 제거 |
| 수동 모드 `M:` 명령 | ps_main.cpp | minStep 제한 없는 정밀 이동 (캘리브레이션) |
| 각도 유틸리티 | control.hpp/cpp | `degreesToSteps(1.0) → 11` |
| 수동 모드 API | serial_port.hpp/cpp | `sendManualMode()`, `sendPanDegrees()` |
| 키보드 조작 | ptcamera_tracker.cpp | 방향키로 1도/0.1도 수동 이동 |
| 해킹 제거 | capture_pantilt_motor_dataset.py | `P:+1` 트리거 불필요 |

---

## Verification Plan

### Automated Tests
1. `degreesToSteps(1.0) == 11`, `degreesToSteps(360.0) == 4096` 확인
2. C++ 빌드 성공 확인
3. FPGA PS 빌드 (Vitis) 성공 확인

### Manual Verification
1. **수동 모드 테스트**: `M:1\n` → `P:+11\n` → 실측 1도 이동 확인
2. **Tilt 단독**: `T:+11\n` (P 없이) → tilt 1도 이동 확인
3. **캘리브레이션 스크립트**: `P:+1` 제거 후 tilt 단독 이동 정상 확인
4. **자동 추적 복귀**: `M:0\n` 후 YOLO 추적 정상 동작 확인
