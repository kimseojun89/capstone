> **상태: 구현 완료 (2026-06-01)**. 이 문서는 변경 계획 단계의 walkthrough.
> 실제 구현 이력은 [motor_control_changes.md](motor_control_changes.md) §수정7~9 참고.

# 모터 제어 개선 — Walkthrough

## 변경 목적
1. **1도 정밀 제어** — 캘리브레이션 시 모터를 정확히 1도(≈11 하프스텝)씩 이동
2. **비직관적 프로토콜 해소** — Tilt 단독 이동 불가 문제, `P:+1` 해킹 제거
3. **수동 모드 추가** — minStep 제한 없는 정밀 이동 모드

## 변경된 파일 (8개)

### FPGA
| 파일 | 변경 |
|---|---|
| [ps_main.cpp](file:///c:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp) | 상태머신 `Pan\|\|Tilt` 조건, `M:` 명령 파서, `g_manual_mode`, minStep 분기 |

### C++ (PC)
| 파일 | 변경 |
|---|---|
| [control.hpp](file:///c:/Users/kimse/capstone/antidrone/cpp/include/ptcamera/control.hpp) | `STEPS_PER_DEGREE`, `degreesToSteps()`, `stepsToDegrees()` |
| [control.cpp](file:///c:/Users/kimse/capstone/antidrone/cpp/src/control.cpp) | 위 함수 구현 |
| [serial_port.hpp](file:///c:/Users/kimse/capstone/antidrone/cpp/include/ptcamera/serial_port.hpp) | `sendManualMode()`, `sendPanDegrees()`, `sendTiltDegrees()` |
| [serial_port.cpp](file:///c:/Users/kimse/capstone/antidrone/cpp/src/serial_port.cpp) | 위 함수 구현 (Windows + Linux 양쪽) |
| [ptcamera_tracker.cpp](file:///c:/Users/kimse/capstone/antidrone/cpp/apps/ptcamera_tracker.cpp) | `M` 키 수동 모드 토글, 방향키 1도 이동, `--manual-mode` CLI |

### Python
| 파일 | 변경 |
|---|---|
| [capture_pantilt_motor_dataset.py](file:///c:/Users/kimse/capstone/scripts/pantilt_calibration/capture_pantilt_motor_dataset.py) | `P:+1` 해킹 제거, `M:1`/`M:0` 자동 전송, `send_manual_mode()` |

### 문서
| 파일 | 변경 |
|---|---|
| [CLAUDE.md](file:///c:/Users/kimse/capstone/CLAUDE.md) | `M:` 프로토콜 추가, 상태머신 조건 갱신, minStep 수동모드 설명 |
| [motor_control_changes.md](file:///c:/Users/kimse/capstone/docs/motor_control_changes.md) | 수정 7 항목 추가 (전체 변경 상세) |
| [motor_control_deep_dive.md](file:///c:/Users/kimse/capstone/docs/motor_control_deep_dive.md) | 상태머신 다이어그램 갱신, 수동 모드 설명 |

## 핵심 변경 3가지

### 1. Tilt 단독 이동 허용
```diff
- if (g_host_pan_steps != 0 && g_host_cmd_cooldown == 0)
+ if ((g_host_pan_steps != 0 || g_host_tilt_steps != 0) && g_host_cmd_cooldown == 0)
```
→ `T:+11\n` 만으로 Tilt 1도 이동 가능. `P:+1` 트리거 해킹 불필요.

### 2. 수동 모드 (`M:1\n` / `M:0\n`)
- `g_manual_mode = true` → `motor_update_full_host()`에서 minStep 클램프 건너뜀
- 1스텝(0.011°)부터 정밀 이동 가능
- 캘리브레이션 스크립트가 startup/shutdown 시 자동 전환

### 3. 각도 기반 API
```cpp
constexpr double STEPS_PER_DEGREE = 4096.0 / 360.0;  // 11.378
serial.sendPanDegrees(1.0);   // → P:+11\n
serial.sendTiltDegrees(-0.5); // → T:-6\n
```

## 재빌드 필요
| 대상 | 명령 |
|---|---|
| PC C++ | `scripts\build_win.bat` |
| FPGA PS | `scripts\run_build.bat` → `run_system.ps1 -Flash` |
| Python / 문서 | 재빌드 불필요 |
