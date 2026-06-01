# P0 통합 계획

> 기준: `CLAUDE.md`와 실제 코드. 코드와 문서가 다르면 코드를 우선한다.

## 목표

P0는 실동작을 막거나 디버깅을 혼동시키는 항목만 처리한다. 범위는 문서 정합, COM4 독점 구조 복구, bbox 전송 주기 정합이다.

## P0-1. 문서 정합

### 문제
- 일부 문서가 구형 `P:/T:` 직접 스텝 AI 제어를 설명한다.
- 현재 코드는 PC가 `B:ex,ey`를 보내고 FPGA가 Pan/Tilt PID를 계산한다.
- 삭제된 이전 문서명을 참조하는 링크가 남아 있다.

### 작업
- `docs/README.md`의 문서 목록을 실제 존재 파일 기준으로 갱신한다.
- `docs/SYSTEM_OVERVIEW.md`의 런타임 흐름, 상태머신, 프로토콜을 `B:ex,ey` 기준으로 갱신한다.
- `docs/INTEGRATION.md`는 변경 이력 문서가 아니라 현재 통합 구조 문서로 축약한다.
- `docs/OPEN_ISSUES.md`에서 처리된 항목을 제거하고 남은 이슈만 유지한다.

### 검증 기준
- 완료 표시와 삭제된 문서명 참조가 남지 않는다.
- `docs/` 내 상대 링크가 실제 파일을 가리킨다.

## P0-2. GUI 시리얼 점유 제거

### 문제
- 문서상 `unified_gui.py`는 표시 전용이어야 한다.
- 현재 `unified_gui.py`는 시작 시 pyserial로 COM 포트를 열려고 하므로 `ptcamera_tracker.exe`의 COM4 독점 구조와 충돌할 수 있다.

### 작업
- `unified_gui.py`에서 pyserial import, serial open, `_serial_send()`, `send_pan()`, `send_tilt()` 경로를 제거한다.
- GUI 상태 표시는 UDP 텔레메트리 기반으로만 구성한다.
- 실행 인자에서 COM 포트 파싱을 제거하거나 무시한다.

### 검증 기준
- `unified_gui.py` 실행 중에도 `ptcamera_tracker.exe --serial-port COM4`가 COM4를 열 수 있다.
- `rg "serial|send_pan|send_tilt|MOTOR_BAUD|COM3|COM4" antidrone/unified_gui.py`에서 시리얼 송신/점유 코드가 남지 않는다.
- 보드 없이 `unified_gui.py` + `scripts/gui_udp_sim.py`로 카메라/PPI/텔레메트리 표시가 동작한다.

## P0-3. bbox 전송 주기 정합

### 문제
- FPGA는 매 루프 끝에 `g_host_bbox_valid=false`로 리셋한다.
- 문서 기준은 30Hz 재전송이지만 `settings.hpp`의 `sendInterval=0.066`은 약 15Hz다.

### 작업
- `antidrone/cpp/include/ptcamera/settings.hpp`의 기본 `sendInterval`을 `0.033`으로 맞춘다.
- 주석을 FPGA cooldown 구조와 일치하게 고친다.
- 필요하면 `ptcamera_tracker.cpp`의 `nextSendAllowed` 주석도 갱신한다.

### 검증 기준
- `ptcamera_tracker.exe`가 target lock 중 `B:ex,ey`를 약 30Hz로 송신한다.
- FPGA 로그/모터 반응에서 bbox 명령 공백 때문에 레이더 단독 모드가 불필요하게 끼어드는 현상이 줄어든다.

## 검증 순서

1. `.\scripts\build_win.bat`
2. `.\antidrone\run_system.ps1 -Preflight`
3. 보드 없이 GUI 점검: `unified_gui.py` + `scripts/gui_udp_sim.py`
4. 보드 연결 후 `.\antidrone\run_system.ps1 -SerialPort COM4 -EnableMotor`
5. 카메라 타겟 추적 중 `B:ex,ey` 송신, RADAR UDP 릴레이, GUI 표시, 모터 반응 확인
