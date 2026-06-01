# 미해결 항목 통합

> 기준: `CLAUDE.md`와 실제 코드. P0 실행 계획은 [P0_PLAN.md](P0_PLAN.md)를 본다.

우선순위: P0(동작/검증 차단) · P1(정밀도/안정성) · P2(개선/편의)

## P0

### P0-1. 문서 정합
- **현상:** 일부 문서가 구형 `P:/T:` 직접 스텝 AI 제어와 삭제된 문서명을 참조한다.
- **다음 액션:** [P0_PLAN.md](P0_PLAN.md)의 문서 정합 항목 수행.

### P0-2. GUI 시리얼 점유 제거
- **현상:** `unified_gui.py`가 표시 전용이어야 하지만 시작 시 pyserial로 COM 포트를 열려고 한다.
- **영향:** `ptcamera_tracker.exe`의 COM4 독점 구조와 충돌할 수 있다.
- **다음 액션:** GUI의 시리얼 open/송신 경로를 제거하고 UDP 수신 전용으로 고정.

### P0-3. bbox 전송 주기 정합
- **현상:** FPGA는 매 루프마다 `g_host_bbox_valid`를 리셋하지만, 호스트 기본 `sendInterval=0.066`은 약 15Hz다.
- **다음 액션:** 호스트 기본 전송 간격을 30Hz 기준인 `0.033`으로 맞추고 실동작 확인.

## P1

### P1-1. Kalman HLS DT 반영 상태 확인
- **현상:** `Vitis/kalman_filter.cpp` 소스는 `DT=0.033f`이나, 생성 IP/bitstream/XSA가 이 값을 반영했는지 별도 확인이 필요하다.
- **다음 액션:** HLS 재합성 여부, Vivado IP upgrade 여부, bitstream/XSA 반영 여부를 확인한다.

### P1-2. 모터 GPIO 핀맵 실보드 검증
- **현상:** XDC에는 `pan_out_0`, `tilt_out_0` 핀 할당이 있으나 실제 ULN2003 배선/회전 방향 실측이 필요하다.
- **위치:** `vivado_project/antidrone.srcs/constrs_1/new/motor_pins.xdc`
- **다음 액션:** `M:1`, `P:+58`, `T:+58`로 PMODA/PMODB 배선과 회전 방향 확인.

### P1-3. UART 256000 bps 실측
- **현상:** FPGA UART0/UART1, 호스트 설정이 모두 256000 bps다.
- **다음 액션:** 사용 USB-UART 어댑터와 레이더 모듈에서 256000 bps 통신을 Tera Term 또는 트래커 로그로 확인.

## P2

### P2-1. ONNX FP16 / TensorRT 최적화
- **현상:** 현재 FP32 ONNX CUDA 추론.
- **다음 액션:** VRAM/지연 시간이 문제가 될 때 FP16 또는 TensorRT 적용.

### P2-2. Linux 빌드 경로 일반화
- **현상:** `antidrone/cpp/CMakeLists.txt`의 ONNX Runtime 기본 경로가 Windows 절대경로다.
- **다음 액션:** Linux 빌드 시 `-DONNXRUNTIME_DIR=...` 오버라이드 또는 preset 추가.
