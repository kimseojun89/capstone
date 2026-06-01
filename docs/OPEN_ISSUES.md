# 미해결 항목 통합

> 기준: `CLAUDE.md`와 실제 코드. P0 범위는 [P0_PLAN.md](P0_PLAN.md)를 기준으로 별도 추적한다.

우선순위: P1(정확도/안정성) -> P2(개선/이식)

## P1

### P1-1. Kalman HLS DT 반영 상태 확인
- **현상:** `Vitis/kalman_filter.cpp` 소스는 `DT=0.033f`이나, 생성 IP/bitstream/XSA가 이 값을 반영했는지는 별도 확인이 필요하다.
- **다음 액션:** HLS 재합성 여부, Vivado IP upgrade 여부, bitstream/XSA 반영 여부를 확인한다.

### P1-2. 모터 GPIO 핀맵 실보드 검증
- **현상:** XDC에는 `pan_out_0`, `tilt_out_0` 핀 할당이 있으나 실제 ULN2003 배선/회전 방향 실측이 필요하다.
- **위치:** `vivado_project/antidrone.srcs/constrs_1/new/motor_pins.xdc`
- **다음 액션:** `M:1`, `P:+58`, `T:+58`로 PMODA/PMODB 배선과 회전 방향을 확인한다.

### P1-3. UART 256000 bps 실측
- **현상:** FPGA UART0/UART1, 호스트 설정은 모두 256000 bps다.
- **다음 액션:** 사용 USB-UART 어댑터와 케이블 조합에서 256000 bps 통신을 Tera Term 또는 tracker 로그로 확인한다.

## P2

### P2-1. ONNX FP16 / TensorRT 최적화
- **현상:** 현재 FP32 ONNX CUDA 추론을 사용한다.
- **다음 액션:** VRAM/지연 시간이 문제가 될 때 FP16 또는 TensorRT 적용을 검토한다.

### P2-2. Linux 빌드 경로 일반화
- **현상:** `antidrone/cpp/CMakeLists.txt`의 ONNX Runtime 기본 경로가 Windows 예제 경로다.
- **다음 액션:** Linux 빌드 시 `-DONNXRUNTIME_DIR=...` 오버라이드 또는 preset 추가를 검토한다.
