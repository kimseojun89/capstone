# 미해결 항목 통합 (OPEN ISSUES)

> 여러 문서에 흩어져 있던 미해결·확인 필요·향후 작업 항목을 한곳에 모은 문서.
> 출처: [FPGA_workflow.md](FPGA_workflow.md), [motor_porting_guide.md](motor_porting_guide.md),
> [motor_control_changes.md](motor_control_changes.md), [ONNX_CUDA_Migration.md](ONNX_CUDA_Migration.md).
> 최종 정리: 2026-05-29.

우선순위: 🔴 높음(동작/검증 차단) · 🟡 중간(정밀도/안정성) · 🟢 낮음(개선/편의)

---

## A. FPGA / 베어메탈 (ps_main.cpp)

### A1. 🔴 MTI 실카메라 미연결 (Mock 상태)
- **현상:** `USE_MOCK_MTI = 1` — MTI IP에 Mock 프레임만 입력, 실제 영상 검증 불가.
- **위치:** `antidrone/vitis_workspace/antidrone_app/src/ps_main.cpp:30`
- **영향:** MTI BBox 정밀도 검증 불가. 카메라→DDR `CURR_BUF` 경로(DMA 또는 PS 직접 쓰기) 미확정.
- **다음 액션:** 카메라 프레임을 DDR에 쓰는 방식 확정 → `USE_MOCK_MTI 0` 전환 후 검증.

### A2. 🟡 Kalman HLS `DT` 불일치 → 속도 3배 오차
- **현상:** Kalman IP 내부 `DT = 0.1f`(10Hz 가정), 실제 메인 루프는 ~33ms(30fps).
- **위치:** HLS `Vitis/kalman_filter.cpp`의 `DT` / 루프 주기 `ps_main.cpp:1026` (`responsive_sleep_us(MAIN_LOOP_SLEEP_US)`).
- **영향:** 속도 추정치(vx, vy)가 실제보다 약 3배 크게 산출.
- **다음 액션:** HLS `DT`를 `0.033f`로 수정 후 재합성, **또는** 루프 주기를 100ms로 변경해 정합.

### A3. 🟡 MTI 폴링이 메인 루프 블로킹
- **현상:** `while(!(MTI_RD(MTI_CTRL_OFF)&AP_DONE)){...}` 동기 폴링 중 UART 수신 정지.
- **위치:** `ps_main.cpp:827`
- **영향:** MTI 처리 동안 레이더 패킷 유실/링버퍼 오버플로 가능.
- **다음 액션:** 인터럽트 기반(`ap_ctrl_hs` IRQ→GIC) 전환 또는 MTI 처리 전후 UART flush 추가.

### A4. 🟡 모터 GPIO 핀 XDC 제약 미문서화
- **현상:** `uln2003_controller`의 `pan_out[3:0]`/`tilt_out[3:0]`(`ap_none`)이 어느 PYNQ-Z2 핀에 매핑됐는지 `.xdc`에 기록 불명확.
- **참조 핀맵(추정, 검증 필요):** [motor_porting_guide.md](motor_porting_guide.md) §3 — Pan=PMODA(U18/U19/W18/W19), Tilt=PMODB(W14/Y14/T11/T10).
- **다음 액션:** `vivado_project/antidrone.srcs/constrs_1/`에 XDC 존재·핀 할당 확인, 없으면 명시 작성.

### A5. 🟢 MTI IP DDR 접근 방식 — 보류
- **현상:** MTI DDR 접근(AXI Master) 경로 미확정. A1과 연동.
- **다음 액션:** MTI 파트 본격 진행 시 DMA vs PS 직접 쓰기 결정.

### A6. 🟢 IP Repository 경로 재등록 (이전/재설치 시)
- **현상:** HLS Export IP는 Vivado에 절대경로로 등록되며 PC 이전/재설치 시 자동 유지 안 됨.
- **경로:** `Vitis/{cordic_prj,kalman_prj,motor_prj,mti_process_prj}/solution1/impl/ip`
- **다음 액션:** [FPGA_workflow.md](FPGA_workflow.md) §6 절차대로 4개 경로 재등록.

---

## B. 통신 / 인터페이스

### B1. 🟡 UART 바우드레이트 정합 확인
- **현상:** FPGA UART0/UART1 모두 256000 bps 설정. 레이더 모듈·호스트 어댑터가 일치하는지 실측 필요.
- **위치:** `ps_main.cpp:38`(RADAR_UART_BAUD), `ps_main.cpp:39`(CONSOLE_UART_BAUD), 호스트 `antidrone/cpp/include/ptcamera/settings.hpp` (`serialBaud=256000`).
- **주의:** 256000 미지원 USB-UART 존재(CH340 일부). FTDI FT232/CP2102 권장.
- **다음 액션:** Tera Term로 256000 통신 확인 후 시스템 기동.

---

## C. 소프트웨어 / 빌드 (호스트 C++)

### C1. 🟢 ONNX FP16 / TensorRT 최적화
- **현상:** 현재 FP32 ONNX CUDA 추론. FP16/TensorRT 미적용.
- **다음 액션:** `yolo export ... half=True`로 FP16(VRAM 절반), 또는 `onnxruntime_providers_tensorrt.dll`로 `.engine` 변환.

### C2. 🟢 Linux 빌드 시 `ONNXRUNTIME_DIR` 하드코딩
- **현상:** `antidrone/cpp/CMakeLists.txt`의 ONNX Runtime 경로가 Windows 절대경로.
- **다음 액션:** Linux 빌드 시 `-DONNXRUNTIME_DIR=...` 오버라이드 ([ONNX_CUDA_Migration.md](ONNX_CUDA_Migration.md) §6 참고).

### C3. 🟢 빌드 환경변수 자동화 — 부분 해소
- **현상:** Windows 빌드가 매 세션 환경변수 설정 필요했음.
- **해소:** [scripts/build_win.bat](../scripts/build_win.bat)가 vcvars64 + Windows SDK 경로를 자동 설정. FPGA는 [scripts/run_build.bat](../scripts/run_build.bat).
- **잔여:** CMake preset화는 미적용(선택).

---

## D. 문서 동기화 (2026-05-29 정리에서 처리/추적)

### D1. ✅ SYSTEM_OVERVIEW 모터값 stale 정정
- 구: "모터 속도 딜레이 300". 실제 `ps_main.cpp:118` = **200**, `MOTOR_KP=2.0`(`:100`). → SYSTEM_OVERVIEW §8/§9 갱신 완료.

### D2. ✅ 3-모드 제어표 정합
- SYSTEM_OVERVIEW §5를 [motor_control_changes.md](motor_control_changes.md) 최종표(Pan=안테나 방위각 절대위치 전용, Tilt만 분기)와 일치시킴.

### D3. ✅ 뷰어 일원화
- 런타임 뷰어 = `antidrone/unified_gui.py`. `ppi_viewer.py`/`ppi_original.py`는 [legacy/](../legacy/)로 격리. INTEGRATION §7 주석 추가.

---

## 처리 우선순위 제안
1. **B1**(baud 실측) → **A1/A4**(카메라·핀 확정) : 실동작 검증의 전제.
2. **A2/A3** : 추적 정밀도·안정성.
3. **C1~C3** : 성능/편의 개선(선택).
