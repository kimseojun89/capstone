# legacy/ — 레거시·미사용 격리 보관소

2026-05-29 폴더 재구성에서 **현재 파이프라인이 쓰지 않는** 파일을 한곳에 모았다.
삭제하지 않고 보관하므로 필요 시 되돌릴 수 있다. (git 이력 + 이 폴더)

| 항목 | 격리 사유 | 대체/현재 |
|---|---|---|
| `mti_subsystem/` | FPGA 온보드 MTI 영상감지. PC YOLO와 중복·Mock 상태(죽은 코드). | PC `ptcamera_tracker.exe` YOLO |
| `serial_stepper_test.cpp` | 구 Arduino 4필드 CSV(`pan,tilt,delay,coil`) 시리얼 테스트. FPGA가 더 이상 미파싱. | `P:`/`T:` 프로토콜 |
| `ppi_viewer.py` | 레이더 PPI 단독 뷰어. 통합 GUI로 대체된 백업. | `antidrone/unified_gui.py` |
| `ppi_original.py` | PPI 뷰어 초기 버전. 어디서도 참조 안 됨. | `antidrone/unified_gui.py` |
| `run_ppi_viewer.bat` | `ppi_viewer.py` 실행 배치. 위와 함께 격리. | `antidrone/run_system.ps1` |
| `cpp_README_openvino.md` | 구 `cpp/README.md`. OpenVINO + Arduino PlatformIO 전제(둘 다 미사용). | 루트 `README.md` + `docs/` |
| `model_README_openvino.md` | 구 모델 README. OpenVINO export·`tapo_tracker.py`(없음) 전제. | `docs/ONNX_CUDA_Migration.md` |
| `best_openvino_model/` | OpenVINO IR(best.bin 272MB/best.xml/metadata.yaml). ONNX 전환으로 미사용. | `antidrone/models/drone_yolov8x/best.onnx` |
| `onnxruntime-win-x64-gpu-1.20.1.zip` | ONNX Runtime SDK 압축본. 이미 `onnxruntime-gpu/`로 해제됨. | `onnxruntime-gpu/` (압축 해제본) |
| `check_arm.bat` | arm-gcc 경로 확인용 1회성 진단 스크립트. | — |
| `detector_test_err.txt`, `detector_test_out.txt` | detector 테스트 출력 덤프. | — |
| `hs_err_pid16400.log` | JVM 크래시 로그(Vitis HLS). | — |
| `vitis_hls.log` | Vitis HLS 실행 로그. | — |

## 복구 방법

```powershell
# 예: PPI 단독 뷰어를 다시 antidrone/로 되돌리기
git mv legacy/ppi_viewer.py     antidrone/ppi_viewer.py
git mv legacy/run_ppi_viewer.bat antidrone/run_ppi_viewer.bat
```

> `best_openvino_model/`, `*.zip`은 `.gitignore`로 git 비추적(대용량). 디스크에만 존재하므로
> 불필요하면 폴더째 삭제해도 무방하다(ONNX 파이프라인과 무관).
