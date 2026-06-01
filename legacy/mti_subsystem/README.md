# legacy/mti_subsystem — MTI(영상 모션 감지) 서브시스템 격리

> 2026-06-01 분리. FPGA 베어메탈 `ps_main.cpp`에서 MTI 코드를 제거하고 여기에 보존.

## 왜 레거시인가

- MTI는 PL의 `mti_process` HLS IP로 **프레임 차분 → box blur → dilate → ROI/BBox 추출**을
  수행해 FPGA가 *자체적으로* 카메라에서 움직이는 표적을 찾으려는 경로였다.
- 그러나 실제 시스템에서 드론 탐지는 **PC YOLOv8x**(`ptcamera_tracker.exe`)가 담당하고
  결과를 `P:`/`T:` 시리얼 명령으로 FPGA에 보낸다. MTI는 이 경로와 **기능 중복**.
- 카메라 프레임을 DDR(`CURR_BUF`)에 넣는 경로가 미확정이라 `USE_MOCK_MTI=1`(가짜 프레임),
  `DISABLE_MTI=1`(IP 호출 자체 비활성) 상태로만 존재 → `mti_run()`은 항상 0 반환,
  `fuse()`는 항상 -1 → **모터 퓨전 분기는 도달 불가능한 죽은 코드**였다.

→ 제거로 `ps_main.cpp`가 단순해지고(모터 분기: AI추적 / 레이더단독 / 정지), OPEN_ISSUES
A1/A3/A5가 해소됨. **MTI 제거 후 FPGA Tilt는 PC `T:` 명령으로만 구동**된다.

## 보존 파일

| 파일 | 내용 |
|---|---|
| `ps_main_mti_excerpt.cpp` | 제거된 MTI 코드 원본(레지스터맵·DDR버퍼·`BBox_t`/`MtiParams_t`·`fuse`·`motor_update`·`mti_init`·`mock_frame`·`mti_run`·메인루프 사용법). 컴파일 대상 아님. |

## 하드웨어 측 현황 (소프트웨어만 제거함)

- **Vivado 블록디자인의 `mti_process` IP는 PL에 그대로 잔존**(미사용). 호출하지 않으므로 무해.
  비트스트림 재생성 불필요. 인스턴스: `vivado_project/.../antidrone_bd_mti_process_0_0.xci`.
- HLS 빌드 산출물: `Vitis/mti_process_prj/`(gitignore 대상, 물리 이동 안 함).

## MTI IP 인터페이스 (참조)

| 오프셋 | 의미 |
|---|---|
| `0x00` | ap_ctrl (AP_START/AP_DONE/AP_IDLE) |
| `0x10/0x1C` | prev/curr 프레임 버퍼 주소 (DDR) |
| `0x28/0x30` | width / height (320×240) |
| `0x38/0x3C` | MtiParams 패킹(dthr, blur, mba) |
| `0x44/0x50` | ROI(BBox) / count 출력 버퍼 주소 |

## 부활 방법

1. `ps_main_mti_excerpt.cpp`의 블록들을 `ps_main.cpp`로 복원
   (레지스터맵/버퍼/구조체/`fuse`/`mti_*`/`mock_frame`/`motor_update` + 메인루프 사용부).
2. 컴파일 스위치 복원: `#define USE_MOCK_MTI 0`(실프레임) / `#define DISABLE_MTI 0`.
3. **카메라 → DDR `CURR_BUF` 쓰기 경로 확정**(DMA 또는 PS 직접 쓰기) — 미해결 전제(구 OPEN_ISSUES A5).
4. Vivado IP는 잔존하므로 재합성 불필요(인터페이스 불변 시).
