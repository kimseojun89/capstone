# Pan/Tilt Webcam ChArUco Calibration Workflow

이 문서는 pan/tilt 웹캠 이미지의 target pixel `(u, v)`를 antenna 평면 좌표 `(x, y)` mm로 변환하기 위한 Python + OpenCV 워크플로우를 정리한다.

## 위치

스크립트는 다음 하위 폴더에 있다.

```text
scripts/pantilt_calibration/
  generate_charuco_board.py
  capture_intrinsic_images.py
  calibrate_intrinsics_charuco.py
  capture_pantilt_charuco_dataset.py
  capture_pantilt_motor_dataset.py
  estimate_pantilt_pose_table.py
  calibrate_antenna_plane.py
  runtime_localize_point.py
  pantilt_calibration_utils.py
```

## 의존성

OpenCV는 `cv2.aruco`가 포함된 contrib 빌드가 필요하다.

```powershell
pip install opencv-contrib-python numpy pyyaml pillow
pip install pyserial
```

모든 길이 단위는 mm로 통일한다.

## 1. ChArUco 보드 생성

기본 설정은 A4 landscape 페이지 기준 `7x5`, square `30 mm`, marker `22 mm`, `DICT_4X4_50`, `600 dpi`, margin `20 mm`이다.

```powershell
python scripts\pantilt_calibration\generate_charuco_board.py
```

출력:

```text
data/charuco/a4/charuco_a4_7x5_30mm_600dpi.png
data/charuco/a4/charuco_a4_7x5_30mm_600dpi.pdf
data/charuco/a4/charuco_a4_7x5_30mm_600dpi.yaml
```

현재 기본 페이지는 A4 landscape, 즉 `297 mm x 210 mm`이다.

캘리브레이션용 A4 출력은 기본값인 `square 30 mm`, `marker 22 mm`를 사용한다. 보드 자체는 `210 mm x 150 mm`, margin 포함 캔버스는 `250 mm x 190 mm`라서 A4 landscape에 들어간다.

```powershell
python scripts\pantilt_calibration\generate_charuco_board.py
```

## 2. Intrinsic 이미지 캡처

웹캠 preview 창에서 `Space`로 저장하고, `q` 또는 `ESC`로 종료한다.

```powershell
python scripts\pantilt_calibration\capture_intrinsic_images.py --show-detections
```

출력:

```text
data/intrinsics/*.png
data/intrinsics/intrinsic_capture_index.yaml
```

## 3. Intrinsic 캘리브레이션

저장된 ChArUco 이미지에서 corner를 검출하고 `camera_intrinsics.yaml`을 만든다.

```powershell
python scripts\pantilt_calibration\calibrate_intrinsics_charuco.py --debug-dir data\intrinsics_debug
```

출력:

```text
data/camera_intrinsics.yaml
data/intrinsics_debug/*_corners.jpg
```

## 4. Pan/Tilt 데이터셋 캡처

### 4a. 수동 각도 입력 방식 (모터 없이)

```powershell
python scripts\pantilt_calibration\capture_pantilt_charuco_dataset.py --show-detections --require-charuco
```

고정 각도로 한 번에 캡처할 때:

```powershell
python scripts\pantilt_calibration\capture_pantilt_charuco_dataset.py --pan-deg 10.0 --tilt-deg -5.0
```

출력:

```text
data/pantilt/*.png
data/pantilt/*.json
data/pantilt/pantilt_dataset_index.yaml
```

### 4b. 모터 시리얼 제어 방식 (`capture_pantilt_motor_dataset.py`)

FPGA UART0 프로토콜: `P:+N\n` (pan), `T:+N\n` (tilt) — 상대 step, 부호 필수, LF 종단.

```powershell
python scripts\pantilt_calibration\capture_pantilt_motor_dataset.py `
  --show-detections `
  --require-charuco `
  --serial-port COM4
```

| 키 | 동작 |
|----|------|
| `←` / `A` | pan -step |
| `→` / `D` | pan +step |
| `↑` / `W` | tilt +step |
| `↓` / `S` | tilt -step |
| `Space` | 현재 pan/tilt로 샘플 저장 |
| `R` | 현재 위치를 pan=0, tilt=0으로 재정의 |
| `+` / `-` | angle_step_deg ±0.5 deg |
| `Q` / `ESC` | 종료 |

주요 옵션:

```text
--serial-port COM4        FPGA UART 포트 (기본: COM4)
--baud 256000             baud rate (기본: 256000)
--angle-step-deg 1.0      키 1회 이동 각도
--steps-per-degree 11.378 4096 steps / 360 deg (기본)
--settle-sec 0.4          모터 이동 후 대기 시간 (카메라 settle)
--flush-frames 4          이동 후 버퍼 클리어 프레임 수
--require-charuco         ChArUco 검출 안 되면 저장 거부
--min-corners 4           최소 corner 수 (기본: 4)
--no-serial               시리얼 없이 드라이런 (각도만 추적)
```

시리얼 없이 테스트 (드라이런):

```powershell
python scripts\pantilt_calibration\capture_pantilt_motor_dataset.py `
  --no-serial `
  --show-detections
```

초기 상대 이동 (홈 복귀용):

```powershell
python scripts\pantilt_calibration\capture_pantilt_motor_dataset.py `
  --show-detections `
  --require-charuco `
  --init-pan-steps -200 `
  --init-tilt-steps 100 `
  --init-settle-sec 0.8
```

출력:

```text
data/pantilt/pantilt_motor_YYYYMMDD_HHMMSS_NNN.png
data/pantilt/pantilt_motor_YYYYMMDD_HHMMSS_NNN.json
data/pantilt/pantilt_motor_dataset_index.yaml
```

## 5. Pan/Tilt Pose Table 생성

각 pan/tilt 샘플에서 ChArUco pose를 추정한다.

```powershell
python scripts\pantilt_calibration\estimate_pantilt_pose_table.py --debug-dir data\pantilt_pose_debug
```

출력:

```text
data/pantilt_pose_table.yaml
data/pantilt_pose_debug/*_pose.jpg
```

좌표계:

```text
X_camera = R_board_to_camera * X_world + tvec_board_to_camera_mm
X_world  = R_camera_to_world * X_camera + t_camera_in_world_mm
```

여기서 world frame은 ChArUco board frame이다. pose table의 translation 단위는 mm이다.

## 6. Antenna Plane 보정

antenna 평면 위 기준점 3개 이상을 입력한다. 기준점 좌표는 pose table과 같은 world frame, 즉 ChArUco board frame 기준 mm 좌표여야 한다.

```powershell
python scripts\pantilt_calibration\calibrate_antenna_plane.py --points "0,0,0;100,0,0;0,100,0"
```

CSV 파일을 사용할 때:

```powershell
python scripts\pantilt_calibration\calibrate_antenna_plane.py --points-file data\antenna_points.csv
```

출력:

```text
data/antenna_plane.yaml
```

평면 좌표는 다음 식으로 정의된다.

```text
x = dot(P_world - origin_mm, x_axis)
y = dot(P_world - origin_mm, y_axis)
```

## 7. Runtime Pixel Localize

입력 pixel을 camera ray로 변환하고, 선택된 pan/tilt pose를 통해 world ray로 변환한 뒤 antenna plane과 교차시킨다.

```powershell
python scripts\pantilt_calibration\runtime_localize_point.py `
  --image data\pantilt\pantilt_YYYYMMDD_HHMMSS_000.png `
  --pan-deg 10.0 `
  --tilt-deg -5.0 `
  --u 640 `
  --v 360 `
  --overlay data\runtime\localization_overlay.jpg
```

출력:

```text
data/runtime/localization_result.yaml
```

터미널에는 `antenna_x_mm`, `antenna_y_mm`, `world_point_mm`, 선택된 pose sample이 출력된다.

## 검증

문법 검증:

```powershell
python -m py_compile scripts\pantilt_calibration\pantilt_calibration_utils.py `
  scripts\pantilt_calibration\generate_charuco_board.py `
  scripts\pantilt_calibration\capture_intrinsic_images.py `
  scripts\pantilt_calibration\calibrate_intrinsics_charuco.py `
  scripts\pantilt_calibration\capture_pantilt_charuco_dataset.py `
  scripts\pantilt_calibration\estimate_pantilt_pose_table.py `
  scripts\pantilt_calibration\calibrate_antenna_plane.py `
  scripts\pantilt_calibration\runtime_localize_point.py
```

웹캠 없이 가능한 smoke test:

```powershell
python scripts\pantilt_calibration\generate_charuco_board.py --output-dir $env:TEMP\capstone_charuco_smoke --name smoke_charuco
python scripts\pantilt_calibration\calibrate_antenna_plane.py --points "0,0,0;100,0,0;0,100,0;100,100,0" --output $env:TEMP\capstone_charuco_smoke\antenna_plane.yaml
```

검증해야 하는 핵심 산출물:

```text
camera_intrinsics.yaml: camera_matrix, distortion_coefficients, RMS reprojection error
pantilt_pose_table.yaml: pan_deg, tilt_deg, R/t, reprojection_error_px
antenna_plane.yaml: origin_mm, normal, x_axis, y_axis, fit_error
localization_result.yaml: ray, intersection_world_mm, antenna_plane_xy_mm
```

## 주의사항

- ChArUco 보드는 출력 후 실제 크기가 설정값과 같은지 확인한다. 기본 A4 보드는 `30 mm` square이다.
- intrinsic 캘리브레이션 이미지는 보드를 화면 전체에 다양한 위치와 각도로 배치해서 촬영한다.
- pan/tilt pose table은 runtime에서 사용하는 pan/tilt 각도와 같은 기준의 각도를 metadata에 저장해야 한다.
- runtime은 기본적으로 가장 가까운 pose table entry를 사용하며, 기본 허용 오차는 `0.25 deg`이다. 필요하면 `--max-angle-error-deg`로 조정한다.
- antenna 기준점이 같은 world frame이 아니면 pixel-to-plane 결과가 틀어진다.
