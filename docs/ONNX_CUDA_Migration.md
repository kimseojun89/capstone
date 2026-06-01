# AI 모델 변경 시 현재 환경 적용 및 검증 가이드

> 기준 환경: Windows 11 / NVIDIA RTX 3050 Laptop 4GB / ONNX Runtime GPU / CUDA 12 계열  
> 목적: 모델을 바꿀 때 현재 C++ 실행 환경에서 바로 로드되고, CUDA 또는 CPU로 정상 추론되는지 확인한다.

---

## 1. 이 문서의 목적

이 문서는 현재 프로젝트에서 AI 모델을 교체하거나 재학습 모델을 반영할 때 필요한 파일 배치, 빌드, 실행, 검증 절차를 정리한다.

핵심 확인 항목은 다음과 같다.

| 항목 | 확인 내용 |
|------|-----------|
| 모델 형식 | ONNX 모델 파일을 사용한다. 기본 파일명은 `best.onnx`이다. |
| 기본 경로 | `antidrone/models/drone_yolov8x/best.onnx` |
| 추론 런타임 | ONNX Runtime C++ |
| 기본 디바이스 | CUDA, 실패 시 CPU fallback |
| 실행 검증 | `detector_viewer.exe`, `tracking_viewer.exe`, `ptcamera_tracker.exe` |

---

## 2. 현재 모델 적용 구조

```text
capstone/
├─ antidrone/
│  ├─ models/
│  │  └─ drone_yolov8x/
│  │     ├─ best.pt      # 학습 원본 또는 재변환용 PyTorch 가중치
│  │     └─ best.onnx    # C++ 앱이 기본으로 읽는 ONNX 모델
│  └─ cpp/
│     ├─ CMakeLists.txt
│     ├─ include/ptcamera/
│     │  ├─ detector.hpp
│     │  └─ settings.hpp
│     ├─ src/
│     │  ├─ detector.cpp
│     │  └─ settings.cpp
│     └─ build_win/
│        ├─ detector_viewer.exe
│        ├─ tracking_viewer.exe
│        ├─ ptcamera_tracker.exe
│        ├─ onnxruntime.dll
│        ├─ onnxruntime_providers_cuda.dll
│        ├─ opencv_world4100.dll
│        └─ CUDA/cuDNN 관련 DLL
└─ onnxruntime-gpu/
   └─ onnxruntime-win-x64-gpu-1.20.1/
```

`settings.cpp`의 기본 모델 경로는 다음 위치를 가리킨다.

```cpp
antidrone/models/drone_yolov8x/best.onnx
```

다른 모델을 테스트할 때는 기존 파일을 덮어쓰기 전에 `--model` 옵션으로 먼저 검증한다.

---

## 3. 새 모델 준비

### 3.1 ONNX로 변환

YOLO 계열 모델은 현재 C++ 후처리 코드와 출력 형태가 맞아야 한다. 현재 코드는 일반적인 YOLO 출력인 `[1, 5, 8400]` 또는 `[1, 8400, 5]` 형태를 처리한다.

예시 변환:

```python
from ultralytics import YOLO

model = YOLO("best.pt")
model.export(
    format="onnx",
    imgsz=640,
    dynamic=False,
    simplify=True,
    opset=17,
    device="cuda",
)
```

변환 후 확인할 사항:

| 항목 | 권장값 |
|------|--------|
| 입력 크기 | 640x640 |
| 입력 layout | NCHW, float32 |
| 입력 채널 | RGB 기준으로 학습된 YOLO 모델 |
| 출력 | detection head 출력, rank 3 |
| 클래스 수 | 현재 후처리와 설정이 기대하는 클래스 구성과 일치 |

### 3.2 모델 파일 배치

기본 모델로 교체하려면 아래 파일을 교체한다.

```text
antidrone/models/drone_yolov8x/best.onnx
```

먼저 테스트만 하려면 별도 경로에 두고 실행 시 `--model`로 지정한다.

```powershell
.\detector_viewer.exe --model C:\path\to\new_model.onnx
```

디렉터리를 지정하는 경우, 해당 디렉터리 안에 `best.onnx`가 있어야 한다.

```powershell
.\detector_viewer.exe --model C:\path\to\model_dir
```

---

## 4. 빌드 환경 확인

현재 C++ 빌드는 ONNX Runtime GPU 패키지를 사용한다.

| 항목 | 현재 기준 |
|------|-----------|
| ONNX Runtime | `onnxruntime-win-x64-gpu-1.20.1` |
| ONNX Runtime 경로 | `C:/Users/kimse/capstone/onnxruntime-gpu/onnxruntime-win-x64-gpu-1.20.1` |
| OpenCV | `C:/opencv/build/x64/vc16/lib` |
| 빌드 출력 | `antidrone/cpp/build_win` |

빌드 명령:

```powershell
$sdkVer  = "10.0.26100.0"
$sdkBase = "C:\Program Files (x86)\Windows Kits\10"
$vsBase  = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools"
$msvcVer = "14.44.35207"

$env:PATH    = "$vsBase\VC\Tools\MSVC\$msvcVer\bin\Hostx64\x64;$sdkBase\bin\$sdkVer\x64;C:\Program Files\CMake\bin;" + $env:PATH
$env:INCLUDE = "$vsBase\VC\Tools\MSVC\$msvcVer\include;$sdkBase\Include\$sdkVer\ucrt;$sdkBase\Include\$sdkVer\um;$sdkBase\Include\$sdkVer\shared"
$env:LIB     = "$vsBase\VC\Tools\MSVC\$msvcVer\lib\x64;$sdkBase\Lib\$sdkVer\ucrt\x64;$sdkBase\Lib\$sdkVer\um\x64"

cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release `
    "-DOpenCV_DIR=C:/opencv/build/x64/vc16/lib" `
    "-DONNXRUNTIME_DIR=C:/Users/kimse/capstone/onnxruntime-gpu/onnxruntime-win-x64-gpu-1.20.1" `
    -S "C:/Users/kimse/capstone/antidrone/cpp" `
    -B "C:/Users/kimse/capstone/antidrone/cpp/build_win"

cmake --build "C:/Users/kimse/capstone/antidrone/cpp/build_win" --config Release
```

빌드 후 실행 폴더에 다음 DLL이 있어야 한다.

```text
onnxruntime.dll
onnxruntime_providers_cuda.dll
onnxruntime_providers_shared.dll
opencv_world4100.dll
cudart64_12.dll
cublas64_12.dll
cublasLt64_12.dll
cudnn64_9.dll
cudnn_ops64_9.dll
cudnn_graph64_9.dll
cudnn_cnn64_9.dll
cudnn_adv64_9.dll
cudnn_heuristic64_9.dll
cudnn_engines_precompiled64_9.dll
cudnn_engines_runtime_compiled64_9.dll
cufft64_11.dll
cufftw64_11.dll
```

---

## 5. 실행 검증 절차

실행 위치:

```powershell
cd C:\Users\kimse\capstone\antidrone\cpp\build_win
```

### 5.1 기본 모델 CUDA 검증

```powershell
.\detector_viewer.exe
```

정상 로그 예시:

```text
[YoloDetector] 모델: "...best.onnx"
[YoloDetector] 디바이스: CUDA
[YoloDetector] 입력 크기: 640x640
Inference device: CUDA
```

### 5.2 새 모델 CUDA 검증

```powershell
.\detector_viewer.exe --model C:\path\to\new_model.onnx
```

확인할 것:

| 항목 | 정상 기준 |
|------|-----------|
| 모델 로드 | 예외 없이 실행 |
| 디바이스 | `CUDA`로 출력 |
| 화면 | 카메라 프레임 표시 |
| 검출 | 박스가 튀거나 전체 화면을 덮지 않음 |
| 종료 | `Q` 또는 `ESC`로 정상 종료 |

### 5.3 CPU fallback 검증

CUDA 문제와 모델 자체 문제를 분리하려면 CPU로도 실행한다.

```powershell
.\detector_viewer.exe --model C:\path\to\new_model.onnx --device CPU
```

CPU에서는 정상인데 CUDA에서만 실패하면 DLL, ONNX Runtime CUDA provider, CUDA/cuDNN 호환성을 먼저 확인한다.

### 5.4 추적 앱 검증

검출 앱이 정상일 때 추적 앱을 확인한다.

```powershell
.\tracking_viewer.exe --model C:\path\to\new_model.onnx
```

### 5.5 전체 시스템 검증

모터 제어까지 포함해 확인할 때만 실행한다.

```powershell
.\ptcamera_tracker.exe --model C:\path\to\new_model.onnx --serial-port COM4 --baud 256000 --enable-motor
```

모터를 움직이지 않고 먼저 확인하려면 `--enable-motor`를 빼고 실행한다.

---

## 6. 모델 변경 시 체크리스트

모델을 기본 모델로 반영하기 전에 아래 순서로 확인한다.

1. 새 `.pt` 모델을 ONNX로 변환한다.
2. `detector_viewer.exe --model <새 모델 경로>`로 CUDA 실행을 확인한다.
3. CUDA 실패 시 `--device CPU`로 모델 자체가 정상인지 분리 확인한다.
4. 검출 박스 위치, confidence, class id가 기존 앱 기대와 맞는지 확인한다.
5. `tracking_viewer.exe --model <새 모델 경로>`로 추적 안정성을 확인한다.
6. 필요하면 `ptcamera_tracker.exe`를 모터 비활성 상태로 먼저 확인한다.
7. 모든 검증이 끝난 뒤 `antidrone/models/drone_yolov8x/best.onnx`를 교체한다.
8. 교체 후 옵션 없이 `detector_viewer.exe`를 실행해 기본 경로 로드를 확인한다.

---

## 7. 자주 나는 문제와 확인 위치

| 증상 | 우선 확인 |
|------|-----------|
| `CUDA 프로바이더 초기화 실패` | `onnxruntime_providers_cuda.dll`, CUDA/cuDNN DLL이 실행 폴더에 있는지 확인 |
| CUDA만 실패하고 CPU는 정상 | ONNX Runtime GPU 버전과 CUDA/cuDNN DLL 호환성 확인 |
| 모델 로드 실패 | `--model` 경로, 파일명, ONNX opset, 모델 손상 여부 확인 |
| 출력 shape 오류 | 변환된 ONNX 출력이 현재 후처리의 `[1,5,N]` 또는 `[1,N,5]` 계열인지 확인 |
| 박스 위치가 어긋남 | 입력 크기, letterbox 방식, YOLO export 옵션 확인 |
| 검출이 전혀 안 됨 | confidence threshold, 클래스 구성, 학습 데이터 label 순서 확인 |

---

## 8. 기본 실행 명령 모음

```powershell
cd C:\Users\kimse\capstone\antidrone\cpp\build_win

# 기본 모델, CUDA
.\detector_viewer.exe

# 새 모델, CUDA
.\detector_viewer.exe --model C:\path\to\new_model.onnx

# 새 모델, CPU
.\detector_viewer.exe --model C:\path\to\new_model.onnx --device CPU

# 추적 검증
.\tracking_viewer.exe --model C:\path\to\new_model.onnx

# 전체 시스템, 모터 활성
.\ptcamera_tracker.exe --model C:\path\to\new_model.onnx --serial-port COM4 --baud 256000 --enable-motor
```
