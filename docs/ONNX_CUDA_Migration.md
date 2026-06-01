# Anti-Drone YOLOv8x — OpenVINO → ONNX Runtime CUDA 전환 작업 정리

> 작업일: 2026-05-28  
> 환경: Windows 11 Home 25H2 / AMD Ryzen 5 6600H / **NVIDIA RTX 3050 Laptop 4GB** / CUDA 12.1

---

## 1. 배경 및 목적

| 항목 | 내용 |
|------|------|
| 기존 추론 엔진 | OpenVINO (`ov::Core`) — CPU 전용 기본값 |
| 기존 모델 포맷 | OpenVINO IR (`best.xml` / `best.bin`, 272MB) |
| 문제점 | OpenVINO GPU 플러그인은 Intel GPU 전용 → RTX 3050 CUDA 미활용 |
| 목표 | ONNX Runtime + CUDA 백엔드로 RTX 3050에서 GPU 가속 추론 |

---

## 2. 작업 결과 요약

```
capstone/
├── antidrone/
│   ├── models/drone_yolov8x/
│   │   ├── best.pt                      # 원본 PyTorch 가중치 (130MB)
│   │   ├── best.onnx                    # ★ 신규 변환 (260MB, opset 17)
│   │   └── best_openvino_model/         # 기존 OpenVINO IR (유지)
│   └── cpp/
│       ├── CMakeLists.txt               # ★ 수정: OpenVINO → ONNX Runtime
│       ├── include/ptcamera/
│       │   ├── detector.hpp             # ★ 수정: ov:: → Ort::
│       │   └── settings.hpp             # ★ 수정: openvinoDevice → inferenceDevice
│       ├── src/
│       │   ├── detector.cpp             # ★ 전면 재작성
│       │   ├── settings.cpp             # ★ 수정: 기본 모델 경로
│       │   └── serial_port.cpp          # WinAPI 시리얼 구현
│       ├── apps/
│       │   ├── detector_viewer.cpp      # ★ 수정: 필드명 업데이트
│       │   ├── ptcamera_tracker.cpp     # ★ 수정: 필드명 업데이트
│       │   └── tracking_viewer.cpp      # ★ 수정: 필드명 업데이트
│       └── build_win/                   # ★ 신규: Windows 빌드 출력
│           ├── detector_viewer.exe
│           ├── tracking_viewer.exe
│           ├── ptcamera_tracker.exe
│           ├── onnxruntime.dll
│           ├── onnxruntime_providers_cuda.dll
│           ├── opencv_world4100.dll
│           └── cudart64_12.dll 외 cuDNN DLLs
├── onnxruntime-gpu/
│   └── onnxruntime-win-x64-gpu-1.20.1/ # ONNX Runtime C++ GPU 패키지
└── ONNX_CUDA_Migration.md              # 이 파일
```

---

## 3. 단계별 작업 내용

### Step 1 — ONNX 모델 변환 (Python)

**설치한 패키지**
```
Python 3.11.9 (winget)
torch 2.5.1+cu121
torchvision 0.20.1+cu121
ultralytics 8.4.56
onnx 1.21.0
onnxruntime-gpu 1.26.0
onnxslim 0.1.94
```

**변환 명령**
```python
from ultralytics import YOLO
model = YOLO("best.pt")
model.export(format="onnx", imgsz=640, dynamic=False, simplify=True, opset=17, device="cuda")
# → best.onnx (260MB, [1,3,640,640] → [1,5,8400])
```

---

### Step 2 — ONNX Runtime C++ GPU 패키지

- **버전:** `onnxruntime-win-x64-gpu-1.20.1`
- **다운로드:** https://github.com/microsoft/onnxruntime/releases/tag/v1.20.1
- **설치 경로:** `C:\Users\kimse\capstone\onnxruntime-gpu\onnxruntime-win-x64-gpu-1.20.1\`
- **포함 DLL:**
  - `onnxruntime.dll` — 메인 라이브러리
  - `onnxruntime_providers_cuda.dll` — CUDA 실행 공급자
  - `onnxruntime_providers_shared.dll` — 공유 공급자
  - `onnxruntime_providers_tensorrt.dll` — TensorRT 공급자 (추후 사용 가능)

---

### Step 3 — C++ 코드 수정

#### `settings.hpp`
```cpp
// 변경 전
std::string openvinoDevice = "CPU";
std::string resolvedOpenvinoDevice;

// 변경 후
std::string inferenceDevice = "CUDA";   // "CUDA" or "CPU"
std::string resolvedDevice;
```

#### `detector.hpp`
```cpp
// 변경 전
#include <openvino/openvino.hpp>
class YoloOpenVinoDetector { ... };

// 변경 후
#include <onnxruntime_cxx_api.h>
class YoloDetector {
    Ort::Env env_;
    Ort::Session session_;
    std::string inputName_, outputName_;
};
using YoloOpenVinoDetector = YoloDetector;  // 하위 호환 alias
```

#### `detector.cpp` — 주요 변경 흐름
```cpp
// 초기화: OpenVINO
ov::Core core_;
auto model = core_.read_model(xmlPath);
compiledModel_ = core_.compile_model(model, "CPU");

// 초기화: ONNX Runtime CUDA
OrtCUDAProviderOptions cudaOptions{};
cudaOptions.device_id = 0;
sessionOptions.AppendExecutionProvider_CUDA(cudaOptions);
session_ = Ort::Session(env_, modelPath.wstring().c_str(), sessionOptions);

// 추론: OpenVINO
inferRequest_.set_input_tensor(inputTensor);
inferRequest_.infer();
auto output = inferRequest_.get_output_tensor();

// 추론: ONNX Runtime
auto outputs = session_.Run(Ort::RunOptions{nullptr},
    inputNames, &inputTensor, 1, outputNames, 1);
const float* data = outputs[0].GetTensorData<float>();
```

#### `settings.cpp`
```cpp
// 변경 전
return (path / "models" / "drone_yolov8x" / "best_openvino_model").string();

// 변경 후
return (path / "models" / "drone_yolov8x" / "best.onnx").string();
```

#### `serial_port.cpp` — Windows WinAPI 시리얼 구현

Windows에서는 `CreateFile`, `SetCommState`, `COMMTIMEOUTS`, `ReadFile`, `WriteFile` 기반으로 COM 포트를 직접 연다. `ptcamera_tracker.exe`가 COM4를 독점하고, FPGA 로그를 읽어 GUI로 UDP 릴레이한다.

#### `CMakeLists.txt`
```cmake
# 변경 전
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
target_link_libraries(ptcamera_core PUBLIC ${OpenCV_LIBS} openvino::runtime)

# 변경 후
set(ONNXRUNTIME_DIR "C:/Users/kimse/capstone/onnxruntime-gpu/onnxruntime-win-x64-gpu-1.20.1")
add_library(onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_IMPLIB   "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib"
    IMPORTED_LOCATION "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_DIR}/include")
target_link_libraries(ptcamera_core PUBLIC ${OpenCV_LIBS} onnxruntime)
# + POST_BUILD: ONNX Runtime DLL 자동 복사
```

---

### Step 4 — Windows 빌드 환경 구성

| 도구 | 버전 | 설치 방법 |
|------|------|-----------|
| Python 3.11 | 3.11.9 | `winget install Python.Python.3.11` |
| CMake | 4.3.3 | `winget install Kitware.CMake` |
| Visual Studio Build Tools 2022 | 17.14 | `winget install Microsoft.VisualStudio.2022.BuildTools` |
| Windows SDK | 10.0.26100.0 | 이미 설치됨 |
| OpenCV | 4.10.0 | https://github.com/opencv/opencv/releases (→ `C:\opencv`) |

**빌드 명령** (PowerShell — 매 세션마다 환경 변수 설정 필요)
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

---

### Step 5 — CUDA DLL 수동 복사 (CUDA Toolkit 미설치 대응)

CUDA Toolkit 없이 PyTorch 번들 DLL을 사용:

```powershell
$src = "C:\Users\kimse\AppData\Local\Programs\Python\Python311\Lib\site-packages\torch\lib"
$dst = "C:\Users\kimse\capstone\antidrone\cpp\build_win"

# 복사한 DLL 목록
cudart64_12.dll, cublas64_12.dll, cublasLt64_12.dll
cudnn64_9.dll, cudnn_ops64_9.dll, cudnn_graph64_9.dll
cudnn_cnn64_9.dll, cudnn_adv64_9.dll, cudnn_heuristic64_9.dll
cudnn_engines_precompiled64_9.dll, cudnn_engines_runtime_compiled64_9.dll
cufft64_11.dll, cufftw64_11.dll
```

---

## 4. 실행 방법

### detector_viewer (카메라 + YOLO 드론 탐지)
```bat
cd C:\Users\kimse\capstone\antidrone\cpp\build_win
.\detector_viewer.exe
```

**옵션**
```bat
.\detector_viewer.exe --camera 0          # 카메라 인덱스 (기본 0)
.\detector_viewer.exe --conf 0.3          # 컨피던스 임계값 (기본 0.25)
.\detector_viewer.exe --device CPU        # CPU 강제 (기본 CUDA)
.\detector_viewer.exe --model <경로>      # 모델 경로 직접 지정
```

**종료:** `Q` 또는 `ESC`

### tracking_viewer (카메라 + 추적)
```bat
.\tracking_viewer.exe
```

### ptcamera_tracker (전체 시스템)
```bat
.\ptcamera_tracker.exe --serial-port COM4 --baud 256000 --enable-motor
```

---

## 5. 동작 확인 로그

```
[YoloDetector] 모델: "...\best.onnx"
[YoloDetector] 디바이스: CUDA          ← RTX 3050 CUDA 활성화 확인
[YoloDetector] 입력 크기: 640x640
Model: "...\best.onnx"
Inference device: CUDA
```

---

## 6. Linux 환경에서 재빌드 시 주의사항

CMakeLists.txt의 `ONNXRUNTIME_DIR` 경로가 Windows 절대경로로 하드코딩되어 있음.  
Linux에서 빌드할 경우 아래와 같이 오버라이드:

```bash
# Linux용 ONNX Runtime GPU 다운로드
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-gpu-1.20.1.tgz
tar xf onnxruntime-linux-x64-gpu-1.20.1.tgz

cmake -DONNXRUNTIME_DIR=/path/to/onnxruntime-linux-x64-gpu-1.20.1 \
      -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
      -S cpp -B cpp/build
cmake --build cpp/build
```

---

## 7. 향후 개선 가능 사항

| 항목 | 설명 |
|------|------|
| **FP16 최적화** | `yolo export ... half=True` → VRAM 사용량 절반, 추론 속도 향상 |
| **TensorRT** | `onnxruntime_providers_tensorrt.dll` 이미 포함됨, `.engine` 파일 변환 시 최고 성능 |
| **Windows 시리얼 포트** | `serial_port.cpp`에 WinAPI(`CreateFile`, `SetCommState`) 구현 추가 시 Windows에서도 FPGA 연동 가능 |
