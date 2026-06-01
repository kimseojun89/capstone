#include "ptcamera/detector.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace ptcamera {

namespace {

std::string trim(const std::string& value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c); });
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    if (first >= last) return {};
    return std::string(first, last);
}

std::string stripQuotes(std::string value) {
    value = trim(value);
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') ||
         (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::vector<std::string> splitLabels(const std::string& labels) {
    std::vector<std::string> output;
    std::string current;
    for (char c : labels) {
        if (c == ',' || c == ';') {
            current = stripQuotes(current);
            if (!current.empty()) output.push_back(current);
            current.clear();
        } else {
            current.push_back(c);
        }
    }
    current = stripQuotes(current);
    if (!current.empty()) output.push_back(current);
    return output;
}

}  // namespace

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
YoloDetector::YoloDetector(const TrackerSettings& settings)
    : settings_(settings),
      env_(ORT_LOGGING_LEVEL_WARNING, "YoloDetector"),
      session_(nullptr) {

    if (settings_.modelPath.empty()) {
        settings_.modelPath = defaultModelPath();
    }

    modelPath_    = resolveModelOnnx(settings_.modelPath);
    metadataPath_ = resolveMetadataPath(settings_.modelPath);

    const ModelMetadata metadata = loadModelMetadata();
    classNames_ = metadata.classNames;
    if (settings_.inputSize <= 0) {
        settings_.inputSize = metadata.inputSize > 0 ? metadata.inputSize : 640;
    }
    resolveTargetClassIds();

    settings_.resolvedDevice = resolveDevice(settings_.inferenceDevice);

    // Session 옵션 구성
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (settings_.resolvedDevice == "CUDA") {
        OrtCUDAProviderOptions cudaOptions{};
        cudaOptions.device_id = 0;
        try {
            sessionOptions.AppendExecutionProvider_CUDA(cudaOptions);
        } catch (const std::exception& e) {
            std::cerr << "[YoloDetector] CUDA 프로바이더 초기화 실패: " << e.what()
                      << " → CPU 폴백\n";
            settings_.resolvedDevice = "CPU";
        }
    }

    // 세션 생성 (CUDA 실패 시 CPU 재시도)
    try {
        session_ = Ort::Session(env_, modelPath_.wstring().c_str(), sessionOptions);
    } catch (const std::exception& error) {
        if (settings_.resolvedDevice == "CPU") throw;
        std::cerr << "[YoloDetector] CUDA 세션 생성 실패: " << error.what()
                  << " → CPU 폴백\n";
        settings_.resolvedDevice = "CPU";
        Ort::SessionOptions cpuOptions;
        cpuOptions.SetIntraOpNumThreads(1);
        cpuOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = Ort::Session(env_, modelPath_.wstring().c_str(), cpuOptions);
    }

    // 입출력 이름 획득
    Ort::AllocatorWithDefaultOptions allocator;
    inputName_  = std::string(session_.GetInputNameAllocated(0, allocator).get());
    outputName_ = std::string(session_.GetOutputNameAllocated(0, allocator).get());

    std::cout << "[YoloDetector] 모델: " << modelPath_ << '\n';
    std::cout << "[YoloDetector] 디바이스: " << settings_.resolvedDevice << '\n';
    std::cout << "[YoloDetector] 입력 크기: " << settings_.inputSize << "x" << settings_.inputSize << '\n';
}

// ---------------------------------------------------------------------------
// detect
// ---------------------------------------------------------------------------
std::vector<Detection> YoloDetector::detect(const cv::Mat& frameBgr) {
    if (frameBgr.empty()) return {};

    LetterboxInfo info;
    const cv::Mat letterboxed = letterbox(frameBgr, info);

    const int size = settings_.inputSize;
    // letterbox된 정사각 영상 → NCHW float32 blob (BGR→RGB, /255 정규화).
    // OpenCV가 SIMD 최적화로 처리 (구 수동 픽셀 루프 makeInputData 대체).
    const cv::Mat blob = cv::dnn::blobFromImage(
        letterboxed, 1.0 / 255.0, cv::Size(size, size), cv::Scalar(),
        /*swapRB=*/true, /*crop=*/false, CV_32F);

    std::array<int64_t, 4> inputShape{1, 3, size, size};
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memInfo,
        const_cast<float*>(blob.ptr<float>()),
        static_cast<size_t>(blob.total()),
        inputShape.data(),
        inputShape.size());

    const char* inputNames[]  = {inputName_.c_str()};
    const char* outputNames[] = {outputName_.c_str()};

    auto outputs = session_.Run(
        Ort::RunOptions{nullptr},
        inputNames, &inputTensor, 1,
        outputNames, 1);

    const float* outputData = outputs[0].GetTensorData<float>();
    const auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    return postprocess(outputData, shape, info);
}

// ---------------------------------------------------------------------------
// letterbox
// ---------------------------------------------------------------------------
cv::Mat YoloDetector::letterbox(const cv::Mat& frameBgr, LetterboxInfo& info) const {
    const int size = settings_.inputSize;
    info.inputSize    = cv::Size(size, size);
    info.originalSize = frameBgr.size();

    const float scale = std::min(size / static_cast<float>(std::max(frameBgr.cols, 1)),
                                 size / static_cast<float>(std::max(frameBgr.rows, 1)));
    const int resizedWidth  = std::max(static_cast<int>(std::round(frameBgr.cols * scale)), 1);
    const int resizedHeight = std::max(static_cast<int>(std::round(frameBgr.rows * scale)), 1);
    info.scale = scale;
    info.padX  = (size - resizedWidth)  / 2;
    info.padY  = (size - resizedHeight) / 2;

    cv::Mat resized;
    cv::resize(frameBgr, resized, cv::Size(resizedWidth, resizedHeight), 0.0, 0.0, cv::INTER_LINEAR);

    cv::Mat output;
    cv::copyMakeBorder(
        resized, output,
        info.padY, size - resizedHeight - info.padY,
        info.padX, size - resizedWidth  - info.padX,
        cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    return output;
}

// ---------------------------------------------------------------------------
// postprocess  (YOLO 출력 디코딩 + NMS)
// ---------------------------------------------------------------------------
std::vector<Detection> YoloDetector::postprocess(
    const float* data,
    const std::vector<int64_t>& shape,
    const LetterboxInfo& info) const {

    if (shape.size() != 3) {
        throw std::runtime_error("Unexpected YOLO output rank. Expected [1,5,8400] or [1,8400,5].");
    }

    const bool channelsFirst = shape[1] <= shape[2];
    const size_t channels = static_cast<size_t>(channelsFirst ? shape[1] : shape[2]);
    const size_t count    = static_cast<size_t>(channelsFirst ? shape[2] : shape[1]);
    if (channels < 5) {
        throw std::runtime_error("Unexpected YOLO output channels (<5).");
    }

    auto valueAt = [&](size_t ch, size_t idx) -> float {
        return channelsFirst ? data[ch * count + idx] : data[idx * channels + ch];
    };

    std::vector<Detection> candidates;
    std::vector<cv::Rect>  nmsBoxes;
    std::vector<float>     scores;
    candidates.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        int   classId    = 0;
        float confidence = valueAt(4, i);

        if (channels > 5) {
            confidence = -1.0F;
            for (size_t ci = 4; ci < channels; ++ci) {
                const float s = valueAt(ci, i);
                if (s > confidence) { confidence = s; classId = static_cast<int>(ci - 4); }
            }
        }

        if (!isTargetClass(classId)) continue;
        if (confidence < settings_.confidenceThreshold) continue;

        const float cx = valueAt(0, i), cy = valueAt(1, i);
        const float w  = valueAt(2, i), h  = valueAt(3, i);

        float x1 = (cx - w * 0.5F - static_cast<float>(info.padX)) / info.scale;
        float y1 = (cy - h * 0.5F - static_cast<float>(info.padY)) / info.scale;
        float x2 = (cx + w * 0.5F - static_cast<float>(info.padX)) / info.scale;
        float y2 = (cy + h * 0.5F - static_cast<float>(info.padY)) / info.scale;

        cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
        box = clampRect(box, info.originalSize);
        if (box.area() < static_cast<float>(settings_.minArea)) continue;

        Detection det;
        det.box        = box;
        det.confidence = confidence;
        det.classId    = classId;
        candidates.push_back(det);
        nmsBoxes.emplace_back(
            static_cast<int>(std::round(box.x)),  static_cast<int>(std::round(box.y)),
            static_cast<int>(std::round(box.width)), static_cast<int>(std::round(box.height)));
        scores.push_back(confidence);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(nmsBoxes, scores,
                      settings_.confidenceThreshold, settings_.nmsIouThreshold, keep);

    std::vector<Detection> detections;
    detections.reserve(keep.size());
    for (int idx : keep) detections.push_back(candidates[static_cast<size_t>(idx)]);
    return detections;
}

// ---------------------------------------------------------------------------
// resolveModelOnnx
// ---------------------------------------------------------------------------
std::filesystem::path YoloDetector::resolveModelOnnx(const std::string& modelPath) const {
    namespace fs = std::filesystem;
    fs::path path(modelPath);

    if (fs::is_directory(path)) {
        if (fs::exists(path / "best.onnx"))             return path / "best.onnx";
        if (fs::exists(path.parent_path() / "best.onnx")) return path.parent_path() / "best.onnx";
    }
    if (path.extension() == ".onnx" && fs::exists(path)) return path;

    // XML/PT 경로가 주어진 경우 동급 ONNX 탐색
    fs::path onnxPath = path.parent_path() / (path.stem().string() + ".onnx");
    if (fs::exists(onnxPath)) return onnxPath;

    throw std::runtime_error(
        "ONNX 모델을 찾을 수 없습니다: " + path.string() +
        "\n  힌트: python -c \"from ultralytics import YOLO; YOLO('best.pt').export(format='onnx')\"");
}

// ---------------------------------------------------------------------------
// resolveMetadataPath
// ---------------------------------------------------------------------------
std::filesystem::path YoloDetector::resolveMetadataPath(const std::string& /*modelPath*/) const {
    namespace fs = std::filesystem;
    const fs::path base = modelPath_.parent_path();

    // 1) ONNX 파일과 같은 디렉터리
    if (fs::exists(base / "metadata.yaml")) return base / "metadata.yaml";
    // 2) 형제 best_openvino_model 디렉터리 (기존 OpenVINO export 산출물)
    if (fs::exists(base / "best_openvino_model" / "metadata.yaml"))
        return base / "best_openvino_model" / "metadata.yaml";

    return base / "metadata.yaml";  // 없어도 loadModelMetadata가 처리
}

// ---------------------------------------------------------------------------
// loadModelMetadata  (metadata.yaml 파싱)
// ---------------------------------------------------------------------------
YoloDetector::ModelMetadata YoloDetector::loadModelMetadata() const {
    ModelMetadata metadata;
    if (!std::filesystem::exists(metadataPath_)) return metadata;

    std::ifstream input(metadataPath_);
    std::string line;
    bool inNames = false;
    bool awaitingImgszList = false;

    while (std::getline(input, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) line = line.substr(0, comment);
        const std::string trimmed = trim(line);
        if (trimmed.empty()) continue;

        if (awaitingImgszList && trimmed.rfind("- ", 0) == 0) {
            try { metadata.inputSize = std::stoi(stripQuotes(trimmed.substr(2))); }
            catch (...) {}
            awaitingImgszList = false;
            continue;
        }

        const auto colon = trimmed.find(':');
        if (colon == std::string::npos) continue;
        const std::string key   = trim(trimmed.substr(0, colon));
        const std::string value = stripQuotes(trimmed.substr(colon + 1));

        if (key == "imgsz") {
            inNames = false;
            if (value.empty()) { awaitingImgszList = true; }
            else { try { metadata.inputSize = std::stoi(value); } catch (...) {} }
            continue;
        }
        if (key == "names") { inNames = true; continue; }
        if (key == "labels" && metadata.classNames.empty()) {
            const auto labels = splitLabels(value);
            for (int i = 0; i < static_cast<int>(labels.size()); ++i)
                metadata.classNames[i] = labels[static_cast<size_t>(i)];
            continue;
        }
        if (inNames) {
            try { metadata.classNames[std::stoi(key)] = value; }
            catch (...) { inNames = false; }
        }
    }
    return metadata;
}

// ---------------------------------------------------------------------------
// resolveDevice
// ---------------------------------------------------------------------------
std::string YoloDetector::resolveDevice(const std::string& requestedDevice) const {
    std::string req = requestedDevice.empty() ? "CUDA" : requestedDevice;
    std::transform(req.begin(), req.end(), req.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (req == "CUDA" || req == "GPU") return "CUDA";
    return "CPU";
}

bool YoloDetector::isTargetClass(int classId) const {
    if (targetClassIds_.empty()) return true;
    return targetClassIds_.count(classId) != 0;
}

void YoloDetector::resolveTargetClassIds() {
    targetClassIds_.clear();
    std::vector<std::string> wanted;
    wanted.reserve(settings_.targetClassNames.size());
    for (const auto& name : settings_.targetClassNames) wanted.push_back(lower(name));

    for (const auto& [id, name] : classNames_) {
        if (std::find(wanted.begin(), wanted.end(), lower(name)) != wanted.end())
            targetClassIds_.insert(id);
    }
}

}  // namespace ptcamera
