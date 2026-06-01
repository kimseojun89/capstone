#pragma once

#include "ptcamera/settings.hpp"
#include "ptcamera/types.hpp"

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ptcamera {

class YoloDetector {
public:
    explicit YoloDetector(const TrackerSettings& settings);

    std::vector<Detection> detect(const cv::Mat& frameBgr);
    cv::Size inputSize() const { return cv::Size(settings_.inputSize, settings_.inputSize); }
    const std::filesystem::path& modelOnnxPath() const { return modelPath_; }
    const std::string& resolvedDevice() const { return settings_.resolvedDevice; }

    // Backward-compat accessors (apps 코드 유지)
    const std::filesystem::path& modelXmlPath() const { return modelPath_; }
    const std::string& resolvedOpenvinoDevice() const { return settings_.resolvedDevice; }

private:
    struct ModelMetadata {
        int inputSize = 0;
        std::map<int, std::string> classNames;
    };

    cv::Mat letterbox(const cv::Mat& frameBgr, LetterboxInfo& info) const;
    std::vector<Detection> postprocess(
        const float* data,
        const std::vector<int64_t>& shape,
        const LetterboxInfo& info) const;
    std::filesystem::path resolveModelOnnx(const std::string& modelPath) const;
    std::filesystem::path resolveMetadataPath(const std::string& modelPath) const;
    ModelMetadata loadModelMetadata() const;
    std::string resolveDevice(const std::string& requestedDevice) const;
    bool isTargetClass(int classId) const;
    void resolveTargetClassIds();

    TrackerSettings settings_;
    std::filesystem::path modelPath_;
    std::filesystem::path metadataPath_;
    std::map<int, std::string> classNames_;
    std::set<int> targetClassIds_;

    Ort::Env env_;
    Ort::Session session_;
    std::string inputName_;
    std::string outputName_;
};

// Backward-compat alias: pipeline.hpp 등 기존 코드가 그대로 컴파일되도록
using YoloOpenVinoDetector = YoloDetector;

}  // namespace ptcamera
