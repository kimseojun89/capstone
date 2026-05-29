#pragma once

#include "ptcamera/control.hpp"
#include "ptcamera/types.hpp"

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace ptcamera {

void drawOverlay(
    cv::Mat& frame,
    const std::optional<Track>& target,
    const ControlTelemetry& telemetry,
    int detectionCount,
    double fps,
    bool motorEnabled,
    const std::vector<std::string>& statusLines);

}  // namespace ptcamera
