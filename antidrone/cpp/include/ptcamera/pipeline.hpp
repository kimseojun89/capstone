#pragma once

#include "ptcamera/detector.hpp"
#include "ptcamera/tracking.hpp"

#include <opencv2/core.hpp>

#include <optional>
#include <vector>

namespace ptcamera {

struct PipelineResult {
    std::vector<Detection> detections;
    std::vector<Track> tracks;
    std::optional<Track> target;
};

class TrackingPipeline {
public:
    explicit TrackingPipeline(const TrackerSettings& settings);

    PipelineResult update(const cv::Mat& frameBgr, double dt);
    void reset();

private:
    TrackerSettings settings_;
    YoloOpenVinoDetector detector_;
    ByteTracker byteTracker_;
    TargetSelector targetSelector_;
    VisualTrackerFallback visualTracker_;
    KalmanTracker2D kalman_;
    int lostFrames_ = 0;
    std::optional<Track> lastTrackingTarget_;
};

}  // namespace ptcamera
