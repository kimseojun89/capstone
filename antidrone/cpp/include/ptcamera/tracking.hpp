#pragma once

#include "ptcamera/settings.hpp"
#include "ptcamera/types.hpp"

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace ptcamera {

class ByteTracker {
public:
    explicit ByteTracker(const TrackerSettings& settings);

    std::vector<Track> update(const std::vector<Detection>& detections, const cv::Size& frameSize);
    void reset();

private:
    struct InternalTrack {
        int id = -1;
        cv::Rect2f box;
        float confidence = 0.0F;
        int missingFrames = 0;
        bool updated = false;
    };

    std::vector<std::pair<int, int>> matchDetections(
        const std::vector<int>& trackIndices,
        const std::vector<int>& detectionIndices,
        const std::vector<Detection>& detections) const;

    TrackerSettings settings_;
    std::vector<InternalTrack> tracks_;
    int nextTrackId_ = 1;
};

class TargetSelector {
public:
    explicit TargetSelector(int maxMissingFrames = 30);

    std::optional<Track> select(const std::vector<Track>& candidates, const cv::Size& frameSize);
    void reset();

private:
    double score(const Track& track, const cv::Size& frameSize) const;

    int maxMissingFrames_;
    int activeTrackId_ = -1;
    int missingFrames_ = 0;
    std::optional<Track> lastSelected_;
};

class VisualTrackerFallback {
public:
    VisualTrackerFallback(std::string trackerName, int maxMissingFrames);
    ~VisualTrackerFallback();

    void reset();
    void start(const cv::Mat& frameBgr, const Track& target);
    std::optional<Track> update(const cv::Mat& frameBgr);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class KalmanTracker2D {
public:
    KalmanTracker2D(double processVariance, double measurementVariance);

    void reset();
    cv::Point2f correct(const cv::Point2f& measurement, double dt);
    std::optional<cv::Point2f> predict(double dt);

private:
    void configure(double dt);

    double processVariance_;
    double measurementVariance_;
    cv::KalmanFilter kalman_;
    bool initialized_ = false;
};

}  // namespace ptcamera
