#include "ptcamera/pipeline.hpp"

namespace ptcamera {

TrackingPipeline::TrackingPipeline(const TrackerSettings& settings)
    : settings_(settings),
      detector_(settings),
      byteTracker_(settings),
      targetSelector_(30),
      visualTracker_(settings.visualTracker, settings.visualTrackerMaxMissingFrames),
      kalman_(settings.kalmanProcessVariance, settings.kalmanMeasurementVariance) {}

void TrackingPipeline::reset() {
    byteTracker_.reset();
    targetSelector_.reset();
    visualTracker_.reset();
    kalman_.reset();
    lostFrames_ = 0;
    lastTrackingTarget_.reset();
}

PipelineResult TrackingPipeline::update(const cv::Mat& frameBgr, double dt) {
    PipelineResult result;
    result.detections = detector_.detect(frameBgr);
    result.tracks = byteTracker_.update(result.detections, frameBgr.size());

    std::optional<Track> rawTarget = targetSelector_.select(result.tracks, frameBgr.size());
    if (rawTarget) {
        visualTracker_.start(frameBgr, *rawTarget);
    } else {
        rawTarget = visualTracker_.update(frameBgr);
    }

    if (rawTarget) {
        lostFrames_ = 0;
        const cv::Point2f corrected = kalman_.correct(rawTarget->center, dt);
        result.target = trackWithCenter(*rawTarget, corrected, frameBgr.size(), rawTarget->source);
        if (!result.target) {
            result.target = rawTarget;
        }
        lastTrackingTarget_ = result.target;
        return result;
    }

    ++lostFrames_;
    const auto prediction = kalman_.predict(dt);
    if (prediction && lastTrackingTarget_ && lostFrames_ <= settings_.maxHoldFrames) {
        result.target = trackWithCenter(*lastTrackingTarget_, *prediction, frameBgr.size(), TrackSource::Predict);
        if (result.target) {
            lastTrackingTarget_ = result.target;
        }
        return result;
    }

    if (lostFrames_ > settings_.maxHoldFrames) {
        kalman_.reset();
        lastTrackingTarget_.reset();
    }
    return result;
}

}  // namespace ptcamera
