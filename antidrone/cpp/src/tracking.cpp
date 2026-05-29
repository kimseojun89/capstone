#include "ptcamera/tracking.hpp"

#include "ptcamera/control.hpp"

#include <opencv2/core/version.hpp>

#if __has_include(<opencv2/tracking.hpp>)
#define PTCAMERA_HAS_OPENCV_TRACKING 1
#include <opencv2/tracking.hpp>
#else
#define PTCAMERA_HAS_OPENCV_TRACKING 0
#endif

#if __has_include(<opencv2/tracking/tracking_legacy.hpp>)
#define PTCAMERA_HAS_OPENCV_LEGACY_TRACKING 1
#include <opencv2/tracking/tracking_legacy.hpp>
#else
#define PTCAMERA_HAS_OPENCV_LEGACY_TRACKING 0
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <utility>

namespace ptcamera {

std::string toString(TrackSource source) {
    switch (source) {
        case TrackSource::Detector:
            return "detector";
        case TrackSource::ByteTrack:
            return "bytetrack";
        case TrackSource::OpenCV:
            return "opencv";
        case TrackSource::Predict:
            return "predict";
    }
    return "unknown";
}

Track trackFromDetection(const Detection& detection, int trackId, TrackSource source) {
    Track track;
    track.box = detection.box;
    track.center = cv::Point2f(detection.box.x + detection.box.width * 0.5F,
                               detection.box.y + detection.box.height * 0.5F);
    track.area = detection.box.area();
    track.trackId = trackId;
    track.source = source;
    track.confidence = detection.confidence;
    return track;
}

cv::Rect2f clampRect(const cv::Rect2f& rect, const cv::Size& frameSize) {
    const float x1 = static_cast<float>(clamp(rect.x, 0.0, std::max(frameSize.width - 1, 0)));
    const float y1 = static_cast<float>(clamp(rect.y, 0.0, std::max(frameSize.height - 1, 0)));
    const float x2 = static_cast<float>(clamp(rect.x + std::max(rect.width, 1.0F),
                                             static_cast<double>(x1 + 1.0F),
                                             static_cast<double>(frameSize.width)));
    const float y2 = static_cast<float>(clamp(rect.y + std::max(rect.height, 1.0F),
                                             static_cast<double>(y1 + 1.0F),
                                             static_cast<double>(frameSize.height)));
    return cv::Rect2f(x1, y1, std::max(x2 - x1, 1.0F), std::max(y2 - y1, 1.0F));
}

std::optional<Track> trackWithCenter(
    const Track& base,
    const cv::Point2f& center,
    const cv::Size& frameSize,
    TrackSource source) {
    cv::Rect2f rect(center.x - base.box.width * 0.5F,
                    center.y - base.box.height * 0.5F,
                    base.box.width,
                    base.box.height);
    rect = clampRect(rect, frameSize);
    if (rect.width <= 1.0F || rect.height <= 1.0F) {
        return std::nullopt;
    }

    Track track = base;
    track.box = rect;
    track.center = cv::Point2f(rect.x + rect.width * 0.5F, rect.y + rect.height * 0.5F);
    track.area = rect.area();
    track.source = source;
    return track;
}

float boxIou(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.width, b.x + b.width);
    const float y2 = std::min(a.y + a.height, b.y + b.height);
    const float intersection = std::max(0.0F, x2 - x1) * std::max(0.0F, y2 - y1);
    const float unionArea = a.area() + b.area() - intersection;
    if (unionArea <= 0.0F) {
        return 0.0F;
    }
    return intersection / unionArea;
}

ByteTracker::ByteTracker(const TrackerSettings& settings) : settings_(settings) {}

void ByteTracker::reset() {
    tracks_.clear();
    nextTrackId_ = 1;
}

std::vector<Track> ByteTracker::update(const std::vector<Detection>& detections, const cv::Size& frameSize) {
    for (auto& track : tracks_) {
        track.updated = false;
    }

    std::vector<int> trackIndices;
    trackIndices.reserve(tracks_.size());
    for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
        trackIndices.push_back(i);
    }

    std::vector<int> highDetections;
    std::vector<int> lowDetections;
    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        if (detections[i].confidence >= settings_.trackHighThreshold) {
            highDetections.push_back(i);
        } else if (detections[i].confidence >= settings_.trackLowThreshold) {
            lowDetections.push_back(i);
        }
    }

    std::set<int> matchedHighDetections;
    for (const auto& [trackIndex, detectionIndex] : matchDetections(trackIndices, highDetections, detections)) {
        auto& track = tracks_[trackIndex];
        track.box = clampRect(detections[detectionIndex].box, frameSize);
        track.confidence = detections[detectionIndex].confidence;
        track.missingFrames = 0;
        track.updated = true;
        matchedHighDetections.insert(detectionIndex);
    }

    std::vector<int> unmatchedTrackIndices;
    for (int index : trackIndices) {
        if (!tracks_[index].updated) {
            unmatchedTrackIndices.push_back(index);
        }
    }

    for (const auto& [trackIndex, detectionIndex] : matchDetections(unmatchedTrackIndices, lowDetections, detections)) {
        auto& track = tracks_[trackIndex];
        track.box = clampRect(detections[detectionIndex].box, frameSize);
        track.confidence = detections[detectionIndex].confidence;
        track.missingFrames = 0;
        track.updated = true;
    }

    for (auto& track : tracks_) {
        if (!track.updated) {
            ++track.missingFrames;
        }
    }

    for (int detectionIndex : highDetections) {
        if (matchedHighDetections.count(detectionIndex) != 0) {
            continue;
        }
        if (detections[detectionIndex].confidence < settings_.newTrackThreshold) {
            continue;
        }

        InternalTrack track;
        track.id = nextTrackId_++;
        track.box = clampRect(detections[detectionIndex].box, frameSize);
        track.confidence = detections[detectionIndex].confidence;
        track.updated = true;
        tracks_.push_back(track);
    }

    tracks_.erase(
        std::remove_if(
            tracks_.begin(),
            tracks_.end(),
            [&](const InternalTrack& track) { return track.missingFrames > settings_.trackBuffer; }),
        tracks_.end());

    std::vector<Track> output;
    for (const auto& track : tracks_) {
        if (track.missingFrames == 0) {
            Detection detection;
            detection.box = track.box;
            detection.confidence = track.confidence;
            output.push_back(trackFromDetection(detection, track.id, TrackSource::ByteTrack));
        }
    }
    return output;
}

std::vector<std::pair<int, int>> ByteTracker::matchDetections(
    const std::vector<int>& trackIndices,
    const std::vector<int>& detectionIndices,
    const std::vector<Detection>& detections) const {
    struct Candidate {
        int trackIndex;
        int detectionIndex;
        float iou;
        float score;
    };

    std::vector<Candidate> candidates;
    for (int trackIndex : trackIndices) {
        for (int detectionIndex : detectionIndices) {
            const float iou = boxIou(tracks_[trackIndex].box, detections[detectionIndex].box);
            if (iou >= settings_.trackMatchThreshold) {
                const float score = settings_.fuseScore
                                        ? iou * (0.5F + 0.5F * detections[detectionIndex].confidence)
                                        : iou;
                candidates.push_back({trackIndex, detectionIndex, iou, score});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.score > b.score;
    });

    std::set<int> usedTracks;
    std::set<int> usedDetections;
    std::vector<std::pair<int, int>> matches;
    for (const auto& candidate : candidates) {
        if (usedTracks.count(candidate.trackIndex) != 0 || usedDetections.count(candidate.detectionIndex) != 0) {
            continue;
        }
        usedTracks.insert(candidate.trackIndex);
        usedDetections.insert(candidate.detectionIndex);
        matches.push_back({candidate.trackIndex, candidate.detectionIndex});
    }
    return matches;
}

TargetSelector::TargetSelector(int maxMissingFrames) : maxMissingFrames_(maxMissingFrames) {}

void TargetSelector::reset() {
    activeTrackId_ = -1;
    missingFrames_ = 0;
    lastSelected_.reset();
}

std::optional<Track> TargetSelector::select(const std::vector<Track>& candidates, const cv::Size& frameSize) {
    if (candidates.empty()) {
        ++missingFrames_;
        if (missingFrames_ >= maxMissingFrames_) {
            activeTrackId_ = -1;
            lastSelected_.reset();
        }
        return std::nullopt;
    }

    if (activeTrackId_ >= 0) {
        for (const auto& candidate : candidates) {
            if (candidate.trackId == activeTrackId_) {
                lastSelected_ = candidate;
                missingFrames_ = 0;
                return candidate;
            }
        }
    }

    const auto best = std::max_element(candidates.begin(), candidates.end(), [&](const Track& a, const Track& b) {
        return score(a, frameSize) < score(b, frameSize);
    });
    if (best == candidates.end()) {
        return std::nullopt;
    }

    activeTrackId_ = best->trackId;
    lastSelected_ = *best;
    missingFrames_ = 0;
    return *best;
}

double TargetSelector::score(const Track& track, const cv::Size& frameSize) const {
    double result = static_cast<double>(track.confidence) * 0.6;
    const double frameDiag = std::max(std::hypot(frameSize.width, frameSize.height), 1.0);

    if (lastSelected_) {
        const double distance = std::hypot(track.center.x - lastSelected_->center.x,
                                          track.center.y - lastSelected_->center.y);
        const double proximityScore = 1.0 - clamp(distance / frameDiag, 0.0, 1.0);
        const double areaRatio = std::max(track.area, 1.0F) / std::max(lastSelected_->area, 1.0F);
        const double areaConsistency = 1.0 - clamp(std::abs(std::log(areaRatio)), 0.0, 1.0);
        result += proximityScore * 0.3;
        result += areaConsistency * 0.1;
    } else {
        const double centerDistance = std::hypot(track.center.x - frameSize.width / 2.0,
                                                track.center.y - frameSize.height / 2.0);
        const double centerScore = 1.0 - clamp(centerDistance / (frameDiag / 2.0), 0.0, 1.0);
        result += centerScore * 0.1;
    }

    return result;
}

struct VisualTrackerFallback::Impl {
    explicit Impl(std::string trackerNameValue, int maxMissingFramesValue)
        : trackerName(std::move(trackerNameValue)), maxMissingFrames(maxMissingFramesValue) {}

    std::string trackerName;
    int maxMissingFrames;
    int missingFrames = 0;
    int trackId = -1;
    bool warnedUnavailable = false;

#if PTCAMERA_HAS_OPENCV_TRACKING
    cv::Ptr<cv::Tracker> tracker;
#endif
};

VisualTrackerFallback::VisualTrackerFallback(std::string trackerName, int maxMissingFrames)
    : impl_(std::make_unique<Impl>(std::move(trackerName), maxMissingFrames)) {}

VisualTrackerFallback::~VisualTrackerFallback() = default;

void VisualTrackerFallback::reset() {
    impl_->missingFrames = 0;
    impl_->trackId = -1;
#if PTCAMERA_HAS_OPENCV_TRACKING
    impl_->tracker.release();
#endif
}

namespace {

cv::Rect expandTrackerRect(const cv::Rect2f& rect, const cv::Size& frameSize) {
    const float pad = std::max(4.0F, std::max(rect.width, rect.height) * 0.15F);
    const float width = std::max(rect.width + pad * 2.0F, 24.0F);
    const float height = std::max(rect.height + pad * 2.0F, 24.0F);
    const cv::Point2f center(rect.x + rect.width * 0.5F, rect.y + rect.height * 0.5F);
    const cv::Rect2f expanded(center.x - width * 0.5F, center.y - height * 0.5F, width, height);
    const auto clamped = clampRect(expanded, frameSize);
    return cv::Rect(
        static_cast<int>(std::round(clamped.x)),
        static_cast<int>(std::round(clamped.y)),
        static_cast<int>(std::round(clamped.width)),
        static_cast<int>(std::round(clamped.height)));
}

#if PTCAMERA_HAS_OPENCV_TRACKING
std::string lowerTrackerName(std::string name) {
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return name;
}

std::vector<std::string> trackerCandidates(const std::string& requestedTrackerName) {
    std::vector<std::string> candidates;
    auto add = [&](std::string name) {
        name = lowerTrackerName(std::move(name));
        if (!name.empty() && std::find(candidates.begin(), candidates.end(), name) == candidates.end()) {
            candidates.push_back(name);
        }
    };

    add(requestedTrackerName.empty() ? "csrt" : requestedTrackerName);
    add("csrt");
    add("kcf");
    add("mil");
    add("mosse");
    return candidates;
}

cv::Ptr<cv::Tracker> createOpenCvTracker(const std::string& trackerName) {
    const std::string name = lowerTrackerName(trackerName.empty() ? "csrt" : trackerName);
    if (name == "csrt") {
        return cv::TrackerCSRT::create();
    }
    if (name == "kcf") {
        return cv::TrackerKCF::create();
    }
    if (name == "mil") {
        return cv::TrackerMIL::create();
    }

#if PTCAMERA_HAS_OPENCV_LEGACY_TRACKING
    if (name == "mosse") {
        return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerMOSSE::create());
    }
    if (name == "legacy_csrt") {
        return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerCSRT::create());
    }
    if (name == "legacy_kcf") {
        return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerKCF::create());
    }
    if (name == "legacy_mil") {
        return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerMIL::create());
    }
#endif

    return {};
}
#endif

}  // namespace

void VisualTrackerFallback::start(const cv::Mat& frameBgr, const Track& target) {
#if PTCAMERA_HAS_OPENCV_TRACKING
    const cv::Rect rect = expandTrackerRect(target.box, frameBgr.size());
    for (const auto& trackerName : trackerCandidates(impl_->trackerName)) {
        try {
            auto tracker = createOpenCvTracker(trackerName);
            if (!tracker) {
                continue;
            }
            tracker->init(frameBgr, rect);
            impl_->tracker = tracker;
            impl_->missingFrames = 0;
            impl_->trackId = target.trackId;
            return;
        } catch (const cv::Exception& error) {
            std::cerr << "OpenCV tracker init failed (" << trackerName << "): " << error.what() << '\n';
        }
    }
    reset();
#else
    (void)frameBgr;
    (void)target;
    if (!impl_->warnedUnavailable) {
        std::cerr << "OpenCV tracking headers are unavailable; visual fallback is disabled.\n";
        impl_->warnedUnavailable = true;
    }
#endif
}

std::optional<Track> VisualTrackerFallback::update(const cv::Mat& frameBgr) {
#if PTCAMERA_HAS_OPENCV_TRACKING
    if (!impl_->tracker) {
        return std::nullopt;
    }

    cv::Rect rect;
    bool ok = false;
    try {
        ok = impl_->tracker->update(frameBgr, rect);
    } catch (const cv::Exception&) {
        ok = false;
    }

    if (!ok) {
        ++impl_->missingFrames;
        if (impl_->missingFrames >= impl_->maxMissingFrames) {
            reset();
        }
        return std::nullopt;
    }

    impl_->missingFrames = 0;
    Detection detection;
    detection.box = clampRect(cv::Rect2f(static_cast<float>(rect.x),
                                         static_cast<float>(rect.y),
                                         static_cast<float>(rect.width),
                                         static_cast<float>(rect.height)),
                              frameBgr.size());
    detection.confidence = 0.0F;
    return trackFromDetection(detection, impl_->trackId, TrackSource::OpenCV);
#else
    (void)frameBgr;
    return std::nullopt;
#endif
}

KalmanTracker2D::KalmanTracker2D(double processVariance, double measurementVariance)
    : processVariance_(processVariance), measurementVariance_(measurementVariance), kalman_(4, 2, 0, CV_32F) {
    configure(1.0);
    kalman_.errorCovPost = cv::Mat::eye(4, 4, CV_32F);
}

void KalmanTracker2D::reset() {
    initialized_ = false;
    configure(1.0);
    kalman_.errorCovPost = cv::Mat::eye(4, 4, CV_32F);
}

cv::Point2f KalmanTracker2D::correct(const cv::Point2f& measurement, double dt) {
    configure(dt);
    if (!initialized_) {
        kalman_.statePost = (cv::Mat_<float>(4, 1) << measurement.x, measurement.y, 0.0F, 0.0F);
        kalman_.statePre = kalman_.statePost.clone();
        initialized_ = true;
        return measurement;
    }

    kalman_.predict();
    const cv::Mat measurementMat = (cv::Mat_<float>(2, 1) << measurement.x, measurement.y);
    const cv::Mat corrected = kalman_.correct(measurementMat);
    return cv::Point2f(corrected.at<float>(0, 0), corrected.at<float>(1, 0));
}

std::optional<cv::Point2f> KalmanTracker2D::predict(double dt) {
    if (!initialized_) {
        return std::nullopt;
    }
    configure(dt);
    const cv::Mat prediction = kalman_.predict();
    return cv::Point2f(prediction.at<float>(0, 0), prediction.at<float>(1, 0));
}

void KalmanTracker2D::configure(double dt) {
    dt = std::max(dt, 1e-3);
    kalman_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1.0F, 0.0F, static_cast<float>(dt), 0.0F,
                                 0.0F, 1.0F, 0.0F, static_cast<float>(dt),
                                 0.0F, 0.0F, 1.0F, 0.0F,
                                 0.0F, 0.0F, 0.0F, 1.0F);
    kalman_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1.0F, 0.0F, 0.0F, 0.0F,
                                  0.0F, 1.0F, 0.0F, 0.0F);
    kalman_.processNoiseCov = cv::Mat::eye(4, 4, CV_32F) * processVariance_;
    kalman_.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * measurementVariance_;
}

}  // namespace ptcamera
