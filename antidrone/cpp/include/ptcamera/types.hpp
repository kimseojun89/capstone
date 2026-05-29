#pragma once

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace ptcamera {

enum class TrackSource {
    Detector,
    ByteTrack,
    OpenCV,
    Predict,
};

std::string toString(TrackSource source);

struct Detection {
    cv::Rect2f box;
    float confidence = 0.0F;
    int classId = 0;
};

struct Track {
    cv::Rect2f box;
    cv::Point2f center;
    float area = 0.0F;
    int trackId = -1;
    TrackSource source = TrackSource::Detector;
    float confidence = 0.0F;

    bool hasTrackId() const { return trackId >= 0; }
};

struct ControlCommand {
    int panSteps = 0;
    int tiltSteps = 0;
};

struct LetterboxInfo {
    float scale = 1.0F;
    int padX = 0;
    int padY = 0;
    cv::Size inputSize;
    cv::Size originalSize;
};

Track trackFromDetection(const Detection& detection, int trackId, TrackSource source);
cv::Rect2f clampRect(const cv::Rect2f& rect, const cv::Size& frameSize);
std::optional<Track> trackWithCenter(
    const Track& base,
    const cv::Point2f& center,
    const cv::Size& frameSize,
    TrackSource source);
float boxIou(const cv::Rect2f& a, const cv::Rect2f& b);

}  // namespace ptcamera
