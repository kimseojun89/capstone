#include "ptcamera/overlay.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ptcamera {

namespace {

std::string formatDouble(double value, int precision) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

void putLine(cv::Mat& frame, const std::string& text, int y, const cv::Scalar& color, double scale = 0.65) {
    cv::putText(frame, text, cv::Point(20, y), cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
}

}  // namespace

void drawOverlay(
    cv::Mat& frame,
    const std::optional<Track>& target,
    const ControlTelemetry& telemetry,
    int detectionCount,
    double fps,
    bool motorEnabled,
    const std::vector<std::string>& statusLines) {
    const int aimX = frame.cols / 2;
    const int aimY = frame.rows / 2;

    cv::line(frame, cv::Point(aimX, 0), cv::Point(aimX, frame.rows), cv::Scalar(255, 0, 0), 1);
    cv::line(frame, cv::Point(0, aimY), cv::Point(frame.cols, aimY), cv::Scalar(255, 0, 0), 1);
    cv::circle(frame, cv::Point(aimX, aimY), 12, cv::Scalar(255, 255, 255), 2);
    cv::circle(frame, cv::Point(aimX, aimY), 3, cv::Scalar(255, 255, 255), -1);

    if (target) {
        const cv::Rect rect(static_cast<int>(std::round(target->box.x)),
                            static_cast<int>(std::round(target->box.y)),
                            static_cast<int>(std::round(target->box.width)),
                            static_cast<int>(std::round(target->box.height)));
        cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
        cv::circle(frame,
                   cv::Point(static_cast<int>(std::round(target->center.x)),
                             static_cast<int>(std::round(target->center.y))),
                   5,
                   cv::Scalar(0, 0, 255),
                   -1);
        if (target->hasTrackId()) {
            cv::putText(frame,
                        "ID " + std::to_string(target->trackId),
                        cv::Point(rect.x, std::max(rect.y - 8, 20)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(0, 255, 0),
                        2,
                        cv::LINE_AA);
        }
        cv::putText(frame,
                    toString(target->source),
                    cv::Point(rect.x, std::min(rect.y + rect.height + 22, frame.rows - 10)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cv::Scalar(0, 255, 0),
                    2,
                    cv::LINE_AA);
    }

    putLine(frame, telemetry.targetFound ? "LOCKED" : "NO TARGET", 35, cv::Scalar(0, 255, 255), 1.0);
    putLine(frame, std::string("MODE webcam/") + toString(telemetry.source), 65, cv::Scalar(200, 255, 255));
    putLine(frame,
            "STEP pan " + std::to_string(telemetry.command.panSteps) + " tilt " +
                std::to_string(telemetry.command.tiltSteps),
            95,
            cv::Scalar(0, 255, 0));
    putLine(frame,
            "ERR_X " + formatDouble(telemetry.errorX, 1) + " ERR_Y " + formatDouble(telemetry.errorY, 1),
            125,
            cv::Scalar(255, 255, 0));
    putLine(frame, "DETECTIONS " + std::to_string(detectionCount), 155, cv::Scalar(0, 200, 255));
    putLine(frame,
            "PID pan " + formatDouble(telemetry.panControl, 2) + " tilt " +
                formatDouble(telemetry.tiltControl, 2),
            185,
            cv::Scalar(180, 255, 180),
            0.6);
    putLine(frame,
            "CMD pan " + formatDouble(telemetry.panCommand, 2) + " tilt " +
                formatDouble(telemetry.tiltCommand, 2),
            215,
            cv::Scalar(200, 220, 255),
            0.6);
    putLine(frame, "FPS " + formatDouble(fps, 1), 245, cv::Scalar(255, 255, 255), 0.6);
    putLine(frame, motorEnabled ? "MOTOR ON" : "MOTOR OFF", 275, motorEnabled ? cv::Scalar(80, 255, 80)
                                                                               : cv::Scalar(180, 180, 180));

    int y = 305;
    for (const auto& line : statusLines) {
        putLine(frame, line, y, cv::Scalar(255, 255, 255), 0.6);
        y += 28;
    }
}

}  // namespace ptcamera
