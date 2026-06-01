#pragma once

#include "ptcamera/settings.hpp"
#include "ptcamera/types.hpp"

#include <opencv2/core.hpp>

#include <optional>

namespace ptcamera {

double clamp(double value, double low, double high);
double slewLimit(double target, double current, double maxDelta);

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double integralLimit = 1.5);

    void reset();
    double update(double error, double dt);

private:
    double kp_;
    double ki_;
    double kd_;
    double integralLimit_;
    double integral_ = 0.0;
    double previousError_ = 0.0;
    bool initialized_ = false;
};

class AxisController {
public:
    AxisController(int minStep, int maxStep, int deadbandPx, bool invert);

    void reset();
    double normalizeCommandFloor(double command) const;
    int stepFromCommand(double command) const;
    int deadbandPx() const { return deadbandPx_; }

private:
    int minStep_;
    int maxStep_;
    int deadbandPx_;
    bool invert_;
};

struct ControlTelemetry {
    bool targetFound = false;
    TrackSource source = TrackSource::Detector;
    double errorX = 0.0;
    double errorY = 0.0;
    double panControl = 0.0;
    double tiltControl = 0.0;
    double panCommand = 0.0;
    double tiltCommand = 0.0;
    ControlCommand command;
};

class ControlLoop {
public:
    explicit ControlLoop(const TrackerSettings& settings);

    void reset();
    ControlTelemetry update(const std::optional<Track>& target, const cv::Size& frameSize, double dt);

private:
    double controlScale(TrackSource source) const;
    std::pair<double, int> updateAxis(
        AxisController& axis,
        PIDController& pid,
        double errorPx,
        double frameHalfPx,
        double dt,
        double& smoothedCommand,
        double rampLimit);

    TrackerSettings settings_;
    AxisController panAxis_;
    AxisController tiltAxis_;
    PIDController panPid_;
    PIDController tiltPid_;
    double panCommand_ = 0.0;
    double tiltCommand_ = 0.0;
    bool hadTarget_ = false;
};

// 각도 ↔ 스텝 변환 상수 (28BYJ-48 하프스텝: 4096 steps/rev)
constexpr double STEPS_PER_DEGREE = 4096.0 / 360.0;  // ≈ 11.378

int degreesToSteps(double degrees);
double stepsToDegrees(int steps);

}  // namespace ptcamera
