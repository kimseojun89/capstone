#include "ptcamera/control.hpp"

#include <algorithm>
#include <cmath>

namespace ptcamera {

double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

double slewLimit(double target, double current, double maxDelta) {
    if (maxDelta <= 0.0) {
        return target;
    }
    return current + clamp(target - current, -maxDelta, maxDelta);
}

PIDController::PIDController(double kp, double ki, double kd, double integralLimit)
    : kp_(kp), ki_(ki), kd_(kd), integralLimit_(integralLimit) {}

void PIDController::reset() {
    integral_ = 0.0;
    previousError_ = 0.0;
    initialized_ = false;
}

double PIDController::update(double error, double dt) {
    dt = std::max(dt, 1e-3);
    if (!initialized_) {
        previousError_ = error;
        initialized_ = true;
    }

    integral_ = clamp(integral_ + error * dt, -integralLimit_, integralLimit_);
    const double derivative = (error - previousError_) / dt;
    previousError_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

AxisController::AxisController(int minStep, int maxStep, int deadbandPx, bool invert)
    : minStep_(minStep), maxStep_(maxStep), deadbandPx_(deadbandPx), invert_(invert) {}

void AxisController::reset() {}

double AxisController::normalizeCommandFloor(double command) const {
    command = clamp(command, -1.0, 1.0);
    if (std::abs(command) < 1e-3) {
        return 0.0;
    }

    const int stepSpan = std::max(maxStep_ - minStep_, 1);
    const double minimumRatio = static_cast<double>(minStep_) / static_cast<double>(minStep_ + stepSpan);
    const double magnitude = std::max(std::abs(command), minimumRatio);
    return command > 0.0 ? magnitude : -magnitude;
}

int AxisController::stepFromCommand(double command) const {
    command = clamp(command, -1.0, 1.0);
    if (std::abs(command) < 1e-3) {
        return 0;
    }
    if (invert_) {
        command *= -1.0;
    }

    const int step = minStep_ + static_cast<int>((maxStep_ - minStep_) * std::abs(command));
    return command > 0.0 ? step : -step;
}

ControlLoop::ControlLoop(const TrackerSettings& settings)
    : settings_(settings),
      panAxis_(settings.panMinStep, settings.panMaxStep, settings.panDeadband, settings.invertPan),
      tiltAxis_(settings.tiltMinStep, settings.tiltMaxStep, settings.tiltDeadband, settings.invertTilt),
      panPid_(settings.panKp, settings.panKi, settings.panKd),
      tiltPid_(settings.tiltKp, settings.tiltKi, settings.tiltKd) {}

void ControlLoop::reset() {
    panPid_.reset();
    tiltPid_.reset();
    panCommand_ = 0.0;
    tiltCommand_ = 0.0;
    hadTarget_ = false;
}

ControlTelemetry ControlLoop::update(const std::optional<Track>& target, const cv::Size& frameSize, double dt) {
    dt = std::max(dt, 1e-3);
    ControlTelemetry telemetry;

    if (target) {
        telemetry.targetFound = true;
        telemetry.source = target->source;
        telemetry.errorX = static_cast<double>(target->center.x) - static_cast<double>(frameSize.width) / 2.0;
        telemetry.errorY = static_cast<double>(frameSize.height) / 2.0 - static_cast<double>(target->center.y);

        double rampLimit = settings_.commandRamp * dt;
        if (!hadTarget_) {
            rampLimit *= settings_.acquireRampScale;
        }

        const double scale = controlScale(target->source);
        const auto pan = updateAxis(
            panAxis_,
            panPid_,
            telemetry.errorX * scale,
            static_cast<double>(frameSize.width) / 2.0,
            dt,
            panCommand_,
            rampLimit);
        const auto tilt = updateAxis(
            tiltAxis_,
            tiltPid_,
            telemetry.errorY * scale,
            static_cast<double>(frameSize.height) / 2.0,
            dt,
            tiltCommand_,
            rampLimit);

        telemetry.panControl = pan.first;
        telemetry.tiltControl = tilt.first;
        telemetry.command.panSteps = pan.second;
        telemetry.command.tiltSteps = tilt.second;
        hadTarget_ = true;
    } else {
        panCommand_ = slewLimit(0.0, panCommand_, settings_.commandRamp * dt);
        tiltCommand_ = slewLimit(0.0, tiltCommand_, settings_.commandRamp * dt);
        telemetry.command.panSteps = panAxis_.stepFromCommand(panCommand_);
        telemetry.command.tiltSteps = tiltAxis_.stepFromCommand(tiltCommand_);
        if (std::abs(panCommand_) < 1e-3 && std::abs(tiltCommand_) < 1e-3) {
            panPid_.reset();
            tiltPid_.reset();
        }
        hadTarget_ = false;
    }

    telemetry.panCommand = panCommand_;
    telemetry.tiltCommand = tiltCommand_;
    return telemetry;
}

double ControlLoop::controlScale(TrackSource source) const {
    if (source == TrackSource::OpenCV) {
        return clamp(settings_.opencvControlScale, 0.0, 1.0);
    }
    if (source == TrackSource::Predict) {
        return clamp(settings_.predictControlScale, 0.0, 1.0);
    }
    return 1.0;
}

std::pair<double, int> ControlLoop::updateAxis(
    AxisController& axis,
    PIDController& pid,
    double errorPx,
    double frameHalfPx,
    double dt,
    double& smoothedCommand,
    double rampLimit) {
    double targetCommand = 0.0;
    if (std::abs(errorPx) <= axis.deadbandPx()) {
        pid.reset();
    } else {
        targetCommand = pid.update(errorPx / std::max(frameHalfPx, 1.0), dt);
        targetCommand = axis.normalizeCommandFloor(targetCommand);
    }

    smoothedCommand = slewLimit(targetCommand, smoothedCommand, rampLimit);
    return {targetCommand, axis.stepFromCommand(smoothedCommand)};
}

}  // namespace ptcamera
