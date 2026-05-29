#pragma once

#include <string>
#include <vector>

namespace ptcamera {

struct TrackerSettings {
    int cameraIndex = 1;      // ABKO APC900 FHD (index 1)
    int cameraWidth = 1920;   // ABKO APC900 FHD
    int cameraHeight = 1080;  // ABKO APC900 FHD
    bool flipFrame = false;

    std::string modelPath;
    int inputSize = 0;
    std::string inferenceDevice = "CUDA";   // "CUDA" or "CPU"
    std::string resolvedDevice;
    float confidenceThreshold = 0.25F;
    float nmsIouThreshold = 0.70F;
    int minArea = 80;
    std::vector<std::string> targetClassNames {"drone"};

    std::string trackerBackend = "bytetrack";
    std::string trackerConfigPath;
    float trackHighThreshold = 0.25F;
    float trackLowThreshold = 0.08F;
    float newTrackThreshold = 0.25F;
    float trackMatchThreshold = 0.80F;
    bool fuseScore = true;
    int trackBuffer = 30;
    int maxHoldFrames = 10;
    std::string visualTracker = "csrt";
    int visualTrackerMaxMissingFrames = 45;

    double sendInterval = 0.12;
    int panMinStep = 8;
    int tiltMinStep = 8;
    int panMaxStep = 48;
    int tiltMaxStep = 48;
    int panDeadband = 35;
    int tiltDeadband = 35;
    bool invertPan = false;
    bool invertTilt = false;
    double panKp = 0.25;
    double panKi = 0.0;
    double panKd = 0.02;
    double tiltKp = 0.25;
    double tiltKi = 0.0;
    double tiltKd = 0.02;
    double commandRamp = 2.2;
    double acquireRampScale = 0.6;
    double opencvControlScale = 0.6;
    double predictControlScale = 0.35;

    double kalmanProcessVariance = 1e-2;
    double kalmanMeasurementVariance = 15.0;

    std::string serialPort = "/dev/ttyACM0";
    int serialBaud = 256000;
    int stepDelayUs = 2000;
    int coilOrder = 0;
};

std::string defaultModelPath();
TrackerSettings defaultSettings();

}  // namespace ptcamera
