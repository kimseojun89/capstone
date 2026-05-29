#include "ptcamera/control.hpp"
#include "ptcamera/overlay.hpp"
#include "ptcamera/pipeline.hpp"
#include "ptcamera/settings.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int readIntArg(int argc, char** argv, int& index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return std::atoi(argv[++index]);
}

float readFloatArg(int argc, char** argv, int& index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return std::stof(argv[++index]);
}

std::string readStringArg(int argc, char** argv, int& index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return argv[++index];
}

void printUsage() {
    std::cout << "Usage: tracking_viewer [--model path_or_dir] [--device CPU] [--camera 0] "
                 "[--conf 0.25]\n";
}

}  // namespace

int main(int argc, char** argv) {
    ptcamera::TrackerSettings settings = ptcamera::defaultSettings();

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--model") {
                settings.modelPath = readStringArg(argc, argv, i);
            } else if (arg == "--device") {
                settings.inferenceDevice = readStringArg(argc, argv, i);
            } else if (arg == "--camera") {
                settings.cameraIndex = readIntArg(argc, argv, i);
            } else if (arg == "--conf") {
                settings.confidenceThreshold = readFloatArg(argc, argv, i);
            } else if (arg == "--help") {
                printUsage();
                return 0;
            } else {
                throw std::runtime_error("unknown argument: " + arg);
            }
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        printUsage();
        return 2;
    }

    try {
        ptcamera::TrackingPipeline pipeline(settings);
        ptcamera::ControlLoop control(settings);

        cv::VideoCapture cap(settings.cameraIndex);
        if (!cap.isOpened()) {
            std::cerr << "Cannot open camera index " << settings.cameraIndex << '\n';
            return 1;
        }

        cv::namedWindow("tracking_viewer", cv::WINDOW_NORMAL);
        auto last = std::chrono::steady_clock::now();
        double fps = 0.0;

        while (true) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "Failed to read frame\n";
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last).count();
            last = now;
            if (dt > 0.0) {
                fps = fps <= 0.0 ? 1.0 / dt : 0.9 * fps + 0.1 * (1.0 / dt);
            }

            auto result = pipeline.update(frame, dt);
            auto telemetry = control.update(result.target, frame.size(), dt);
            drawOverlay(frame,
                        result.target,
                        telemetry,
                        static_cast<int>(result.detections.size()),
                        fps,
                        false,
                        {"tracking viewer", "space unused, ESC/q exits"});

            cv::imshow("tracking_viewer", frame);
            const int key = cv::waitKey(1) & 0xff;
            if (key == 27 || key == 'q') {
                break;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
