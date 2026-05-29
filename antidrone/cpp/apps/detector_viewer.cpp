#include "ptcamera/detector.hpp"
#include "ptcamera/settings.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
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
    std::cout << "Usage: detector_viewer [--model path_or_dir] [--device CPU] [--camera 0] "
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
        ptcamera::YoloOpenVinoDetector detector(settings);
        std::cout << "Model: " << detector.modelOnnxPath() << '\n';
        std::cout << "Inference device: " << detector.resolvedDevice() << '\n';

        cv::VideoCapture cap(settings.cameraIndex);
        if (!cap.isOpened()) {
            std::cerr << "Cannot open camera index " << settings.cameraIndex << '\n';
            return 1;
        }

        cv::namedWindow("detector_viewer", cv::WINDOW_NORMAL);
        auto last = std::chrono::steady_clock::now();
        double fps = 0.0;

        while (true) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "Failed to read frame\n";
                break;
            }

            const auto detections = detector.detect(frame);
            for (const auto& detection : detections) {
                const cv::Rect rect(static_cast<int>(std::round(detection.box.x)),
                                    static_cast<int>(std::round(detection.box.y)),
                                    static_cast<int>(std::round(detection.box.width)),
                                    static_cast<int>(std::round(detection.box.height)));
                cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame,
                            "drone " + std::to_string(detection.confidence).substr(0, 4),
                            cv::Point(rect.x, std::max(rect.y - 8, 20)),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.55,
                            cv::Scalar(0, 255, 0),
                            2);
            }

            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last).count();
            last = now;
            if (dt > 0.0) {
                fps = 0.9 * fps + 0.1 * (1.0 / dt);
            }
            cv::putText(frame,
                        "detections " + std::to_string(detections.size()) + " fps " +
                            std::to_string(fps).substr(0, 4),
                        cv::Point(20, 35),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.8,
                        cv::Scalar(0, 255, 255),
                        2);

            cv::imshow("detector_viewer", frame);
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
