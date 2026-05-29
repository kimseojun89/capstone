#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

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

}  // namespace

int main(int argc, char** argv) {
    int cameraIndex = 0;
    int width = 0;
    int height = 0;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--camera") {
                cameraIndex = readIntArg(argc, argv, i);
            } else if (arg == "--width") {
                width = readIntArg(argc, argv, i);
            } else if (arg == "--height") {
                height = readIntArg(argc, argv, i);
            } else if (arg == "--help") {
                std::cout << "Usage: camera_smoke_test [--camera 0] [--width 1280] [--height 720]\n";
                return 0;
            } else {
                throw std::runtime_error("unknown argument: " + arg);
            }
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }

    cv::VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open camera index " << cameraIndex << '\n';
        return 1;
    }
    if (width > 0) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    }
    if (height > 0) {
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }

    cv::namedWindow("camera_smoke_test", cv::WINDOW_NORMAL);
    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Failed to read frame\n";
            return 1;
        }
        cv::imshow("camera_smoke_test", frame);
        const int key = cv::waitKey(1) & 0xff;
        if (key == 27 || key == 'q') {
            break;
        }
    }
    return 0;
}
