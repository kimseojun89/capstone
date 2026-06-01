#include "ptcamera/control.hpp"
#include "ptcamera/overlay.hpp"
#include "ptcamera/pipeline.hpp"
#include "ptcamera/serial_port.hpp"
#include "ptcamera/settings.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <unistd.h>
#endif

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int readIntArg(int argc, char** argv, int& index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return std::atoi(argv[++index]);
}

double readDoubleArg(int argc, char** argv, int& index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return std::stod(argv[++index]);
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
    std::cout << "Usage: ptcamera_tracker [--model path_or_dir] [--device CPU] [--camera 0]\n"
                 "                        [--camera-width 1920] [--camera-height 1080]\n"
                 "                        [--serial-port COM4] [--baud 256000]\n"
                 "                        [--enable-motor] [--conf 0.25] [--send-interval 0.033]\n"
                 "                        [--manual-mode]\n";
}

bool ensureSerialOpen(ptcamera::SerialPort& serial, const ptcamera::TrackerSettings& settings) {
    if (serial.isOpen()) {
        return true;
    }

    std::string error;
    if (!serial.openPort(settings.serialPort, settings.serialBaud, &error)) {
        std::cerr << error << '\n';
        return false;
    }
    std::cout << "Opened serial " << settings.serialPort << " at " << settings.serialBaud
              << ". Connected to FPGA.\n";
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    ptcamera::TrackerSettings settings = ptcamera::defaultSettings();
    bool motorEnabled = false;
    bool manualMode = false;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--model") {
                settings.modelPath = readStringArg(argc, argv, i);
            } else if (arg == "--device") {
                settings.inferenceDevice = readStringArg(argc, argv, i);
            } else if (arg == "--camera") {
                settings.cameraIndex = readIntArg(argc, argv, i);
            } else if (arg == "--camera-width") {
                settings.cameraWidth = readIntArg(argc, argv, i);
            } else if (arg == "--camera-height") {
                settings.cameraHeight = readIntArg(argc, argv, i);
            } else if (arg == "--serial-port") {
                settings.serialPort = readStringArg(argc, argv, i);
            } else if (arg == "--baud") {
                settings.serialBaud = readIntArg(argc, argv, i);
            } else if (arg == "--conf") {
                settings.confidenceThreshold = readFloatArg(argc, argv, i);
            } else if (arg == "--send-interval") {
                settings.sendInterval = readDoubleArg(argc, argv, i);
            } else if (arg == "--step-delay-us") {
                settings.stepDelayUs = readIntArg(argc, argv, i);
            } else if (arg == "--coil-order") {
                settings.coilOrder = readIntArg(argc, argv, i);
            } else if (arg == "--enable-motor") {
                motorEnabled = true;
            } else if (arg == "--manual-mode") {
                manualMode = true;
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
        ptcamera::SerialPort serial;

        if (!ensureSerialOpen(serial, settings)) {
            std::cerr << "Serial not open; motor disabled, no radar relay\n";
            motorEnabled = false;
        } else if (!motorEnabled) {
            std::cout << "Serial open for radar relay (motor disabled; press space to enable)\n";
        }

#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        SOCKET udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#else
        int udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#endif
        sockaddr_in udpDest = {};
        udpDest.sin_family = AF_INET;
        udpDest.sin_port   = htons(9999);
        inet_pton(AF_INET, "127.0.0.1", &udpDest.sin_addr);

#ifdef _WIN32
        SOCKET camSock   = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        SOCKET telemSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#else
        int camSock   = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        int telemSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#endif
        sockaddr_in camDest = {};
        camDest.sin_family = AF_INET;
        camDest.sin_port   = htons(9998);
        inet_pton(AF_INET, "127.0.0.1", &camDest.sin_addr);

        sockaddr_in telemDest = {};
        telemDest.sin_family = AF_INET;
        telemDest.sin_port   = htons(10000);
        inet_pton(AF_INET, "127.0.0.1", &telemDest.sin_addr);

        cv::VideoCapture cap(settings.cameraIndex);
        if (!cap.isOpened()) {
            std::cerr << "Cannot open camera index " << settings.cameraIndex << '\n';
            return 1;
        }
        if (settings.cameraWidth > 0) {
            cap.set(cv::CAP_PROP_FRAME_WIDTH, settings.cameraWidth);
        }
        if (settings.cameraHeight > 0) {
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, settings.cameraHeight);
        }
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

        std::cout << "ptcamera_tracker started\n";
        std::cout << "Model: " << settings.modelPath << '\n';
        std::cout << "Inference device: " << settings.inferenceDevice << '\n';
        std::cout << "Camera index: " << settings.cameraIndex << '\n';
        std::cout << "Motor: " << (motorEnabled ? "enabled" : "disabled") << " (space toggles)\n";
        std::cout << "Manual: " << (manualMode ? "enabled" : "disabled") << " (m toggles, arrows move 1 deg)\n";
        std::cout << "Radar relay: UDP 127.0.0.1:9999\n";

        cv::namedWindow("ptcamera_tracker", cv::WINDOW_NORMAL);

        auto last = std::chrono::steady_clock::now();
        auto nextSendAllowed = std::chrono::steady_clock::now();
        double fps = 0.0;
        std::vector<std::string> statusLines;
        std::string relayBuf;

        while (true) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "Failed to read frame\n";
                break;
            }
            if (settings.flipFrame) {
                cv::flip(frame, frame, 1);
            }

            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last).count();
            last = now;
            if (dt > 0.0) {
                fps = fps <= 0.0 ? 1.0 / dt : 0.9 * fps + 0.1 * (1.0 / dt);
            }

            if (serial.isOpen()) {
                std::string chunk;
                serial.readAvailable(chunk);
                if (!chunk.empty()) {
                    relayBuf += chunk;
                    while (true) {
                        const auto nl = relayBuf.find('\n');
                        if (nl == std::string::npos) {
                            break;
                        }
                        std::string line = relayBuf.substr(0, nl);
                        relayBuf.erase(0, nl + 1);
                        if (line.find("[RADAR]") != std::string::npos) {
                            line += '\n';
                            sendto(udpSock, line.c_str(), static_cast<int>(line.size()), 0,
                                   reinterpret_cast<const sockaddr*>(&udpDest), sizeof(udpDest));
                        }
                    }
                }
            }

            auto result = pipeline.update(frame, dt);
            statusLines.clear();

            ptcamera::ControlTelemetry telemetry;
            if (result.target) {
                telemetry.targetFound = true;
                telemetry.source      = result.target->source;
                const float fw2 = static_cast<float>(frame.cols) / 2.0f;
                const float fh2 = static_cast<float>(frame.rows) / 2.0f;
                telemetry.errorX = (result.target->center.x - fw2) / fw2 * 160.0f;
                telemetry.errorY = (result.target->center.y - fh2) / fh2 * 120.0f;
            }

            if (motorEnabled) {
                if (now >= nextSendAllowed && telemetry.targetFound) {
                    if (!serial.isOpen()) {
                        statusLines.push_back("serial not open, motor off");
                    } else {
                        std::string errMsg;
                        const int ex = static_cast<int>(telemetry.errorX);
                        const int ey = static_cast<int>(telemetry.errorY);
                        if (!serial.sendBBox(ex, ey, &errMsg)) {
                            statusLines.push_back(errMsg);
                        } else {
                            nextSendAllowed =
                                now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                          std::chrono::duration<double>(settings.sendInterval));
                        }
                    }
                }
            } else {
                statusLines.push_back("press space to enable motor");
            }

            drawOverlay(frame,
                        result.target,
                        telemetry,
                        static_cast<int>(result.detections.size()),
                        fps,
                        motorEnabled,
                        statusLines);

            cv::imshow("ptcamera_tracker", frame);

            {
                static cv::Mat streamBuf;
                static std::vector<uchar> jpegBuf;
                static const std::vector<int> jpegParams{cv::IMWRITE_JPEG_QUALITY, 60};
                cv::resize(frame, streamBuf, cv::Size(640, 360));
                if (cv::imencode(".jpg", streamBuf, jpegBuf, jpegParams) &&
                    jpegBuf.size() <= 65000u) {
                    sendto(camSock,
                           reinterpret_cast<const char*>(jpegBuf.data()),
                           static_cast<int>(jpegBuf.size()), 0,
                           reinterpret_cast<const sockaddr*>(&camDest), sizeof(camDest));
                }
            }

            {
                float cx = 0.0f;
                float cy = 0.0f;
                float conf = 0.0f;
                if (result.target) {
                    cx   = result.target->center.x;
                    cy   = result.target->center.y;
                    conf = result.target->confidence;
                }
                char telemJson[256];
                const int n = static_cast<int>(std::snprintf(
                    telemJson,
                    sizeof(telemJson),
                    "{\"motor_enabled\":%s,\"target_found\":%s,"
                    "\"bbox_ex\":%d,\"bbox_ey\":%d,"
                    "\"center_x\":%.1f,\"center_y\":%.1f,"
                    "\"confidence\":%.3f,\"fps\":%.1f}",
                    motorEnabled ? "true" : "false",
                    telemetry.targetFound ? "true" : "false",
                    static_cast<int>(telemetry.errorX),
                    static_cast<int>(telemetry.errorY),
                    cx, cy, conf, static_cast<float>(fps)));
                if (n > 0 && n < static_cast<int>(sizeof(telemJson))) {
                    sendto(telemSock, telemJson, n, 0,
                           reinterpret_cast<const sockaddr*>(&telemDest),
                           sizeof(telemDest));
                }
            }

            const int key = cv::waitKeyEx(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
            if (key == ' ') {
                motorEnabled = !motorEnabled;
                if (motorEnabled && !serial.isOpen()) {
                    motorEnabled = ensureSerialOpen(serial, settings);
                }
                std::cout << "Motor " << (motorEnabled ? "enabled" : "disabled") << '\n';
            }
            if (key == 'm' || key == 'M') {
                manualMode = !manualMode;
                if (serial.isOpen()) {
                    serial.sendManualMode(manualMode);
                }
                if (manualMode) {
                    motorEnabled = false;
                }
                std::cout << "Manual mode "
                          << (manualMode ? "ON (arrows: 1deg)" : "OFF") << '\n';
            }

            if (manualMode && serial.isOpen()) {
                constexpr double degNormal = 1.0;
                if (key == 2424832) {
                    serial.sendPanDegrees(-degNormal);
                    std::cout << "Manual: pan -1 deg\n";
                } else if (key == 2555904) {
                    serial.sendPanDegrees(+degNormal);
                    std::cout << "Manual: pan +1 deg\n";
                } else if (key == 2490368) {
                    serial.sendTiltDegrees(+degNormal);
                    std::cout << "Manual: tilt +1 deg\n";
                } else if (key == 2621440) {
                    serial.sendTiltDegrees(-degNormal);
                    std::cout << "Manual: tilt -1 deg\n";
                }
            }
        }

#ifdef _WIN32
        if (udpSock != INVALID_SOCKET) {
            closesocket(udpSock);
        }
        if (camSock != INVALID_SOCKET) {
            closesocket(camSock);
        }
        if (telemSock != INVALID_SOCKET) {
            closesocket(telemSock);
        }
        WSACleanup();
#else
        if (udpSock >= 0) {
            ::close(udpSock);
        }
        if (camSock >= 0) {
            ::close(camSock);
        }
        if (telemSock >= 0) {
            ::close(telemSock);
        }
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
