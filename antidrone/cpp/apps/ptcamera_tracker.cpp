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
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <unistd.h>
#endif

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
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
                 "                        [--serial-port COM3] [--baud 256000]\n"
                 "                        [--enable-motor] [--conf 0.25] [--send-interval 0.12]\n";
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
        ptcamera::SerialPort serial;

        // Always open serial — needed for radar relay even when motor is off
        if (!ensureSerialOpen(serial, settings)) {
            std::cerr << "Serial not open — motor disabled, no radar relay\n";
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

        // Camera frame stream -> unified GUI (UDP 9998)
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

        // Telemetry -> unified GUI (UDP 10000, JSON)
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
        std::cout << "Radar relay: UDP 127.0.0.1:9999\n";

        cv::namedWindow("ptcamera_tracker", cv::WINDOW_NORMAL);

        auto last            = std::chrono::steady_clock::now();
        auto nextSendAllowed = std::chrono::steady_clock::now();  // 처음부터 전송 가능
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

            // Drain FPGA UART0 output, relay [RADAR] lines to ppi_viewer via UDP
            if (serial.isOpen()) {
                std::string chunk;
                serial.readAvailable(chunk);
                if (!chunk.empty()) {
                    relayBuf += chunk;
                    while (true) {
                        const auto nl = relayBuf.find('\n');
                        if (nl == std::string::npos) break;
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
            auto telemetry = control.update(result.target, frame.size(), dt);
            statusLines.clear();

            if (motorEnabled) {
                // Move & Re-detect: 이동 완료 시간 기반 동적 쿨다운
                // - 명령 전송 후 모터 실제 이동 시간 + 1프레임 안정화 대기
                // - 대기 중엔 PID 상태도 리셋하여 다음 명령은 최신 bbox 기준 fresh start
                if (now >= nextSendAllowed &&
                    (telemetry.command.panSteps != 0 || telemetry.command.tiltSteps != 0)) {
                    if (!serial.isOpen()) {
                        statusLines.push_back("serial not open, motor off");
                    } else {
                        std::string errMsg;
                        bool ok = true;
                        if (telemetry.targetFound && telemetry.command.panSteps != 0) {
                            ok = serial.sendPanCommand(telemetry.command.panSteps, &errMsg);
                        }
                        if (ok) {
                            ok = serial.sendTiltCommand(telemetry.command.tiltSteps, &errMsg);
                        }
                        if (ok) {
                            // 동적 쿨다운: max(이동시간 + 안정화, sendInterval 최솟값)
                            // MOTOR_SPEED_DELAY=200 → 0.57ms/step
                            constexpr double STEP_SEC   = 0.00057;
                            constexpr double STABLE_SEC = 0.033;   // 1프레임 안정화 여유
                            const double moveSec =
                                std::abs(telemetry.command.panSteps) * STEP_SEC;
                            const double cooldown =
                                std::max(moveSec + STABLE_SEC, settings.sendInterval);
                            nextSendAllowed =
                                now + std::chrono::duration<double>(cooldown);
                            control.reset();  // PID 리셋: 다음 명령은 최신 bbox 기준으로 계산
                        } else {
                            statusLines.push_back(errMsg);
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

            // Stream annotated frame to unified GUI (640x360 JPEG, UDP 9998)
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

            // Telemetry JSON → unified GUI (UDP 10000)
            {
                float cx = 0.0f, cy = 0.0f, conf = 0.0f;
                if (result.target) {
                    cx   = result.target->center.x;
                    cy   = result.target->center.y;
                    conf = result.target->confidence;
                }
                char telemJson[256];
                const int n = snprintf(telemJson, sizeof(telemJson),
                    "{\"motor_enabled\":%s,\"target_found\":%s,"
                    "\"pan_steps\":%d,\"tilt_steps\":%d,"
                    "\"center_x\":%.1f,\"center_y\":%.1f,"
                    "\"confidence\":%.3f,\"fps\":%.1f}",
                    motorEnabled ? "true" : "false",
                    telemetry.targetFound ? "true" : "false",
                    telemetry.command.panSteps, telemetry.command.tiltSteps,
                    cx, cy, conf, static_cast<float>(fps));
                if (n > 0 && n < static_cast<int>(sizeof(telemJson))) {
                    sendto(telemSock, telemJson, n, 0,
                           reinterpret_cast<const sockaddr*>(&telemDest),
                           sizeof(telemDest));
                }
            }

            const int key = cv::waitKey(1) & 0xff;
            if (key == 27 || key == 'q') {
                break;
            }
            if (key == ' ') {
                motorEnabled = !motorEnabled;
                control.reset();
                if (motorEnabled && !serial.isOpen()) {
                    motorEnabled = ensureSerialOpen(serial, settings);
                }
                std::cout << "Motor " << (motorEnabled ? "enabled" : "disabled") << '\n';
            }
        }

#ifdef _WIN32
        if (udpSock   != INVALID_SOCKET) closesocket(udpSock);
        if (camSock   != INVALID_SOCKET) closesocket(camSock);
        if (telemSock != INVALID_SOCKET) closesocket(telemSock);
        WSACleanup();
#else
        if (udpSock   >= 0) ::close(udpSock);
        if (camSock   >= 0) ::close(camSock);
        if (telemSock >= 0) ::close(telemSock);
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
