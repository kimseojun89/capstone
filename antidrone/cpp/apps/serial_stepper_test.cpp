#include "ptcamera/serial_port.hpp"
#include "ptcamera/settings.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

int readIntArg(int argc, char** argv, int& index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return std::atoi(argv[++index]);
}

std::string readStringArg(int argc, char** argv, int& index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return argv[++index];
}

void printUsage() {
    std::cout << "Usage: serial_stepper_test --port /dev/ttyACM0 [--baud 115200] "
                 "[--pan 200] [--tilt 0] [--step-delay-us 2000] [--coil-order 0]\n";
}

}  // namespace

int main(int argc, char** argv) {
    ptcamera::TrackerSettings settings = ptcamera::defaultSettings();
    int pan = 0;
    int tilt = 0;
    bool hasPort = false;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--port") {
                settings.serialPort = readStringArg(argc, argv, i);
                hasPort = true;
            } else if (arg == "--baud") {
                settings.serialBaud = readIntArg(argc, argv, i);
            } else if (arg == "--pan") {
                pan = readIntArg(argc, argv, i);
            } else if (arg == "--tilt") {
                tilt = readIntArg(argc, argv, i);
            } else if (arg == "--step-delay-us") {
                settings.stepDelayUs = readIntArg(argc, argv, i);
            } else if (arg == "--coil-order") {
                settings.coilOrder = readIntArg(argc, argv, i);
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

    if (!hasPort) {
        std::cerr << "Missing --port\n";
        printUsage();
        return 2;
    }

    ptcamera::SerialPort serial;
    std::string error;
    if (!serial.openPort(settings.serialPort, settings.serialBaud, &error)) {
        std::cerr << error << '\n';
        return 1;
    }

    std::cout << "Opened " << settings.serialPort << ". Waiting for Arduino reset...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::string line;
    while (serial.readLine(line, 100, nullptr)) {
        if (!line.empty()) {
            std::cout << line << '\n';
        }
    }

    const ptcamera::ControlCommand command {pan, tilt};
    if (!serial.sendStepperCommand(command, settings.stepDelayUs, settings.coilOrder, &error)) {
        std::cerr << error << '\n';
        return 1;
    }
    std::cout << "SEND " << pan << ',' << tilt << ',' << settings.stepDelayUs << ',' << settings.coilOrder << '\n';

    const int maxSteps = std::max(std::abs(pan), std::abs(tilt));
    const auto expectedMoveTime = std::chrono::microseconds(
        static_cast<long long>(maxSteps) * static_cast<long long>(settings.stepDelayUs));
    const auto deadline = std::chrono::steady_clock::now() + expectedMoveTime + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        if (serial.readLine(line, 250, &error) && !line.empty()) {
            std::cout << line << '\n';
            if (line.rfind("OK ", 0) == 0 || line.rfind("ERR ", 0) == 0) {
                break;
            }
        }
    }

    return 0;
}
