#pragma once

#include "ptcamera/types.hpp"

#include <string>

namespace ptcamera {

class SerialPort {
public:
    SerialPort() = default;
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    ~SerialPort();

    bool openPort(const std::string& port, int baudRate, std::string* error = nullptr);
    void closePort();
    bool isOpen() const {
#ifdef _WIN32
        return handle_ != reinterpret_cast<void*>(-1);
#else
        return fd_ >= 0;
#endif
    }

    bool writeLine(const std::string& line, std::string* error = nullptr);
    bool readLine(std::string& line, int timeoutMs, std::string* error = nullptr);
    bool readAvailable(std::string& buf);  // non-blocking drain of RX buffer

    bool sendTiltCommand(int tiltSteps, std::string* error = nullptr);
    bool sendPanCommand(int panSteps,   std::string* error = nullptr);

    // AI 추적 모드: bbox 중심 오차 (320×240 기준 스케일) → FPGA PID
    bool sendBBox(int ex, int ey, std::string* error = nullptr);

    bool sendManualMode(bool enable, std::string* error = nullptr);
    bool sendPanDegrees(double degrees, std::string* error = nullptr);
    bool sendTiltDegrees(double degrees, std::string* error = nullptr);

private:
    void* handle_ = reinterpret_cast<void*>(-1);  // HANDLE; windows.h 없이 저장
};

}  // namespace ptcamera
