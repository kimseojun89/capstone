#include "ptcamera/serial_port.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <cmath>
#include <sstream>
#include <cstring>

namespace ptcamera {

static std::string win32Err(const std::string& prefix) {
    DWORD e = GetLastError();
    char buf[256] = {};
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, e, 0, buf, static_cast<DWORD>(sizeof(buf)), nullptr);
    for (int i = static_cast<int>(strlen(buf)) - 1;
         i >= 0 && (buf[i] == '\r' || buf[i] == '\n'); --i) buf[i] = '\0';
    return prefix + ": " + buf;
}

SerialPort::~SerialPort() { closePort(); }

bool SerialPort::openPort(const std::string& port, int baudRate, std::string* error) {
    closePort();
    // COM10 이상을 위한 "\\.\" 접두사 처리
    std::string fp = (port.rfind("\\\\.\\", 0) == 0) ? port : "\\\\.\\" + port;
    HANDLE h = CreateFileA(fp.c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0, nullptr, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        if (error) *error = win32Err("CreateFile " + port);
        return false;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        if (error) *error = win32Err("GetCommState");
        CloseHandle(h); return false;
    }
    dcb.BaudRate       = static_cast<DWORD>(baudRate);
    dcb.ByteSize       = 8;
    dcb.Parity         = NOPARITY;
    dcb.StopBits       = ONESTOPBIT;
    dcb.fBinary        = TRUE;
    dcb.fParity        = FALSE;
    dcb.fOutxCtsFlow   = FALSE;
    dcb.fOutxDsrFlow   = FALSE;
    dcb.fDtrControl    = DTR_CONTROL_DISABLE;
    dcb.fRtsControl    = RTS_CONTROL_DISABLE;
    dcb.fOutX          = FALSE;
    dcb.fInX           = FALSE;
    if (!SetCommState(h, &dcb)) {
        if (error) *error = win32Err("SetCommState");
        CloseHandle(h); return false;
    }

    // 논블로킹 읽기 (VMIN=0, VTIME=0 에 해당)
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout         = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = 100;  // 쓰기 최대 100ms
    if (!SetCommTimeouts(h, &timeouts)) {
        if (error) *error = win32Err("SetCommTimeouts");
        CloseHandle(h); return false;
    }

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    handle_ = reinterpret_cast<void*>(h);
    return true;
}

void SerialPort::closePort() {
    HANDLE h = reinterpret_cast<HANDLE>(handle_);
    if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
    handle_ = reinterpret_cast<void*>(INVALID_HANDLE_VALUE);
}

bool SerialPort::writeLine(const std::string& line, std::string* error) {
    HANDLE h = reinterpret_cast<HANDLE>(handle_);
    if (h == INVALID_HANDLE_VALUE) {
        if (error) *error = "serial port is not open";
        return false;
    }
    const char* data = line.data();
    DWORD rem = static_cast<DWORD>(line.size());
    while (rem > 0) {
        DWORD written = 0;
        if (!WriteFile(h, data, rem, &written, nullptr)) {
            if (error) *error = win32Err("WriteFile");
            return false;
        }
        data += written;
        rem  -= written;
    }
    return true;
}

bool SerialPort::readLine(std::string& line, int timeoutMs, std::string* error) {
    line.clear();
    HANDLE h = reinterpret_cast<HANDLE>(handle_);
    if (h == INVALID_HANDLE_VALUE) {
        if (error) *error = "serial port is not open";
        return false;
    }
    int elapsed = 0;
    while (elapsed <= timeoutMs) {
        char c = '\0';
        DWORD got = 0;
        if (!ReadFile(h, &c, 1, &got, nullptr)) {
            if (error) *error = win32Err("ReadFile");
            return false;
        }
        if (got == 0) { Sleep(1); elapsed += 1; continue; }
        if (c == '\r') continue;
        if (c == '\n') return true;
        line.push_back(c);
    }
    return !line.empty();
}

bool SerialPort::sendTiltCommand(int tiltSteps, std::string* error) {
    std::ostringstream ss;
    ss << "T:" << tiltSteps << '\n';
    return writeLine(ss.str(), error);
}

bool SerialPort::sendPanCommand(int panSteps, std::string* error) {
    std::ostringstream ss;
    ss << "P:" << panSteps << '\n';
    return writeLine(ss.str(), error);
}

bool SerialPort::sendBBox(int ex, int ey, std::string* error) {
    std::ostringstream ss;
    ss << "B:" << ex << ',' << ey << '\n';
    return writeLine(ss.str(), error);
}

bool SerialPort::sendManualMode(bool enable, std::string* error) {
    return writeLine(enable ? "M:1\n" : "M:0\n", error);
}

bool SerialPort::sendPanDegrees(double degrees, std::string* error) {
    int steps = static_cast<int>(std::round(degrees * 4096.0 / 360.0));
    return sendPanCommand(steps, error);
}

bool SerialPort::sendTiltDegrees(double degrees, std::string* error) {
    int steps = static_cast<int>(std::round(degrees * 4096.0 / 360.0));
    return sendTiltCommand(steps, error);
}

bool SerialPort::readAvailable(std::string& buf) {
    buf.clear();
    HANDLE h = reinterpret_cast<HANDLE>(handle_);
    if (h == INVALID_HANDLE_VALUE) return false;
    char tmp[256];
    DWORD got = 0;
    while (ReadFile(h, tmp, sizeof(tmp), &got, nullptr) && got > 0)
        buf.append(tmp, static_cast<size_t>(got));
    return true;
}

}  // namespace ptcamera
