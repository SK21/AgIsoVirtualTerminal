//=============================================================================
/// @file slcan_interface.cpp
/// @brief SLCAN (Serial Line CAN) driver implementation
//=============================================================================

#include "slcan_interface.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include <cstring>
#include <sstream>
#include <iomanip>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#endif

namespace aog {

SLCANInterface::SLCANInterface(const std::string& portName,
                               uint32_t baudRate,
                               uint32_t canBitrate)
    : m_portName(portName)
    , m_serialBaudRate(baudRate)
    , m_canBitrate(canBitrate)
{
}

SLCANInterface::~SLCANInterface()
{
    close();
}

std::string SLCANInterface::get_name() const
{
    return "SLCAN (" + m_portName + ")";
}

bool SLCANInterface::get_is_valid() const
{
    return m_isValid;
}

void SLCANInterface::set_port_name(const std::string& port)
{
    m_portName = port;
}

void SLCANInterface::open()
{
    if (m_isOpen) {
        return;
    }

#ifdef _WIN32
    // Open serial port on Windows
    std::string fullPortName = "\\\\.\\" + m_portName;
    m_serialHandle = CreateFileA(
        fullPortName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        0,
        nullptr
    );

    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        isobus::CANStackLogger::error("SLCAN: Failed to open " + m_portName + " - error " + std::to_string(GetLastError()));
        return;
    }

    // Configure serial port
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(m_serialHandle, &dcb)) {
        isobus::CANStackLogger::error("SLCAN: Failed to get comm state");
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
        return;
    }

    dcb.BaudRate = m_serialBaudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState(m_serialHandle, &dcb)) {
        isobus::CANStackLogger::error("SLCAN: Failed to set comm state");
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
        return;
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;

    if (!SetCommTimeouts(m_serialHandle, &timeouts)) {
        isobus::CANStackLogger::error("SLCAN: Failed to set timeouts");
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
        return;
    }

    // Clear buffers
    PurgeComm(m_serialHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);

#else
    // Linux serial port
    m_serialFd = ::open(m_portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_serialFd < 0) {
        isobus::CANStackLogger::error("SLCAN: Failed to open " + m_portName);
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(m_serialFd, &tty) != 0) {
        isobus::CANStackLogger::error("SLCAN: Failed to get termios");
        ::close(m_serialFd);
        m_serialFd = -1;
        return;
    }

    // Set baud rate
    speed_t speed = B115200;
    if (m_serialBaudRate == 9600) speed = B9600;
    else if (m_serialBaudRate == 19200) speed = B19200;
    else if (m_serialBaudRate == 38400) speed = B38400;
    else if (m_serialBaudRate == 57600) speed = B57600;
    else if (m_serialBaudRate == 115200) speed = B115200;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(m_serialFd, TCSANOW, &tty) != 0) {
        isobus::CANStackLogger::error("SLCAN: Failed to set termios");
        ::close(m_serialFd);
        m_serialFd = -1;
        return;
    }

    tcflush(m_serialFd, TCIOFLUSH);
#endif

    m_isOpen = true;

    // Initialize slcan adapter
    // First close any existing CAN connection
    sendCommand("C\r");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Set CAN bitrate
    char bitrateCmd[4] = {'S', getBitrateCommand(m_canBitrate), '\r', '\0'};
    if (!sendCommand(bitrateCmd)) {
        isobus::CANStackLogger::error("SLCAN: Failed to set bitrate");
        close();
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Open CAN channel
    if (!sendCommand("O\r")) {
        isobus::CANStackLogger::error("SLCAN: Failed to open CAN channel");
        close();
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Start receive thread
    m_rxThreadRunning = true;
    m_rxThread = std::thread(&SLCANInterface::receiveThreadFunc, this);

    m_isValid = true;
    isobus::CANStackLogger::info("SLCAN: Opened " + m_portName + " at " + std::to_string(m_canBitrate) + " bps");
}

void SLCANInterface::close()
{
    if (!m_isOpen) {
        return;
    }

    m_isValid = false;
    m_rxThreadRunning = false;

    if (m_rxThread.joinable()) {
        m_rxThread.join();
    }

    // Close CAN channel
    sendCommand("C\r");

#ifdef _WIN32
    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
    }
#else
    if (m_serialFd >= 0) {
        ::close(m_serialFd);
        m_serialFd = -1;
    }
#endif

    m_isOpen = false;
    isobus::CANStackLogger::info("SLCAN: Closed " + m_portName);
}

char SLCANInterface::getBitrateCommand(uint32_t bitrate)
{
    // slcan bitrate codes
    switch (bitrate) {
        case 10000:   return '0';
        case 20000:   return '1';
        case 50000:   return '2';
        case 100000:  return '3';
        case 125000:  return '4';
        case 250000:  return '5';  // ISOBUS
        case 500000:  return '6';
        case 800000:  return '7';
        case 1000000: return '8';
        default:      return '5';  // Default to 250k
    }
}

bool SLCANInterface::sendCommand(const std::string& cmd)
{
#ifdef _WIN32
    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        return false;
    }

    DWORD written = 0;
    if (!WriteFile(m_serialHandle, cmd.c_str(), (DWORD)cmd.length(), &written, nullptr)) {
        return false;
    }
    return written == cmd.length();
#else
    if (m_serialFd < 0) {
        return false;
    }
    ssize_t written = write(m_serialFd, cmd.c_str(), cmd.length());
    return written == (ssize_t)cmd.length();
#endif
}

int SLCANInterface::readData(char* buffer, size_t maxLen, uint32_t timeoutMs)
{
#ifdef _WIN32
    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        return -1;
    }

    DWORD bytesRead = 0;
    if (!ReadFile(m_serialHandle, buffer, (DWORD)maxLen, &bytesRead, nullptr)) {
        return -1;
    }
    return (int)bytesRead;
#else
    if (m_serialFd < 0) {
        return -1;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_serialFd, &fds);

    struct timeval tv;
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;

    int ret = select(m_serialFd + 1, &fds, nullptr, nullptr, &tv);
    if (ret > 0) {
        return read(m_serialFd, buffer, maxLen);
    }
    return 0;
#endif
}

bool SLCANInterface::read_frame(isobus::CANMessageFrame& canFrame)
{
    std::lock_guard<std::mutex> lock(m_rxMutex);

    if (m_rxQueue.empty()) {
        return false;
    }

    canFrame = m_rxQueue.front();
    m_rxQueue.pop();
    return true;
}

bool SLCANInterface::write_frame(const isobus::CANMessageFrame& canFrame)
{
    if (!m_isValid) {
        return false;
    }

    std::string encoded = encodeFrame(canFrame);
    bool result = sendCommand(encoded);

    if (result) {
        isobus::CANStackLogger::debug("SLCAN TX: " + encoded.substr(0, encoded.length() - 1));  // Remove \r for logging
    } else {
        isobus::CANStackLogger::warn("SLCAN TX failed");
    }

    return result;
}

void SLCANInterface::receiveThreadFunc()
{
    char tempBuffer[64];

    while (m_rxThreadRunning) {
        int bytesRead = readData(tempBuffer, sizeof(tempBuffer), 10);

        if (bytesRead > 0) {
            for (int i = 0; i < bytesRead; i++) {
                char c = tempBuffer[i];

                if (c == '\r' || c == '\n') {
                    // End of frame
                    if (m_rxBufferPos > 0) {
                        m_rxBuffer[m_rxBufferPos] = '\0';

                        isobus::CANMessageFrame frame;
                        if (parseFrame(m_rxBuffer, m_rxBufferPos, frame)) {
                            std::lock_guard<std::mutex> lock(m_rxMutex);
                            m_rxQueue.push(frame);
                        }
                        m_rxBufferPos = 0;
                    }
                } else if (m_rxBufferPos < sizeof(m_rxBuffer) - 1) {
                    m_rxBuffer[m_rxBufferPos++] = c;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

bool SLCANInterface::parseFrame(const char* data, size_t len, isobus::CANMessageFrame& frame)
{
    if (len < 4) {
        return false;
    }

    frame.channel = 0;
    frame.timestamp_us = 0;

    if (data[0] == 't') {
        // Standard frame: tIIILDD...
        if (len < 5) return false;

        frame.isExtendedFrame = false;

        // Parse 3-digit hex ID
        frame.identifier = (hexCharToNibble(data[1]) << 8) |
                           (hexCharToNibble(data[2]) << 4) |
                           hexCharToNibble(data[3]);

        // Length
        frame.dataLength = hexCharToNibble(data[4]);
        if (frame.dataLength > 8) return false;

        // Data bytes
        for (size_t i = 0; i < frame.dataLength; i++) {
            size_t pos = 5 + i * 2;
            if (pos + 1 >= len) return false;
            frame.data[i] = (hexCharToNibble(data[pos]) << 4) |
                            hexCharToNibble(data[pos + 1]);
        }

        return true;

    } else if (data[0] == 'T') {
        // Extended frame: TIIIIIIIILDD...
        if (len < 10) return false;

        frame.isExtendedFrame = true;

        // Parse 8-digit hex ID
        frame.identifier = 0;
        for (int i = 0; i < 8; i++) {
            frame.identifier = (frame.identifier << 4) | hexCharToNibble(data[1 + i]);
        }

        // Length
        frame.dataLength = hexCharToNibble(data[9]);
        if (frame.dataLength > 8) return false;

        // Data bytes
        for (size_t i = 0; i < frame.dataLength; i++) {
            size_t pos = 10 + i * 2;
            if (pos + 1 >= len) return false;
            frame.data[i] = (hexCharToNibble(data[pos]) << 4) |
                            hexCharToNibble(data[pos + 1]);
        }

        return true;
    }

    return false;  // Unknown frame type
}

std::string SLCANInterface::encodeFrame(const isobus::CANMessageFrame& frame)
{
    std::ostringstream ss;

    if (frame.isExtendedFrame) {
        // Extended frame: TIIIIIIIILDD...
        ss << 'T';
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << frame.identifier;
        ss << static_cast<int>(frame.dataLength);  // Cast to int to output as digit, not char
    } else {
        // Standard frame: tIIILDD...
        ss << 't';
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(3) << frame.identifier;
        ss << static_cast<int>(frame.dataLength);  // Cast to int to output as digit, not char
    }

    // Data bytes
    for (size_t i = 0; i < frame.dataLength; i++) {
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
           << static_cast<int>(frame.data[i]);
    }

    ss << '\r';
    return ss.str();
}

uint8_t SLCANInterface::hexCharToNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

char SLCANInterface::nibbleToHexChar(uint8_t n)
{
    if (n < 10) return '0' + n;
    return 'A' + n - 10;
}

} // namespace aog
