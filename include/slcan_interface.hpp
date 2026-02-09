//=============================================================================
/// @file slcan_interface.hpp
/// @brief SLCAN (Serial Line CAN) driver for AgIsoStack++
/// @details Implements CANHardwarePlugin for slcan-compatible adapters
//=============================================================================

#ifndef SLCAN_INTERFACE_HPP
#define SLCAN_INTERFACE_HPP

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include <string>
#include <atomic>
#include <mutex>
#include <queue>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#endif

namespace aog {

/// @brief SLCAN CAN driver implementation
class SLCANInterface : public isobus::CANHardwarePlugin
{
public:
    /// @brief Constructor
    /// @param portName COM port name (e.g., "COM3" on Windows)
    /// @param baudRate Serial baud rate (typically 115200 or 3000000)
    /// @param canBitrate CAN bus bitrate (250000 for ISOBUS)
    SLCANInterface(const std::string& portName,
                   uint32_t baudRate = 115200,
                   uint32_t canBitrate = 250000);

    ~SLCANInterface() override;

    // CANHardwarePlugin interface
    std::string get_name() const override;
    bool get_is_valid() const override;
    void close() override;
    void open() override;
    bool read_frame(isobus::CANMessageFrame& canFrame) override;
    bool write_frame(const isobus::CANMessageFrame& canFrame) override;

    /// @brief Set the COM port name (call before open())
    void set_port_name(const std::string& port);

private:
    /// @brief Get slcan bitrate command for CAN bitrate
    char getBitrateCommand(uint32_t bitrate);

    /// @brief Send command to slcan adapter
    bool sendCommand(const std::string& cmd);

    /// @brief Read response/data from adapter
    int readData(char* buffer, size_t maxLen, uint32_t timeoutMs = 100);

    /// @brief Parse slcan frame from buffer
    bool parseFrame(const char* data, size_t len, isobus::CANMessageFrame& frame);

    /// @brief Encode frame to slcan format
    std::string encodeFrame(const isobus::CANMessageFrame& frame);

    /// @brief Receive thread function
    void receiveThreadFunc();

    /// @brief Convert hex char to nibble
    static uint8_t hexCharToNibble(char c);

    /// @brief Convert nibble to hex char
    static char nibbleToHexChar(uint8_t n);

    std::string m_portName;
    uint32_t m_serialBaudRate;
    uint32_t m_canBitrate;
    std::atomic<bool> m_isOpen{false};
    std::atomic<bool> m_isValid{false};

    // Receive buffer and queue
    std::mutex m_rxMutex;
    std::queue<isobus::CANMessageFrame> m_rxQueue;
    char m_rxBuffer[256];
    size_t m_rxBufferPos = 0;

    // Receive thread
    std::thread m_rxThread;
    std::atomic<bool> m_rxThreadRunning{false};

#ifdef _WIN32
    HANDLE m_serialHandle = INVALID_HANDLE_VALUE;
#else
    int m_serialFd = -1;
#endif
};

} // namespace aog

#endif // SLCAN_INTERFACE_HPP
