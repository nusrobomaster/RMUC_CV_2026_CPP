// protocol_data.hpp
#pragma once
#include <cstdint>
#include <cstddef>

struct Protocol {
    // MUST match STM32 usb_task.h
    enum class Type : std::uint8_t {
        aimbot = 0xA1,   // yaw, pitch, fire (3 floats)
        nav    = 0xA2    // vx, vy, vz (3 floats) - optional if you need later
    };

    struct AimbotData {
        float yaw;
        float pitch;
        float fire;  // IMPORTANT: float, because STM32 parses 3 floats (len==12)
    };

    struct NavData {
        float vx;
        float vy;
        float vz;
    };

    static constexpr std::uint8_t HEADER_BYTE = 0xAA;     // MUST match STM32 USB_MAGIC_BYTE
    static constexpr std::size_t  MAX_PACKET_SIZE = 256;
    static constexpr std::size_t  MAX_PAYLOAD_SIZE = 240; // MUST match STM32 USB_MAX_PAYLOAD_SIZE
    static constexpr std::size_t  CRC_SIZE = 2;
    static constexpr std::size_t  HEADER_SIZE = 4;
};