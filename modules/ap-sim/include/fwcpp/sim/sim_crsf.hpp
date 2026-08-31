#pragma once

// Port of libraries/SITL/SIM_CRSF.{h,cpp}. Original cycles three hardcoded
// CRSF frames every 400ms (VTX_FRAME, VTX_TELEM, battery 15.8V) while
// draining the autopilot UART into a 64-byte buffer.

#include <cstdint>
#include <cstring>

#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

class CRSF : public SerialDevice {
public:
    enum DataID : std::uint8_t {
        VTX_FRAME = 0x00,
        VTX_TELEM = 0x01,
        VTX_UNKNOWN = 0x02,
        MAX_DATA_FRAMES = 0x03
    };

    char _buffer[64]{};
    std::uint16_t _buflen = 0;
    std::uint8_t _id = 0;
    std::uint32_t _last_update_ms = 0;

    static const char* dataid_string(DataID id, ssize_t& len) {
        switch (id) {
            case DataID::VTX_FRAME: {
                static const std::uint8_t vtx_frame[] = {0xC8, 0x8, 0xF, 0xCE, 0x30, 0x8, 0x16, 0xE9, 0x0, 0x5F};
                len = static_cast<ssize_t>(sizeof(vtx_frame));
                return reinterpret_cast<const char*>(vtx_frame);
            }
            case DataID::VTX_TELEM: {
                static const std::uint8_t vtx_frame[] = {0xC8, 0x7, 0x10, 0xCE, 0xE, 0x16, 0x65, 0x0, 0x1B};
                len = static_cast<ssize_t>(sizeof(vtx_frame));
                return reinterpret_cast<const char*>(vtx_frame);
            }
            case DataID::VTX_UNKNOWN: {
                static const std::uint8_t vtx_frame[] = {0xC8, 0x9, 0x8, 0x0, 0x9E, 0x0, 0x0, 0x0, 0x0, 0x0, 0x95};
                len = static_cast<ssize_t>(sizeof(vtx_frame));
                return reinterpret_cast<const char*>(vtx_frame);
            }
            default:
                break;
        }
        len = 0;
        return "UNKNOWN";
    }

    void update(std::uint32_t now_ms) {
        const ssize_t n = read_from_autopilot(&_buffer[_buflen], sizeof(_buffer) - _buflen - 1);
        if (n > 0) {
            _buflen = static_cast<std::uint16_t>(_buflen + n);
        }

        if (now_ms - _last_update_ms < 400) {
            return;
        }
        _last_update_ms = now_ms;

        ssize_t len = 0;
        ssize_t index = 0;
        const char* bytes = dataid_string(static_cast<DataID>(_id), len);
        while (len > 0) {
            const ssize_t nwrite = write_to_autopilot(&bytes[index], static_cast<std::size_t>(len));
            if (nwrite <= 0) {
                break;
            }
            len -= nwrite;
            index += nwrite;
        }
        _id = static_cast<std::uint8_t>((_id + 1) % MAX_DATA_FRAMES);
    }
};

}  // namespace fwcpp::sim
