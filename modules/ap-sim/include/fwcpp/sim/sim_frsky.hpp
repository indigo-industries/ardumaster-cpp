#pragma once

// Port of libraries/SITL/SIM_Frsky.{h,cpp} and SIM_Frsky_D.{h,cpp}.
// Original Frsky_D is a receive-side D-telemetry parser: START_STOP_D 0x5E,
// byte-stuff 0x5D -> 0x3E/0x3D, two-byte little-endian payload. It does not
// emit packets toward the autopilot.

#include <cstdint>
#include <cstring>
#include <vector>

#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

class Frsky : public SerialDevice {
public:
    enum class DataID : std::uint8_t {
        GPS_ALT_BP = 0x01,
        TEMP1 = 0x02,
        FUEL = 0x04,
        TEMP2 = 0x05,
        GPS_ALT_AP = 0x09,
        BARO_ALT_BP = 0x10,
        GPS_SPEED_BP = 0x11,
        GPS_LONG_BP = 0x12,
        GPS_LAT_BP = 0x13,
        GPS_COURS_BP = 0x14,
        GPS_SPEED_AP = 0x19,
        GPS_LONG_AP = 0x1A,
        GPS_LAT_AP = 0x1B,
        BARO_ALT_AP = 0x21,
        GPS_LONG_EW = 0x22,
        GPS_LAT_NS = 0x23,
        CURRENT = 0x28,
        VFAS = 0x39,
    };

    static const char* dataid_string(DataID id) {
        switch (id) {
            case DataID::GPS_ALT_BP: return "GPS_ALT_BP";
            case DataID::TEMP1: return "TEMP1";
            case DataID::FUEL: return "FUEL";
            case DataID::TEMP2: return "TEMP2";
            case DataID::GPS_ALT_AP: return "GPS_ALT_AP";
            case DataID::BARO_ALT_BP: return "BARO_ALT_BP";
            case DataID::GPS_SPEED_BP: return "GPS_SPEED_BP";
            case DataID::GPS_LONG_BP: return "GPS_LONG_BP";
            case DataID::GPS_LAT_BP: return "GPS_LAT_BP";
            case DataID::GPS_COURS_BP: return "GPS_COURS_BP";
            case DataID::GPS_SPEED_AP: return "GPS_SPEED_AP";
            case DataID::GPS_LONG_AP: return "GPS_LONG_AP";
            case DataID::GPS_LAT_AP: return "GPS_LAT_AP";
            case DataID::BARO_ALT_AP: return "BARO_ALT_AP";
            case DataID::GPS_LONG_EW: return "GPS_LONG_EW";
            case DataID::GPS_LAT_NS: return "GPS_LAT_NS";
            case DataID::CURRENT: return "CURRENT";
            case DataID::VFAS: return "VFAS";
        }
        return "UNKNOWN";
    }
};

class Frsky_D : public Frsky {
public:
    static constexpr std::uint8_t START_STOP_D = 0x5E;
    static constexpr std::uint8_t BYTESTUFF_D = 0x5D;

    struct Sample {
        std::uint8_t id = 0;
        std::uint16_t data = 0;
    };

    enum class State : std::uint8_t {
        WANT_START_STOP_D = 16,
        WANT_ID = 17,
        WANT_BYTE1 = 18,
        WANT_BYTE2 = 19,
    };

    State _state = State::WANT_START_STOP_D;
    char _buffer[32]{};
    std::uint16_t _buflen = 0;
    std::uint8_t _id = 0;
    std::uint16_t _data = 0;
    std::vector<Sample> received;

    void handle_data(std::uint8_t id, std::uint16_t data) {
        received.push_back(Sample{id, data});
    }

    void update() {
        const ssize_t n = read_from_autopilot(&_buffer[_buflen], sizeof(_buffer) - _buflen - 1);
        if (n > 0) {
            _buflen = static_cast<std::uint16_t>(_buflen + n);
        }
        if (_buflen == 0) {
            return;
        }

        while (_buflen) {
            switch (_state) {
            case State::WANT_START_STOP_D:
                if (static_cast<std::uint8_t>(_buffer[0]) != START_STOP_D) {
                    // Original panics on corrupt; consume the byte so the
                    // state machine can resync instead of spinning.
                    std::memmove(&_buffer[0], &_buffer[1], --_buflen);
                    continue;
                }
                std::memmove(&_buffer[0], &_buffer[1], --_buflen);
                _state = State::WANT_ID;
                break;
            case State::WANT_ID:
                _id = static_cast<std::uint8_t>(_buffer[0]);
                std::memmove(&_buffer[0], &_buffer[1], --_buflen);
                _state = State::WANT_BYTE1;
                break;
            case State::WANT_BYTE1:
            case State::WANT_BYTE2: {
                std::uint8_t byte = 0;
                std::uint8_t consume = 1;
                if (static_cast<std::uint8_t>(_buffer[0]) == BYTESTUFF_D) {
                    if (_buflen < 2) {
                        return;
                    }
                    const std::uint8_t stuffed = static_cast<std::uint8_t>(_buffer[1]);
                    if (stuffed == 0x3E) {
                        byte = START_STOP_D;
                    } else if (stuffed == 0x3D) {
                        byte = BYTESTUFF_D;
                    } else {
                        // Original panics on unknown stuffed byte; drop both.
                        std::memmove(&_buffer[0], &_buffer[2], _buflen - 2);
                        _buflen = static_cast<std::uint16_t>(_buflen - 2);
                        _state = State::WANT_START_STOP_D;
                        continue;
                    }
                    consume = 2;
                } else {
                    byte = static_cast<std::uint8_t>(_buffer[0]);
                }
                std::memmove(&_buffer[0], &_buffer[consume], _buflen - consume);
                _buflen = static_cast<std::uint16_t>(_buflen - consume);
                if (_state == State::WANT_BYTE1) {
                    _data = byte;
                    _state = State::WANT_BYTE2;
                } else {
                    _data = static_cast<std::uint16_t>(_data | (static_cast<std::uint16_t>(byte) << 8));
                    handle_data(_id, _data);
                    _state = State::WANT_START_STOP_D;
                }
                break;
            }
            }
        }
    }
};

}  // namespace fwcpp::sim
