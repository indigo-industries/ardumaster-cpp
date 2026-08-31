#pragma once

// Shared endian / NMEA / CRC-xmodem helpers used by original-source SIM ports.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_crc.hpp>

namespace fwcpp::sim {

inline constexpr float kNteslaToMgauss = 10.0f;

inline std::uint8_t crc_xor_of_bytes(const std::uint8_t* data, std::uint16_t n) {
    std::uint8_t x = 0;
    while (n--) {
        x ^= *data++;
    }
    return x;
}

inline void put_be16(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v >> 8);
    p[1] = static_cast<std::uint8_t>(v);
}
inline void put_be32(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v >> 24);
    p[1] = static_cast<std::uint8_t>(v >> 16);
    p[2] = static_cast<std::uint8_t>(v >> 8);
    p[3] = static_cast<std::uint8_t>(v);
}
inline void put_be64(std::uint8_t* p, std::uint64_t v) {
    put_be32(p, static_cast<std::uint32_t>(v >> 32));
    put_be32(p + 4, static_cast<std::uint32_t>(v));
}
inline void put_le16(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}
inline void put_le32(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
    p[2] = static_cast<std::uint8_t>(v >> 16);
    p[3] = static_cast<std::uint8_t>(v >> 24);
}

inline math::Quaternion quat_from_aircraft(const Aircraft& a) {
    float r = 0, p = 0, y = 0;
    a.get_dcm().to_euler(&r, &p, &y);
    math::Quaternion q;
    q.from_euler(r, p, y);
    return q;
}

inline std::string nmea_checksum_line(const char* body) {
    std::uint8_t cs = 0;
    for (const char* p = body; *p; ++p) {
        if (*p != '$' && *p != '*') {
            cs ^= static_cast<std::uint8_t>(*p);
        }
        if (*p == '*') {
            break;
        }
    }
    char out[256];
    const int n = std::snprintf(out, sizeof(out), "%s*%02X\r\n", body, cs);
    return n > 0 ? std::string(out, static_cast<std::size_t>(n)) : std::string();
}

}  // namespace fwcpp::sim
