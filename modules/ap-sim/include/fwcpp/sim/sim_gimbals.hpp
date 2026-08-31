#pragma once

// Original-source ports of SIM_Siyi / SIM_Siyi_ZT30, SIM_Topotek, SIM_Viewpro.
// AVT_CM62: vendor/model/fw/limits + attitude; MAVLink sockets skipped (needs GCS).

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class Siyi : public SerialDevice {
public:
    static constexpr std::uint8_t HEADER1 = 0x55;
    static constexpr std::uint8_t HEADER2 = 0x66;
    enum class CommandID : std::uint8_t {
        ACQUIRE_FIRMWARE_VERSION = 0x01,
        HARDWARE_ID = 0x02,
        GIMBAL_ROTATION = 0x07,
        ACQUIRE_GIMBAL_CONFIG_INFO = 0x0A,
        PHOTO = 0x0C,
        ACQUIRE_GIMBAL_ATTITUDE = 0x0D,
        SET_CAMERA_IMAGE_TYPE = 0x11,
        GET_TEMP_FULL_IMAGE = 0x14,
        READ_RANGEFINDER = 0x15,
        EXTERNAL_ATTITUDE = 0x22,
        SET_TIME = 0x30,
        POSITION_DATA = 0x3e,
    };
    float pitch_deg = 0;
    float yaw_deg = 0;
    std::uint16_t seq_out = 0;
    std::uint8_t rx[256]{};
    std::uint8_t rxlen = 0;

    void send_packed(std::uint8_t ctrl, CommandID id, const void* payload, std::uint16_t plen) {
        std::uint8_t buf[256]{};
        buf[0] = HEADER1;
        buf[1] = HEADER2;
        buf[2] = ctrl;
        put_le16(&buf[3], plen);
        put_le16(&buf[5], seq_out++);
        buf[7] = static_cast<std::uint8_t>(id);
        if (plen && payload) {
            std::memcpy(&buf[8], payload, plen);
        }
        const std::uint16_t crc = crc16_ccitt(buf, static_cast<std::uint32_t>(8 + plen), 0);
        put_le16(&buf[8 + plen], crc);
        write_to_autopilot(reinterpret_cast<const char*>(buf), 10 + plen);
    }

    void handle_cmd(CommandID id, const std::uint8_t* payload, std::uint16_t plen) {
        switch (id) {
        case CommandID::ACQUIRE_FIRMWARE_VERSION: {
            const std::uint8_t fw[12]{1, 2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0};
            send_packed(0x01, id, fw, sizeof(fw));
            break;
        }
        case CommandID::HARDWARE_ID: {
            const char hw[] = "SIMSIYI01";
            send_packed(0x01, id, hw, 9);
            break;
        }
        case CommandID::ACQUIRE_GIMBAL_ATTITUDE: {
            std::int16_t att[3] = {static_cast<std::int16_t>(yaw_deg * 10),
                                   static_cast<std::int16_t>(pitch_deg * 10), 0};
            send_packed(0x01, id, att, sizeof(att));
            break;
        }
        case CommandID::GIMBAL_ROTATION:
            if (plen >= 2) {
                yaw_deg += (static_cast<std::int8_t>(payload[0]) / 100.0f) * 5;
                pitch_deg += (static_cast<std::int8_t>(payload[1]) / 100.0f) * 5;
            }
            send_packed(0x01, id, nullptr, 0);
            break;
        case CommandID::ACQUIRE_GIMBAL_CONFIG_INFO: {
            const std::uint8_t cfg[8]{1, 0, 0, 0, 0, 0, 0, 0};
            send_packed(0x01, id, cfg, sizeof(cfg));
            break;
        }
        case CommandID::READ_RANGEFINDER: {
            const std::uint16_t mm = 1000;
            send_packed(0x01, id, &mm, 2);
            break;
        }
        default:
            send_packed(0x01, id, nullptr, 0);
            break;
        }
    }

    void update(const SitlInput& input) {
        pitch_deg = (input.servos[5] ? input.servos[5] : 1500) - 1500;
        pitch_deg *= 0.09f;
        yaw_deg = (input.servos[6] ? input.servos[6] : 1500) - 1500;
        yaw_deg *= 0.09f;
        const ssize_t n = read_from_autopilot(reinterpret_cast<char*>(rx + rxlen), sizeof(rx) - rxlen);
        if (n > 0) {
            rxlen = static_cast<std::uint8_t>(rxlen + n);
        }
        while (rxlen >= 10) {
            std::uint8_t i = 0;
            while (i + 1 < rxlen && !(rx[i] == HEADER1 && rx[i + 1] == HEADER2)) {
                i++;
            }
            if (i) {
                std::memmove(rx, rx + i, rxlen - i);
                rxlen = static_cast<std::uint8_t>(rxlen - i);
            }
            if (rxlen < 10) {
                return;
            }
            const std::uint16_t plen = static_cast<std::uint16_t>(rx[3] | (rx[4] << 8));
            const std::uint16_t need = static_cast<std::uint16_t>(10 + plen);
            if (rxlen < need) {
                return;
            }
            const std::uint16_t got = static_cast<std::uint16_t>(rx[8 + plen] | (rx[9 + plen] << 8));
            const std::uint16_t calc = crc16_ccitt(rx, static_cast<std::uint32_t>(8 + plen), 0);
            if (got == calc) {
                handle_cmd(static_cast<CommandID>(rx[7]), &rx[8], plen);
            }
            std::memmove(rx, rx + need, rxlen - need);
            rxlen = static_cast<std::uint8_t>(rxlen - need);
        }
    }
};
class Siyi_ZT30 : public Siyi {};

class Topotek : public SerialDevice {
public:
    float zoom = 1;
    float pitch_deg = 0;
    float yaw_deg = 0;
    std::int16_t _commanded_pitch_cd = 0;
    std::int16_t _commanded_yaw_cd = 0;
    std::uint32_t _last_attitude_ms = 0;
    char rx[256]{};
    std::uint8_t rxlen = 0;

    static void uint16_to_hex4(std::uint16_t v, std::uint8_t* out) {
        static constexpr char hex[] = "0123456789ABCDEF";
        out[0] = hex[(v >> 12) & 0xF];
        out[1] = hex[(v >> 8) & 0xF];
        out[2] = hex[(v >> 4) & 0xF];
        out[3] = hex[v & 0xF];
    }
    static std::uint8_t hexval(char c) {
        if (c >= '0' && c <= '9') return static_cast<std::uint8_t>(c - '0');
        if (c >= 'A' && c <= 'F') return static_cast<std::uint8_t>(c - 'A' + 10);
        if (c >= 'a' && c <= 'f') return static_cast<std::uint8_t>(c - 'a' + 10);
        return 0;
    }
    void send_frame(const char* ident3, const std::uint8_t* data, std::uint8_t data_len) {
        // #TP / #tp + ASCII-hex data_len + ident + data + 2 hex CRC (sum of bytes)
        char frame[128];
        int n = std::snprintf(frame, sizeof(frame), "#tp%02X%.3s", data_len, ident3);
        for (std::uint8_t i = 0; i < data_len; i++) {
            n += std::snprintf(frame + n, sizeof(frame) - n, "%02X", data[i]);
        }
        std::uint8_t sum = 0;
        for (int i = 0; i < n; i++) {
            sum = static_cast<std::uint8_t>(sum + static_cast<std::uint8_t>(frame[i]));
        }
        std::snprintf(frame + n, sizeof(frame) - n, "%02X", sum);
        write_to_autopilot(frame, std::strlen(frame));
    }
    void send_attitude() {
        std::uint8_t data[12]{};
        uint16_to_hex4(static_cast<std::uint16_t>(static_cast<std::int16_t>(yaw_deg * 100)), &data[0]);
        uint16_to_hex4(static_cast<std::uint16_t>(static_cast<std::int16_t>(-pitch_deg * 100)), &data[4]);
        uint16_to_hex4(0, &data[8]);
        send_frame("GIA", data, 12);
    }
    void update(const SitlInput* input = nullptr, std::uint32_t now_ms = 0) {
        if (input) {
            pitch_deg = ((input->servos[5] ? input->servos[5] : 1500) - 1500) * 0.09f;
            yaw_deg = ((input->servos[6] ? input->servos[6] : 1500) - 1500) * 0.09f;
        }
        pitch_deg = -_commanded_pitch_cd * 0.01f;
        yaw_deg = _commanded_yaw_cd * 0.01f;
        const ssize_t n = read_from_autopilot(rx + rxlen, sizeof(rx) - rxlen - 1);
        if (n > 0) {
            rxlen = static_cast<std::uint8_t>(rxlen + n);
            rx[rxlen] = 0;
        }
        // parse #TP / #tp commands: ident GIP/GIY set commanded angles
        char* hash = std::strchr(rx, '#');
        while (hash && rxlen >= 8) {
            if ((hash[1] == 'T' || hash[1] == 't') && (hash[2] == 'P' || hash[2] == 'p')) {
                const std::uint8_t dlen = static_cast<std::uint8_t>((hexval(hash[3]) << 4) | hexval(hash[4]));
                const char id0 = hash[5], id1 = hash[6], id2 = hash[7];
                if (id0 == 'G' && id1 == 'I' && id2 == 'P' && dlen >= 4) {
                    _commanded_pitch_cd = static_cast<std::int16_t>(
                        (hexval(hash[8]) << 12) | (hexval(hash[9]) << 8) | (hexval(hash[10]) << 4) | hexval(hash[11]));
                }
                if (id0 == 'G' && id1 == 'I' && id2 == 'Y' && dlen >= 4) {
                    _commanded_yaw_cd = static_cast<std::int16_t>(
                        (hexval(hash[8]) << 12) | (hexval(hash[9]) << 8) | (hexval(hash[10]) << 4) | hexval(hash[11]));
                }
            }
            *hash = ' ';
            hash = std::strchr(rx, '#');
        }
        if (rxlen > 200) {
            rxlen = 0;
        }
        if (now_ms == 0 || now_ms - _last_attitude_ms >= 100) {
            _last_attitude_ms = now_ms;
            send_attitude();
        }
    }
};

class Viewpro : public SerialDevice {
public:
    static constexpr float DEG_TO_OUTPUT = 65536.0f / 360.0f;
    float yaw_deg = 0;
    float pitch_deg = 0;
    std::int16_t _target_pitch_raw = 0;
    std::int16_t _target_yaw_raw = 0;
    std::uint32_t _last_attitude_ms = 0;
    std::uint8_t rx[256]{};
    std::uint8_t rxlen = 0;
    enum class Parse { WANT_H0, WANT_H1, WANT_H2, BODY };
    Parse st = Parse::WANT_H0;
    std::uint8_t body[64]{};
    std::uint8_t body_need = 0;
    std::uint8_t body_got = 0;

    void send_t1_f1_b1_d1() {
        // 0x55 0xAA 0xDC then length+id 0x40 (T1_F1_B1_D1) attitude
        std::uint8_t pkt[32]{};
        pkt[0] = 0x55;
        pkt[1] = 0xAA;
        pkt[2] = 0xDC;
        const std::uint8_t body_len = 12;
        pkt[3] = body_len;  // bits 0-5 = body length
        pkt[4] = 0x40;      // T1_F1_B1_D1
        const std::int16_t yaw_raw = static_cast<std::int16_t>(yaw_deg * DEG_TO_OUTPUT);
        const std::int16_t pitch_raw = static_cast<std::int16_t>(-pitch_deg * DEG_TO_OUTPUT);
        pkt[5] = static_cast<std::uint8_t>(yaw_raw >> 8);
        pkt[6] = static_cast<std::uint8_t>(yaw_raw);
        pkt[7] = static_cast<std::uint8_t>(pitch_raw >> 8);
        pkt[8] = static_cast<std::uint8_t>(pitch_raw);
        pkt[9] = 0;
        pkt[10] = 0;
        pkt[11] = 0;
        pkt[12] = 0;
        pkt[13] = 0;
        pkt[14] = 0;
        pkt[15] = 0;
        pkt[16] = 0;
        std::uint8_t crc = 0;
        for (int i = 3; i < 17; i++) {
            crc ^= pkt[i];
        }
        pkt[17] = crc;
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 18);
    }

    void update(const SitlInput& input, std::uint32_t now_ms = 0) {
        yaw_deg = ((input.servos[6] ? input.servos[6] : 1500) - 1500) * 0.09f;
        pitch_deg = ((input.servos[5] ? input.servos[5] : 1500) - 1500) * 0.09f;
        char tmp[64];
        const ssize_t n = read_from_autopilot(tmp, sizeof(tmp));
        for (ssize_t i = 0; i < n; i++) {
            const std::uint8_t b = static_cast<std::uint8_t>(tmp[i]);
            switch (st) {
            case Parse::WANT_H0:
                if (b == 0x55) st = Parse::WANT_H1;
                break;
            case Parse::WANT_H1:
                st = (b == 0xAA) ? Parse::WANT_H2 : Parse::WANT_H0;
                break;
            case Parse::WANT_H2:
                st = (b == 0xDC) ? Parse::BODY : Parse::WANT_H0;
                body_got = 0;
                body_need = 0;
                break;
            case Parse::BODY:
                if (body_got == 0) {
                    body_need = static_cast<std::uint8_t>((b & 0x3F) + 2);  // body + id + crc
                    body[body_got++] = b;
                } else {
                    body[body_got++] = b;
                    if (body_got >= body_need && body_need > 1) {
                        const std::uint8_t frame_id = body[1];
                        if (frame_id == 0x41) {  // A1 command
                            if (body_got >= 6) {
                                _target_yaw_raw = static_cast<std::int16_t>((body[2] << 8) | body[3]);
                                _target_pitch_raw = static_cast<std::int16_t>((body[4] << 8) | body[5]);
                                yaw_deg = _target_yaw_raw / DEG_TO_OUTPUT;
                                pitch_deg = -_target_pitch_raw / DEG_TO_OUTPUT;
                            }
                        }
                        st = Parse::WANT_H0;
                    }
                }
                break;
            }
        }
        if (now_ms == 0 || now_ms - _last_attitude_ms >= 100) {
            _last_attitude_ms = now_ms;
            send_t1_f1_b1_d1();
        }
    }
};

class AVT_CM62 {
public:
    const char* vendor = "AVTA";
    const char* model = "SIM_AVTA";
    std::uint32_t firmware_version = (3U << 16) | (2U << 8) | 1U;
    float pitch_min_rad = math::radians(-45.0f);
    float pitch_max_rad = math::radians(45.0f);
    float yaw_min_rad = math::radians(-180.0f);
    float yaw_max_rad = math::radians(180.0f);
    float roll = 0, pitch = 0, yaw = 0;
    void update(const SitlInput& input) {
        roll = ((input.servos[4] ? input.servos[4] : 1500) - 1500) * 0.09f;
        pitch = math::constrain_value(((input.servos[5] ? input.servos[5] : 1500) - 1500) * 0.09f, -45.0f, 45.0f);
        yaw = math::constrain_value(((input.servos[6] ? input.servos[6] : 1500) - 1500) * 0.09f, -180.0f, 180.0f);
    }
};

}  // namespace fwcpp::sim
