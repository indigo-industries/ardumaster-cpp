#pragma once

// Isolated original-source ports of SIM_RF_MAVLink and SIM_RF_LightWare_GRF.
// MAVLink DISTANCE_SENSOR is encoded without GCS_MAVLink (sim_mavlink_min).
// GRF process_input is the original 0xAA LightWare config state machine.

#include <cstdint>
#include <cstring>

#include <fwcpp/sim/sim_mavlink_min.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_serial_rangefinder.hpp>

namespace fwcpp::sim {

class RF_MAVLink : public SerialRangeFinder {
public:
    mavmin::Status st{};
    std::uint8_t sysid = 32;
    std::uint8_t compid = 32;
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        auto bytes = mavmin::encode_distance_sensor(sysid, compid, st, 0, static_cast<std::uint16_t>(alt_m * 100));
        if (bytes.size() > buflen) {
            return 0;
        }
        std::memcpy(buffer, bytes.data(), bytes.size());
        return static_cast<std::uint32_t>(bytes.size());
    }
};

class RF_LightWareGRF : public SerialRangeFinder, public SerialDevice {
public:
    static constexpr std::uint8_t PREAMBLE = 0xAA;
    enum class MsgID : std::uint8_t {
        PRODUCT_NAME = 0,
        DISTANCE_OUTPUT = 27,
        STREAM = 30,
        DISTANCE_DATA_CM = 44,
        UPDATE_RATE = 74,
    };
    bool stream_enabled = false;
    std::uint32_t update_period_ms = 200;
    std::uint8_t rxbuf[64]{};
    std::uint8_t rxlen = 0;

    static std::uint16_t compute_crc(const std::uint8_t* buf, std::uint8_t len) {
        std::uint16_t crc = 0;
        for (std::uint8_t i = 0; i < len; i++) {
            crc = crc_xmodem_update(crc, buf[i]);
        }
        return crc;
    }

    void send_product_name() {
        std::uint8_t payload[7] = {'G', 'R', 'F', '2', '5', '0', 0};
        std::uint8_t msg[32]{};
        std::uint8_t len = 0;
        msg[len++] = PREAMBLE;
        std::uint16_t flags = 0x1;
        flags = static_cast<std::uint16_t>(flags | ((1 + sizeof(payload)) << 6));
        msg[len++] = static_cast<std::uint8_t>(flags);
        msg[len++] = static_cast<std::uint8_t>(flags >> 8);
        msg[len++] = static_cast<std::uint8_t>(MsgID::PRODUCT_NAME);
        std::memcpy(&msg[len], payload, sizeof(payload));
        len = static_cast<std::uint8_t>(len + sizeof(payload));
        const std::uint16_t crc = compute_crc(msg, len);
        msg[len++] = static_cast<std::uint8_t>(crc);
        msg[len++] = static_cast<std::uint8_t>(crc >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(msg), len);
    }
    void send_ack_u32(MsgID id, std::uint32_t value) {
        std::uint8_t msg[24]{};
        std::uint8_t len = 0;
        msg[len++] = PREAMBLE;
        std::uint16_t flags = 0x1;
        flags = static_cast<std::uint16_t>(flags | ((1 + 4) << 6));
        msg[len++] = static_cast<std::uint8_t>(flags);
        msg[len++] = static_cast<std::uint8_t>(flags >> 8);
        msg[len++] = static_cast<std::uint8_t>(id);
        std::memcpy(&msg[len], &value, 4);
        len = static_cast<std::uint8_t>(len + 4);
        const std::uint16_t crc = compute_crc(msg, len);
        msg[len++] = static_cast<std::uint8_t>(crc);
        msg[len++] = static_cast<std::uint8_t>(crc >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(msg), len);
    }

    void try_parse_message() {
        if (rxlen < 6) {
            return;
        }
        std::uint8_t i = 0;
        while (i < rxlen && rxbuf[i] != PREAMBLE) {
            i++;
        }
        if (i == rxlen) {
            rxlen = 0;
            return;
        }
        if (i > 0) {
            std::memmove(rxbuf, rxbuf + i, rxlen - i);
            rxlen = static_cast<std::uint8_t>(rxlen - i);
        }
        if (rxlen < 6) {
            return;
        }
        const std::uint16_t flags = static_cast<std::uint16_t>(rxbuf[1] | (rxbuf[2] << 8));
        const std::uint8_t plen = static_cast<std::uint8_t>((flags >> 6) - 1);
        if (plen > 100) {
            std::memmove(rxbuf, rxbuf + 1, --rxlen);
            return;
        }
        const std::uint8_t need = static_cast<std::uint8_t>(1 + 2 + 1 + plen + 2);
        if (rxlen < need) {
            return;
        }
        const std::uint16_t gotcrc = static_cast<std::uint16_t>(rxbuf[need - 2] | (rxbuf[need - 1] << 8));
        const std::uint16_t calcrc = compute_crc(rxbuf, static_cast<std::uint8_t>(need - 2));
        if (gotcrc == calcrc) {
            const auto msgid = static_cast<MsgID>(rxbuf[3]);
            switch (msgid) {
            case MsgID::PRODUCT_NAME:
                send_product_name();
                break;
            case MsgID::UPDATE_RATE: {
                std::uint8_t rate = rxbuf[4] ? rxbuf[4] : 1;
                update_period_ms = 1000 / rate;
                send_ack_u32(MsgID::UPDATE_RATE, rate);
                break;
            }
            case MsgID::DISTANCE_OUTPUT: {
                std::uint32_t fields = 0;
                std::memcpy(&fields, &rxbuf[4], 4);
                send_ack_u32(MsgID::DISTANCE_OUTPUT, fields);
                break;
            }
            case MsgID::STREAM: {
                std::uint32_t stream = 0;
                std::memcpy(&stream, &rxbuf[4], 4);
                stream_enabled = (stream == 5);
                if (stream_enabled) {
                    send_ack_u32(MsgID::STREAM, stream);
                }
                break;
            }
            default:
                break;
            }
        }
        const std::uint8_t remaining = static_cast<std::uint8_t>(rxlen - need);
        std::memmove(rxbuf, rxbuf + need, remaining);
        rxlen = remaining;
        if (rxlen >= 4) {
            try_parse_message();
        }
    }

    void process_input() {
        char buffer[50]{};
        const ssize_t n = read_from_autopilot(buffer, sizeof(buffer));
        if (n <= 0) {
            return;
        }
        for (ssize_t i = 0; i < n && rxlen < sizeof(rxbuf); i++) {
            rxbuf[rxlen++] = static_cast<std::uint8_t>(buffer[i]);
        }
        try_parse_message();
    }

    void build_distance_packet(float alt_m, std::uint8_t* out, std::uint32_t& outlen) {
        float dist_m = alt_m;
        std::uint32_t strength = 100;
        if (dist_m < 0.2f) {
            dist_m = 0.2f;
            strength = 0;
        }
        if (dist_m > 500) {
            dist_m = 500;
            strength = 0;
        }
        const std::uint32_t cm = static_cast<std::uint32_t>(dist_m * 100.0f);
        const std::uint32_t raw = cm / 10;
        std::uint8_t len = 0;
        out[len++] = PREAMBLE;
        std::uint16_t flags = 0x1;
        flags = static_cast<std::uint16_t>(flags | ((1 + 8) << 6));
        out[len++] = static_cast<std::uint8_t>(flags);
        out[len++] = static_cast<std::uint8_t>(flags >> 8);
        out[len++] = static_cast<std::uint8_t>(MsgID::DISTANCE_DATA_CM);
        std::memcpy(&out[len], &raw, 4);
        len = static_cast<std::uint8_t>(len + 4);
        std::memcpy(&out[len], &strength, 4);
        len = static_cast<std::uint8_t>(len + 4);
        const std::uint16_t crc = compute_crc(out, len);
        out[len++] = static_cast<std::uint8_t>(crc);
        out[len++] = static_cast<std::uint8_t>(crc >> 8);
        outlen = len;
    }

    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        process_input();
        if (!stream_enabled) {
            return 0;
        }
        if (buflen < 16) {
            return 0;
        }
        std::uint32_t outlen = 0;
        build_distance_packet(alt_m, buffer, outlen);
        return outlen;
    }
};

}  // namespace fwcpp::sim
