#pragma once

// Port of libraries/SITL/SIM_SensAItion.cpp — 1 kHz tick, packet 0 IMU (36 BE
// bytes) every 5 ticks; interleaved packets 1 (quat) and 2 (INS 69 bytes).
// Header 0xFA, XOR CRC over id+payload (interleaved) or payload (legacy).

#include <cstdint>
#include <cstring>

#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_gps.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

class SensAItion : public SerialDevice {
public:
    explicit SensAItion(bool interleaved_mode = true) : _interleaved_mode(interleaved_mode) {}
    bool _interleaved_mode = true;
    int _tick = 0;
    int _periodMessage0 = 5;
    int _phaseMessage0 = 0;
    int _periodMessage1 = 100;
    int _phaseMessage1 = 0;
    int _periodMessage2 = 100;
    int _phaseMessage2 = 5;
    std::uint8_t _buffert[512]{};
    int _buffert_cnt = 0;

    void flush_packets() {
        if (_buffert_cnt > 0) {
            write_to_autopilot(reinterpret_cast<const char*>(_buffert), static_cast<std::size_t>(_buffert_cnt));
            _buffert_cnt = 0;
        }
    }
    void write_to_autopilot_buf(const void* data, int length) {
        if (_buffert_cnt + length > static_cast<int>(sizeof(_buffert))) {
            flush_packets();
        }
        std::memcpy(&_buffert[_buffert_cnt], data, static_cast<std::size_t>(length));
        _buffert_cnt += length;
    }
    static std::uint8_t calculate_crc(std::uint8_t msg_id, const std::uint8_t* payload, std::uint16_t length, bool use_id) {
        std::uint8_t crc = 0;
        if (use_id) {
            crc ^= msg_id;
        }
        for (std::uint16_t i = 0; i < length; i++) {
            crc ^= payload[i];
        }
        return crc;
    }
    void write_packet(std::uint8_t msg_id, const std::uint8_t* payload, std::uint16_t length) {
        const std::uint8_t header = 0xFA;
        write_to_autopilot_buf(&header, 1);
        write_to_autopilot_buf(&msg_id, 1);
        write_to_autopilot_buf(payload, length);
        const std::uint8_t crc = calculate_crc(msg_id, payload, length, true);
        write_to_autopilot_buf(&crc, 1);
    }
    void write_legacy_packet(const std::uint8_t* payload, std::uint16_t length) {
        const std::uint8_t header = 0xFA;
        write_to_autopilot_buf(&header, 1);
        write_to_autopilot_buf(payload, length);
        const std::uint8_t crc = calculate_crc(0, payload, length, false);
        write_to_autopilot_buf(&crc, 1);
    }

    void send_packet_0_imu(const Aircraft& ac) {
        std::uint8_t pkt[36]{};
        put_be32(&pkt[0], static_cast<std::uint32_t>(static_cast<std::int32_t>(ac.accel_body.x / kGravityMss * 1e6f)));
        put_be32(&pkt[4], static_cast<std::uint32_t>(static_cast<std::int32_t>(ac.accel_body.y / kGravityMss * 1e6f)));
        put_be32(&pkt[8], static_cast<std::uint32_t>(static_cast<std::int32_t>(ac.accel_body.z / kGravityMss * 1e6f)));
        put_be32(&pkt[12], static_cast<std::uint32_t>(static_cast<std::int32_t>(math::degrees(ac.gyro.x) * 1e6f)));
        put_be32(&pkt[16], static_cast<std::uint32_t>(static_cast<std::int32_t>(math::degrees(ac.gyro.y) * 1e6f)));
        put_be32(&pkt[20], static_cast<std::uint32_t>(static_cast<std::int32_t>(math::degrees(ac.gyro.z) * 1e6f)));
        put_be16(&pkt[24], static_cast<std::uint16_t>(static_cast<std::int16_t>((25.0f - 20.0f) / 0.008f)));
        put_be16(&pkt[26], static_cast<std::uint16_t>(static_cast<std::int16_t>(ac.mag_bf.x * 1000)));
        put_be16(&pkt[28], static_cast<std::uint16_t>(static_cast<std::int16_t>(ac.mag_bf.y * 1000)));
        put_be16(&pkt[30], static_cast<std::uint16_t>(static_cast<std::int16_t>(ac.mag_bf.z * 1000)));
        float p = 0, t_k = 0;
        get_pressure_temperature_for_alt_amsl(ac.location.alt * 0.01f, p, t_k);
        put_be32(&pkt[32], static_cast<std::uint32_t>(static_cast<std::int32_t>(p * 10.0f)));
        if (_interleaved_mode) {
            write_packet(0x00, pkt, sizeof(pkt));
        } else {
            write_legacy_packet(pkt, sizeof(pkt));
        }
    }
    void send_packet_1_orientation(const Aircraft& ac) {
        const auto q = quat_from_aircraft(ac);
        std::uint8_t pkt[16]{};
        put_be32(&pkt[0], static_cast<std::uint32_t>(static_cast<std::int32_t>(q.q1 * 1.0e6f)));
        put_be32(&pkt[4], static_cast<std::uint32_t>(static_cast<std::int32_t>(q.q2 * 1.0e6f)));
        put_be32(&pkt[8], static_cast<std::uint32_t>(static_cast<std::int32_t>(q.q3 * 1.0e6f)));
        put_be32(&pkt[12], static_cast<std::uint32_t>(static_cast<std::int32_t>(q.q4 * 1.0e6f)));
        write_packet(0x01, pkt, sizeof(pkt));
    }
    void send_packet_2_ins(const Aircraft& ac) {
        std::uint8_t pkt[69]{};
        put_be32(&pkt[0], 0x0C0C0C0C);
        put_be32(&pkt[4], 0);
        pkt[8] = 0xFF;
        put_be32(&pkt[9], static_cast<std::uint32_t>(ac.location.lat));
        put_be32(&pkt[13], static_cast<std::uint32_t>(ac.location.lng));
        put_be32(&pkt[17], static_cast<std::uint32_t>(static_cast<std::int32_t>(ac.velocity_ef.x * 1000)));
        put_be32(&pkt[21], static_cast<std::uint32_t>(static_cast<std::int32_t>(ac.velocity_ef.y * 1000)));
        put_be32(&pkt[25], static_cast<std::uint32_t>(static_cast<std::int32_t>(ac.velocity_ef.z * 1000)));
        put_be32(&pkt[29], static_cast<std::uint32_t>(static_cast<std::int32_t>(ac.location.alt * 10)));  // cm -> mm
        pkt[33] = 1;
        const auto gps_tow = GPS_Backend::gps_time();
        put_be32(&pkt[34], gps_tow.ms);
        pkt[38] = 3;
        pkt[39] = 3;
        put_be16(&pkt[40], 2025);
        put_be16(&pkt[42], 12);
        pkt[44] = 7;
        put_be32(&pkt[45], 100);
        put_be32(&pkt[49], 100);
        put_be32(&pkt[53], 20);
        put_be32(&pkt[57], 20);
        put_be32(&pkt[61], 20);
        put_be32(&pkt[65], 100);
        write_packet(0x02, pkt, sizeof(pkt));
    }

    void update(const Aircraft& ac) {
        const int tick1kHz = static_cast<int>((ac.time_now_us + 500) / 1000);
        if (tick1kHz <= _tick) {
            return;
        }
        _tick = tick1kHz;
        char trash[64]{};
        read_from_autopilot(trash, sizeof(trash));
        if (ac.time_now_us / 1000 < 1000) {
            return;
        }
        if ((_tick % _periodMessage0) == _phaseMessage0) {
            send_packet_0_imu(ac);
        }
        if (_interleaved_mode) {
            if ((_tick % _periodMessage1) == _phaseMessage1) {
                send_packet_1_orientation(ac);
            }
            if ((_tick % _periodMessage2) == _phaseMessage2) {
                send_packet_2_ins(ac);
            }
        }
        flush_packets();
    }
};

}  // namespace fwcpp::sim
