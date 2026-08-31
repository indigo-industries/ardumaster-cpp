#pragma once

// Port of libraries/SITL/SIM_MicroStrain.cpp + SIM_MicroStrain7.cpp
// MIP packets: 0x75 0x65 <desc> <len> payload Fletcher checksum.
// IMU 40 ms, GNSS 500 ms, filter 40 ms. Floats/doubles/ints are big-endian.

#include <cstdint>
#include <cstring>

#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

struct MicroStrain_Packet {
    std::uint8_t header[4]{};
    std::uint8_t payload[256]{};
    std::uint8_t checksum[2]{};
    std::uint8_t payload_size = 0;
};

class MicroStrain : public SerialDevice {
public:
    std::uint32_t last_imu_pkt_ms = 0;
    std::uint32_t last_gnss_pkt_ms = 0;
    std::uint32_t last_filter_pkt_ms = 0;
    std::uint64_t start_us = 0;

    static void put_float(MicroStrain_Packet& packet, float f) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &f, 4);
        put_be32(&packet.payload[packet.payload_size], bits);
        packet.payload_size = static_cast<std::uint8_t>(packet.payload_size + 4);
    }
    static void put_double(MicroStrain_Packet& packet, double d) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &d, 8);
        put_be64(&packet.payload[packet.payload_size], bits);
        packet.payload_size = static_cast<std::uint8_t>(packet.payload_size + 8);
    }
    static void put_int(MicroStrain_Packet& packet, std::uint16_t t) {
        put_be16(&packet.payload[packet.payload_size], t);
        packet.payload_size = static_cast<std::uint8_t>(packet.payload_size + 2);
    }
    static void generate_checksum(MicroStrain_Packet& packet) {
        std::uint8_t b1 = 0, b2 = 0;
        for (int i = 0; i < 4; i++) {
            b1 = static_cast<std::uint8_t>(b1 + packet.header[i]);
            b2 = static_cast<std::uint8_t>(b2 + b1);
        }
        for (int i = 0; i < packet.header[3]; i++) {
            b1 = static_cast<std::uint8_t>(b1 + packet.payload[i]);
            b2 = static_cast<std::uint8_t>(b2 + b1);
        }
        packet.checksum[0] = b1;
        packet.checksum[1] = b2;
    }
    void send_packet(MicroStrain_Packet packet) {
        generate_checksum(packet);
        write_to_autopilot(reinterpret_cast<const char*>(packet.header), 4);
        write_to_autopilot(reinterpret_cast<const char*>(packet.payload), packet.payload_size);
        write_to_autopilot(reinterpret_cast<const char*>(packet.checksum), 2);
    }

    void send_imu_packet(const Aircraft& ac) {
        MicroStrain_Packet packet{};
        packet.header[0] = 0x75;
        packet.header[1] = 0x65;
        packet.header[2] = 0x80;
        packet.payload[packet.payload_size++] = 0x06;
        packet.payload[packet.payload_size++] = 0x17;
        float p = 0, t_k = 0;
        get_pressure_temperature_for_alt_amsl(ac.location.alt * 0.01f, p, t_k);
        put_float(packet, p * 0.001f);
        packet.payload[packet.payload_size++] = 0x0E;
        packet.payload[packet.payload_size++] = 0x06;
        put_float(packet, ac.mag_bf.x * 0.001f);
        put_float(packet, ac.mag_bf.y * 0.001f);
        put_float(packet, ac.mag_bf.z * 0.001f);
        packet.payload[packet.payload_size++] = 0x0E;
        packet.payload[packet.payload_size++] = 0x04;
        put_float(packet, ac.accel_body.x / kGravityMss);
        put_float(packet, ac.accel_body.y / kGravityMss);
        put_float(packet, ac.accel_body.z / kGravityMss);
        packet.payload[packet.payload_size++] = 0x0E;
        packet.payload[packet.payload_size++] = 0x05;
        put_float(packet, ac.gyro.x);
        put_float(packet, ac.gyro.y);
        put_float(packet, ac.gyro.z);
        const auto q = quat_from_aircraft(ac);
        packet.payload[packet.payload_size++] = 0x12;
        packet.payload[packet.payload_size++] = 0x0A;
        put_float(packet, q.q1);
        put_float(packet, q.q2);
        put_float(packet, q.q3);
        put_float(packet, q.q4);
        packet.header[3] = packet.payload_size;
        send_packet(packet);
    }

    virtual void send_gnss_packet(const Aircraft& ac) {
        MicroStrain_Packet packet{};
        packet.header[0] = 0x75;
        packet.header[1] = 0x65;
        packet.header[2] = 0x81;
        fill_gnss_fields(packet, ac);
        packet.header[3] = packet.payload_size;
        send_packet(packet);
    }
    virtual void send_filter_packet(const Aircraft& ac) {
        MicroStrain_Packet packet{};
        packet.header[0] = 0x75;
        packet.header[1] = 0x65;
        packet.header[2] = 0x82;
        fill_filter_common(packet, ac);
        packet.payload[packet.payload_size++] = 0x08;
        packet.payload[packet.payload_size++] = 0x10;
        put_int(packet, 0x02);
        put_int(packet, 0x03);
        put_int(packet, 0);
        packet.header[3] = packet.payload_size;
        send_packet(packet);
    }

    void fill_gnss_fields(MicroStrain_Packet& packet, const Aircraft& ac) {
        const double tsec = static_cast<double>(ac.time_now_us) * 1e-6;
        packet.payload[packet.payload_size++] = 0x0E;
        packet.payload[packet.payload_size++] = 0xD3;
        put_double(packet, tsec);
        put_int(packet, 0);
        put_int(packet, 0);
        packet.payload[packet.payload_size++] = 0x08;
        packet.payload[packet.payload_size++] = 0x0B;
        packet.payload[packet.payload_size++] = 0x00;
        packet.payload[packet.payload_size++] = 19;
        put_int(packet, 0);
        put_int(packet, 0);
        packet.payload[packet.payload_size++] = 0x2C;
        packet.payload[packet.payload_size++] = 0x03;
        put_double(packet, ac.location.lat * 1.0e-7);
        put_double(packet, ac.location.lng * 1.0e-7);
        put_double(packet, 0);
        put_double(packet, ac.location.alt * 0.01);
        put_float(packet, 0.5f);
        put_float(packet, 0.5f);
        put_int(packet, 31);
        packet.payload[packet.payload_size++] = 0x20;
        packet.payload[packet.payload_size++] = 0x07;
        for (int i = 0; i < 7; i++) {
            put_float(packet, 0);
        }
        put_int(packet, 127);
        packet.payload[packet.payload_size++] = 0x24;
        packet.payload[packet.payload_size++] = 0x05;
        put_float(packet, ac.velocity_ef.x);
        put_float(packet, ac.velocity_ef.y);
        put_float(packet, ac.velocity_ef.z);
        put_float(packet, 0);
        put_float(packet, 0);
        put_float(packet, 0);
        put_float(packet, 0.25f);
        put_float(packet, 0);
        put_int(packet, 31);
    }
    void fill_filter_common(MicroStrain_Packet& packet, const Aircraft& ac) {
        packet.payload[packet.payload_size++] = 0x0E;
        packet.payload[packet.payload_size++] = 0xD3;
        put_double(packet, static_cast<double>(ac.time_now_us % 1000000ULL) * 1e-6);
        put_int(packet, 0);
        put_int(packet, 0x0001);
        packet.payload[packet.payload_size++] = 0x10;
        packet.payload[packet.payload_size++] = 0x02;
        put_float(packet, ac.velocity_ef.x);
        put_float(packet, ac.velocity_ef.y);
        put_float(packet, ac.velocity_ef.z);
        put_int(packet, 0x0001);
        packet.payload[packet.payload_size++] = 0x1C;
        packet.payload[packet.payload_size++] = 0x01;
        put_double(packet, ac.location.lat * 1.0e-7);
        put_double(packet, ac.location.lng * 1.0e-7);
        put_double(packet, 0);
        put_int(packet, 0x0001);
    }

    void update(const Aircraft& ac) {
        const std::uint32_t now = static_cast<std::uint32_t>(ac.time_now_us / 1000ULL);
        if (now - last_imu_pkt_ms >= 40) {
            last_imu_pkt_ms = now;
            send_imu_packet(ac);
        }
        if (now - last_gnss_pkt_ms >= 500) {
            last_gnss_pkt_ms = now;
            send_gnss_packet(ac);
        }
        if (now - last_filter_pkt_ms >= 40) {
            last_filter_pkt_ms = now;
            send_filter_packet(ac);
        }
    }
};

class MicroStrain5 : public MicroStrain {};

class MicroStrain7 : public MicroStrain {
public:
    void send_gnss_packet(const Aircraft& ac) override {
        const std::uint8_t descriptors[2] = {0x91, 0x92};
        for (std::uint8_t d : descriptors) {
            MicroStrain_Packet packet{};
            packet.header[0] = 0x75;
            packet.header[1] = 0x65;
            packet.header[2] = d;
            fill_gnss_fields(packet, ac);
            packet.header[3] = packet.payload_size;
            send_packet(packet);
        }
    }
    void send_filter_packet(const Aircraft& ac) override {
        MicroStrain_Packet packet{};
        packet.header[0] = 0x75;
        packet.header[1] = 0x65;
        packet.header[2] = 0x82;
        fill_filter_common(packet, ac);
        packet.payload[packet.payload_size++] = 0x08;
        packet.payload[packet.payload_size++] = 0x10;
        put_int(packet, 0x04);  // GQ7_FULL_NAV
        put_int(packet, 0x03);
        put_int(packet, 0);
        const auto q = quat_from_aircraft(ac);
        packet.payload[packet.payload_size++] = 0x12;
        packet.payload[packet.payload_size++] = 0x03;
        put_float(packet, q.q1);
        put_float(packet, q.q2);
        put_float(packet, q.q3);
        put_float(packet, q.q4);
        packet.header[3] = packet.payload_size;
        send_packet(packet);
    }
};

}  // namespace fwcpp::sim
