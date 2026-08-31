#pragma once

// Port of libraries/SITL/SIM_InertialLabs.cpp/.h — packed Table4 32-message
// ILabsPacket, magic 0x55AA, 200 Hz, GNSS every 20 packets, CRC from byte 2.

#include <cmath>
#include <cstdint>
#include <cstring>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_gps.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

#pragma pack(push, 1)
struct ILabsVec3_16 { std::int16_t x, y, z; };
struct ILabsVec3_32 { std::int32_t x, y, z; };
struct ILabsVec3_u8 { std::uint8_t x, y, z; };
struct ILabsVec3_u16 { std::uint16_t x, y, z; };
struct ILabsPacket {
    std::uint16_t magic = 0x55AA;
    std::uint8_t msg_type = 0x01;
    std::uint8_t msg_id = 0x95;
    std::uint16_t msg_len = 0;
    std::uint8_t num_messages = 32;
    std::uint8_t messages[32] = {0x01, 0x3C, 0x23, 0x21, 0x25, 0x24, 0x07, 0x12, 0x10, 0x58, 0x57, 0x53, 0x4a,
                                 0x3b, 0x30, 0x32, 0x3e, 0x36, 0x41, 0xc0, 0x28, 0x86, 0x8a, 0x8d, 0x50, 0x52,
                                 0x5a, 0x33, 0x3a, 0x40, 0x42, 0x54};
    std::uint32_t gnss_ins_time_ms = 0;
    std::uint16_t gnss_week = 0;
    ILabsVec3_32 accel_data_hr{};
    ILabsVec3_32 gyro_data_hr{};
    struct {
        std::uint16_t pressure_pa2;
        std::int32_t baro_alt;
    } baro_data{};
    ILabsVec3_16 mag_data{};
    struct {
        std::uint16_t yaw;
        std::int16_t pitch;
        std::int16_t roll;
    } orientation_angles{};
    ILabsVec3_32 velocity{};
    struct {
        std::int32_t lat, lon, alt;
    } position{};
    ILabsVec3_u8 kf_vel_covariance{};
    ILabsVec3_u16 kf_pos_covariance{};
    std::uint16_t unit_status = 0;
    struct {
        std::uint8_t fix_type;
        std::uint8_t spoofing_status;
    } gnss_extended_info{};
    std::uint8_t num_sats = 0;
    struct {
        std::int32_t lat, lon, alt;
    } gnss_position{};
    struct {
        std::int32_t hor_speed;
        std::uint16_t track_over_ground;
        std::int32_t ver_speed;
    } gnss_vel_track{};
    std::uint32_t gnss_pos_timestamp = 0;
    struct {
        std::uint8_t info1, info2;
    } gnss_info_short{};
    std::uint8_t gnss_new_data = 0;
    std::uint8_t gnss_jam_status = 0;
    std::int32_t differential_pressure = 0;
    std::int16_t true_airspeed = 0;
    ILabsVec3_16 wind_speed{};
    std::uint16_t air_data_status = 0;
    std::uint16_t supply_voltage = 0;
    std::int16_t temperature = 0;
    std::uint16_t unit_status2 = 0;
    struct {
        std::uint16_t heading;
        std::int16_t pitch;
    } gnss_angles{};
    std::uint8_t gnss_angle_pos_type = 0;
    std::uint32_t gnss_heading_timestamp = 0;
    struct {
        std::uint16_t gdop, pdop, hdop, vdop, tdop;
    } gnss_dop{};
    std::uint8_t ins_sol_status = 0;
    std::uint16_t crc = 0;
};
#pragma pack(pop)

class InertialLabs : public SerialDevice {
public:
    ILabsPacket pkt{};
    std::uint32_t last_pkt_us = 0;
    std::uint32_t packets_sent = 0;
    static constexpr std::uint16_t pkt_rate_hz = 200;
    static constexpr std::uint16_t gnss_rate_hz = 10;
    static constexpr std::uint16_t gnss_frequency = pkt_rate_hz / gnss_rate_hz;

    void send_packet(const Aircraft& ac) {
        pkt.msg_len = static_cast<std::uint16_t>(sizeof(pkt) - 2);
        const auto gps_tow = GPS_Backend::gps_time();
        pkt.gnss_ins_time_ms = gps_tow.ms;
        pkt.accel_data_hr.x = static_cast<std::int32_t>((ac.accel_body.y / kGravityMss) * 1.0e6f);
        pkt.accel_data_hr.y = static_cast<std::int32_t>((ac.accel_body.x / kGravityMss) * 1.0e6f);
        pkt.accel_data_hr.z = static_cast<std::int32_t>((-ac.accel_body.z / kGravityMss) * 1.0e6f);
        pkt.gyro_data_hr.y = static_cast<std::int32_t>(math::degrees(ac.gyro.x) * 1.0e5f);
        pkt.gyro_data_hr.x = static_cast<std::int32_t>(math::degrees(ac.gyro.y) * 1.0e5f);
        pkt.gyro_data_hr.z = static_cast<std::int32_t>(-math::degrees(ac.gyro.z) * 1.0e5f);
        float p = 0, t_k = 0;
        get_pressure_temperature_for_alt_amsl(ac.location.alt * 0.01f, p, t_k);
        pkt.baro_data.pressure_pa2 = static_cast<std::uint16_t>(p * 0.5f);
        pkt.baro_data.baro_alt = static_cast<std::int32_t>(ac.location.alt);
        pkt.mag_data.x = static_cast<std::int16_t>((ac.mag_bf.y / kNteslaToMgauss) * 0.1f);
        pkt.mag_data.y = static_cast<std::int16_t>((ac.mag_bf.x / kNteslaToMgauss) * 0.1f);
        pkt.mag_data.z = static_cast<std::int16_t>((-ac.mag_bf.z / kNteslaToMgauss) * 0.1f);
        float r = 0, pit = 0, y = 0;
        ac.get_dcm().to_euler(&r, &pit, &y);
        float heading_deg = std::fmod(math::degrees(y), 360.0f);
        if (heading_deg < 0) {
            heading_deg += 360.0f;
        }
        pkt.orientation_angles.roll = static_cast<std::int16_t>(math::degrees(r) * 100);
        pkt.orientation_angles.pitch = static_cast<std::int16_t>(math::degrees(pit) * 100);
        pkt.orientation_angles.yaw = static_cast<std::uint16_t>(heading_deg * 100);
        pkt.velocity.x = static_cast<std::int32_t>(ac.velocity_ef.y * 100);
        pkt.velocity.y = static_cast<std::int32_t>(ac.velocity_ef.x * 100);
        pkt.velocity.z = static_cast<std::int32_t>(-ac.velocity_ef.z * 100);
        pkt.position.lat = ac.location.lat;
        pkt.position.lon = ac.location.lng;
        pkt.position.alt = ac.location.alt;
        pkt.kf_vel_covariance = {10, 10, 10};
        pkt.kf_pos_covariance = {20, 20, 20};
        pkt.unit_status = 0;
        pkt.differential_pressure = static_cast<std::int32_t>(0);
        pkt.true_airspeed = static_cast<std::int16_t>(ac.airspeed * 100);
        pkt.wind_speed.x = static_cast<std::int16_t>(ac.wind_ef.y * 100);
        pkt.wind_speed.y = static_cast<std::int16_t>(ac.wind_ef.x * 100);
        pkt.wind_speed.z = 0;
        pkt.air_data_status = 0;
        pkt.supply_voltage = static_cast<std::uint16_t>(ac.battery_voltage * 100);
        pkt.temperature = static_cast<std::int16_t>((t_k - 273.15f) * 10);
        pkt.unit_status2 = 0;
        pkt.ins_sol_status = 0;
        pkt.gnss_new_data = 0;
        if (packets_sent % gnss_frequency == 0) {
            pkt.gnss_week = gps_tow.week;
            pkt.gnss_extended_info.fix_type = 2;
            pkt.gnss_extended_info.spoofing_status = 1;
            pkt.num_sats = 32;
            pkt.gnss_position.lat = ac.location.lat;
            pkt.gnss_position.lon = ac.location.lng;
            pkt.gnss_position.alt = ac.location.alt;
            const float hs = std::hypot(ac.velocity_ef.x, ac.velocity_ef.y);
            pkt.gnss_vel_track.hor_speed = static_cast<std::int32_t>(hs * 100);
            pkt.gnss_vel_track.track_over_ground =
                static_cast<std::uint16_t>(math::wrap_360(math::degrees(std::atan2(ac.velocity_ef.y, ac.velocity_ef.x))) * 100);
            pkt.gnss_vel_track.ver_speed = static_cast<std::int32_t>(-ac.velocity_ef.z * 100);
            pkt.gnss_pos_timestamp = gps_tow.ms;
            pkt.gnss_info_short.info1 = 0;
            pkt.gnss_info_short.info2 = 12;
            pkt.gnss_new_data = 3;
            pkt.gnss_jam_status = 1;
            pkt.gnss_dop = {1000, 1000, 1000, 1000, 1000};
        }
        const auto* buffer = reinterpret_cast<const std::uint8_t*>(&pkt);
        pkt.crc = crc_sum_of_bytes_16(&buffer[2], static_cast<std::uint16_t>(sizeof(pkt) - 4));
        write_to_autopilot(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
        packets_sent++;
    }

    void update(const Aircraft& ac) {
        const std::uint32_t now = static_cast<std::uint32_t>(ac.time_now_us);
        if (now - last_pkt_us >= 5000) {
            last_pkt_us = now;
            send_packet(ac);
        }
    }
};

}  // namespace fwcpp::sim
