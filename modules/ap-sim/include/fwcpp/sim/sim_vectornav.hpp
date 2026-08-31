#pragma once

// Port of libraries/SITL/SIM_VectorNav.cpp/.h — VN-100 / VN-300 binary
// packets (sync 0xFA + group headers + crc16-ccitt swabbed).

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

#pragma pack(push, 1)
struct VN_IMU_packet_sim {
    static constexpr std::uint8_t header[3]{0x01, 0x21, 0x07};
    std::uint64_t timeStartup;
    float gyro[3];
    float accel[3];
    float uncompAccel[3];
    float uncompAngRate[3];
    float mag[3];
    float temp;
    float pressure;
};
struct VN_INS_ekf_packet_sim {
    static constexpr std::uint8_t header[7]{0x31, 0x01, 0x00, 0x06, 0x01, 0x13, 0x06};
    std::uint64_t timeStartup;
    float ypr[3];
    float quaternion[4];
    float yprU[3];
    std::uint16_t insStatus;
    double posLla[3];
    float velNed[3];
    float posU;
    float velU;
};
struct VN_INS_gnss_packet_sim {
    static constexpr std::uint8_t header[7]{0x49, 0x03, 0x00, 0xB8, 0x26, 0x18, 0x00};
    std::uint64_t timeStartup;
    std::uint64_t timeGps;
    std::uint8_t numSats1;
    std::uint8_t fix1;
    double posLla1[3];
    float velNed1[3];
    float posU1[3];
    float velU1;
    float dop1[7];
    std::uint8_t numSats2;
    std::uint8_t fix2;
};
struct VN_AHRS_ekf_packet_sim {
    static constexpr std::uint8_t header[5]{0x11, 0x01, 0x00, 0x06, 0x01};
    std::uint64_t timeStartup;
    float ypr[3];
    float quaternion[4];
    float yprU[3];
};
#pragma pack(pop)

class VectorNav : public SerialDevice {
public:
    enum class VNModel { VN100, VN300 };
    explicit VectorNav(VNModel m = VNModel::VN300) : model(m) {}

    VNModel model = VNModel::VN300;
    std::uint32_t last_imu_pkt_us = 0;
    std::uint32_t last_ekf_pkt_us = 0;
    std::uint32_t last_gnss_pkt_us = 0;
    std::uint32_t last_ahrs_pkt_us = 0;
    float gyro_noise = 0.0f;  // original 0.05; 0 in tests for deterministic packets

    void write_crc_swabbed(const std::uint8_t* hdr, std::size_t hdrlen, const void* pkt, std::size_t pktlen) {
        std::uint16_t crc = crc16_ccitt(hdr, static_cast<std::uint32_t>(hdrlen), 0);
        crc = crc16_ccitt(static_cast<const std::uint8_t*>(pkt), static_cast<std::uint32_t>(pktlen), crc);
        const std::uint16_t crc2 = static_cast<std::uint16_t>((crc << 8) | (crc >> 8));  // swab
        write_to_autopilot(reinterpret_cast<const char*>(&crc2), sizeof(crc2));
    }

    void send_imu_packet(const Aircraft& ac) {
        VN_IMU_packet_sim pkt{};
        pkt.timeStartup = ac.time_now_us * 1000ULL;
        pkt.gyro[0] = ac.gyro.x;
        pkt.gyro[1] = ac.gyro.y;
        pkt.gyro[2] = ac.gyro.z;
        pkt.accel[0] = ac.accel_body.x;
        pkt.accel[1] = ac.accel_body.y;
        pkt.accel[2] = ac.accel_body.z;
        pkt.uncompAccel[0] = ac.accel_body.x;
        pkt.uncompAccel[1] = ac.accel_body.y;
        pkt.uncompAccel[2] = ac.accel_body.z;
        pkt.uncompAngRate[0] = ac.gyro.x;
        pkt.uncompAngRate[1] = ac.gyro.y;
        pkt.uncompAngRate[2] = ac.gyro.z;
        pkt.mag[0] = ac.mag_bf.x * 0.001f;
        pkt.mag[1] = ac.mag_bf.y * 0.001f;
        pkt.mag[2] = ac.mag_bf.z * 0.001f;
        float p = 0, t_k = 0;
        get_pressure_temperature_for_alt_amsl((ac.location.alt * 0.01f), p, t_k);
        pkt.temp = t_k - 273.15f;
        pkt.pressure = p * 0.001f;
        const std::uint8_t sync = 0xFA;
        write_to_autopilot(reinterpret_cast<const char*>(&sync), 1);
        write_to_autopilot(reinterpret_cast<const char*>(VN_IMU_packet_sim::header), sizeof(VN_IMU_packet_sim::header));
        write_to_autopilot(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
        write_crc_swabbed(VN_IMU_packet_sim::header, sizeof(VN_IMU_packet_sim::header), &pkt, sizeof(pkt));
    }

    void send_ins_ekf_packet(const Aircraft& ac) {
        VN_INS_ekf_packet_sim pkt{};
        pkt.timeStartup = ac.time_now_us * 1000ULL;
        float r, p, y;
        ac.get_dcm().to_euler(&r, &p, &y);
        pkt.ypr[0] = math::degrees(y);
        pkt.ypr[1] = math::degrees(p);
        pkt.ypr[2] = math::degrees(r);
        const auto q = quat_from_aircraft(ac);
        pkt.quaternion[0] = q.q2;
        pkt.quaternion[1] = q.q3;
        pkt.quaternion[2] = q.q4;
        pkt.quaternion[3] = q.q1;
        pkt.yprU[0] = 0.03f;
        pkt.yprU[1] = 0.03f;
        pkt.yprU[2] = 0.15f;
        pkt.insStatus = 0x0306;
        pkt.posLla[0] = (ac.location.lat * 1.0e-7);
        pkt.posLla[1] = (ac.location.lng * 1.0e-7);
        pkt.posLla[2] = (ac.location.alt * 0.01f);
        pkt.velNed[0] = ac.velocity_ef.x;
        pkt.velNed[1] = ac.velocity_ef.y;
        pkt.velNed[2] = ac.velocity_ef.z;
        pkt.posU = 0.5f;
        pkt.velU = 0.25f;
        const std::uint8_t sync = 0xFA;
        write_to_autopilot(reinterpret_cast<const char*>(&sync), 1);
        write_to_autopilot(reinterpret_cast<const char*>(VN_INS_ekf_packet_sim::header), sizeof(VN_INS_ekf_packet_sim::header));
        write_to_autopilot(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
        write_crc_swabbed(VN_INS_ekf_packet_sim::header, sizeof(VN_INS_ekf_packet_sim::header), &pkt, sizeof(pkt));
    }

    void send_ins_gnss_packet(const Aircraft& ac) {
        VN_INS_gnss_packet_sim pkt{};
        pkt.timeStartup = ac.time_now_us * 1000ULL;
        pkt.timeGps = (ac.time_now_us % 1000000ULL) * 1000ULL;
        pkt.numSats1 = 19;
        pkt.fix1 = 3;
        pkt.posLla1[0] = (ac.location.lat * 1.0e-7);
        pkt.posLla1[1] = (ac.location.lng * 1.0e-7);
        pkt.posLla1[2] = (ac.location.alt * 0.01f);
        pkt.velNed1[0] = 0.05f;
        pkt.velNed1[1] = 0.05f;
        pkt.velNed1[2] = 0.05f;
        pkt.posU1[0] = 1;
        pkt.posU1[1] = 1;
        pkt.posU1[2] = 1.5f;
        pkt.numSats2 = 18;
        pkt.fix2 = 3;
        const std::uint8_t sync = 0xFA;
        write_to_autopilot(reinterpret_cast<const char*>(&sync), 1);
        write_to_autopilot(reinterpret_cast<const char*>(VN_INS_gnss_packet_sim::header), sizeof(VN_INS_gnss_packet_sim::header));
        write_to_autopilot(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
        write_crc_swabbed(VN_INS_gnss_packet_sim::header, sizeof(VN_INS_gnss_packet_sim::header), &pkt, sizeof(pkt));
    }

    void send_ahrs_packet(const Aircraft& ac) {
        VN_AHRS_ekf_packet_sim pkt{};
        pkt.timeStartup = ac.time_now_us * 1000ULL;
        float r, p, y;
        ac.get_dcm().to_euler(&r, &p, &y);
        pkt.ypr[0] = math::degrees(y);
        pkt.ypr[1] = math::degrees(p);
        pkt.ypr[2] = math::degrees(r);
        const auto q = quat_from_aircraft(ac);
        pkt.quaternion[0] = q.q2;
        pkt.quaternion[1] = q.q3;
        pkt.quaternion[2] = q.q4;
        pkt.quaternion[3] = q.q1;
        pkt.yprU[0] = 0.03f;
        pkt.yprU[1] = 0.03f;
        pkt.yprU[2] = 0.15f;
        const std::uint8_t sync = 0xFA;
        write_to_autopilot(reinterpret_cast<const char*>(&sync), 1);
        write_to_autopilot(reinterpret_cast<const char*>(VN_AHRS_ekf_packet_sim::header), sizeof(VN_AHRS_ekf_packet_sim::header));
        write_to_autopilot(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
        write_crc_swabbed(VN_AHRS_ekf_packet_sim::header, sizeof(VN_AHRS_ekf_packet_sim::header), &pkt, sizeof(pkt));
    }

    void nmea_printf(const char* body_without_star) {
        const std::string s = nmea_checksum_line(body_without_star);
        write_to_autopilot(s.data(), s.size());
    }

    void update(const Aircraft& ac) {
        const std::uint32_t now = static_cast<std::uint32_t>(ac.time_now_us);
        if (now - last_imu_pkt_us >= 20000) {
            last_imu_pkt_us = now;
            send_imu_packet(ac);
        }
        if (model == VNModel::VN300) {
            if (now - last_ekf_pkt_us >= 20000) {
                last_ekf_pkt_us = now;
                send_ins_ekf_packet(ac);
            }
            if (now - last_gnss_pkt_us >= 200000) {
                last_gnss_pkt_us = now;
                send_ins_gnss_packet(ac);
            }
        } else if (now - last_ahrs_pkt_us >= 20000) {
            last_ahrs_pkt_us = now;
            send_ahrs_packet(ac);
        }
        char receive_buf[50]{};
        const ssize_t n = read_from_autopilot(receive_buf, sizeof(receive_buf));
        if (n <= 0) {
            return;
        }
        ssize_t remain = n;
        if (remain >= 9 && std::strncmp(receive_buf, "$VNRRG,01", 9) == 0) {
            if (model == VNModel::VN100) {
                nmea_printf("$VNRRG,01,VN-100-SITL");
            } else {
                nmea_printf("$VNRRG,01,VN-300-SITL");
            }
            std::memmove(receive_buf, receive_buf + 9, static_cast<std::size_t>(remain - 9));
            remain -= 9;
        }
        if (remain > 0) {
            write_to_autopilot(receive_buf, static_cast<std::size_t>(remain));
        }
    }
};

}  // namespace fwcpp::sim
