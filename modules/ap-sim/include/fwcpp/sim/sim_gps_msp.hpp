#pragma once

// Port of libraries/SITL/SIM_GPS_MSP.h/.cpp. MSP $X< GPS (cmd 0x1F03)
// with crc8_dvb_s2_update over flags..sec.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_gps.hpp>

namespace fwcpp::sim {

#if defined(__GNUC__)
#define FWCPP_SIM_PACKED __attribute__((packed))
#else
#define FWCPP_SIM_PACKED
#endif

class GPS_MSP : public GPS_Backend {
public:
    using GPS_Backend::GPS_Backend;
    GPS_MSP(const GPS_MSP&) = delete;
    GPS_MSP& operator=(const GPS_MSP&) = delete;
    void publish(const GPS_Data* d) override;
};

inline void GPS_MSP::publish(const GPS_Data* d) {
    struct FWCPP_SIM_PACKED {
        struct FWCPP_SIM_PACKED {
            std::uint8_t dollar = 0x24;
            std::uint8_t magic = 0x58;
            std::uint8_t code = 0x3C;
            std::uint8_t flags;
            std::uint16_t cmd = 0x1F03;
            std::uint16_t size = 52;
        } hdr;
        std::uint8_t instance;
        std::uint16_t gps_week;
        std::uint32_t ms_tow;
        std::uint8_t fix_type;
        std::uint8_t satellites_in_view;
        std::uint16_t horizontal_pos_accuracy;
        std::uint16_t vertical_pos_accuracy;
        std::uint16_t horizontal_vel_accuracy;
        std::uint16_t hdop;
        std::int32_t longitude;
        std::int32_t latitude;
        std::int32_t msl_altitude;
        std::int32_t ned_vel_north;
        std::int32_t ned_vel_east;
        std::int32_t ned_vel_down;
        std::uint16_t ground_course;
        std::uint16_t true_yaw;
        std::uint16_t year;
        std::uint8_t month;
        std::uint8_t day;
        std::uint8_t hour;
        std::uint8_t min;
        std::uint8_t sec;
        std::uint8_t crc;
    } msp_gps {};

    auto t = gps_time();
    struct timeval tv {};
    simulation_timeval(&tv);
    struct tm tvd {};
    // tv_sec is time_t on Linux but a 32-bit long on Windows, so it cannot be
    // passed as time_t* directly. Copy through a time_t local -- correct on
    // both platforms and no #ifdef needed.
    const time_t tv_secs = static_cast<time_t>(tv.tv_sec);
    auto* tm = gmtime_r(&tv_secs, &tvd);
    msp_gps.gps_week = t.week;
    msp_gps.ms_tow = t.ms;
    msp_gps.fix_type = d->have_lock ? 3 : 0;
    msp_gps.satellites_in_view = d->have_lock ? d->num_sats : 3;
    msp_gps.horizontal_pos_accuracy = static_cast<std::uint16_t>(d->horizontal_acc * 100);
    msp_gps.vertical_pos_accuracy = static_cast<std::uint16_t>(d->vertical_acc * 100);
    msp_gps.horizontal_vel_accuracy = 30;
    msp_gps.hdop = 100;
    msp_gps.longitude = static_cast<std::int32_t>(d->longitude * 1.0e7);
    msp_gps.latitude = static_cast<std::int32_t>(d->latitude * 1.0e7);
    msp_gps.msl_altitude = static_cast<std::int32_t>(d->altitude * 100);
    msp_gps.ned_vel_north = static_cast<std::int32_t>(100 * d->speedN);
    msp_gps.ned_vel_east = static_cast<std::int32_t>(100 * d->speedE);
    msp_gps.ned_vel_down = static_cast<std::int32_t>(100 * d->speedD);
    msp_gps.ground_course = static_cast<std::uint16_t>(math::degrees(static_cast<float>(std::atan2(d->speedE, d->speedN))) * 100);
    msp_gps.true_yaw = static_cast<std::uint16_t>(math::wrap_360(d->yaw_deg) * 100U);
    msp_gps.year = static_cast<std::uint16_t>(tm->tm_year);
    msp_gps.month = static_cast<std::uint8_t>(tm->tm_mon);
    msp_gps.day = static_cast<std::uint8_t>(tm->tm_mday);
    msp_gps.hour = static_cast<std::uint8_t>(tm->tm_hour);
    msp_gps.min = static_cast<std::uint8_t>(tm->tm_min);
    msp_gps.sec = static_cast<std::uint8_t>(tm->tm_sec);
    msp_gps.crc = crc8_dvb_s2_update(0, &msp_gps.hdr.flags, sizeof(msp_gps) - 4);
    write_to_autopilot(reinterpret_cast<const char*>(&msp_gps), sizeof(msp_gps));
}

#undef FWCPP_SIM_PACKED

}  // namespace fwcpp::sim
