#pragma once

// Port of libraries/SITL/SIM_GPS_NMEA.h/.cpp plus AP_Common/NMEA checksum
// trailer (*CS\r\n). Sentences: GPGGA, GPVTG, GPRMC, optional GPHDT/GPTHS/KSXT.

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_gps.hpp>

namespace fwcpp::sim {

inline std::string nmea_vaprintf(const char* fmt, va_list ap) {
    va_list ap_copy;
    va_copy(ap_copy, ap);
    const int len = std::vsnprintf(nullptr, 0, fmt, ap_copy);
    va_end(ap_copy);
    if (len <= 0) {
        return {};
    }
    std::string s(static_cast<std::size_t>(len) + 6, char(0));
    if (std::vsnprintf(s.data(), static_cast<std::size_t>(len) + 5, fmt, ap) < len) {
        return {};
    }
    std::uint8_t cs = 0;
    const std::uint8_t* b = reinterpret_cast<const std::uint8_t*>(s.data()) + 1;
    while (*b) {
        cs ^= *b++;
    }
    std::snprintf(s.data() + len, 6, "*%02X\r\n", static_cast<unsigned>(cs));
    s.resize(static_cast<std::size_t>(len) + 5);
    return s;
}

class GPS_NMEA : public GPS_Backend {
public:
    using GPS_Backend::GPS_Backend;
    GPS_NMEA(const GPS_NMEA&) = delete;
    GPS_NMEA& operator=(const GPS_NMEA&) = delete;
    void publish(const GPS_Data* d) override;

private:
    void nmea_printf(const char* fmt, ...);
};

inline void GPS_NMEA::nmea_printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const std::string s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (!s.empty()) {
        write_to_autopilot(s.c_str(), s.size());
    }
}

inline void GPS_NMEA::publish(const GPS_Data* d) {
    struct timeval tv {};
    simulation_timeval(&tv);
    struct tm tvd {};
    // tv_sec is time_t on Linux but a 32-bit long on Windows, so it cannot be
    // passed as time_t* directly. Copy through a time_t local -- correct on
    // both platforms and no #ifdef needed.
    const time_t tv_secs = static_cast<time_t>(tv.tv_sec);
    struct tm* tm = gmtime_r(&tv_secs, &tvd);
    char tstring[20];
    char dstring[20];
    char lat_string[20];
    char lng_string[20];
    std::snprintf(tstring, sizeof(tstring), "%02u%02u%06.3f", tm->tm_hour, tm->tm_min,
                  tm->tm_sec + tv.tv_usec * 1.0e-6);
    std::snprintf(dstring, sizeof(dstring), "%02u%02u%02u", tm->tm_mday, tm->tm_mon + 1, tm->tm_year % 100);
    double deg = std::fabs(d->latitude);
    std::snprintf(lat_string, sizeof(lat_string), "%02u%08.5f,%c", static_cast<unsigned>(deg),
                  (deg - int(deg)) * 60, d->latitude < 0 ? char(0x53) : char(0x4E));
    deg = std::fabs(d->longitude);
    std::snprintf(lng_string, sizeof(lng_string), "%03u%08.5f,%c", static_cast<unsigned>(deg),
                  (deg - int(deg)) * 60, d->longitude < 0 ? char(0x57) : char(0x45));
    nmea_printf("$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,", tstring, lat_string, lng_string,
                d->have_lock ? 1 : 0, d->have_lock ? d->num_sats : 3, 1.2, d->altitude);
    const float speed_mps = d->speed_2d();
    const float speed_knots = speed_mps * 1.9438444924406046f;
    const float ground_track_deg = math::degrees(d->ground_track_rad());
    nmea_printf("$GPVTG,%.2f,T,%.2f,M,%.2f,N,%.2f,K,A", ground_track_deg, ground_track_deg, speed_knots,
                speed_knots * (1852.0f / 3600.0f) * 3.6f);
    nmea_printf("$GPRMC,%s,%c,%s,%s,%.2f,%.2f,%s,,", tstring, d->have_lock ? char(0x41) : char(0x56), lat_string, lng_string,
                speed_knots, ground_track_deg, dstring);
    if (front.parms().hdg_enabled == GpsHeading::HDT) {
        nmea_printf("$GPHDT,%.2f,T", d->yaw_deg);
    } else if (front.parms().hdg_enabled == GpsHeading::THS) {
        nmea_printf("$GPTHS,%.2f,%c,T", d->yaw_deg, d->have_lock ? char(0x41) : char(0x56));
    } else if (front.parms().hdg_enabled == GpsHeading::KSXT) {
        nmea_printf("$KSXT,%04u%02u%02u%02u%02u%02u.%02u,%.8f,%.8f,%.4f,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,,,,%.3f,%.3f,%.3f,,",
                    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
                    static_cast<unsigned>(tv.tv_usec * 1.e-4), d->longitude, d->latitude, d->altitude,
                    math::wrap_360(d->yaw_deg), d->pitch_deg, ground_track_deg, speed_mps, d->roll_deg,
                    d->have_lock ? 1 : 0, 3, d->have_lock ? d->num_sats : 3, d->have_lock ? d->num_sats : 3,
                    d->speedE * 3.6, d->speedN * 3.6, -d->speedD * 3.6);
    }
}

}  // namespace fwcpp::sim
