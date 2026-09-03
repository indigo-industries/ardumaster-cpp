#pragma once

// Port of Aircraft::rangefinder_range / SIM_SerialRangeFinder plant:
// perpendicular distance to the ground plane along body-z, plus a
// serial rangefinder sample the harness can consume.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>

namespace fwcpp::sim {

struct SitlRangefinderSample {
    float distance_m{0.0f};
    bool healthy{true};
};

inline float rangefinder_range(const Aircraft& aircraft) {
    // Original: perpendicular_distance_to_rangefinder_surface = hagl,
    // adjusted by beam orientation. Flat-earth: hagl / cos(tilt).
    float r = 0.0f;
    float p = 0.0f;
    float y = 0.0f;
    aircraft.dcm.to_euler(&r, &p, &y);
    const float c = std::cos(r) * std::cos(p);
    const float h = aircraft.hagl();
    if (c <= 0.01f) {
        return INFINITY;
    }
    return h / c;
}

inline SitlRangefinderSample sitl_rangefinder_from_aircraft(const Aircraft& aircraft) {
    SitlRangefinderSample s;
    s.distance_m = rangefinder_range(aircraft);
    s.healthy = std::isfinite(s.distance_m) && s.distance_m < 100000.0f;
    return s;
}

}  // namespace fwcpp::sim
