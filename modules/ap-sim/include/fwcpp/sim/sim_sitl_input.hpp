#pragma once

// Port of libraries/SITL/SITL_Input.h (Copter-4.7.0 / Plane-4.7.0).
// SitlInput is the original sitl_input: servo PWM microseconds plus wind.

#include <cstdint>

namespace fwcpp::sim {

inline constexpr std::uint8_t kSitlServoChannels = 32;

struct SitlInput {
    std::uint16_t servos[kSitlServoChannels]{};
    struct {
        float speed{0.0f};       // m/s
        float direction{0.0f};   // degrees 0..360
        float turbulence{0.0f};
        float dir_z{0.0f};       // degrees -90..90
    } wind;
};

}  // namespace fwcpp::sim
