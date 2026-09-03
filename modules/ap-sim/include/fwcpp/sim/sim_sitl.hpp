#pragma once

// Port of the SIM_* parameter surface from libraries/SITL/SITL.h that the
// plants actually read: battery, wind, shove, twist, vibe_motor, clamp,
// mag anomaly / motor interference. No AP_Param singleton — callers hold
// a SitlParams and pass it into Aircraft.

#include <cstdint>

#include <fwcpp/math/vector3.hpp>

namespace fwcpp::sim {

struct ImpulseForce {
    std::uint32_t t{0};
    std::uint32_t start_ms{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct SitlParams {
    float batt_voltage{12.6f};
    float batt_capacity_ah{0.0f};
    float batt_resistance{-1.0f};

    float wind_speed{0.0f};
    float wind_direction{0.0f};
    float wind_turbulance{0.0f};
    float wind_dir_z{0.0f};

    ImpulseForce shove{};
    ImpulseForce twist{};

    float vibe_motor{0.0f};
    float vibe_motor_scale{0.0f};
    std::int16_t vibe_motor_harmonics{0};
    std::int32_t vibe_motor_mask{0};

    std::int8_t gnd_behav{-1};
    std::int8_t clamp_ch{0};

    math::Vector3f mag_anomaly_ned{};
    float mag_anomaly_hgt{1.0f};
    math::Vector3f mag_mot{};
};

}  // namespace fwcpp::sim
