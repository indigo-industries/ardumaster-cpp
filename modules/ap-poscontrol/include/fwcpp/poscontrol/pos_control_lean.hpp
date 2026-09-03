#pragma once

// CCP-027 slice 1: lean-angle / thrust-direction helpers from AC_PosControl
// (Plane-4.7.0 AC_PosControl.cpp get_lean_angle_max_rad,
// lean_angles_rad_to_accel_NED_mss, accel_NE_mss_to_lean_angles_rad,
// get_thrust_vector). Free functions with explicit inputs ΓÇö the full
// AC_PosControl class: pos_control_class.hpp (slice 8).

#include <algorithm>
#include <cmath>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::poscontrol {

// upstream AP_Math/definitions.h GRAVITY_MSS
inline constexpr float kGravityMss = 9.80665f;

// Matches upstream AP_Math/control.cpp angle_rad_to_accel_mss (deferred in
// fwcpp::math::control.hpp vector half).
[[nodiscard]] inline float angle_rad_to_accel_mss(float angle_rad) {
    return kGravityMss * tanf(angle_rad);
}

// Matches upstream AP_Math/control.cpp accel_mss_to_angle_rad.
[[nodiscard]] inline float accel_mss_to_angle_rad(float accel_mss) {
    return atanf(accel_mss / kGravityMss);
}

/// Parameter bundle for upstream `get_lean_angle_max_rad`.
struct LeanAngleMaxConfig {
    float lean_angle_max_deg{0.0f};
    float angle_max_override_rad{0.0f};
    float attitude_lean_angle_max_rad{0.0f};
};

/// upstream AC_PosControl::get_lean_angle_max_rad
[[nodiscard]] inline float get_lean_angle_max_rad(const LeanAngleMaxConfig& cfg) {
    if (math::is_positive(cfg.angle_max_override_rad)) {
        return cfg.angle_max_override_rad;
    }
    if (!math::is_positive(cfg.lean_angle_max_deg)) {
        return cfg.attitude_lean_angle_max_rad;
    }
    return math::radians(cfg.lean_angle_max_deg);
}

/// upstream AC_PosControl::lean_angles_rad_to_accel_NED_mss
[[nodiscard]] inline math::Vector3f lean_angles_rad_to_accel_ned_mss(
    const math::Vector3f& att_target_euler_rad) {
    const float sin_roll = sinf(att_target_euler_rad.x);
    const float cos_roll = cosf(att_target_euler_rad.x);
    const float sin_pitch = sinf(att_target_euler_rad.y);
    const float cos_pitch = cosf(att_target_euler_rad.y);
    const float sin_yaw = sinf(att_target_euler_rad.z);
    const float cos_yaw = cosf(att_target_euler_rad.z);

    const float denom = std::max(cos_roll * cos_pitch, 0.1f);

    return math::Vector3f{
        kGravityMss * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / denom,
        kGravityMss * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / denom,
        -kGravityMss,
    };
}

/// upstream AC_PosControl::accel_NE_mss_to_lean_angles_rad (yaw from AHRS).
inline void accel_ne_mss_to_lean_angles_rad(float accel_n_mss, float accel_e_mss,
                                            float cos_yaw, float sin_yaw,
                                            float& roll_target_rad,
                                            float& pitch_target_rad) {
    const float accel_forward_mss = accel_n_mss * cos_yaw + accel_e_mss * sin_yaw;
    const float accel_right_mss = -accel_n_mss * sin_yaw + accel_e_mss * cos_yaw;

    pitch_target_rad = accel_mss_to_angle_rad(-accel_forward_mss);
    const float cos_pitch_target = cosf(pitch_target_rad);
    roll_target_rad = accel_mss_to_angle_rad(accel_right_mss * cos_pitch_target);
}

/// upstream AC_PosControl::get_thrust_vector
[[nodiscard]] inline math::Vector3f get_thrust_vector(const math::Vector3f& accel_target_ned_mss) {
    return math::Vector3f{accel_target_ned_mss.x, accel_target_ned_mss.y, -kGravityMss};
}

}  // namespace fwcpp::poscontrol
