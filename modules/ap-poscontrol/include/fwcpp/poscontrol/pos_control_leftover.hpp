#pragma once

// CCP-027 completeness catalog for AC_PosControl (Plane-4.7.0).

#include <cstddef>
#include <cstdint>

namespace fwcpp::poscontrol {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PosControlPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PosControlPortItem kPosControlCompleteness[] = {
    {"POSCONTROL_* default constants", PortStatus::kThisSlice,
     "pos_control_defaults.hpp mirrors AC_PosControl.h macros"},
    {"get_lean_angle_max_rad", PortStatus::kThisSlice,
     "LeanAngleMaxConfig + get_lean_angle_max_rad()"},
    {"lean_angles_rad_to_accel_NED_mss", PortStatus::kThisSlice,
     "lean_angles_rad_to_accel_ned_mss() free function"},
    {"accel_NE_mss_to_lean_angles_rad", PortStatus::kThisSlice,
     "accel_ne_mss_to_lean_angles_rad() free function"},
    {"get_thrust_vector", PortStatus::kThisSlice, "get_thrust_vector() free function"},
    {"leftover-complete catalog", PortStatus::kThisSlice, "this table"},
        {"AC_PosControl class / constructor / singleton", PortStatus::kThisSlice,
     "pos_control_class.hpp PosControl + PosControlInjectedDeps (no AP singleton)"},
    {"AP_Param var_info and gain subgroups", PortStatus::kOutOfScope,
     "PosControlConfig + Rust pos_control_var_info; no AP_Param in fw-cpp"},
    {"update_estimates", PortStatus::kThisSlice, "pos_control_accessors.hpp update_estimates()"},
    {"3D input_pos_NED_m path shaper", PortStatus::kThisSlice, "pos_control_path.hpp input_pos_ned_m()"},
    {"NE_set_max_speed_accel_* / NE limits", PortStatus::kThisSlice,
     "NeLimits + ne_set_max_speed_accel_* (pos_control_ne.hpp)"},
    {"NE input_* / init / relax / soften / stop", PortStatus::kThisSlice,
     "input_*, init_controller*, relax/soften/stop (pos_control_ne.hpp)"},
    {"NE_update_controller", PortStatus::kThisSlice, "pos_control_ne.hpp update_controller()"},
    {"yaw_from_ne_motion", PortStatus::kThisSlice, "pos_control_ne.hpp yaw_from_ne_motion()"},
    {"AcP2d / AcPid2d", PortStatus::kThisSlice, "ap-pid ac_p_2d.hpp ac_pid_2d.hpp"},
    {"AcP1d / AcPidBasic", PortStatus::kThisSlice, "ap-pid ac_p_1d.hpp ac_pid_basic.hpp"},
    {"limit_accel_xy / sqrt_controller_xy", PortStatus::kThisSlice, "ap-math control_vector.hpp"},
    {"D_update_controller", PortStatus::kThisSlice, "pos_control_d.hpp update_controller()"},
    {"D_set_max_speed_accel_* / D limits", PortStatus::kThisSlice,
     "DLimits + d_set_max_speed_accel_* (pos_control_d.hpp)"},
    {"D init / relax / stopping point", PortStatus::kThisSlice,
     "init_controller*, d_relax_controller, stopping_point_d (pos_control_d.hpp)"},
    {"D throttle vibration override", PortStatus::kThisSlice,
     "throttle_with_vibration_override (pos_control_d.hpp)"},
    {"Offsets / terrain / stopping point accessors", PortStatus::kThisSlice,
     "pos_control_accessors.hpp init_terrain, get_stopping_point_*, offset targets"},
    {"AC_PosControl::_singleton", PortStatus::kOutOfScope,
     "fw-cpp uses explicit PosControl instances"},
    {"write_log / HAL_LOGGING_ENABLED", PortStatus::kOutOfScope,
     "no onboard logger in fw-cpp unit port"},
    {"AP_SCRIPTING_ENABLED LUA offsets", PortStatus::kOutOfScope,
     "scripting not in scope"},
    {"ArduPlane get_fwd_pitch_is_limited", PortStatus::kOutOfScope,
     "fixed-wing-only branch; copter-cpp module"},
};

[[nodiscard]] inline constexpr std::size_t pos_control_completeness_size() {
    return sizeof(kPosControlCompleteness) / sizeof(kPosControlCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kPosControlCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kPosControlCompleteness) {
        bool match = true;
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                match = false;
                break;
            }
            ++a;
            ++b;
        }
        if (match && *a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t remaining_count() { return count_status(PortStatus::kRemaining); }
[[nodiscard]] inline constexpr std::size_t on_main_count() { return count_status(PortStatus::kOnMain); }
[[nodiscard]] inline constexpr std::size_t this_slice_count() { return count_status(PortStatus::kThisSlice); }
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() { return count_status(PortStatus::kOutOfScope); }

}  // namespace fwcpp::poscontrol
