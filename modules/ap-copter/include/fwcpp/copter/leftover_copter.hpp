#pragma once

// CCP-043: thin leftover Copter vehicle shell for SitlCopterHarness.
// Upstream counterpart is ArduCopter/Copter.h state the SITL HAL feeds;
// this port has no full Copter object yet (CCP-035 leftovers are free
// functions + Mode*). ADR-0012: no AP:: singletons — sensor samples are
// buffers/flags the harness writes, then leftover_copter_tick() consumes.
//
// CCP-045: motor_pwm[32] is the sitl_input.servos[] path. SitlCopterHarness
// feeds these into SimMulticopter Frame/Motor (not leftover body-z).
//
// CCP-064: leftover_copter_tick is Copter::loop (leftover scheduler free
// functions in FAST_TASK / SCHED_TASK order). AC_PosControl D cascade
// lives on this shell so sitl hold/takeoff/land is not a 1-line vz damper.

#include <cstdint>

#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/pid/ac_pid_2d.hpp>
#include <fwcpp/pid/ac_pid_basic.hpp>
#include <fwcpp/poscontrol/pos_control_d.hpp>
#include <fwcpp/poscontrol/pos_control_ne.hpp>
namespace fwcpp::copter {

struct LeftoverCopter {
    math::Vector3f gyro_buffer{};
    math::Vector3f accel_buffer{};
    float baro_altitude_m{0.0f};
    std::int32_t gps_lat{0};
    std::int32_t gps_lng{0};
    math::Vector3f compass_field_bf{};

    bool gyro_injected{false};
    bool accel_injected{false};
    bool baro_injected{false};
    bool gps_injected{false};
    bool compass_injected{false};

    bool motors_armed{false};
    bool motors_armed_injected{false};
    SpoolState spool_state{SpoolState::SHUT_DOWN};
    bool spool_injected{false};
    bool attitude_hold{false};
    bool attitude_hold_injected{false};

    // CCP-045: PWM microseconds, sitl_input.servos layout (motor.servo index).
    std::uint16_t motor_pwm[32]{};

    std::int32_t home_lat{-353632621};
    std::int32_t home_lng{1491652374};

    std::uint32_t tick_count{0};
    Mode* current{nullptr};
    bool land_complete{false};
    bool move_vehicle_on_ekf_reset{false};

    // CCP-064: Copter::loop timing + AC_PosControl D state.
    float loop_dt{0.0025f};
    std::uint32_t now_ms{0};
    Location current_loc{};
    poscontrol::PosControlD pos_d{};
    pid::AcP1d p_pos_d{};
    pid::AcPidBasic pid_vel_d{};
    pid::AcPid pid_accel_d{pid::AcPid::Gains{}};
    poscontrol::DLimits d_limits{};
    bool pos_d_inited{false};
    float throttle_out{0.0f};
    poscontrol::PosControlNe pos_ne{};
    pid::AcP2d p_pos_ne{};
    pid::AcPid2d pid_vel_ne{};
    poscontrol::NeLimits ne_limits{};
    poscontrol::NeOffsetState ne_offsets{};
    poscontrol::NeDisturbance ne_disturb{};
    bool pos_ne_inited{false};
    float roll_target_rad{0.0f};
    float pitch_target_rad{0.0f};
    bool loop_ran_rate_controller{false};
    bool loop_ran_motors_output{false};
    bool loop_ran_read_ahrs{false};
    bool loop_ran_read_inertia{false};
    bool loop_ran_check_ekf_reset{false};
    bool loop_ran_update_flight_mode{false};
    bool loop_ran_rc_loop{false};
    bool loop_ran_throttle_loop{false};
};

}  // namespace fwcpp::copter

#include <fwcpp/copter/leftover_copter_loop.hpp>
