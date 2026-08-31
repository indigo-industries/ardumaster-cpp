#pragma once

// CCP-065: leftover mission (arm / takeoff / outbound 1 mile North / RTL /
// land) driving SIM_Multicopter Frame/Motor plant via SitlCopterHarness::step.
//
// Horizontal motion is original-source AC_PosControl NE (CCP-027) plus WPNav
// destination/RTL (CCP-028). leftover_apply_collective is NOT the XY path —
// it only stores collective throttle. SitlCopterHarness mixes MotorsMatrix PWM
// from PosControl lean targets + D throttle (differential PWM into SimMulticopter).

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <fwcpp/copter/land_detector.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/takeoff.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/pid/ac_pid_basic.hpp>
#include <fwcpp/poscontrol/pos_control_d.hpp>
#include <fwcpp/poscontrol/pos_control_defaults.hpp>
#include <fwcpp/poscontrol/pos_control_ne.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

namespace fwcpp::hal_sitl::copter_sitl_run {

// 1 statute mile. Outbound heading is North (NED +X, 0 deg).
inline constexpr float kOutboundMileM = 1609.34f;
inline constexpr float kOutboundHeadingDeg = 0.0f;
inline constexpr float kWpSpeedMs = 10.0f;
inline constexpr float kLeanAngleMaxDeg = 30.0f;

enum class MissionPhase : std::uint8_t {
    kDisarmed = 0,
    kTakeoff = 1,
    kOutbound = 2,
    kRtl = 3,
    kLand = 4,
    kLanded = 5,
};

struct LeftoverMission {
    MissionPhase phase{MissionPhase::kDisarmed};
    float takeoff_alt_m{10.0f};
    float climb_command{0.70f};
    float land_command{0.20f};
    float hold_s{2.0f};
    float hold_elapsed_s{0.0f};
    float command{0.0f};
    copter::TakeOffState takeoff{};
    // Horizontal WP: default 1 mile North. Unit tests may shorten this.
    float outbound_n_m{kOutboundMileM};
    float outbound_e_m{0.0f};
    wpnav::WpNav wpnav{};
    bool wpnav_inited{false};
    bool outbound_wp_set{false};
    bool rtl_wp_set{false};
    bool phase_changed{false};
    float land_start_n_m{0.0f};
    float land_start_e_m{0.0f};
    float land_start_d_m{0.0f};
};

[[nodiscard]] inline const char* mission_phase_name(MissionPhase phase) {
    switch (phase) {
    case MissionPhase::kDisarmed:
        return "DISARMED";
    case MissionPhase::kTakeoff:
        return "TAKEOFF";
    case MissionPhase::kOutbound:
        return "OUTBOUND";
    case MissionPhase::kRtl:
        return "RTL";
    case MissionPhase::kLand:
        return "LAND";
    case MissionPhase::kLanded:
        return "LANDED";
    }
    return "?";
}

inline void leftover_set_phase(LeftoverMission& mission, MissionPhase next) {
    if (mission.phase != next) {
        mission.phase = next;
        mission.phase_changed = true;
    }
}

// CCP-064: AC_PosControl D cascade (pos -> vel -> accel -> throttle).
// Replaces the leftover 1-line vz damper. NED +z down. throttle_hover is
// Frame::hover_command() so harness PWM expo matches hover.
inline void leftover_init_poscontrol(copter::LeftoverCopter& copter) {
    if (copter.pos_d_inited) {
        return;
    }
    copter.p_pos_d = pid::AcP1d::with_kp(1.0f);
    copter.pid_vel_d = pid::AcPidBasic::with_gains(5.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    copter.pid_accel_d.set_kP(0.5f);
    copter.pid_accel_d.set_kI(1.0f);
    copter.pid_accel_d.set_imax(1.0f);
    copter.d_limits = poscontrol::d_set_max_speed_accel_m(
        copter.d_limits, poscontrol::kPoscontrolSpeedDownMs, poscontrol::kPoscontrolSpeedUpMs,
        poscontrol::kPoscontrolAccelDMss, poscontrol::kPoscontrolJerkDMsss, copter.pid_accel_d);
    copter.pos_d_inited = true;
}

[[nodiscard]] inline float leftover_poscontrol_throttle(copter::LeftoverCopter& copter,
                                                        const sim::SimMulticopter& sim,
                                                        float pos_d_target_m,
                                                        float vel_d_desired_ms) {
    leftover_init_poscontrol(copter);
    copter.pos_d.pos_desired_m = pos_d_target_m;
    copter.pos_d.vel_desired_ms = vel_d_desired_ms;
    poscontrol::DUpdateInputs inp{};
    inp.dt = copter.loop_dt > 0.0f ? copter.loop_dt : 0.0025f;
    inp.now_ms = copter.now_ms;
    inp.estimates.pos_m = sim.position.z;
    inp.estimates.vel_ms = sim.velocity_ef.z;
    inp.estimated_accel_d_mss = 0.0f;
    inp.throttle_hover = sim.hover_command();
    inp.vel_max_down_ms = copter.d_limits.vel_max_down_ms;
    if (inp.vel_max_down_ms <= 0.0f) {
        inp.vel_max_down_ms = poscontrol::kPoscontrolSpeedDownMs;
    }
    const auto out = copter.pos_d.update_controller(copter.p_pos_d, copter.pid_vel_d, copter.pid_accel_d, inp);
    copter.throttle_out = math::constrain_value(out.throttle_out, 0.0f, 1.0f);
    return copter.throttle_out;
}

// Hold: AC_PosControl D to the leftover mission altitude (not vz damper).
[[nodiscard]] inline float leftover_hold_command(copter::LeftoverCopter& copter,
                                                 const sim::SimMulticopter& sim,
                                                 float hold_alt_m) {
    return leftover_poscontrol_throttle(copter, sim, -hold_alt_m, 0.0f);
}

inline void leftover_init_poscontrol_ne(copter::LeftoverCopter& copter, const sim::SimMulticopter& sim) {
    if (copter.pos_ne_inited) {
        return;
    }
    copter.p_pos_ne = pid::AcP2d::with_kp(poscontrol::kNePosP);
    copter.pid_vel_ne = pid::AcPid2d::ne_velocity();
    poscontrol::AttitudeCapability att{};
    att.ang_vel_roll_max_rads = 3.0f;
    att.ang_vel_pitch_max_rads = 3.0f;
    att.accel_roll_max_radss = 20.0f;
    att.accel_pitch_max_radss = 20.0f;
    att.bf_feedforward = false;
    copter.ne_limits = poscontrol::ne_set_max_speed_accel_m(kWpSpeedMs, wpnav::kWpnavAccelerationMss,
                                                            poscontrol::kPoscontrolJerkNeMsss, att);
    poscontrol::ne_set_correction_speed_accel_m(copter.p_pos_ne, kWpSpeedMs, wpnav::kWpnavAccelerationMss);

    poscontrol::NeInitInputs inp{};
    inp.estimates.pos_m = {sim.position.x, sim.position.y};
    inp.estimates.vel_ms = {sim.velocity_ef.x, sim.velocity_ef.y};
    inp.lean_angle_max_rad = math::radians(kLeanAngleMaxDeg);
    inp.now_ms = copter.now_ms;
    inp.ticks = copter.tick_count;
    (void)copter.pos_ne.init_controller(copter.ne_offsets, copter.pid_vel_ne, inp);
    copter.pos_ne_inited = true;
}

// Original-source PosControl NE: shape toward dest (WPNav) then update_controller
// lean angles. Hold (shape_to_wp=false) parks pos_desired at dest N/E.
inline void leftover_poscontrol_ne_update(copter::LeftoverCopter& copter, const sim::SimMulticopter& sim,
                                          bool shape_to_wp, float dest_n, float dest_e) {
    leftover_init_poscontrol_ne(copter, sim);
    const float dt = copter.loop_dt > 0.0f ? copter.loop_dt : 0.0025f;
    math::Vector2f pos_error{static_cast<float>(copter.pos_ne.pos_desired_m.x) - sim.position.x,
                             static_cast<float>(copter.pos_ne.pos_desired_m.y) - sim.position.y};
    math::Vector2f vel_error{copter.pos_ne.vel_desired_ms.x - sim.velocity_ef.x,
                             copter.pos_ne.vel_desired_ms.y - sim.velocity_ef.y};
    if (shape_to_wp) {
        math::Vector2<math::postype_t> pos_ne_m{dest_n, dest_e};
        math::Vector2f vel_ne_ms{};
        math::Vector2f accel_ne_mss{};
        copter.pos_ne.input_pos_vel_accel(pos_ne_m, vel_ne_ms, accel_ne_mss, copter.ne_limits, dt, true,
                                          pos_error, vel_error);
    } else {
        copter.pos_ne.pos_desired_m = {dest_n, dest_e};
        copter.pos_ne.vel_desired_ms.zero();
        copter.pos_ne.accel_desired_mss.zero();
    }

    poscontrol::NeUpdateInputs inp{};
    inp.dt = dt;
    inp.vel_max_ne_ms = copter.ne_limits.vel_max_ne_ms;
    inp.estimates.pos_m = {sim.position.x, sim.position.y};
    inp.estimates.vel_ms = {sim.velocity_ef.x, sim.velocity_ef.y};
    inp.offsets = copter.ne_offsets.current;
    inp.lean_angle_max_rad = math::radians(kLeanAngleMaxDeg);
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    sim.dcm.to_euler(&roll, &pitch, &yaw);
    inp.cos_yaw = std::cos(yaw);
    inp.sin_yaw = std::sin(yaw);
    inp.att_yaw_target_rad = 0.0f;  // hold North

    const auto out =
        copter.pos_ne.update_controller(copter.p_pos_ne, copter.pid_vel_ne, inp, copter.ne_disturb);
    copter.roll_target_rad = out.roll_target_rad;
    copter.pitch_target_rad = out.pitch_target_rad;
}

inline void leftover_init_wpnav(LeftoverMission& mission, copter::LeftoverCopter& copter,
                                const sim::SimMulticopter& sim) {
    if (mission.wpnav_inited) {
        return;
    }
    wpnav::AttitudeJerkLimits att{};
    att.ang_vel_roll_max_rads = 3.0f;
    att.ang_vel_pitch_max_rads = 3.0f;
    mission.wpnav.set_wp_speed_ms(kWpSpeedMs);
    mission.wpnav.set_wp_accel_mss(wpnav::kWpnavAccelerationMss);
    mission.wpnav.set_wp_radius_m(5.0f);
    const math::Vector3<float> stopping{sim.position.x, sim.position.y, sim.position.z};
    mission.wpnav.wp_and_spline_init_m(kWpSpeedMs, stopping, copter.now_ms, att);
    mission.wpnav_inited = true;
}

inline bool leftover_set_wp_ned(LeftoverMission& mission, copter::LeftoverCopter& copter,
                                const sim::SimMulticopter& sim, float n, float e, float d) {
    leftover_init_wpnav(mission, copter, sim);
    wpnav::SetWpDestinationContext ctx{};
    ctx.now_ms = copter.now_ms;
    ctx.attitude.ang_vel_roll_max_rads = 3.0f;
    ctx.attitude.ang_vel_pitch_max_rads = 3.0f;
    ctx.stopping_point_ned_m = {sim.position.x, sim.position.y, sim.position.z};
    return mission.wpnav.set_wp_destination_ned_m({n, e, d}, false, 0.0f, ctx);
}

inline void leftover_tick_wpnav(LeftoverMission& mission, copter::LeftoverCopter& copter) {
    if (!mission.wpnav_inited) {
        return;
    }
    wpnav::UpdateWpNavContext ctx{};
    ctx.now_ms = copter.now_ms;
    ctx.dt_s = copter.loop_dt > 0.0f ? copter.loop_dt : 0.0025f;
    (void)mission.wpnav.update_wpnav(ctx);
}

[[nodiscard]] inline bool leftover_wp_arrived(const LeftoverMission& mission, const sim::SimMulticopter& sim) {
    const math::Vector3<float> pos{sim.position.x, sim.position.y, sim.position.z};
    const float dist = mission.wpnav.get_wp_distance_to_destination_m(pos);
    const float spd = std::hypot(sim.velocity_ef.x, sim.velocity_ef.y);
    const float radius = std::max(mission.wpnav.wp_radius_m(), 8.0f);
    return dist < radius && spd < 2.0f;
}

// leftover_apply_collective is NOT the XY path. It only stores collective
// throttle (and zeros lean) so SitlCopterHarness MotorsMatrix mixing can
// run. Hover tests use this; the mile mission uses PosControl NE lean.
inline void leftover_apply_collective(copter::LeftoverCopter& copter, const sim::SimMulticopter& sim, float command,
                                      float dt = 0.0025f) {
    (void)sim;
    (void)dt;
    copter.throttle_out = math::constrain_value(command, 0.0f, 1.0f);
    copter.roll_target_rad = 0.0f;
    copter.pitch_target_rad = 0.0f;
}

inline void leftover_mission_begin_takeoff(LeftoverMission& mission) {
    leftover_set_phase(mission, MissionPhase::kTakeoff);
    mission.hold_elapsed_s = 0.0f;
}

inline void leftover_mission_advance(copter::LeftoverCopter& copter, sim::SimMulticopter& sim, LeftoverMission& mission,
                                     float dt) {
    (void)dt;
    mission.phase_changed = false;
    const float alt_m = -sim.position.z;
    leftover_tick_wpnav(mission, copter);

    switch (mission.phase) {
    case MissionPhase::kDisarmed:
        copter.motors_armed = false;
        copter.land_complete = true;
        mission.command = 0.0f;
        copter.throttle_out = 0.0f;
        copter.roll_target_rad = 0.0f;
        copter.pitch_target_rad = 0.0f;
        break;

    case MissionPhase::kTakeoff: {
        copter.motors_armed = true;
        if (copter.land_complete) {
            copter::UserTakeoffInputs in;
            in.motors_armed = true;
            in.land_complete = true;
            in.has_user_takeoff = true;
            in.takeoff_alt_m = mission.takeoff_alt_m;
            in.current_alt_m = alt_m;
            copter::UserTakeoffEffects fx;
            if (leftover_do_user_takeoff_U_m(in, fx, &mission.takeoff, alt_m) && fx.leftover_takeoff_start_m) {
                copter.land_complete = false;
            }
        }
        mission.command = leftover_poscontrol_throttle(copter, sim, -mission.takeoff_alt_m, -2.5f);
        leftover_poscontrol_ne_update(copter, sim, false, 0.0f, 0.0f);
        if (alt_m >= mission.takeoff_alt_m) {
            mission.takeoff._running = false;
            leftover_set_phase(mission, MissionPhase::kOutbound);
        }
        break;
    }

    case MissionPhase::kOutbound: {
        copter.motors_armed = true;
        leftover_init_wpnav(mission, copter, sim);
        if (!mission.outbound_wp_set) {
            leftover_set_wp_ned(mission, copter, sim, mission.outbound_n_m, mission.outbound_e_m,
                                -mission.takeoff_alt_m);
            mission.outbound_wp_set = true;
        }
        leftover_tick_wpnav(mission, copter);
        mission.command = leftover_hold_command(copter, sim, mission.takeoff_alt_m);
        leftover_poscontrol_ne_update(copter, sim, true, mission.outbound_n_m, mission.outbound_e_m);
        if (leftover_wp_arrived(mission, sim)) {
            leftover_set_phase(mission, MissionPhase::kRtl);
        }
        break;
    }

    case MissionPhase::kRtl: {
        copter.motors_armed = true;
        if (!mission.rtl_wp_set) {
            leftover_set_wp_ned(mission, copter, sim, 0.0f, 0.0f, -mission.takeoff_alt_m);
            mission.rtl_wp_set = true;
        }
        leftover_tick_wpnav(mission, copter);
        mission.command = leftover_hold_command(copter, sim, mission.takeoff_alt_m);
        leftover_poscontrol_ne_update(copter, sim, true, 0.0f, 0.0f);
        if (leftover_wp_arrived(mission, sim)) {
            leftover_set_phase(mission, MissionPhase::kLand);
            mission.land_start_n_m = sim.position.x;
            mission.land_start_e_m = sim.position.y;
            mission.land_start_d_m = sim.position.z;
        }
        break;
    }

    case MissionPhase::kLand: {
        copter.motors_armed = true;
        mission.command = leftover_poscontrol_throttle(copter, sim, 0.25f, 1.5f);
        leftover_poscontrol_ne_update(copter, sim, false, 0.0f, 0.0f);
        if (sim.on_ground()) {
            copter::LandDetectorInputs lin;
            lin.motors_armed = true;
            lin.land_complete = copter.land_complete;
            lin.descent_rate_low = true;
            lin.throttle_at_lower_limit = true;
            lin.motors_throttle_low = true;
            lin.throttle_mix_min = true;
            lin.accel_stationary = true;
            lin.rangefinder_check = true;
            lin.wow_check = true;
            copter::LandDetectorEffects lfx;
            leftover_update_land_and_crash_detectors(lin, lfx);
            if (lfx.land_complete) {
                copter.land_complete = true;
                copter.motors_armed = false;
                mission.command = 0.0f;
                copter.throttle_out = 0.0f;
                leftover_set_phase(mission, MissionPhase::kLanded);
            }
        }
        break;
    }

    case MissionPhase::kLanded:
        copter.motors_armed = false;
        copter.land_complete = true;
        mission.command = 0.0f;
        copter.throttle_out = 0.0f;
        copter.roll_target_rad = 0.0f;
        copter.pitch_target_rad = 0.0f;
        break;
    }
}

// Mission leftover then CCP-043/045/065 harness (sensors + NE/D motors + plant).
inline void leftover_copter_sitl_step(SitlCopterHarness& harness, LeftoverMission& mission, float dt) {
    leftover_mission_advance(harness.copter(), harness.sim(), mission, dt);
    harness.step(dt);
}

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table; CCP-065 mile/RTL mission + CCP-045 plant"},
    {"copter_sitl_run scaffold", PortStatus::kThisSlice,
     "sitl/copter_main.cpp + CMake copter_sitl_run target"},
    {"leftover_mission_advance", PortStatus::kThisSlice, "arm / takeoff / outbound / RTL / land leftover state machine"},
    {"leftover_hold_command", PortStatus::kThisSlice,
     "CCP-064: altitude via AC_PosControl D cascade (not leftover vz damper)"},
    {"leftover_poscontrol_throttle", PortStatus::kThisSlice,
     "CCP-064: pos_desired/vel_desired -> PosControlD::update_controller throttle_out"},
    {"leftover_poscontrol_ne", PortStatus::kThisSlice,
     "CCP-065: AC_PosControl NE input_pos_vel_accel + update_controller lean -> harness PWM"},
    {"leftover_wpnav_rtl", PortStatus::kThisSlice,
     "CCP-065: WPNav set_wp_destination_ned_m outbound 1 mile North then RTL origin NED 0,0"},
    {"leftover_copter_loop", PortStatus::kOnMain,
     "CCP-064: leftover_copter_tick walks Copter scheduler leftover free functions"},
    {"leftover_apply_collective", PortStatus::kThisSlice,
     "stores collective throttle only; NOT the XY path (PosControl NE lean is)"},
    {"leftover_copter_sitl_step", PortStatus::kThisSlice,
     "mission then SitlCopterHarness::step (sensors + MotorsMatrix PWM + SimMulticopter)"},
    {"copter_sitl_run takeoff/outbound/rtl/land", PortStatus::kThisSlice,
     "main() 1 mile North then RTL/land on the real Frame/Motor plant"},
    {"SitlCopterHarness sensor synth (CCP-043)", PortStatus::kOnMain,
     "sitl_copter_harness.hpp"},
    {"leftover takeoff / land_detector (CCP-041)", PortStatus::kOnMain,
     "takeoff.hpp + land_detector.hpp remaining_count()==0"},
    {"SIM_Multicopter Frame/Motor mixing", PortStatus::kOnMain,
     "CCP-045: sim_multicopter.hpp / sim_frame.hpp / sim_motor.hpp - not leftover body-z"},
    {"GCS / MAVLink / interactive run", PortStatus::kOutOfScope,
     "no GCS in this port; bounded duration like CPP-085"},
    {"AP:: / HAL SITL singletons", PortStatus::kOutOfScope, "ADR-0012 explicit refs"},
    {"Rust copter-sitl", PortStatus::kOutOfScope, "Do not copy Rust"},
    {"JSON custom frame models / battery drain / shove-twist-clamp", PortStatus::kOutOfScope,
     "optional original extras; default_model + constant voltage"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace fwcpp::hal_sitl::copter_sitl_run
