#pragma once

// CCP-068: general-purpose, arbitrary-length Copter mission runner — the
// Copter analogue of Plane's ModeAUTO dispatch (plane.hpp), built entirely
// from primitives copter_sitl_run_leftover.hpp (CCP-064/065) already proved
// out on the real Frame/Motor plant: leftover_init_poscontrol /
// leftover_poscontrol_throttle / leftover_hold_command (altitude, AC_PosControl
// D), leftover_init_poscontrol_ne / leftover_poscontrol_ne_update (horizontal
// lean, AC_PosControl NE), and a NEW small set of WPNav helpers below (mirrors
// of leftover_init_wpnav / leftover_set_wp_ned / leftover_tick_wpnav /
// leftover_wp_arrived, duplicated rather than generalized in place so this
// slice touches ZERO lines of copter_sitl_run_leftover.hpp — its own CCP-065
// tests keep asserting against the exact fixed
// arm/takeoff/outbound-1-mile/RTL/land demo, unmodified).
//
// WHAT THIS ADDS OVER leftover_mission_advance: an arbitrary-length
// fwcpp::copter::Mission (Waypoint/Takeoff/Rtl/Land items, up to
// kMaxMissionItems) instead of a mission fixed at exactly
// takeoff -> one outbound point -> RTL -> land. Every item's Waypoint leg can
// carry its OWN altitude (WPNav's set_wp_destination_ned_m already takes a
// full 3D NED point — leftover_mission_advance just never varied it leg to
// leg) — this is a strict superset of what the CCP-065 demo could fly, not a
// narrower reimplementation of it.
//
// WHAT THIS DOES NOT ADD: DO-commands, loiter-for-time, splines, jump/repeat,
// conditional commands — same exclusion list Plane's own MissionItem/Mission
// documents, for the same reason (this port's mission vocabulary is
// deliberately the minimal set its Mode dispatch can act on, not a shrunken
// copy of upstream AP_Mission's ~80-command vocabulary).

#include <cstdint>

#include <fwcpp/copter/land_detector.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/mission.hpp>
#include <fwcpp/copter/takeoff.hpp>
#include <fwcpp/hal_sitl/copter_sitl_run_leftover.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

namespace fwcpp::hal_sitl::copter_auto {

// Lifecycle phase — separate from the mission ITEM's own command, because
// Land needs the same multi-tick descend-to-touchdown logic
// leftover_mission_advance already has regardless of which mission index
// commanded it, and because Disarmed/Landed are terminal states no mission
// item represents on its own.
enum class AutoPhase : std::uint8_t {
    kDisarmed = 0,
    kTakeoff = 1,
    kAuto = 2,  // flying mission.current(); dispatches on its ->command
    kLand = 3,
    kLanded = 4,
};

[[nodiscard]] inline const char* auto_phase_name(AutoPhase phase) {
    switch (phase) {
    case AutoPhase::kDisarmed:
        return "DISARMED";
    case AutoPhase::kTakeoff:
        return "TAKEOFF";
    case AutoPhase::kAuto:
        return "AUTO";
    case AutoPhase::kLand:
        return "LAND";
    case AutoPhase::kLanded:
        return "LANDED";
    }
    return "?";
}

struct AutoMissionState {
    AutoPhase phase{AutoPhase::kDisarmed};
    bool phase_changed{false};
    // Altitude (metres AGL, positive-up) the most recently completed Takeoff
    // item climbed to. This is what Rtl returns to and what a Land item
    // starts its descent from if reached without an intervening Waypoint —
    // matches leftover_mission_advance's own "RTL/land hold at the takeoff
    // altitude" behavior exactly, just no longer hardcoded to one value.
    float hold_alt_m{10.0f};
    copter::TakeOffState takeoff{};
    // Own WPNav instance + init flag — NOT copter_sitl_run::LeftoverMission's
    // (that struct is CCP-065's own, untouched). Same wp_and_spline_init_m /
    // set_wp_destination_ned_m / update_wpnav / get_wp_distance_to_destination_m
    // API, just owned here.
    wpnav::WpNav wpnav{};
    bool wpnav_inited{false};
    // Identity of the mission item we last called set_wp_destination for —
    // compared by POINTER (Mission::current() returns a stable address into
    // its fixed backing array) so a fresh destination is set exactly once
    // per item, on the tick the runner first starts flying toward it.
    const copter::MissionItem* wp_dest_set_for{nullptr};
    float land_start_n_m{0.0f};
    float land_start_e_m{0.0f};
    float land_start_d_m{0.0f};
};

inline void auto_set_phase(AutoMissionState& state, AutoPhase next) {
    if (state.phase != next) {
        state.phase = next;
        state.phase_changed = true;
    }
}

inline void auto_init_wpnav(AutoMissionState& state, copter::LeftoverCopter& copter,
                            const sim::SimMulticopter& sim) {
    if (state.wpnav_inited) {
        return;
    }
    wpnav::AttitudeJerkLimits att{};
    att.ang_vel_roll_max_rads = 3.0f;
    att.ang_vel_pitch_max_rads = 3.0f;
    state.wpnav.set_wp_speed_ms(copter_sitl_run::kWpSpeedMs);
    state.wpnav.set_wp_accel_mss(wpnav::kWpnavAccelerationMss);
    state.wpnav.set_wp_radius_m(5.0f);
    const math::Vector3<float> stopping{sim.position.x, sim.position.y, sim.position.z};
    state.wpnav.wp_and_spline_init_m(copter_sitl_run::kWpSpeedMs, stopping, copter.now_ms, att);
    state.wpnav_inited = true;
}

inline bool auto_set_wp_ned(AutoMissionState& state, copter::LeftoverCopter& copter,
                            const sim::SimMulticopter& sim, float n, float e, float d) {
    auto_init_wpnav(state, copter, sim);
    wpnav::SetWpDestinationContext ctx{};
    ctx.now_ms = copter.now_ms;
    ctx.attitude.ang_vel_roll_max_rads = 3.0f;
    ctx.attitude.ang_vel_pitch_max_rads = 3.0f;
    ctx.stopping_point_ned_m = {sim.position.x, sim.position.y, sim.position.z};
    return state.wpnav.set_wp_destination_ned_m({n, e, d}, false, 0.0f, ctx);
}

inline void auto_tick_wpnav(AutoMissionState& state, copter::LeftoverCopter& copter) {
    if (!state.wpnav_inited) {
        return;
    }
    wpnav::UpdateWpNavContext ctx{};
    ctx.now_ms = copter.now_ms;
    ctx.dt_s = copter.loop_dt > 0.0f ? copter.loop_dt : 0.0025f;
    (void)state.wpnav.update_wpnav(ctx);
}

[[nodiscard]] inline bool auto_wp_arrived(const AutoMissionState& state, const sim::SimMulticopter& sim,
                                          float acceptance_radius_m) {
    const math::Vector3<float> pos{sim.position.x, sim.position.y, sim.position.z};
    const float dist = state.wpnav.get_wp_distance_to_destination_m(pos);
    const float spd = std::hypot(sim.velocity_ef.x, sim.velocity_ef.y);
    const float radius =
        acceptance_radius_m > 0.0f ? acceptance_radius_m : std::max(state.wpnav.wp_radius_m(), 8.0f);
    return dist < radius && spd < 2.0f;
}

// Begins flying `mission` from its first item. That item MUST be
// MissionCommand::Takeoff — same "NAV_TAKEOFF is required as the first
// flown item" rule this project already enforces for every other vehicle
// type's missions (a takeoff a plan doesn't declare is a plan bug, not a
// shortcut). Returns false (mission untouched, state unchanged) if the
// mission is empty or does not start with Takeoff.
[[nodiscard]] inline bool auto_mission_begin(AutoMissionState& state, copter::Mission& mission) {
    mission.reset();
    const copter::MissionItem* first = mission.current();
    if (first == nullptr || first->command != copter::MissionCommand::Takeoff) {
        return false;
    }
    state.hold_alt_m = -first->down_m;
    state.wp_dest_set_for = nullptr;
    auto_set_phase(state, AutoPhase::kTakeoff);
    return true;
}

// Advances one tick. Call once per SitlCopterHarness::step — see
// auto_mission_sitl_step below for the combined convenience form.
inline void auto_mission_advance(copter::LeftoverCopter& copter, sim::SimMulticopter& sim,
                                 AutoMissionState& state, copter::Mission& mission, float dt) {
    (void)dt;
    state.phase_changed = false;
    const float alt_m = -sim.position.z;
    auto_tick_wpnav(state, copter);

    switch (state.phase) {
    case AutoPhase::kDisarmed:
        copter.motors_armed = false;
        copter.land_complete = true;
        copter.throttle_out = 0.0f;
        copter.roll_target_rad = 0.0f;
        copter.pitch_target_rad = 0.0f;
        break;

    case AutoPhase::kTakeoff: {
        copter.motors_armed = true;
        if (copter.land_complete) {
            copter::UserTakeoffInputs in;
            in.motors_armed = true;
            in.land_complete = true;
            in.has_user_takeoff = true;
            in.takeoff_alt_m = state.hold_alt_m;
            in.current_alt_m = alt_m;
            copter::UserTakeoffEffects fx;
            if (leftover_do_user_takeoff_U_m(in, fx, &state.takeoff, alt_m) && fx.leftover_takeoff_start_m) {
                copter.land_complete = false;
            }
        }
        (void)copter_sitl_run::leftover_poscontrol_throttle(copter, sim, -state.hold_alt_m, -2.5f);
        copter_sitl_run::leftover_poscontrol_ne_update(copter, sim, false, sim.position.x, sim.position.y);
        if (alt_m >= state.hold_alt_m) {
            state.takeoff._running = false;
            mission.advance();  // Takeoff was item 0; move to item 1 (or hold on it if it's the only item)
            state.wp_dest_set_for = nullptr;
            auto_set_phase(state, AutoPhase::kAuto);
        }
        break;
    }

    case AutoPhase::kAuto: {
        copter.motors_armed = true;
        const copter::MissionItem* item = mission.current();
        if (item == nullptr) {
            // No mission (or ran past the last item with nothing to hold on,
            // which at_last()/advance()'s own contract makes unreachable in
            // practice) — hold current position and altitude rather than
            // ever cutting throttle mid-air.
            (void)copter_sitl_run::leftover_hold_command(copter, sim, alt_m);
            copter_sitl_run::leftover_poscontrol_ne_update(copter, sim, false, sim.position.x, sim.position.y);
            break;
        }
        switch (item->command) {
        case copter::MissionCommand::Waypoint: {
            auto_init_wpnav(state, copter, sim);
            if (state.wp_dest_set_for != item) {
                auto_set_wp_ned(state, copter, sim, item->north_m, item->east_m, item->down_m);
                state.wp_dest_set_for = item;
            }
            auto_tick_wpnav(state, copter);
            (void)copter_sitl_run::leftover_hold_command(copter, sim, -item->down_m);
            copter_sitl_run::leftover_poscontrol_ne_update(copter, sim, true, item->north_m, item->east_m);
            if (auto_wp_arrived(state, sim, item->acceptance_radius_m)) {
                state.hold_alt_m = -item->down_m;  // Rtl/Land after this leg return to THIS altitude
                mission.advance();
                state.wp_dest_set_for = nullptr;
            }
            break;
        }
        case copter::MissionCommand::Rtl: {
            auto_init_wpnav(state, copter, sim);
            if (state.wp_dest_set_for != item) {
                auto_set_wp_ned(state, copter, sim, 0.0f, 0.0f, -state.hold_alt_m);
                state.wp_dest_set_for = item;
            }
            auto_tick_wpnav(state, copter);
            (void)copter_sitl_run::leftover_hold_command(copter, sim, state.hold_alt_m);
            copter_sitl_run::leftover_poscontrol_ne_update(copter, sim, true, 0.0f, 0.0f);
            if (auto_wp_arrived(state, sim, item->acceptance_radius_m)) {
                mission.advance();
                state.wp_dest_set_for = nullptr;
            }
            break;
        }
        case copter::MissionCommand::Land: {
            state.land_start_n_m = sim.position.x;
            state.land_start_e_m = sim.position.y;
            state.land_start_d_m = sim.position.z;
            auto_set_phase(state, AutoPhase::kLand);
            break;
        }
        case copter::MissionCommand::Takeoff: {
            // A Takeoff item anywhere but index 0 has no real meaning yet
            // (this port has no "climb from current position mid-mission"
            // concept beyond the initial one) — treat it as a no-op advance
            // rather than silently re-running takeoff logic mid-flight.
            mission.advance();
            state.wp_dest_set_for = nullptr;
            break;
        }
        }
        break;
    }

    case AutoPhase::kLand: {
        copter.motors_armed = true;
        (void)copter_sitl_run::leftover_poscontrol_throttle(copter, sim, 0.25f, 1.5f);
        copter_sitl_run::leftover_poscontrol_ne_update(copter, sim, false, state.land_start_n_m,
                                                       state.land_start_e_m);
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
                copter.throttle_out = 0.0f;
                auto_set_phase(state, AutoPhase::kLanded);
            }
        }
        break;
    }

    case AutoPhase::kLanded:
        copter.motors_armed = false;
        copter.land_complete = true;
        copter.throttle_out = 0.0f;
        copter.roll_target_rad = 0.0f;
        copter.pitch_target_rad = 0.0f;
        break;
    }
}

// Mission advance then SitlCopterHarness::step — same pairing
// leftover_copter_sitl_step already establishes for the CCP-065 demo.
inline void auto_mission_sitl_step(SitlCopterHarness& harness, AutoMissionState& state,
                                   copter::Mission& mission, float dt) {
    auto_mission_advance(harness.copter(), harness.sim(), state, mission, dt);
    harness.step(dt);
}

}  // namespace fwcpp::hal_sitl::copter_auto
