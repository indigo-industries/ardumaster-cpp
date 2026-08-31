#pragma once

// CCP-064: Copter::loop as leftover scheduler free functions.
// Upstream ArduCopter/Copter.cpp scheduler_tasks[]: FAST_TASK every loop,
// SCHED_TASK at rate_hz. This port's leftover bodies are flag/effect
// functions (CCP-035) plus Mode::run via update_flight_mode. Do not invent
// a new vehicle architecture — walk the existing table in order.
//
// leftover_copter_tick is the SitlCopterHarness vehicle tick (the Copter
// analogue of vehicle::tick for Plane). PWM is mixed in SitlCopterHarness
// from AC_PosControl NE lean + D throttle via MotorsMatrix (CCP-065).
// leftover_apply_collective is not the XY path.

#include <algorithm>
#include <cstdint>

#include <fwcpp/copter/auto_disarm_check.hpp>
#include <fwcpp/copter/check_ekf_reset.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/motors_output.hpp>
#include <fwcpp/copter/rc_loop.hpp>
#include <fwcpp/copter/read_ahrs.hpp>
#include <fwcpp/copter/read_inertia.hpp>
#include <fwcpp/copter/run_nav_updates.hpp>
#include <fwcpp/copter/run_rate_controller.hpp>
#include <fwcpp/copter/scheduler_tasks.hpp>
#include <fwcpp/copter/takeoff_check.hpp>
#include <fwcpp/copter/three_hz_loop.hpp>
#include <fwcpp/copter/throttle_loop.hpp>
#include <fwcpp/copter/update_altitude.hpp>
#include <fwcpp/copter/update_batt_compass.hpp>
#include <fwcpp/copter/update_flight_mode.hpp>
#include <fwcpp/copter/update_home_from_ekf.hpp>
#include <fwcpp/copter/update_throttle_hover.hpp>
#include <fwcpp/location.hpp>

namespace fwcpp::copter {

[[nodiscard]] inline bool leftover_scheduler_due(std::uint32_t tick_count, float rate_hz) {
    if (rate_hz <= 0.0f) {
        return true;
    }
    const std::uint32_t interval =
        std::max(std::uint32_t{1}, static_cast<std::uint32_t>(kCopterLoopRateHz / rate_hz));
    return (tick_count % interval) == 0;
}

inline void leftover_copter_loop(LeftoverCopter& copter) {
    ++copter.tick_count;
    const float dt = copter.loop_dt > 0.0f ? copter.loop_dt : 0.0025f;
    copter.now_ms = static_cast<std::uint32_t>(static_cast<float>(copter.tick_count) * dt * 1000.0f);

    // FAST_TASK run_rate_controller_main
    RateControllerMainInputs rate_in;
    rate_in.last_loop_time_s = dt;
    rate_in.using_rate_thread = false;
    const auto rate_fx = run_rate_controller_main(rate_in);
    copter.loop_ran_rate_controller = rate_fx.rate_controller_run;

    // FAST_TASK motors_output_main (flags; PWM is leftover_apply_collective)
    MotorsOutputInputs mo_in;
    mo_in.motors_armed = copter.motors_armed;
    mo_in.now_ms = copter.now_ms;
    const auto mo_fx = motors_output_main(mo_in);
    copter.loop_ran_motors_output = mo_fx.leftover == MotorsOutputMainLeftover::kRan;

    // FAST_TASK read_AHRS
    const auto ahrs_fx = read_ahrs();
    copter.loop_ran_read_ahrs = ahrs_fx.skip_ins_update;

    // FAST_TASK read_inertia
    ReadInertiaInputs ri;
    ri.ahrs_lat = copter.gps_lat != 0 ? copter.gps_lat : copter.home_lat;
    ri.ahrs_lng = copter.gps_lng != 0 ? copter.gps_lng : copter.home_lng;
    ri.has_rel_pos_d = copter.baro_injected;
    ri.pos_d_m = -copter.baro_altitude_m;
    ri.home_is_set = true;
    AltitudeContext alt_ctx{};
    (void)read_inertia(ri, copter.current_loc, alt_ctx);
    copter.loop_ran_read_inertia = true;

    // FAST_TASK check_ekf_reset
    (void)check_ekf_reset({});
    copter.loop_ran_check_ekf_reset = true;

    // FAST_TASK update_flight_mode — Mode::run() when current is set
    UpdateFlightModeInputs fm;
    fm.land_complete = copter.land_complete;
    fm.move_vehicle_on_ekf_reset = copter.move_vehicle_on_ekf_reset;
    fm.current = copter.current;
    (void)update_flight_mode(fm);
    copter.loop_ran_update_flight_mode = true;

    // FAST_TASK update_home_from_EKF (no-op once home_is_set)
    UpdateHomeFromEkfInputs home_in;
    home_in.home_is_set = true;
    home_in.armed = copter.motors_armed;
    Location home{};
    (void)update_home_from_ekf(home_in, home);

    // SCHED_TASK rc_loop @ 250 Hz
    if (leftover_scheduler_due(copter.tick_count, kRcLoopRateHz)) {
        ModeSwitchReadInputs rc_in;
        rc_in.has_valid_input = true;
        rc_in.flight_mode_channel = 4;
        (void)rc_loop(rc_in);
        copter.loop_ran_rc_loop = true;
    }

    // SCHED_TASK throttle_loop @ 50 Hz
    if (leftover_scheduler_due(copter.tick_count, 50.0f)) {
        (void)throttle_loop();
        copter.loop_ran_throttle_loop = true;
    }

    // SCHED_TASK update_batt_compass @ 10 Hz, update_altitude @ 10 Hz
    if (leftover_scheduler_due(copter.tick_count, 10.0f)) {
        UpdateBattCompassInputs batt;
        batt.compass_available = copter.compass_injected;
        batt.throttle = copter.throttle_out;
        (void)update_batt_compass(batt);
        UpdateAltitudeInputs alt_in;
        alt_in.baro_alt_m = copter.baro_altitude_m;
        (void)update_altitude(alt_in);
        AutoDisarmCheckInputs ad;
        ad.tnow_ms = copter.now_ms;
        ad.armed = copter.motors_armed;
        ad.land_complete = copter.land_complete;
        ad.spool_state = copter.spool_state;
        (void)auto_disarm_check(ad);
    }

    // SCHED_TASK run_nav_updates @ 50 Hz, takeoff_check @ 50 Hz
    if (leftover_scheduler_due(copter.tick_count, 50.0f)) {
        (void)run_nav_updates();
        TakeoffCheckInputs tk;
        tk.now_ms = copter.now_ms;
        tk.land_complete = copter.land_complete;
        (void)takeoff_check(tk);
    }
    if (leftover_scheduler_due(copter.tick_count, 100.0f)) {
        UpdateThrottleHoverInputs hv;
        hv.armed = copter.motors_armed;
        hv.land_complete = copter.land_complete;
        hv.throttle = copter.throttle_out;
        hv.velocity_D_ok = copter.baro_injected;
        (void)update_throttle_hover(hv);
    }
    if (leftover_scheduler_due(copter.tick_count, 3.0f)) {
        (void)three_hz_loop();
    }
}

inline void leftover_copter_tick(LeftoverCopter& copter) { leftover_copter_loop(copter); }

}  // namespace fwcpp::copter
