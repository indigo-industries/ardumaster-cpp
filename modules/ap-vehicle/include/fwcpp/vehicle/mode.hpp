#pragma once

// Out-of-line method bodies for the Mode class hierarchy (declared in
// plane.hpp - see that file's own "CPP-031 SLICE 7 ADDENDUM" banner note
// for exactly why the split exists: Plane needs each concrete Mode
// subclass to be a COMPLETE type to hold it by value, while every
// Plane-touching Mode/ModeXXX method body needs PLANE to be a complete
// type - an ordering cycle broken the same way upstream's own mode.h/
// mode.cpp split breaks it, reproduced within this port's existing
// two-header convention instead of adding six new mode_*.cpp-equivalent
// files. This file `#include`s plane.hpp (Plane, complete, plus every
// Mode class declaration) and ONLY defines method bodies + tick() - no
// new class declarations live here anymore as of CPP-031 slice 7.
//
// Every judgment call, upstream citation, and exclusion for each method
// body below is documented on that method's DECLARATION in plane.hpp (the
// Mode class hierarchy section) and in plane.hpp's own SLICE 1/2/4/5/6/7
// file-banner addenda - not repeated here beyond a short pointer, to keep
// a single source of truth per judgment call rather than two copies that
// could drift.
//
// TICK() - see this file's own comment on the function below for the
// CPP-031 slice 4 navigate()-vs-update()/run() ordering decision (SHAPE
// CHOICE unchanged since slice 1: "a single fixed sequence suffices") and
// slice 7's own change (dispatching through `plane.control_mode` instead
// of taking an explicit `Mode&` parameter - the real payoff of this
// slice's set_mode() work: a set_mode() call made mid-tick, e.g. from
// ModeAUTO::navigate()'s own mission-complete transition, takes effect
// starting the FOLLOWING tick() call, not the one still in progress - see
// that comment for why and how this is verified).

#include <optional>

#include <fwcpp/vehicle/plane.hpp>

namespace fwcpp::vehicle {

// ---------------------------------------------------------------------
// Mode (base class) - out-of-line bodies. Declarations + full judgment-
// call documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void Mode::run(const StabilizeInputs& in) {
    plane_.stabilize_roll(in);
    plane_.stabilize_pitch(in);
    plane_.stabilize_yaw(in);
}

inline void Mode::reset_controllers() {
    plane_.roll_controller.reset_i();
    plane_.pitch_controller.reset_i();
    plane_.yaw_controller.reset_I();

    // GROUND STEERING ADDENDUM - see plane.hpp's own file banner and this
    // method's own declaration comment (plane.hpp). Upstream's own real
    // reset_controllers() body ("reset steering controls") does exactly
    // this, unconditionally.
    plane_.steer_state.locked_course = false;
    plane_.steer_state.locked_course_err = 0.0f;

    plane_.tecs.reset();
}

inline void Mode::output_pilot_throttle() {
    if (plane_.aparm.throttle_passthru_stabilize) {
        plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, plane_.get_throttle_input(true));
        return;
    }
    plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, plane_.get_adjusted_throttle_input(true));
}

inline void Mode::output_rudder_and_steering(float val) {
    plane_.srv_channels.set_output_scaled(srv::Function::kRudder, val);
    plane_.srv_channels.set_output_scaled(srv::Function::kSteering, val);
}

// ---------------------------------------------------------------------
// ModeManual - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void ModeManual::update(const StabilizeInputs&) {
    plane_.srv_channels.set_output_scaled(srv::Function::kAileron, plane_.roll_in_expo(false));
    plane_.srv_channels.set_output_scaled(srv::Function::kElevator, plane_.pitch_in_expo(false));
    output_rudder_and_steering(plane_.rudder_in_expo(false));

    const float throttle = plane_.get_throttle_input(true);
    plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, throttle);

    // nav_roll_cd/nav_pitch_cd are set from the AHRS's OWN current
    // attitude here, NOT a demand - matches upstream exactly (kept
    // for logging/consistency with other modes; MANUAL never
    // stabilizes toward these, see run() below - direct stick
    // passthrough only).
    plane_.nav_roll_cd = plane_.roll_sensor_cd();
    plane_.nav_pitch_cd = plane_.pitch_sensor_cd();
}

inline void ModeManual::run(const StabilizeInputs&) { reset_controllers(); }

// ---------------------------------------------------------------------
// ModeFBWA - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

// CPP-042 - upstream: Mode::enter() shared wrapper body (mode.cpp ~line
// 72), the two lines this port's ModeFBWA needs: `plane.auto_state.
// initial_pitch_cd = ahrs.pitch_sensor;` and (per plane.hpp own
// "HIGHEST_AIRSPEED" file banner note, extended this ticket) `plane.
// auto_state.highest_airspeed = 0;`. This port has no shared Mode::
// enter() wrapper (CPP-031 slice 7 own precedent) - ModeTAKEOFF::enter()
// (below) already does the highest_airspeed reset for its own real need;
// ModeFBWA::update() new FbwaTaildragger engage check (above) is a
// second real reader of both fields, so ModeFBWA now needs its own
// enter() too, doing exactly the same two lines.
inline bool ModeFBWA::enter() {
    plane_.takeoff_state.initial_pitch_cd = plane_.pitch_sensor_cd();
    plane_.takeoff_state.highest_airspeed = 0.0f;
    return true;
}

inline void ModeFBWA::update(const StabilizeInputs&) {
    // set nav_roll and nav_pitch using sticks
    plane_.nav_roll_cd =
        static_cast<std::int32_t>(plane_.channel_roll()->norm_input() * static_cast<float>(plane_.roll_limit_cd));
    plane_.update_load_factor();

    const float pitch_input = plane_.channel_pitch()->norm_input();
    if (pitch_input > 0.0f) {
        plane_.nav_pitch_cd = static_cast<std::int32_t>(pitch_input * plane_.aparm.pitch_limit_max_deg * 100.0f);
    } else {
        plane_.nav_pitch_cd = static_cast<std::int32_t>(-(pitch_input * plane_.pitch_limit_min * 100.0f));
    }
    plane_.adjust_nav_pitch_throttle();
    plane_.nav_pitch_cd = math::constrain_value(plane_.nav_pitch_cd, static_cast<std::int32_t>(plane_.pitch_limit_min * 100.0f),
                                                  static_cast<std::int32_t>(plane_.aparm.pitch_limit_max_deg * 100.0f));
    if (plane_.fly_inverted()) {
        plane_.nav_pitch_cd = -plane_.nav_pitch_cd;
    }

    // CPP-042 - upstream: ModeFBWA::update() own AuxFunc::FBWA_TAILDRAGGER
    // check (mode_fbwa.cpp, read in full): a channel tagged FBWA_
    // TAILDRAGGER is looked up via find_channel_for_option(); when found
    // and its raw switch position is HIGH, and fbwa_tdrag_takeoff_mode
    // isn't already set, and highest_airspeed is still below TKOFF_
    // TDRAG_SPD1, engage fbwa_tdrag_takeoff_mode. channel_for() (CPP-038,
    // rc_channels.hpp) is this port's find_channel_for_option()
    // equivalent - reused directly, not duplicated. get_aux_switch_pos()
    // (RC_Channel.cpp ~line 2053) calls read_3pos_switch() DIRECTLY - the
    // RAW, un-debounced switch position, deliberately bypassing read_
    // aux()'s own debounce/first-read-suppression machinery (CPP-037) -
    // and falls back to LOW if reading the switch fails (that function's
    // own doc comment), reproduced here by seeding `pos` to kLow before
    // the call and ignoring read_3pos_switch()'s bool return, exactly
    // matching that fallback.
    if (rc::RcChannel* chan = plane_.rc_channels.channel_for(rc::AuxFunc::FbwaTaildragger); chan != nullptr) {
        rc::AuxSwitchPos pos = rc::AuxSwitchPos::kLow;
        static_cast<void>(chan->read_3pos_switch(pos));
        const bool tdrag_mode = pos == rc::AuxSwitchPos::kHigh;
        if (tdrag_mode && !plane_.takeoff_state.fbwa_tdrag_takeoff_mode) {
            if (plane_.takeoff_state.highest_airspeed < plane_.aparm.takeoff_tdrag_speed1) {
                plane_.takeoff_state.fbwa_tdrag_takeoff_mode = true;
                // upstream: gcs().send_text(MAV_SEVERITY_WARNING, "FBWA
                // tdrag mode") - excluded, no GCS subsystem (disclosed).
            }
        }
    }
}

inline void ModeFBWA::run(const StabilizeInputs& in) {
    Mode::run(in);
    output_pilot_throttle();
}

// ---------------------------------------------------------------------
// ModeFBWB - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void ModeFBWB::update(const StabilizeInputs& in) {
    plane_.nav_roll_cd =
        static_cast<std::int32_t>(plane_.channel_roll()->norm_input() * static_cast<float>(plane_.roll_limit_cd));
    plane_.update_load_factor();
    plane_.update_fbwb_speed_height(in);
}

// ---------------------------------------------------------------------
// ModeCRUISE - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void ModeCRUISE::update(const StabilizeInputs& in) {
    if (plane_.channel_roll()->control_in != 0 || plane_.rudder_input() != 0) {
        locked_heading_ = false;
        lock_timer_ms_ = 0;
    }

    if (!locked_heading_) {
        plane_.nav_roll_cd =
            static_cast<std::int32_t>(plane_.channel_roll()->norm_input() * static_cast<float>(plane_.roll_limit_cd));
        plane_.update_load_factor();
    } else {
        plane_.calc_nav_roll(in);
    }
    plane_.update_fbwb_speed_height(in);
}

inline void ModeCRUISE::navigate(const StabilizeInputs& in) {
    const ahrs::GpsSample& gps_sample = plane_.gps.sample();
    const std::int32_t ground_course_cd = static_cast<std::int32_t>(gps_sample.ground_course_deg * 100.0f);
    const bool moving_forwards = std::fabs(math::wrap_PI(
        math::cd_to_rad(static_cast<float>(ground_course_cd)) - plane_.ahrs->get_yaw())) < static_cast<float>(M_PI_2);

    if (!locked_heading_ && plane_.channel_roll()->control_in == 0 && plane_.rudder_input() == 0 && gps_sample.has_fix &&
        gps_sample.ground_speed_ms >= kGpsGndCrsMinSpd && moving_forwards && lock_timer_ms_ == 0) {
        // user wants to lock the heading - start the timer.
        lock_timer_ms_ = in.now_ms;
    }
    if (lock_timer_ms_ != 0 && (in.now_ms - lock_timer_ms_) > 500U) {
        // lock the heading after 0.5 seconds of zero heading input from
        // the pilot.
        locked_heading_ = true;
        lock_timer_ms_ = 0;
        locked_heading_cd_ = ground_course_cd;
        plane_.prev_WP_loc = plane_.current_loc;
    }
    if (locked_heading_) {
        plane_.next_WP_loc = plane_.prev_WP_loc;
        // always look 1km ahead.
        plane_.next_WP_loc.offset_bearing(static_cast<float>(locked_heading_cd_) * 0.01f,
                                           plane_.prev_WP_loc.get_distance(plane_.current_loc) + 1000.0f);
        const nav::L1Inputs l1_in = plane_.build_l1_inputs(in);
        plane_.nav_controller.update_waypoint(plane_.prev_WP_loc, plane_.next_WP_loc, l1_in);
    }
}

// ---------------------------------------------------------------------
// ModeAUTO - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp (both the ModeAUTO class banner and the CPP-031
// SLICE 7 ADDENDUM's "MISSION-COMPLETE-TO-RTL"/"HOME-BEFORE-AUTO-RTL"
// notes).
// ---------------------------------------------------------------------

inline bool ModeAUTO::enter() {
    // CPP-031 slice 7 - see plane.hpp's own "HOME-BEFORE-AUTO-RTL" note:
    // a real, minimal fallback for a caller that never called
    // plane.set_home() at all - treats "home is still exactly the
    // untouched default Location()" as "never explicitly set", and sets
    // it to wherever THIS mission is starting from. Does NOT override any
    // other value a caller may have already set.
    if (plane_.home.lat == 0 && plane_.home.lng == 0 && plane_.home.alt == 0) {
        plane_.set_home(plane_.current_loc);
    }

    plane_.next_WP_loc = plane_.prev_WP_loc = plane_.current_loc;
    // CPP-039 - upstream: start_command()'s own `switch(cmd.id)` dispatch
    // (commands_logic.cpp), restricted to this port's two-command
    // vocabulary (MissionCommand::Waypoint/Takeoff - see plane.hpp's own
    // MissionItem/MissionCommand doc comments). ModeAuto::_enter()'s real
    // body calls `plane.mission.start_or_resume()`, which itself dispatches
    // to start_command() for whatever the first command happens to be -
    // this port's own smaller Mission has no separate start_or_resume()
    // state machine (SLICE 5's own scope boundary), so ModeAUTO::enter()
    // dispatches directly on mission.current()'s own command tag instead,
    // exactly reproducing the one real behavior start_or_resume() would
    // have produced here (do_takeoff()/do_nav_wp() called once, for
    // whichever command is first).
    const MissionItem* first_item = plane_.mission.current();
    if (first_item != nullptr) {
        if (first_item->command == MissionCommand::Takeoff) {
            plane_.do_takeoff();
        } else if (first_item->command == MissionCommand::Land) {
            // CPP-041 - upstream: start_command()'s NAV_LAND case
            // (commands_logic.cpp) dispatching to Plane::do_land(). A
            // mission that starts directly on a Land item is an edge case
            // upstream itself supports (no ordering requirement on
            // start_or_resume()'s own dispatch) - reproduced the same way.
            plane_.do_land();
        } else {
            plane_.do_nav_wp();
        }
    }
    return true;
}

inline void ModeAUTO::update(const StabilizeInputs& in) {
    plane_.update_auto_speed_height(in);

    // CPP-039 - upstream: mode_auto.cpp's own `if (nav_cmd_id ==
    // MAV_CMD_NAV_TAKEOFF || (nav_cmd_id == MAV_CMD_NAV_LAND &&
    // flight_stage == ABORT_LANDING)) { takeoff_calc_roll(); takeoff_calc_
    // pitch(); takeoff_calc_throttle(); } else { calc_nav_roll(); calc_nav_
    // pitch(); calc_throttle(); }` (ModeAuto::update(), ~line 82-90),
    // verified directly. The MAV_CMD_NAV_LAND/ABORT_LANDING half of the
    // OR is dropped - no NAV_LAND/landing subsystem in this port's
    // MissionItem vocabulary at all (ticket's own explicit exclusion,
    // separate future ticket) - so this collapses to a plain command-type
    // check. takeoff_calc_roll()/pitch()/throttle() (plane.hpp, CPP-031
    // slice 12) are called completely UNMODIFIED - the literal "thin
    // wrapper, not a re-implementation" that slice's own commit message
    // anticipated, including their own reads of `mode_takeoff.level_alt`/
    // `.target_alt`/`.ground_pitch` (plane.hpp's takeoff_calc_roll()/pitch(),
    // traced directly against upstream's own takeoff.cpp: these three
    // functions read the mode_takeoff OBJECT's own tunables regardless of
    // which mode/command actually triggered the takeoff - a single global
    // TKOFF_* parameter set, not a per-mode copy - verified, not assumed).
    const MissionItem* item = plane_.mission.current();
    if (item != nullptr && item->command == MissionCommand::Takeoff) {
        plane_.takeoff_calc_roll(in);
        plane_.takeoff_calc_pitch(in);
        plane_.takeoff_calc_throttle();
    } else if (item != nullptr && item->command == MissionCommand::Land) {
        // CPP-041 - upstream: mode_auto.cpp's own `else if (nav_cmd_id ==
        // MAV_CMD_NAV_LAND) { calc_nav_roll(); calc_nav_pitch();
        // nav_roll_cd = landing.constrain_roll(nav_roll_cd,
        // g.level_roll_limit*100UL); if (landing.is_throttle_suppressed())
        // { SRV_Channels::set_output_scaled(k_throttle, 0.0); } else {
        // calc_throttle(); } }`, verified directly, reproduced in full
        // (minus the ABORT_LANDING/quadplane branches - excluded, see
        // plane.hpp's own "CPP-041 ADDENDUM").
        plane_.calc_nav_roll(in);
        plane_.calc_nav_pitch();
        plane_.nav_roll_cd = plane_.constrain_landing_roll(plane_.nav_roll_cd);
        if (plane_.is_landing_final_flare()) {
            // if landing is considered complete throttle is never allowed,
            // regardless of landing type (upstream's own comment).
            plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, 0.0f);
        } else {
            plane_.calc_throttle();
        }
    } else {
        plane_.calc_nav_roll(in);
        plane_.calc_nav_pitch();
        plane_.calc_throttle();
    }
}

inline void ModeAUTO::navigate(const StabilizeInputs& in) {
    const MissionItem* item = plane_.mission.current();
    if (item == nullptr) {
        return; // no mission loaded
    }
    const nav::L1Inputs l1_in = plane_.build_l1_inputs(in);

    // CPP-039/CPP-041 - upstream: Plane::verify_command()'s own
    // `switch(cmd.id)` dispatch (commands_logic.cpp) - `case
    // MAV_CMD_NAV_TAKEOFF: return verify_takeoff(); case MAV_CMD_NAV_WAYPOINT:
    // return verify_nav_wp(cmd); case MAV_CMD_NAV_LAND: return
    // landing.verify_land(...);` - restricted to this port's three-command
    // vocabulary, same reasoning as ModeAUTO::enter() above. verify_land()
    // always returns false (see plane.hpp's own verify_land() doc comment),
    // so item_complete is always false while the current item is Land - the
    // mission simply never advances off it, matching upstream's real
    // continue_after_land()-defaults-false behavior (see plane.hpp's "CPP-041
    // ADDENDUM").
    bool item_complete;
    if (item->command == MissionCommand::Takeoff) {
        item_complete = plane_.verify_takeoff(l1_in);
    } else if (item->command == MissionCommand::Land) {
        item_complete = plane_.verify_land(l1_in);
    } else {
        item_complete = plane_.verify_nav_wp(l1_in);
    }

    if (item_complete) {
        if (plane_.mission.advance()) {
            // upstream: AP_Mission::update()'s own advance-then-dispatch-
            // start_command() sequence - the newly-current item's REAL
            // command type decides do_takeoff()/do_land()/do_nav_wp(), not
            // whichever one just completed.
            const MissionItem* next_item = plane_.mission.current();
            if (next_item != nullptr && next_item->command == MissionCommand::Takeoff) {
                plane_.do_takeoff();
            } else if (next_item != nullptr && next_item->command == MissionCommand::Land) {
                plane_.do_land();
            } else {
                plane_.do_nav_wp();
            }
        } else {
            // CPP-031 slice 7 - reached/passed the FINAL waypoint with no
            // more legs to advance to: the real upstream mission-complete
            // trigger (AP_Mission::complete() -> Plane::
            // exit_mission_callback() -> `if (control_mode == &mode_auto)
            // set_mode(mode_rtl, ...)` - see plane.hpp's own "CPP-031
            // SLICE 7 ADDENDUM" "MISSION-COMPLETE-TO-RTL" note for the
            // full trace) reproduced directly at the one place this
            // port's own smaller Mission detects the same condition.
            // Replaces slice 5's own "hold the final leg forever" no-op -
            // the documented gap this slice closes.
            plane_.set_mode(plane_.mode_rtl);
        }
    }
}

// ---------------------------------------------------------------------
// ModeRTL - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline bool ModeRTL::enter() {
    plane_.prev_WP_loc = plane_.current_loc;
    plane_.do_RTL(plane_.get_RTL_altitude_cm());
    plane_.rtl.done_climb = false;
    return true;
}

// CPP-034 FIX - see plane.hpp's own "UPDATE_AUTO_SPEED_HEIGHT()" note
// (Plane class, just above that method) and calc_throttle()'s own doc
// comment ("This is called by TECS-enabled flight modes"): any mode that
// calls calc_nav_pitch()/calc_throttle() only READS Tecs's last computed
// pitch/throttle demand - something else has to actually DRIVE that
// demand (tecs.update_50hz() + tecs.update_pitch_throttle(), bundled as
// update_auto_speed_height()) earlier the SAME tick, exactly as
// ModeAUTO::update() does above. ModeRTL::update() has called calc_nav_
// pitch()/calc_throttle() since RTL was first added (CPP-031 slice 6) but
// NEVER called update_auto_speed_height() - a genuine port-side gap, not
// an upstream-fidelity choice: upstream's real TECS update runs from a
// separate, mode-independent scheduled task (Plane::update_speed_height(),
// scheduler_tasks[], called every loop regardless of control_mode), so
// upstream never needed a "which mode is responsible for driving TECS"
// convention at all. This port chose instead to drive TECS from inside
// each auto-throttle mode's own update() (see UPDATE_AUTO_SPEED_HEIGHT()/
// UPDATE_FBWB_SPEED_HEIGHT()'s own notes) - a reasonable substitute for
// the missing scheduler, but that self-imposed convention was simply
// never extended to ModeRTL when slice 6 added it.
//
// EFFECT, ROOT-CAUSED VIA A REAL CLOSED-LOOP REPRO (CPP-034 ticket -
// vehicle_test.cpp's own "Closed loop: CRUISE-then-RTL converges toward
// home" test, which FAILED to converge before this fix and converges
// after it, with no other change): without this call, tecs.get_pitch_
// demand()/get_throttle_demand() (read by calc_nav_pitch()/calc_throttle()
// below) stayed FROZEN at whatever the PREVIOUS active mode last computed
// - e.g. ModeCRUISE's own last update_fbwb_speed_height() call, tuned for
// level flight at CRUISE's OWN altitude/speed, not RTL's real RTL_ALTITUDE
// climb target (do_RTL() sets target_altitude_cm correctly, but nothing
// ever fed it to Tecs again once RTL took over). The aircraft therefore
// never actually climbed to home.alt+RTL_ALTITUDE, and flew its entire
// loiter approach on a stale throttle/pitch trim never re-tuned for RTL's
// own speed/energy regime - which combined with L1Control's loiter
// capture-then-circle law (l1_control.hpp) to produce a large,
// non-decaying orbit oscillation (radius swinging roughly 45m-330m,
// never settling) instead of the tight, steady loiter every OTHER passing
// RTL closed-loop test already reaches. RTL-alone and AUTO-then-RTL both
// already converged fine despite this same gap - their own frozen/default
// Tecs demand at the moment RTL took over happened to still be close
// enough to survivable for L1's lateral loop to visibly work - which is
// exactly why this was invisible until the RC-failsafe slice's own agent
// tried CRUISE-then-RTL specifically (see vehicle_test.cpp's own "WHY
// AUTO, NOT CRUISE" note, CPP-031 slice 8) and flagged it for this ticket
// rather than assuming CRUISE was simply unlucky.
inline void ModeRTL::update(const StabilizeInputs& in) {
    plane_.update_auto_speed_height(in);
    plane_.calc_nav_roll(in);
    plane_.calc_nav_pitch();
    plane_.calc_throttle();

    if (plane_.aparm.rtl_climb_min <= 0.0f) {
        return;
    }

    // when RTL first starts, limit bank angle to LEVEL_ROLL_LIMIT
    // until we have climbed by RTL_CLIMB_MIN meters.
    const bool alt_threshold_reached =
        static_cast<float>(plane_.current_loc.alt - plane_.prev_WP_loc.alt) * 0.01f > plane_.aparm.rtl_climb_min;

    if (!plane_.rtl.done_climb && alt_threshold_reached) {
        plane_.prev_WP_loc = plane_.current_loc;
        // setup_alt_slope() - deferred, see plane.hpp's own note;
        // nothing left to do in this port's flat-altitude model.
        plane_.rtl.done_climb = true;
    }
    if (!plane_.rtl.done_climb) {
        // Constrain the roll limit as a failsafe, that way if
        // something goes wrong the plane will eventually turn back
        // and go to RTL instead of going perfectly straight. This
        // also leaves some leeway for fighting wind.
        plane_.roll_limit_cd = std::min(plane_.roll_limit_cd, static_cast<std::int32_t>(plane_.aparm.level_roll_limit_deg * 100.0f));
        plane_.nav_roll_cd = math::constrain_value(plane_.nav_roll_cd, -plane_.roll_limit_cd, plane_.roll_limit_cd);
    }
}

inline void ModeRTL::navigate(const StabilizeInputs& in) {
    const std::uint16_t radius = static_cast<std::uint16_t>(std::fabs(plane_.aparm.rtl_radius));
    if (radius > 0) {
        plane_.loiter.direction = (plane_.aparm.rtl_radius < 0.0f) ? -1 : 1;
    }
    plane_.update_loiter(radius, in);
}

// ---------------------------------------------------------------------
// ModeLOITER - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp (class banner and file banner's own "CPP-031
// SLICE 10 ADDENDUM").
// ---------------------------------------------------------------------

inline bool ModeLOITER::enter() {
    plane_.do_loiter_at_location();
    return true;
}

// CPP-031 SLICE 10 - see plane.hpp's own "CPP-031 SLICE 10 ADDENDUM" file
// banner note ("UPDATE_AUTO_SPEED_HEIGHT()") for why this calls
// update_auto_speed_height(in) even though upstream's own literal
// mode_loiter.cpp does not - the same "an auto-throttle mode calling
// calc_nav_pitch()/calc_throttle() must drive Tecs itself, since this port
// has no mode-independent scheduled TECS task" reasoning CPP-034
// established for ModeRTL::update() above, applied here from the start
// rather than needing its own later fix.
inline void ModeLOITER::update(const StabilizeInputs& in) {
    plane_.update_auto_speed_height(in);
    plane_.calc_nav_roll(in);
    plane_.calc_nav_pitch();
    plane_.calc_throttle();
}

inline void ModeLOITER::navigate(const StabilizeInputs& in) {
    // Zero indicates to use WP_LOITER_RAD (upstream's own comment,
    // mode_loiter.cpp) - see plane.hpp's update_loiter() doc comment for
    // why a zero radius genuinely reaches its real `radius <= 1` default
    // fallback. Contrast with ModeRTL::navigate() just above, which
    // passes a real nonzero RTL_RADIUS-derived radius when configured -
    // LOITER always wants the plain default.
    plane_.update_loiter(0, in);
}

// ---------------------------------------------------------------------
// ModeTAKEOFF - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp (ModeTAKEOFF class banner and the "CPP-031
// SLICE 12 ADDENDUM" file banner).
// ---------------------------------------------------------------------

inline bool ModeTAKEOFF::enter() {
    takeoff_mode_setup_ = false;
    climb_out_complete_ = false;
    plane_.takeoff_state.highest_airspeed = 0.0f;
    // CPP-042 - upstream: Mode::enter() shared `plane.auto_state.
    // initial_pitch_cd = ahrs.pitch_sensor;` (mode.cpp ~line 72) - the
    // only other real reader of this field is takeoff_tail_hold() itself
    // (plane.hpp), gated on in_takeoff (TAKEOFF or FBWA-tdrag-mode), so
    // capturing it here and in ModeFBWA::enter() (this file, above) is
    // the complete, honest substitute for upstream's real per-mode-change
    // capture - see plane.hpp own "CPP-042 ADDENDUM" file banner.
    plane_.takeoff_state.initial_pitch_cd = plane_.pitch_sensor_cd();
    plane_.takeoff_state.rotation_complete = false;
    plane_.steer_state.hold_course_cd = -1;

    // upstream: Mode::enter()'s own unconditional `plane.steerController.
    // reset_I();` - see plane.hpp's file banner "STEERCONTROLLER::RESET_I()
    // ON MODE ENTRY" note for why this is ported on ModeTAKEOFF's own
    // enter() rather than the shared Mode base class.
    plane_.steer_controller.reset_I();

    // See class banner's "ENTER()-SIDE next_WP_loc SEEDING" note - a
    // vertical-only climb target directly above the entry point, seeded so
    // the very first navigate() call (before update() has run even once -
    // see mode.hpp's tick() ordering note) doesn't loiter around a stale/
    // default Location. Superseded within one tick by update()'s own real
    // horizontal-offset setup below.
    start_loc_ = plane_.current_loc;
    plane_.prev_WP_loc = plane_.current_loc;
    plane_.next_WP_loc = plane_.current_loc;
    plane_.next_WP_loc.alt += static_cast<std::int32_t>(target_alt * 100.0f);
    plane_.target_altitude_cm = plane_.next_WP_loc.alt;

    return true;
}

// upstream: ModeTakeoff::update() (mode_takeoff.cpp), read in full - see
// class banner (plane.hpp) for every exclusion (home/position gating,
// is_flying()-skip, fence auto-enable, check_takeoff_timeout()) and the
// climb_out_complete_ substitution for `flight_stage == TAKEOFF`.
inline void ModeTAKEOFF::update(const StabilizeInputs& in) {
    const float alt = target_alt;
    const float dist = target_dist;

    if (!takeoff_mode_setup_) {
        plane_.takeoff_state.takeoff_altitude_rel_cm = static_cast<std::int32_t>(alt * 100.0f);
        plane_.takeoff_state.takeoff_pitch_cd = static_cast<std::int32_t>(level_pitch * 100.0f);
        // upstream: auto_state.baro_takeoff_alt = barometer.get_altitude()
        // - see plane.hpp file banner's "BAROMETRIC ALTITUDE SUBSTITUTION"
        // note. Re-primed every tick until locked in below, same as
        // everything else in this setup block - a stationary/slow-rolling
        // vehicle's current_altitude_m barely moves before lock, so this
        // converges to the same real "altitude at takeoff start" value
        // upstream's own one-shot assignment captures.
        plane_.takeoff_state.takeoff_start_alt_m = in.current_altitude_m;

        // upstream: `ahrs.groundspeed_vector()`'s bearing/length - this
        // port's own established GPS-ground-course/ground-speed substitute
        // (matches calc_nav_yaw_ground()'s own use of the same gps.sample()
        // fields, plane.hpp).
        const ahrs::GpsSample& gps_sample = plane_.gps.sample();
        const float direction = gps_sample.ground_course_deg;
        const float groundspeed = gps_sample.ground_speed_ms;

        start_loc_ = plane_.current_loc;
        plane_.prev_WP_loc = plane_.current_loc;
        plane_.next_WP_loc = plane_.current_loc;
        plane_.next_WP_loc.alt += static_cast<std::int32_t>(alt * 100.0f); // upstream: offset_up_m(alt)
        plane_.next_WP_loc.offset_bearing(direction, dist);
        plane_.target_altitude_cm = plane_.next_WP_loc.alt;

        // upstream: `!plane.throttle_suppressed && groundspeed >
        // GPS_GND_CRS_MIN_SPD` - see class banner's "GROUND-SPEED LOCK"
        // note for why `!throttle_suppressed` always evaluates true here.
        if (groundspeed > kGpsGndCrsMinSpd) {
            takeoff_mode_setup_ = true;
            // Necessary to allow Plane::takeoff_calc_roll() to function -
            // upstream's own comment, reproduced verbatim in intent.
            plane_.steer_state.hold_course_cd = static_cast<std::int32_t>(math::wrap_360_cd(direction * 100.0f));
        }
    }

    // We update the waypoint to follow once we're past TKOFF_LVL_ALT or we
    // pass the target location, correcting any yaw estimation error using
    // position bearing instead of GPS ground course - upstream's own
    // "enter-once" fallback lock, gated on hold_course_cd still being -1
    // (i.e. the ground-speed lock above never fired).
    const std::int32_t altitude_cm = plane_.current_loc.alt - start_loc_.alt;
    if (!climb_out_complete_ && plane_.steer_state.hold_course_cd == -1 &&
        (static_cast<float>(altitude_cm) >= level_alt * 100.0f || start_loc_.get_distance(plane_.current_loc) >= dist)) {
        const float direction = static_cast<float>(start_loc_.get_bearing_to(plane_.current_loc)) * 0.01f;
        plane_.next_WP_loc = start_loc_;
        plane_.next_WP_loc.offset_bearing(direction, dist);
        plane_.next_WP_loc.alt = start_loc_.alt + static_cast<std::int32_t>(alt * 100.0f);
        plane_.steer_state.hold_course_cd = static_cast<std::int32_t>(math::wrap_360_cd(direction * 100.0f));
    }

    // We finish the initial level takeoff if we climb past TKOFF_ALT (less
    // a 2m margin, matching upstream) or we pass the target location.
    if (!climb_out_complete_ &&
        (altitude_cm >= static_cast<std::int32_t>(alt * 100.0f) - 200 || start_loc_.get_distance(plane_.current_loc) >= dist)) {
        climb_out_complete_ = true;
    }

    // upstream: Plane::update_alt()'s mode-independent TECS drive - see
    // plane.hpp's own CPP-034/ModeRTL "no mode-independent scheduled TECS
    // task" precedent, applied here the same way LOITER/RTL/AUTO already
    // do. Runs regardless of climb_out_complete_ - TAKEOFF is an auto-
    // throttle mode throughout (does_auto_throttle() returns true
    // unconditionally, plane.hpp), matching upstream's own real dispatch.
    plane_.update_auto_speed_height(in);

    if (!climb_out_complete_) {
        // below TKOFF_ALT
        plane_.takeoff_calc_roll(in);
        plane_.takeoff_calc_pitch(in);
        plane_.takeoff_calc_throttle();
    } else {
        plane_.calc_nav_roll(in);
        plane_.calc_nav_pitch();
        plane_.calc_throttle();

        // CPP-036 - upstream: ModeTakeoff::update()'s own real recall,
        // read directly (mode_takeoff.cpp ~192-196): "check if in long
        // failsafe due to being in initial TAKEOFF stage; if it is,
        // recall long failsafe now to get fs action via events call" -
        // `if (plane.long_failsafe_pending) { plane.long_failsafe_pending
        // = false; plane.failsafe_long_on_event(FAILSAFE_LONG, ModeReason
        // ::MODE_TAKEOFF_FAILSAFE); }`. See plane.hpp file banner's
        // "CPP-036 ADDENDUM" ("LONG_FAILSAFE_PENDING" section) for why
        // this is a real, NECESSARY mechanism (not optional bookkeeping):
        // failsafe_long_on_event() stamps failsafe.state = Long
        // unconditionally on the FIRST (deferred) call, so check_long_
        // failsafe() alone would never re-invoke it once climb-out
        // completes - only this explicit recall, placed exactly where
        // upstream places it (the same `else` branch, i.e. only once
        // climb_out_complete_ has actually become true this tick or
        // earlier), applies the deferred escalation for real.
        if (plane_.long_failsafe_pending) {
            plane_.long_failsafe_pending = false;
            plane_.failsafe_long_on_event();
        }
    }
}

inline void ModeTAKEOFF::navigate(const StabilizeInputs& in) {
    // Zero indicates to use WP_LOITER_RAD - see this method's own
    // declaration comment (plane.hpp): called unconditionally, matching
    // upstream's real, literal ModeTakeoff::navigate() exactly (no
    // flight_stage branch there at all).
    plane_.update_loiter(0, in);
}

// upstream: the real scheduler task-table sequence (AHRS update ->
// update_control_mode/navigate -> Plane::stabilize() -> Plane::
// set_servos()/output), inferred from Mode::run()'s own body plus
// ModeFBWA::run()'s "run base then output throttle" pattern, per the
// ticket's own instruction. Folds in Plane::ahrs_update()'s roll/pitch-
// limit scaling, Plane::calc_airspeed_errors()'s speed-scaler filter
// update, and Plane::stabilize()'s 2-second-stale reset_controllers()
// check (Attitude.cpp) - the pre-takeoff integrator-zeroing check right
// after it in upstream's stabilize() is excluded (needs barometer/
// relative-altitude/groundspeed, no such subsystem in this port), as is
// the nav_scripting_active()/mode_training special dispatch in stabilize()
// (both always just call mode.run() unconditionally here - matching this
// slice's modes, none of which is mode_training or scripting-driven).
//
// CPP-031 SLICE 2 (FBWB) NOTE: this function is UNCHANGED by ModeFBWB's
// addition - Tecs::update_50hz()/update_pitch_throttle() are called from
// within Plane::update_fbwb_speed_height() (plane.hpp), reached only via
// ModeFBWB::update() above, not from here. See plane.hpp's file banner
// addendum ("SURPRISING UPSTREAM FINDING #1") for why: upstream itself
// gates both calls on `does_auto_throttle()` (true for FBWB only), and
// calling them unconditionally from this shared tick() would run them for
// MANUAL/FBWA too - wrong per upstream's own real behavior - without
// resurrecting the mode-identification machinery this port deliberately
// left unported.
//
// CPP-031 SLICE 3 NOTE: this function is what actually closes the real gap
// CPP-033 was built for - see plane.hpp's own file banner addendum for the
// full design rationale (StabilizeInputs's four new fields, the
// fly_forward()/accel_healthy()/ins_healthy() Plane methods, and the
// CALL-ORDER NOTE explaining why drift correction runs AFTER, not
// interleaved with, ahrs.update() this tick). Previously this function
// called ONLY `plane.ahrs.update(gyro_sample)` - pure gyro integration,
// with NO drift correction at all, despite CPP-028 slices 2/3 having fully
// ported and unit-tested drift_correction_yaw()/drift_correction_accel().
// Steps 2 and 3b below are what were missing.
//
// CPP-031 SLICE 4 NOTE: adds step 5b (plane.update_current_loc()) and the
// mode.navigate(in) call in step 6, both new shared infrastructure this
// slice's ModeCRUISE needs. See plane.hpp's file banner addendum for
// current_loc's own design rationale, and plane.hpp's own "CPP-031 SLICE 4
// ADDENDUM" note (on the Mode class hierarchy section) for the navigate()-
// vs-update()/run() ordering decision and its upstream justification.
// Neither change alters MANUAL/FBWA/FBWB's behavior: update_current_loc()
// only writes plane.current_loc (a field no pre-existing mode reads), and
// navigate() is a no-op on every mode but ModeCRUISE (Mode's own base
// implementation).
//
// CPP-031 SLICE 7 NOTE - THE REAL PAYOFF OF THIS SLICE: `mode` is bound
// ONCE, at the top of this function, from `plane.control_mode` - NOT
// re-read at each of the three dispatch points below. This is a
// deliberate choice, not an oversight: it reproduces upstream's own real
// same-iteration ordering, traced directly rather than assumed. Upstream's
// `control_mode->navigate()` (a separate, slower-rate SCHED_TASK - see
// this file's own "CPP-031 SLICE 4 ADDENDUM" note on the timing
// investigation) runs, WHEN IT FIRES, strictly AFTER that same loop
// iteration's `control_mode->update()`/`run()` (both FAST_TASKs, always
// run first) - so if navigate() triggers a mode switch via set_mode()
// (e.g. ModeAUTO's own mission-complete transition, plane.hpp), that
// iteration's update()/stabilize() have ALREADY run against the OLD mode;
// only the NEXT iteration's update_control_mode()/stabilize() (and
// navigate()) read the new `control_mode`. Binding `Mode& mode =
// *plane.control_mode;` once here and reusing it for navigate()/update()/
// run() reproduces exactly that: this tick()'s own update()/run() finish
// out against whichever mode was active when THIS tick() call began, even
// if navigate() (called first, per SLICE 4's own ordering choice) just
// switched `plane.control_mode` out from under it - the switch is only
// OBSERVABLE, and only dispatched through, starting the FOLLOWING tick()
// call, which re-reads `plane.control_mode` fresh. Verified directly by a
// dedicated test (vehicle_test.cpp) that installs a mode whose navigate()
// calls set_mode() mid-tick and confirms THIS tick's update()/run() still
// ran on the old mode, with the switch only visible on the next tick().
//
// CPP-031 SLICE 8 NOTE: adds step 1b (plane.update_throttle_failsafe()/
// plane.check_short_rc_failsafe()) - see plane.hpp's own "CPP-031 SLICE 8
// ADDENDUM" file banner for the full RC short (throttle) failsafe design.
// Placed immediately after step 1's RC read, matching upstream's own
// adjacent same-rate scheduling of read_radio()/control_failsafe() and
// check_short_rc_failsafe() (Plane.cpp's scheduler_tasks[], priorities 6
// and 9, both 50Hz). Neither call changes any EXISTING mode's behavior
// for a caller that never lets rc_failsafe actually latch (the real,
// in-scope default: THR_FAILSAFE defaults Enabled, but FixedWingTunables::
// fs_action_short/throttle_fs_value/rc_fs_timeout_ms all default to
// upstream's own real values, and every existing test's own
// set_sticks()-driven throttle PWM stays comfortably above
// THR_FS_VALUE's default threshold (950) every tick it runs) - verified
// directly by running every pre-existing test unchanged (vehicle_test.cpp).
//
// CPP-031 SLICE 11 NOTE: adds step 1c (the RC mode-switch channel dispatch)
// - see plane.hpp's own "CPP-031 SLICE 11 ADDENDUM" file banner for the
// full design. Placed immediately after step 1b, not before it: the
// freshness guard this step reproduces (upstream's RC_Channels_Plane::
// read_mode_switch() override, control_modes.cpp) reads plane.failsafe.
// last_valid_rc_ms, which step 1b's update_throttle_failsafe() call is
// what actually refreshes every tick - reading it any earlier in this
// function would see the PREVIOUS tick's value, off by one tick's worth of
// staleness. Matches upstream's own real scheduler ordering too: read_
// mode_switch() (Plane.cpp's scheduler_tasks[], gated inside AP_Vehicle's
// update_mode() task) always runs after read_radio()/control_failsafe()
// have already updated last_valid_rc_ms for the current frame.
//
// CPP-036 NOTE: adds step 1b-2 (plane.check_long_failsafe()) - see
// plane.hpp's own "CPP-036 ADDENDUM" file banner for the full RC long
// failsafe escalation design. Placed immediately after step 1b's
// check_short_rc_failsafe(), matching upstream's own relative scheduler
// ordering (check_short_rc_failsafe() priority 9, check_long_failsafe()
// priority 96 - short always runs first), and before step 1c is
// unaffected (1c's freshness guard reads last_valid_rc_ms, untouched by
// either failsafe-tier check).
//
// CPP-037 NOTE: adds step 1d (the RC aux-function switch dispatch) - see
// plane.hpp's own "CPP-037 ADDENDUM" file banner for the full design.
// Placed immediately after step 1c (mode-switch dispatch), matching
// upstream's own real relative scheduler ordering (RC_Channels::read_
// mode_switch() priority 7 before RC_Channels::read_aux_all() priority
// 10, both 50Hz, Plane.cpp's scheduler_tasks[]) - see step 1d's own
// comment below for exactly why this ordering is required (a same-tick
// MODE_SWITCH_RESET/aux-mode-release must not be read back by a mode-
// switch dispatch that has already run this tick).
//
// CPP-038 NOTE: adds step 6b (plane.set_servos_flaps()) - see plane.hpp's
// own "CPP-038 ADDENDUM" file banner for the full design. Placed strictly
// AFTER step 6 (mode.run(), which is what actually writes k_aileron's
// scaled output every tick) and strictly BEFORE step 7 (the hardware PWM
// write) - flaperon_update() (called from inside set_servos_flaps())
// reads k_aileron's just-written output back to mix into k_flaperon_
// left/right, so it must observe THIS tick's value, not a stale one from
// before mode.run() ran.
//
// CPP-055 NOTE (2026-08-27): a reader wondering why this is a single
// hand-sequenced function rather than a runtime walk over a registered
// task table (upstream's real AP_Scheduler/Plane::scheduler_tasks[]
// design, and this port's own fwcpp::scheduler::Scheduler, ap-scheduler/
// scheduler.hpp, CPP-026 slice 1) - see that header's own CPP-055 note:
// investigated and decided SUPERSEDED by this function's own design, not
// abandoned. Every SLICE/step NOTE above already cites the specific
// upstream scheduler_tasks[] priority number it reproduces at that exact
// point in this sequence - this function's accumulated ordering already
// IS this port's answer to "in what relative order do these tasks run,"
// verified through closed-loop tests slice by slice. A generic dispatcher
// walking a table to call these same steps in this same order would add
// indirection, not new behavior; CPP-026's own Scheduler class remains
// available and independently verified (scheduler_test.cpp) for a future
// caller that genuinely needs runtime-registered/variable-rate tasks,
// which this vehicle does not today.
//
// CPP-082 NOTE: `in` is now taken BY VALUE, not `const StabilizeInputs&`.
// This is the FIRST tick() slice to need a value this function itself
// computes (the real airspeed sensor's output) to flow into every
// existing downstream consumer of in.airspeed_valid/in.airspeed_eas
// (this function's own airspeed_tas/update_speed_scaler() below, AND
// mode.navigate(in)/mode.update(in)/mode.run(in)'s internal stabilize_
// roll()/pitch()/yaw()/build_l1_inputs()/takeoff_calc_pitch() reads,
// plane.hpp) - every one of which reads `in` again after this function's
// own call site, not a value this function could thread through as a
// separate parameter. Taking `in` by value gives this function its OWN
// local copy to override in place (see step 3/3b below) so every one of
// those later reads - inside THIS function and inside mode.navigate/
// update/run() - sees the overridden value automatically, without
// renaming every "in." reference in this file. This is transparent to
// every existing caller: tick(plane, gyro, in) already passed an
// lvalue StabilizeInputs at every call site (grep confirms it), so this
// change only adds one (cheap - a handful of floats/bools/Vector3f, no
// heap allocation) copy per call; the CALLER's own `in` object is never
// mutated, exactly as when this was a const reference.
inline void tick(Plane& plane, const ahrs::GyroSample& gyro_sample, StabilizeInputs in) {
    Mode& mode = *plane.control_mode; // see this function's own "CPP-031 SLICE 7 NOTE" above

    // 1. pull RC input (upstream: AP_Vehicle's read_radio() scheduled task)
    plane.rc_channels.read_input(plane.hal.rc_input);

    // 1b. RC short (throttle) failsafe - CPP-031 slice 8, see plane.hpp's
    //     own "CPP-031 SLICE 8 ADDENDUM" file banner for the full design
    //     (detection debounce, mode-switch on/off events, and the
    //     NEW-FRAME-DETECTION note explaining why this reads
    //     RcChannels::input_update_count() rather than re-checking
    //     RcInput::new_input(), already consumed by vehicle_test.cpp's own
    //     set_sticks() helper in every closed-loop test that calls it
    //     before tick()). Matches upstream's own adjacent, same-rate
    //     scheduling (read_radio() -> control_failsafe() ->
    //     check_short_rc_failsafe(), Plane.cpp's scheduler_tasks[]
    //     priorities 6/9, both 50Hz) - this port's single fixed-sequence
    //     tick() reproduces that ordering directly rather than needing a
    //     separate task-table entry.
    plane.update_throttle_failsafe(in.now_ms);
    plane.check_short_rc_failsafe();

    // 1b-2. RC long failsafe escalation - CPP-036, see plane.hpp's own
    //     "CPP-036 ADDENDUM" file banner for the full design. Upstream
    //     schedules check_long_failsafe() at 3Hz (Plane.cpp's scheduler_
    //     tasks[], priority 96) versus check_short_rc_failsafe()'s 50Hz
    //     (priority 9) - this port's single fixed-sequence tick() checks
    //     every tick instead (a named simplification, see file banner -
    //     both functions are pure timestamp-threshold comparisons against
    //     a monotonic clock, so checking more often can only detect the
    //     timeout SOONER, never later or less often). Placed immediately
    //     after check_short_rc_failsafe(), matching upstream's own
    //     relative ordering (short's lower priority number runs first).
    plane.check_long_failsafe(in.now_ms);

    // 1c. RC mode-switch channel dispatch - CPP-031 slice 11, see this
    //     function's own "CPP-031 SLICE 11 NOTE" and plane.hpp's "CPP-031
    //     SLICE 11 ADDENDUM" file banner for the full design. Upstream:
    //     RC_Channels_Plane::read_mode_switch()'s own guard (control_
    //     modes.cpp) - "only use signals that are less than 0.1s old" -
    //     reproduced literally before ever calling into RcChannels::
    //     read_mode_switch() (which layers its own has_valid_input()
    //     check on top, ap-rc-channel module).
    if (in.now_ms - plane.failsafe.last_valid_rc_ms <= 100U) {
        const std::optional<std::int8_t> new_pos = plane.rc_channels.read_mode_switch(in.now_ms);
        if (new_pos.has_value()) {
            plane.mode_switch_changed(*new_pos);
        }
    }

    // 1d. RC aux-function switch dispatch - CPP-037, see this function's
    //     own "CPP-037 NOTE" and plane.hpp's "CPP-037 ADDENDUM" file
    //     banner for the full design. Placed immediately after step 1c
    //     (mode-switch dispatch), matching upstream's own real relative
    //     scheduler ordering (RC_Channels::read_mode_switch() at priority
    //     7 always runs before RC_Channels::read_aux_all() at priority
    //     10, both 50Hz, Plane.cpp's scheduler_tasks[]) - this ordering
    //     matters because a MODE_SWITCH_RESET or aux-engaged-mode-release
    //     dispatched THIS call can call RcChannels::reset_mode_switch(),
    //     which must not be read back by THIS SAME tick's already-
    //     finished step 1c (it isn't - step 1c already ran). Unlike step
    //     1c, read_aux_all() has no extra "signal freshness" guard of its
    //     own to reproduce - upstream's real RC_Channels::read_aux_all()
    //     is the UNMODIFIED base-class version (RC_Channels_Plane never
    //     overrides it, unlike read_mode_switch()), gated only by has_
    //     valid_input() internally (rc_channels.hpp's own read_aux_all()).
    plane.rc_channels.read_aux_all(
        in.now_ms, [&plane, &in](fwcpp::rc::AuxFunc func, fwcpp::rc::AuxSwitchPos pos) {
            plane.dispatch_aux_function(func, pos, in.now_ms);
        });

    // 2. GPS update (upstream: AP_GPS::update(), a separate, earlier
    //    scheduled task feeding the AHRS update that follows it - see
    //    ap-gps/gps.hpp's own file banner for what this reproduces from
    //    AP_GPS_SITL). Always called every tick; internally rate-limited to
    //    200ms, exactly like the real backend, so most calls are a no-op.
    plane.gps.update(in.true_velocity_ned, in.now_ms);

    // 3/3b. AHRS full estimator cycle (upstream: Plane::ahrs_update()'s
    //    ahrs.update() call plus the REST of AP_AHRS_DCM::update() -
    //    drift_correction(delta_t) - which this port's AhrsDcm (CPP-028
    //    slices 2/3) split into accumulate_accel() (every tick, unrated)
    //    plus drift_correction_yaw()/drift_correction_accel() (each
    //    internally gated on a new GPS-fix-time observation - see their own
    //    doc comments in ahrs_dcm.hpp). CPP-035: plane.compass is now a
    //    real (fixed-earth-field) compass model - see plane.hpp's file
    //    banner "CPP-035 ADDENDUM" and modules/ap-compass/include/fwcpp/
    //    compass/compass.hpp's own file banner for the full design. It is
    //    only updated this tick when the caller's StabilizeInputs::
    //    compass_healthy is true (in.compass_field_bf then holds the
    //    already-body-frame field the caller computed from true attitude -
    //    Compass::update() itself never touches attitude, see compass.hpp's
    //    "WHO COMPUTES..." note) - a caller that never populates these two
    //    fields gets EXACTLY the prior behavior (plane.compass.sample()
    //    stays default-constructed, healthy=false, forever). With a real
    //    compass wired in, drift_correction_yaw()'s use_compass() prefers
    //    it over GPS ground course - see ahrs_dcm.hpp's use_compass() - so
    //    yaw drift can now be corrected even below kGpsSpeedMinMs (3 m/s),
    //    closing the gap CPP-035's own ticket exists to close.
    //
    //    CPP-078: the four separate AhrsDcm calls this call site used to
    //    make (update(), accumulate_accel(), drift_correction_yaw(),
    //    drift_correction_accel()) are now ONE call to AhrsDcm::
    //    update_full_cycle() - see that method's own doc comment in
    //    ahrs_dcm.hpp for its full parameter-provenance rationale. This
    //    is a pure call-site collapse, not a behavior change:
    //    update_full_cycle() still calls update() first, then
    //    accumulate_accel(), then drift_correction_yaw(), then
    //    drift_correction_accel(), in that exact order with the exact
    //    same arguments each received before this change. Nothing
    //    between the old update() call and the old accumulate_accel()
    //    call reads AhrsDcm's own state: compass.update() below only
    //    touches plane.compass (in.compass_field_bf is the caller's own
    //    true-attitude-derived field, not anything read back from ahrs),
    //    and gps_sample/wind_speed_ms/airspeed_tas/armed_and_safety_off
    //    are all computed independently of ahrs's attitude/velocity
    //    estimate too - so folding update() into the single call below
    //    (which necessarily runs textually after these independent
    //    computations, since it needs their results as arguments)
    //    changes nothing observable.
    if (in.compass_healthy) {
        plane.compass.update(in.compass_field_bf, in.now_us);
    }

    // CPP-082: real airspeed sensor wiring - see modules/ap-airspeed's
    // own file banner for AirspeedSensor's full read()-formula design,
    // and StabilizeInputs::airspeed_sensor_enabled's own doc comment
    // (plane.hpp) for why this defaults false/inert. MUST run before
    // airspeed_tas below (the first of MANY real consumers of in.
    // airspeed_valid/in.airspeed_eas this tick - see this function's own
    // "CPP-082 NOTE" above for the full list, which also includes mode.
    // navigate/update/run()'s internal stabilize_roll/pitch/yaw()/
    // build_l1_inputs()/takeoff_calc_pitch() reads further down this same
    // call): overriding `in`'s own copy of these two fields here, before
    // any of them are read, is what makes the override visible to every
    // one of those later reads without threading a separate value
    // through each of them individually.
    // CPP-083: now_ms is passed through so AirspeedSensor::update() can
    // service its own real boot-time zero-offset calibration internally
    // (gated on calibration_state() == InProgress) - no separate
    // calibration call site is needed here, matching the ticket's own
    // instruction. A caller must still call plane.airspeed_sensor.
    // start_calibration(now_ms) itself once (this port has no
    // AP_Vehicle::init_ardupilot() equivalent boot sequence yet to do it
    // unconditionally, unlike real upstream's own boot-time call - see
    // airspeed_sensor.hpp's file banner) for calibration to ever run;
    // until then calibration_state() stays NotStarted and this call is
    // simply the CPP-082 read()-formula pipeline, byte-for-byte.
    if (in.airspeed_sensor_enabled) {
        plane.airspeed_sensor.update(in.airspeed_raw_pressure_pa, in.now_ms);
        in.airspeed_valid = plane.airspeed_sensor.healthy();
        in.airspeed_eas = plane.airspeed_sensor.airspeed();
    }

    // BARO -> eas2tas. Upstream this is ahrs.get_EAS2TAS() delegating to
    // AP_Baro::get_EAS2TAS(); here plane.baro does the same arithmetic from
    // MEASURED pressure against its calibrated ground reference. Gated the
    // same way as the airspeed sensor directly above, for the same reason: a
    // caller that sets in.eas2tas itself (the fw_control / l1_control tests,
    // which have no baro at all) must not have it silently overwritten.
    if (in.baro_sensor_enabled) {
        plane.baro.update(in.baro_pressure_pa, in.baro_temperature_c, in.now_ms);
        if (plane.baro.healthy()) {
            in.eas2tas = plane.baro.get_eas2tas();
        }
    }
    // else: leave in.eas2tas exactly as the caller set it - plane.baro is
    // never touched, so a caller with no barometer keeps whatever convention
    // it was already using (1.0 by default = "true == equivalent airspeed").
    // else: leave airspeed_valid/airspeed_eas exactly as the caller set
    // them - plane.airspeed_sensor is never even touched, so a caller
    // that never sets airspeed_sensor_enabled gets bit-for-bit this
    // port's pre-CPP-082 behavior (mode.hpp used to read these two
    // fields as pure caller-supplied input, unconditionally; it still
    // does, unless this flag opts in).

    const ahrs::CompassSample compass = plane.compass.sample();
    const ahrs::GpsSample& gps_sample = plane.gps.sample();
    const float wind_speed_ms = in.wind_estimate.xy().length(); // see plane.hpp's file banner addendum
    const float airspeed_tas = in.airspeed_valid ? in.airspeed_eas * in.eas2tas : 0.0f; // matches ap-tecs's own EAS*eas2tas->TAS precedent
    // CPP-031 slice 9: armed_and_safety_off is now COMPUTED from the real
    // Plane::armed/RcOutput::safety_state() this slice wires together,
    // not a StabilizeInputs field a caller sets directly - see plane.hpp
    // file banner's "IS_ARMED_AND_SAFETY_OFF() BECOMES COMPUTED" note.
    const bool armed_and_safety_off = plane.is_armed_and_safety_off();

    plane.ahrs->update_full_cycle(gyro_sample, in.accel_sample, in.dt, compass, gps_sample, plane.fly_forward(),
                                  armed_and_safety_off, in.gps_use_enabled, wind_speed_ms, in.wind_estimate,
                                  airspeed_tas, plane.accel_healthy(), plane.ins_healthy(), in.now_ms);

    // 4. scaled roll/pitch limits from current attitude (upstream: the
    //    rest of Plane::ahrs_update())
    plane.update_flight_limits();

    // 5. speed-scaler low-pass filter (upstream: calc_airspeed_errors(),
    //    normally a separate 10Hz scheduled task - see Plane::
    //    update_speed_scaler()'s own doc comment)
    plane.update_speed_scaler(in.airspeed_valid, in.airspeed_eas, armed_and_safety_off, in.dt);

    // 5b. current position as a Location (upstream: nothing this simple -
    //     see plane.hpp's file banner addendum). Always called, every mode:
    //     cheap, and no pre-existing mode reads plane.current_loc, so this
    //     cannot change MANUAL/FBWA/FBWB's behavior.
    plane.update_current_loc(in.position_ned);

    // 6. navigate + mode update + stabilize (upstream: Plane::navigate() ->
    //    control_mode->navigate(), then Plane::stabilize()). See this
    //    file's own "CPP-031 SLICE 4 ADDENDUM" note (plane.hpp, Mode class
    //    hierarchy section) for why mode.navigate(in) is called HERE -
    //    before update()/run(), not after - despite upstream's real
    //    navigate() being a separate, slower-rate task that (when it does
    //    fire) runs AFTER that same iteration's stabilize(). See this
    //    function's own "CPP-031 SLICE 7 NOTE" above for why `mode` is
    //    bound once, at the top of this function, rather than re-read at
    //    each dispatch point.
    if (in.now_ms - plane.last_stabilize_ms > 2000U) {
        mode.reset_controllers();
    }
    plane.last_stabilize_ms = in.now_ms;
    mode.navigate(in);
    mode.update(in);
    mode.run(in);

    // 6b. flap servo output - CPP-038, see plane.hpp's own "CPP-038
    //     ADDENDUM" (set_servos_flaps()) file banner for the full design.
    //     Upstream: Plane::set_servos_flaps() is one of several steps
    //     inside the real Plane::set_servos() (servos.cpp), called AFTER
    //     the active mode has already written k_aileron's scaled output
    //     (step 6 above, mode.run()) - flaperon_update() (called from
    //     inside set_servos_flaps()) reads that value back to mix into
    //     k_flaperon_left/right, so this must run strictly after mode.
    //     run() and strictly before step 7's hardware write below.
    plane.set_servos_flaps(in.dt);

    // 7. write computed PWM to hardware (upstream: Plane::set_servos() ->
    //    SRV_Channels::output_ch_all())
    plane.srv_channels.output_ch_all(plane.hal.rc_output);
}

} // namespace fwcpp::vehicle
