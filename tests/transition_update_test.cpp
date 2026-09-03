#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane_transition/transition_fsm.hpp>
#include <fwcpp/quadplane_transition/transition_timing.hpp>

using fwcpp::quadplane_transition::SltTransition;
using fwcpp::quadplane_transition::TransFailAction;
using fwcpp::quadplane_transition::TransFailOutcome;
using fwcpp::quadplane_transition::TransitionState;
using fwcpp::quadplane_transition::back_transition_time_s;
using fwcpp::quadplane_transition::constrain_transition_time_ms;
using fwcpp::quadplane_transition::kModeQland;
using fwcpp::quadplane_transition::kModeQrtl;
using fwcpp::quadplane_transition::kQOptionsTransFailToFw;
using fwcpp::quadplane_transition::kQTransDecelDefault;
using fwcpp::quadplane_transition::kQTransitionMsDefault;
using fwcpp::quadplane_transition::kQTransitionMsMax;
using fwcpp::quadplane_transition::kQTransitionMsMin;
using fwcpp::quadplane_transition::stopping_distance_m;
using fwcpp::quadplane_transition::trans_fail_outcome_fallback_mode;
using fwcpp::quadplane_transition::trans_fail_to_fw_set;

TEST_CASE("slt defaults include transition timing params", "[transition][update]") {
    const SltTransition fsm = SltTransition::with_defaults();
    REQUIRE(fsm.transition_time_ms() == kQTransitionMsDefault);
    REQUIRE(fsm.transition_decel_mss() == kQTransDecelDefault);
    REQUIRE(fsm.timer_duration_ms() == 5000U);
    REQUIRE(fsm.transition_fail_timeout_s() == 0);
    REQUIRE(fsm.transition_fail_action() == TransFailAction::kQland);
    REQUIRE_FALSE(fsm.transition_fail_warned());
}

TEST_CASE("constrain q transition ms", "[transition][update]") {
    REQUIRE(constrain_transition_time_ms(5000) == 5000U);
    REQUIRE(constrain_transition_time_ms(500) == 500U);
    REQUIRE(constrain_transition_time_ms(30000) == 30000U);
    REQUIRE(constrain_transition_time_ms(100) == static_cast<std::uint32_t>(kQTransitionMsMin));
    REQUIRE(constrain_transition_time_ms(-1) == static_cast<std::uint32_t>(kQTransitionMsMin));
    REQUIRE(constrain_transition_time_ms(32767) == static_cast<std::uint32_t>(kQTransitionMsMax));
}

TEST_CASE("airspeed wait lasts until airspeed not q transition ms", "[transition][update]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.update_airspeed_wait(1, false, 0.0f, 10.0f, false);
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    REQUIRE(fsm.transition_start_ms() == 1U);
    fsm.update_airspeed_wait(1U + 5000U + 5000U, false, 0.0f, 10.0f, false);
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    fsm.update_airspeed_wait(20000, true, 9.0f, 10.0f, false);
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    fsm.update_airspeed_wait(21000, true, 10.0f, 10.0f, false);
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    fsm.update_airspeed_wait(22000, true, 12.0f, 10.0f, true);
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    fsm.update_airspeed_wait(23000, true, 12.0f, 10.0f, false);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    REQUIRE(fsm.transition_low_airspeed_ms() == 23000U);
}

TEST_CASE("timer completes after constrained q transition ms", "[transition][update]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.update_airspeed_wait(1000, true, 12.0f, 10.0f, false);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    const std::uint32_t dwell = fsm.timer_duration_ms();
    fsm.update_timer(1000U + dwell, true);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    fsm.update_timer(1000U + dwell + 1, false);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    fsm.update_timer(1000U + dwell + 1, true);
    REQUIRE(fsm.complete());
    REQUIRE(fsm.transition_start_ms() == 0U);
    REQUIRE(fsm.transition_low_airspeed_ms() == 0U);
}

TEST_CASE("custom q transition ms and assist back via forward timing", "[transition][update]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.set_transition_time_ms(1000);
    fsm.update_forward_timing(100, true, 20.0f, 10.0f, false, true);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    fsm.update_forward_timing(1100, true, 20.0f, 10.0f, false, true);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    fsm.update_forward_timing(1101, true, 8.0f, 10.0f, true, true);
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    fsm.update_forward_timing(1200, true, 20.0f, 10.0f, false, true);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    fsm.update_forward_timing(2201, true, 20.0f, 10.0f, false, true);
    REQUIRE(fsm.complete());
}

TEST_CASE("trans fail zero timeout never fires", "[transition][update]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.update_airspeed_wait(1, false, 0.0f, 10.0f, false);
    REQUIRE(fsm.apply_transition_fail(1U + 60000U, false) == TransFailOutcome::kContinue);
    REQUIRE_FALSE(fsm.transition_fail_warned());
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
}

TEST_CASE("trans fail qland fallback after timeout", "[transition][update]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.set_transition_fail_timeout_s(5);
    fsm.update_airspeed_wait(1000, false, 0.0f, 10.0f, false);
    REQUIRE(fsm.apply_transition_fail(1000U + 5000U, false) == TransFailOutcome::kContinue);
    REQUIRE_FALSE(fsm.transition_fail_warned());
    REQUIRE(fsm.apply_transition_fail(1000U + 5001U, false) == TransFailOutcome::kFallbackQland);
    REQUIRE(fsm.transition_fail_warned());
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    REQUIRE(trans_fail_outcome_fallback_mode(TransFailOutcome::kFallbackQland) == kModeQland);
}

TEST_CASE("trans fail to fw completes timer when tiltrotor has speed", "[transition][update]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.set_transition_fail_timeout_s(3);
    fsm.set_q_options(kQOptionsTransFailToFw);
    fsm.update_airspeed_wait(10, false, 0.0f, 10.0f, false);
    REQUIRE(fsm.apply_transition_fail(10U + 3001U, false) == TransFailOutcome::kFallbackQland);
    fsm.restart();
    fsm.update_airspeed_wait(10, false, 0.0f, 10.0f, false);
    REQUIRE(fsm.apply_transition_fail(10U + 3001U, true) == TransFailOutcome::kCompleteToFw);
    REQUIRE(fsm.state() == TransitionState::kTimer);
    REQUIRE(fsm.in_forced_transition());
    fsm.apply_assist_back(20000, true);
    REQUIRE(fsm.state() == TransitionState::kTimer);
}

TEST_CASE("decel stopping distance and back time", "[transition][update]") {
    const SltTransition fsm = SltTransition::with_defaults();
    REQUIRE(fsm.stopping_distance_m(100.0f) == 25.0f);
    REQUIRE(fsm.back_transition_time_s(10.0f) == 5.0f);
    REQUIRE(stopping_distance_m(100.0f, 2.0f) == 25.0f);
    REQUIRE(back_transition_time_s(10.0f, 2.0f) == 5.0f);
    REQUIRE(trans_fail_to_fw_set(kQOptionsTransFailToFw));
    REQUIRE_FALSE(trans_fail_to_fw_set(0));
    REQUIRE(trans_fail_outcome_fallback_mode(TransFailOutcome::kFallbackQrtl) == kModeQrtl);
}
