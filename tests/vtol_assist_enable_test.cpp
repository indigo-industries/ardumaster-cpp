#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/vtol_assist.hpp>

using fwcpp::vtol_assist::AssistState;
using fwcpp::vtol_assist::AuxSwitchPos;
using fwcpp::vtol_assist::VtolAssist;
using fwcpp::vtol_assist::kAssistAngleDefault;
using fwcpp::vtol_assist::kAssistDelayDefault;
using fwcpp::vtol_assist::kAssistSpeedDefault;

TEST_CASE("vtol assist defaults", "[vtol_assist][enable]") {
    const VtolAssist assist = VtolAssist::with_defaults();
    REQUIRE(assist.speed() == kAssistSpeedDefault);
    REQUIRE(assist.angle() == kAssistAngleDefault);
    REQUIRE(assist.delay() == kAssistDelayDefault);
    REQUIRE(assist.state() == AssistState::kAssistEnabled);
    REQUIRE_FALSE(assist.speed_checks_enabled());
    REQUIRE_FALSE(assist.should_check());
    REQUIRE_FALSE(assist.is_enabled());
}

TEST_CASE("vtol assist speed gate", "[vtol_assist][enable]") {
    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_speed(8.0f);
    REQUIRE(assist.should_check());
    REQUIRE(assist.is_enabled());
    assist.set_state(AssistState::kAssistDisabled);
    REQUIRE_FALSE(assist.should_check());
    assist.set_state(AssistState::kForceEnabled);
    REQUIRE(assist.is_enabled());
    assist.set_state_from_aux(AuxSwitchPos::kHigh);
    REQUIRE(assist.state() == AssistState::kForceEnabled);
}
