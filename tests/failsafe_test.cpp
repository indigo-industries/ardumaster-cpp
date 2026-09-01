#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/failsafe.hpp>
#include <fwcpp/copter/failsafe_leftover.hpp>

using fwcpp::copter::FailsafeAction;
using fwcpp::copter::FailsafeEffects;
using fwcpp::copter::FailsafeInputs;
using fwcpp::copter::FsThrEnable;
using fwcpp::copter::leftover_do_failsafe_action;
using fwcpp::copter::leftover_failsafe_gcs_check;
using fwcpp::copter::leftover_failsafe_radio_check;
using fwcpp::copter::leftover_failsafe_radio_on_event;
using fwcpp::copter::leftover_set_mode_rtl_or_land;
using fwcpp::copter::failsafe::PortStatus;
using fwcpp::copter::failsafe::completeness_has;
using fwcpp::copter::failsafe::completeness_size;
using fwcpp::copter::failsafe::on_main_count;
using fwcpp::copter::failsafe::out_of_scope_count;
using fwcpp::copter::failsafe::remaining_count;
using fwcpp::copter::failsafe::this_slice_count;

TEST_CASE("leftover_failsafe_radio_check disarmed ignores radio failsafe",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = false;
    in.radio_failsafe = true;
    FailsafeEffects fx{};
    leftover_failsafe_radio_check(in, fx);
    REQUIRE_FALSE(fx.radio_failsafe_acted);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.gcs_announce_radio_failsafe);
    REQUIRE_FALSE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_check armed without radio inject is quiet",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = true;
    in.radio_failsafe = false;
    FailsafeEffects fx{};
    leftover_failsafe_radio_check(in, fx);
    REQUIRE_FALSE(fx.radio_failsafe_acted);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.gcs_announce_radio_failsafe);
}

TEST_CASE("leftover_failsafe_radio_check armed + radio inject sets RTL-or-land flags",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = true;
    in.radio_failsafe = true;
    FailsafeEffects fx{};
    leftover_failsafe_radio_check(in, fx);
    REQUIRE(fx.radio_failsafe_acted);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE(fx.gcs_announce_radio_failsafe);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_set_mode_rtl_or_land sets mode-change flags only",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_set_mode_rtl_or_land(fx);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
    REQUIRE_FALSE(fx.gcs_announce_radio_failsafe);
    REQUIRE_FALSE(fx.radio_failsafe_acted);
    REQUIRE_FALSE(fx.leftover_do_failsafe_action);
}

TEST_CASE("leftover_do_failsafe_action None sets dispatcher flag only",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::None, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.leftover_set_mode_land_with_pause);
    REQUIRE_FALSE(fx.leftover_set_mode_smart_rtl_or_rtl);
    REQUIRE_FALSE(fx.leftover_set_mode_smart_rtl_or_land);
    REQUIRE_FALSE(fx.leftover_set_mode_auto_do_land_start_or_rtl);
    REQUIRE_FALSE(fx.leftover_set_mode_brake_or_land);
    REQUIRE_FALSE(fx.leftover_terminate);
    REQUIRE_FALSE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_do_failsafe_action Land -> land_with_pause flag",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::Land, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_land_with_pause);
    REQUIRE(fx.notify_failsafe_mode_change);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
}

TEST_CASE("leftover_do_failsafe_action Rtl -> rtl_or_land flag",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::Rtl, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
    REQUIRE_FALSE(fx.leftover_set_mode_land_with_pause);
}

TEST_CASE("leftover_do_failsafe_action SmartRtl -> smart_rtl_or_rtl flag",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::SmartRtl, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_smart_rtl_or_rtl);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_do_failsafe_action SmartRtlLand -> smart_rtl_or_land flag",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::SmartRtlLand, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_smart_rtl_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_do_failsafe_action Terminate -> terminate flag",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::Terminate, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_terminate);
    REQUIRE_FALSE(fx.notify_failsafe_mode_change);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
}

TEST_CASE("leftover_do_failsafe_action AutoDoLandStart -> auto_do_land_start flag",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::AutoDoLandStart, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_auto_do_land_start_or_rtl);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_do_failsafe_action BrakeLand -> brake_or_land flag",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_do_failsafe_action(FailsafeAction::BrakeLand, fx);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_brake_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_on_event Disabled -> None (no mode flags)",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::Disabled, fx) ==
            FailsafeAction::None);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_on_event AlwaysRtl -> Rtl + rtl_or_land",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::AlwaysRtl, fx) ==
            FailsafeAction::Rtl);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_on_event ContinueMission -> Rtl + rtl_or_land",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::ContinueMission, fx) ==
            FailsafeAction::Rtl);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
}

TEST_CASE("leftover_failsafe_radio_on_event AlwaysLand -> Land + land_with_pause",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::AlwaysLand, fx) ==
            FailsafeAction::Land);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_land_with_pause);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_on_event AlwaysSmartRtlOrRtl -> SmartRtl flags",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::AlwaysSmartRtlOrRtl, fx) ==
            FailsafeAction::SmartRtl);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_smart_rtl_or_rtl);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_on_event AlwaysSmartRtlOrLand -> SmartRtlLand flags",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::AlwaysSmartRtlOrLand, fx) ==
            FailsafeAction::SmartRtlLand);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_smart_rtl_or_land);
}

TEST_CASE("leftover_failsafe_radio_on_event AutoRtlOrRtl -> AutoDoLandStart flags",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::AutoRtlOrRtl, fx) ==
            FailsafeAction::AutoDoLandStart);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_auto_do_land_start_or_rtl);
}

TEST_CASE("leftover_failsafe_radio_on_event BrakeOrLand -> BrakeLand flags",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(FsThrEnable::BrakeOrLand, fx) ==
            FailsafeAction::BrakeLand);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_brake_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_on_event unknown FS_THR -> Land flags",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    REQUIRE(leftover_failsafe_radio_on_event(static_cast<FsThrEnable>(255), fx) ==
            FailsafeAction::Land);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_land_with_pause);
}

TEST_CASE("leftover_failsafe_gcs_check disarmed ignores gcs failsafe",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = false;
    in.gcs_failsafe = true;
    FailsafeEffects fx{};
    leftover_failsafe_gcs_check(in, fx);
    REQUIRE_FALSE(fx.gcs_failsafe_acted);
    REQUIRE_FALSE(fx.leftover_do_failsafe_action);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.gcs_announce_gcs_failsafe);
}

TEST_CASE("leftover_failsafe_gcs_check armed without gcs inject is quiet",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = true;
    in.gcs_failsafe = false;
    FailsafeEffects fx{};
    leftover_failsafe_gcs_check(in, fx);
    REQUIRE_FALSE(fx.gcs_failsafe_acted);
    REQUIRE_FALSE(fx.leftover_do_failsafe_action);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.gcs_announce_gcs_failsafe);
}

TEST_CASE("leftover_failsafe_gcs_check armed + gcs inject -> do_failsafe_action RTL",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = true;
    in.gcs_failsafe = true;
    FailsafeEffects fx{};
    leftover_failsafe_gcs_check(in, fx);
    REQUIRE(fx.gcs_failsafe_acted);
    REQUIRE(fx.gcs_announce_gcs_failsafe);
    REQUIRE(fx.leftover_do_failsafe_action);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
    REQUIRE_FALSE(fx.radio_failsafe_acted);
    REQUIRE_FALSE(fx.gcs_announce_radio_failsafe);
}

TEST_CASE("failsafe leftover catalog remaining_count",
          "[copter][failsafe][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 6);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 7);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_failsafe_radio_check", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_set_mode_rtl_or_land", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_failsafe_radio_on_event", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_do_failsafe_action", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_failsafe_gcs_check", PortStatus::kThisSlice));
    REQUIRE(completeness_has("failsafe_enable call site", PortStatus::kOnMain));
    REQUIRE(completeness_has("ModeRTL / ModeLand", PortStatus::kOnMain));
    REQUIRE(completeness_has("failsafe_radio_on_event override ladder", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("battery / terrain / deadreckon failsafe", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("failsafe.cpp CPU watchdog", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("crash_check / thrust_loss / yaw_imbalance", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("ModeBrake failsafe path", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("GCS / Notify / logger objects", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("AP:: singletons", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("leftover_do_failsafe_action", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("failsafe_gcs_check / failsafe_gcs_on_event",
                                   PortStatus::kRemaining));
}
