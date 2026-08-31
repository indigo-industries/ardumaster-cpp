// CCP-043/045: SitlCopterHarness sensor synth + motor PWM → SimMulticopter.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/compass/compass.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

using fwcpp::Location;
using fwcpp::compass::Compass;
using fwcpp::copter::LeftoverCopter;
using fwcpp::copter::ModeAltHold;
using fwcpp::copter::ModeStabilize;
using fwcpp::copter::SpoolState;
using fwcpp::copter::leftover_copter_tick;
using fwcpp::hal_sitl::SitlCopterHarness;
using fwcpp::hal_sitl::sitl_copter::PortStatus;
using fwcpp::hal_sitl::sitl_copter::completeness_has;
using fwcpp::hal_sitl::sitl_copter::completeness_size;
using fwcpp::hal_sitl::sitl_copter::on_main_count;
using fwcpp::hal_sitl::sitl_copter::out_of_scope_count;
using fwcpp::hal_sitl::sitl_copter::remaining_count;
using fwcpp::hal_sitl::sitl_copter::this_slice_count;
using fwcpp::math::Vector3f;
using fwcpp::sim::SimMulticopter;

TEST_CASE("SitlCopterHarness step synthesizes gyro accel baro GPS compass",
          "[copter][sitl][ccp-043]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    sim.gyro = Vector3f{0.1f, -0.2f, 0.3f};
    sim.accel_body = Vector3f{0.0f, 0.0f, -9.81f};
    sim.position = Vector3f{10.0f, -20.0f, -50.0f};

    const auto dcm_before = sim.dcm;
    SitlCopterHarness harness(copter, sim);
    REQUIRE(harness.tick_count() == 0);
    REQUIRE(copter.tick_count == 0);
    REQUIRE_FALSE(copter.gyro_injected);
    REQUIRE_FALSE(copter.accel_injected);
    REQUIRE_FALSE(copter.baro_injected);
    REQUIRE_FALSE(copter.gps_injected);
    REQUIRE_FALSE(copter.compass_injected);

    harness.step(0.0025f);
    REQUIRE(harness.tick_count() == 1);
    REQUIRE(copter.tick_count == 1);
    REQUIRE(copter.gyro_injected);
    REQUIRE(copter.accel_injected);
    REQUIRE(copter.gyro_buffer.x == Catch::Approx(0.1f));
    REQUIRE(copter.gyro_buffer.y == Catch::Approx(-0.2f));
    REQUIRE(copter.gyro_buffer.z == Catch::Approx(0.3f));
    REQUIRE(copter.accel_buffer.z == Catch::Approx(-9.81f));

    REQUIRE(copter.baro_injected);
    REQUIRE(copter.baro_altitude_m == Catch::Approx(50.0f));

    REQUIRE(copter.gps_injected);
    Location expected(copter.home_lat, copter.home_lng, 0, Location::AltFrame::ABSOLUTE);
    expected.offset(10.0f, -20.0f);
    REQUIRE(copter.gps_lat == expected.lat);
    REQUIRE(copter.gps_lng == expected.lng);

    REQUIRE(copter.compass_injected);
    REQUIRE(copter.compass_field_bf.length() > 0.1f);
    (void)dcm_before;

    harness.step(0.0025f);
    REQUIRE(copter.tick_count == 2);
}

TEST_CASE("SitlCopterHarness arm spool hold smoke", "[copter][sitl][ccp-043]") {
    LeftoverCopter copter{};
    ModeAltHold althold{};
    copter.current = &althold;
    copter.motors_armed = true;

    SimMulticopter sim{};
    sim.gyro = Vector3f{0.0f, 0.0f, 0.0f};
    sim.accel_body = Vector3f{0.0f, 0.0f, -9.81f};
    sim.position = Vector3f{0.0f, 0.0f, -10.0f};

    SitlCopterHarness harness(copter, sim);
    REQUIRE_FALSE(copter.motors_armed_injected);
    REQUIRE_FALSE(copter.spool_injected);
    REQUIRE_FALSE(copter.attitude_hold_injected);
    REQUIRE(copter.spool_state == SpoolState::SHUT_DOWN);
    REQUIRE_FALSE(copter.attitude_hold);

    harness.step(0.0025f);

    REQUIRE(copter.motors_armed_injected);
    REQUIRE(copter.motors_armed);
    REQUIRE(copter.spool_injected);
    REQUIRE(copter.spool_state == SpoolState::THROTTLE_UNLIMITED);
    REQUIRE(copter.attitude_hold_injected);
    REQUIRE(copter.attitude_hold);
    REQUIRE(copter.tick_count == 1);
    REQUIRE(copter.gyro_injected);
    REQUIRE(copter.baro_injected);

    copter.motors_armed = false;
    harness.step(0.0025f);
    REQUIRE(copter.motors_armed_injected);
    REQUIRE(copter.spool_state == SpoolState::SHUT_DOWN);
    REQUIRE_FALSE(copter.attitude_hold);
    REQUIRE(copter.tick_count == 2);
}

TEST_CASE("SitlCopterHarness motor PWM drives Frame mixing", "[copter][sitl][ccp-045]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    sim.position.z = -15.0f;
    copter.motors_armed = true;
    const std::uint16_t high = sim.command_to_pwm(0.70f);
    const std::uint16_t low = sim.command_to_pwm(0.20f);
    copter.motor_pwm[0] = low;
    copter.motor_pwm[1] = high;
    copter.motor_pwm[2] = high;
    copter.motor_pwm[3] = low;

    SitlCopterHarness harness(copter, sim);
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 20; ++i) {
        harness.step(kDt);
    }
    REQUIRE(sim.gyro.x > 0.05f);  // +roll rate from left-high differential
}

TEST_CASE("SitlCopterHarness mixer_ is a distinct per-instance object, not shared",
          "[copter][sitl][ccp-066]") {
    // CCP-066: mixer_ used to be a function-local `static` in
    // apply_motor_pwm, so every SitlCopterHarness instance in the process
    // shared exactly one MotorsMatrix. This is the direct, unambiguous
    // regression test for that: two independently-constructed harnesses
    // must have two distinct mixer_ objects. This would have FAILED under
    // the old code (both accessors would have returned the address of the
    // one shared function-local static, regardless of `this`).
    LeftoverCopter copter_a{};
    SimMulticopter sim_a{};
    SitlCopterHarness harness_a(copter_a, sim_a);

    LeftoverCopter copter_b{};
    SimMulticopter sim_b{};
    SitlCopterHarness harness_b(copter_b, sim_b);

    REQUIRE(&harness_a.mixer_for_test() != &harness_b.mixer_for_test());
}

TEST_CASE("SitlCopterHarness two instances mix independently without cross-contamination",
          "[copter][sitl][ccp-066]") {
    // Two harnesses given genuinely different, non-trivial roll/pitch/
    // throttle commands (large enough to clear apply_motor_pwm's own
    // near-zero early-return) must each mix motor_pwm from their OWN
    // commands only. Interleaving their steps must not perturb either
    // one's own trajectory - a real, behavioral consequence of CCP-066's
    // fix, complementing the direct address-identity test above.
    constexpr std::size_t kQuadMotors = 4;
    constexpr float kDt = 0.0025f;

    LeftoverCopter copter_b{};
    copter_b.motors_armed = true;
    copter_b.throttle_out = 0.3f;
    copter_b.roll_target_rad = -0.20f;
    copter_b.pitch_target_rad = 0.0f;
    SimMulticopter sim_b{};
    SitlCopterHarness harness_b(copter_b, sim_b);

    // Run A alone for two steps, recording its own uninterrupted result.
    LeftoverCopter copter_a{};
    copter_a.motors_armed = true;
    copter_a.throttle_out = 0.6f;
    copter_a.roll_target_rad = 0.20f;
    copter_a.pitch_target_rad = 0.0f;
    SimMulticopter sim_a{};
    SitlCopterHarness harness_a(copter_a, sim_a);
    harness_a.step(kDt);
    harness_a.step(kDt);
    std::uint16_t pwm_a_uninterrupted[kQuadMotors];
    for (std::size_t i = 0; i < kQuadMotors; ++i) {
        pwm_a_uninterrupted[i] = copter_a.motor_pwm[i];
    }

    // Reset A to the same starting condition and re-run its own two steps,
    // this time interleaved with two steps of B carrying opposite-sign
    // commands in between.
    LeftoverCopter copter_a2{};
    copter_a2.motors_armed = true;
    copter_a2.throttle_out = 0.6f;
    copter_a2.roll_target_rad = 0.20f;
    copter_a2.pitch_target_rad = 0.0f;
    SimMulticopter sim_a2{};
    SitlCopterHarness harness_a2(copter_a2, sim_a2);

    harness_a2.step(kDt);
    harness_b.step(kDt);
    harness_a2.step(kDt);
    harness_b.step(kDt);

    for (std::size_t i = 0; i < kQuadMotors; ++i) {
        REQUIRE(copter_a2.motor_pwm[i] == pwm_a_uninterrupted[i]);
    }

    // Also confirm A and B's own left/right-differential roll commands
    // produced genuinely different mixing (a basic correctness sanity
    // check, not itself the regression test above).
    bool any_motor_differs = false;
    for (std::size_t i = 0; i < kQuadMotors; ++i) {
        if (copter_a2.motor_pwm[i] != copter_b.motor_pwm[i]) {
            any_motor_differs = true;
            break;
        }
    }
    REQUIRE(any_motor_differs);
}

TEST_CASE("leftover_copter_tick wires update_flight_mode when Mode* set",
          "[copter][sitl][ccp-043]") {
    LeftoverCopter copter{};
    ModeStabilize stabilize{};
    copter.current = &stabilize;
    REQUIRE(copter.tick_count == 0);
    leftover_copter_tick(copter);
    REQUIRE(copter.tick_count == 1);
    REQUIRE(copter.loop_ran_update_flight_mode);
    REQUIRE(copter.loop_ran_rate_controller);
    REQUIRE(copter.loop_ran_motors_output);
    REQUIRE(copter.loop_ran_read_ahrs);
}

TEST_CASE("SitlCopterHarness leftover catalog remaining_count",
          "[copter][sitl][ccp-043][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 10);
    REQUIRE(on_main_count() == 3);
    REQUIRE(out_of_scope_count() == 2);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("SitlCopterHarness scaffold", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_copter_tick", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_copter_loop", PortStatus::kThisSlice));
    REQUIRE(completeness_has("gyro/accel synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("baro synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("GPS synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("compass synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("closed-loop arm/spool/hold", PortStatus::kThisSlice));
    REQUIRE(completeness_has("motor PWM to SimMulticopter", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SIM_Multicopter Frame/Motor plant", PortStatus::kOnMain));
    REQUIRE(completeness_has("SitlHarness Plane path (CPP-084)", PortStatus::kOnMain));
}
