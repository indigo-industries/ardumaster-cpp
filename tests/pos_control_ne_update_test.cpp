// CCP-027 slice 2: NE_update_controller PID path (Rust pos_control_ne parity cases).

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/poscontrol/pos_control.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/pid/ac_pid_2d.hpp>

using namespace fwcpp::poscontrol;
using fwcpp::math::Vector2f;
using fwcpp::pid::AcP2d;
using fwcpp::pid::AcPid2d;

static NeUpdateInputs default_inputs() {
    NeUpdateInputs inp{};
    inp.dt = 0.02f;
    inp.ahrs_control_scale_xy = 1.0f;
    inp.ne_control_scale_factor = 1.0f;
    inp.vel_max_ne_ms = 10.0f;
    inp.lean_angle_max_rad = 0.8f;
    inp.cos_yaw = 1.0f;
    inp.sin_yaw = 0.0f;
    inp.att_yaw_target_rad = 0.3f;
    return inp;
}

TEST_CASE("NE update_controller P then velocity PID", "[poscontrol][ne]") {
    PosControlNe ne{};
    ne.pos_desired_m = {1.0, 0.0};
    AcP2d pos_p = AcP2d::with_kp(kNePosP);
    AcPid2d vel_pid = AcPid2d::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    NeDisturbance disturb{};
    const NeUpdateOutput out = ne.update_controller(pos_p, vel_pid, default_inputs(), disturb);

    REQUIRE(out.vel_target_ms.x == Catch::Approx(1.0f).margin(1e-5f));
    REQUIRE(out.vel_target_ms.y == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(out.accel_target_mss.x == Catch::Approx(1.0f).margin(1e-5f));
    REQUIRE(out.ne_control_scale_factor == Catch::Approx(1.0f));
}

TEST_CASE("NE update_controller AHRS scale on both loops", "[poscontrol][ne]") {
    PosControlNe ne{};
    ne.pos_desired_m = {1.0, 0.0};
    AcP2d pos_p = AcP2d::with_kp(1.0f);
    AcPid2d vel_pid = AcPid2d::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    NeUpdateInputs inp = default_inputs();
    inp.ahrs_control_scale_xy = 0.5f;
    NeDisturbance disturb{};
    const NeUpdateOutput out = ne.update_controller(pos_p, vel_pid, inp, disturb);

    REQUIRE(out.vel_target_ms.x == Catch::Approx(0.5f).margin(1e-5f));
    REQUIRE(out.accel_target_mss.x == Catch::Approx(0.25f).margin(1e-5f));
}

TEST_CASE("NE one-shot ne_control_scale_factor consumed", "[poscontrol][ne]") {
    PosControlNe ne{};
    ne.pos_desired_m = {1.0, 0.0};
    AcP2d pos_p = AcP2d::with_kp(1.0f);
    AcPid2d vel_pid = AcPid2d::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    NeUpdateInputs inp = default_inputs();
    inp.ne_control_scale_factor = 2.0f;
    NeDisturbance disturb{};
    const NeUpdateOutput out = ne.update_controller(pos_p, vel_pid, inp, disturb);

    REQUIRE(out.vel_target_ms.x == Catch::Approx(2.0f).margin(1e-5f));
    REQUIRE(out.accel_target_mss.x == Catch::Approx(4.0f).margin(1e-5f));
    REQUIRE(out.ne_control_scale_factor == Catch::Approx(1.0f));
}

TEST_CASE("NE disturbance applied once then cleared", "[poscontrol][ne]") {
    PosControlNe ne{};
    AcP2d pos_p = AcP2d::with_kp(1.0f);
    AcPid2d vel_pid = AcPid2d::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    NeDisturbance disturb{};
    disturb.pos_m = {-2.0f, 0.0f};

    const NeUpdateOutput first = ne.update_controller(pos_p, vel_pid, default_inputs(), disturb);
    REQUIRE(first.vel_target_ms.x == Catch::Approx(2.0f).margin(1e-5f));
    REQUIRE(disturb.pos_m.is_zero());

    const NeUpdateOutput second = ne.update_controller(pos_p, vel_pid, default_inputs(), disturb);
    REQUIRE(second.vel_target_ms.x == Catch::Approx(0.0f).margin(1e-5f));
}

TEST_CASE("yaw_from_ne_motion fast vs slow", "[poscontrol][ne]") {
    auto [yaw, rate] = yaw_from_ne_motion({2.0f, 0.0f}, {0.0f, 4.0f}, 10.0f, 0.3f);
    REQUIRE(yaw == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(rate == Catch::Approx(2.0f).margin(1e-5f));

    std::tie(yaw, rate) = yaw_from_ne_motion({0.1f, 0.0f}, {0.0f, 4.0f}, 10.0f, 0.3f);
    REQUIRE(yaw == Catch::Approx(0.3f));
    REQUIRE(rate == Catch::Approx(0.0f));
}
