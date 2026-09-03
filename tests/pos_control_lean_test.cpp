// CCP-027 slice 1: POSCONTROL defaults + lean-angle helpers.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/poscontrol/pos_control.hpp>

using namespace fwcpp::poscontrol;
using fwcpp::math::Vector3f;

TEST_CASE("POSCONTROL default constants match AC_PosControl.h", "[poscontrol][defaults]") {
    REQUIRE(kPoscontrolJerkNeMsss == Catch::Approx(5.0f));
    REQUIRE(kPoscontrolRelaxTc == Catch::Approx(0.16f));
    REQUIRE(kPoscontrolSpeedMs == Catch::Approx(5.0f));
    REQUIRE(kPoscontrolAccelDMss == Catch::Approx(2.5f));
}

TEST_CASE("get_lean_angle_max_rad precedence matches upstream", "[poscontrol][lean]") {
    LeanAngleMaxConfig cfg{};
    cfg.attitude_lean_angle_max_rad = 0.5f;
    cfg.lean_angle_max_deg = 30.0f;
    REQUIRE(get_lean_angle_max_rad(cfg) == Catch::Approx(0.523598776f).margin(1e-5f));

    cfg.lean_angle_max_deg = 0.0f;
    REQUIRE(get_lean_angle_max_rad(cfg) == Catch::Approx(0.5f));

    cfg.angle_max_override_rad = 0.25f;
    REQUIRE(get_lean_angle_max_rad(cfg) == Catch::Approx(0.25f));
}

TEST_CASE("lean_angles_rad_to_accel_ned_mss level hover", "[poscontrol][lean]") {
    const Vector3f euler{0.0f, 0.0f, 0.0f};
    const Vector3f accel = lean_angles_rad_to_accel_ned_mss(euler);
    REQUIRE(accel.x == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(accel.y == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(accel.z == Catch::Approx(-kGravityMss));
}

TEST_CASE("accel_ne round trip at zero yaw", "[poscontrol][lean]") {
    const float accel_n = 1.2f;
    const float accel_e = -0.4f;
    float roll = 0.0f;
    float pitch = 0.0f;
    accel_ne_mss_to_lean_angles_rad(accel_n, accel_e, 1.0f, 0.0f, roll, pitch);

    Vector3f att{roll, pitch, 0.0f};
    const Vector3f ned = lean_angles_rad_to_accel_ned_mss(att);
    REQUIRE(ned.x == Catch::Approx(accel_n).margin(0.05f));
    REQUIRE(ned.y == Catch::Approx(accel_e).margin(0.05f));
}

TEST_CASE("get_thrust_vector fixes vertical at -g", "[poscontrol][lean]") {
    const Vector3f target{2.0f, -1.0f, 99.0f};
    const Vector3f thrust = get_thrust_vector(target);
    REQUIRE(thrust.x == Catch::Approx(2.0f));
    REQUIRE(thrust.y == Catch::Approx(-1.0f));
    REQUIRE(thrust.z == Catch::Approx(-kGravityMss));
}
