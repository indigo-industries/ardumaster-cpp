// CCP-027 slice 4: NeLimits + NE entry point tests.

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/poscontrol/pos_control.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/pid/ac_pid_2d.hpp>

using namespace fwcpp::poscontrol;
using fwcpp::pid::AcP2d;
using fwcpp::pid::AcPid2d;

static float f32_from_bits(const std::string& s) {
    const std::uint32_t bits = static_cast<std::uint32_t>(std::stoul(s));
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

static std::vector<std::vector<std::string>> fixture_rows(const char* section) {
    // Shared parity data lives in the SEPARATE ardumaster-rust repo, which may
    // not be checked out beside this one. Location comes from CMake
    // (FWCPP_RUST_FIXTURES_DIR, default ../ardumaster-rust/fixtures) and can be
    // redirected at run time by the same-named env var; SKIP rather than fail
    // when it is absent, so a cpp-only checkout still gets a green suite.
    std::string dir = FWCPP_RUST_FIXTURES_DIR;
    if (const char* env = std::getenv("FWCPP_RUST_FIXTURES_DIR")) {
        dir = env;
    }
    const std::string path = dir + "/pos_control_ne.csv";
    std::ifstream in(path);
    if (!in.good()) {
        SKIP("ardumaster-rust fixture not found: " + path);
    }
    std::vector<std::vector<std::string>> rows;
    std::string current;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line[0] == '#') {
            current = line.substr(1);
            continue;
        }
        if (line.empty() || std::isalpha(static_cast<unsigned char>(line[0]))) {
            continue;
        }
        if (current == section) {
            std::vector<std::string> cols;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                cols.push_back(cell);
            }
            rows.push_back(std::move(cols));
        }
    }
    return rows;
}

TEST_CASE("NeLimits derive matches upstream fixture", "[poscontrol][ne][limits]") {
    const auto rows = fixture_rows("limits");
    REQUIRE(!rows.empty());

    std::unordered_set<std::uint32_t> with_ff;
    std::unordered_set<std::uint32_t> without_ff;

    for (const auto& r : rows) {
        REQUIRE(r.size() == 12);
        const bool ff = (r[8] == "1");
        AttitudeCapability att{
            f32_from_bits(r[4]),
            f32_from_bits(r[5]),
            f32_from_bits(r[6]),
            f32_from_bits(r[7]),
            ff,
        };
        const NeLimits got =
            NeLimits::derive(f32_from_bits(r[1]), f32_from_bits(r[2]), f32_from_bits(r[3]), att);
        REQUIRE(got.vel_max_ne_ms == Catch::Approx(f32_from_bits(r[9])).margin(3e-5f));
        REQUIRE(got.accel_max_ne_mss == Catch::Approx(f32_from_bits(r[10])).margin(3e-5f));
        REQUIRE(got.jerk_max_ne_msss == Catch::Approx(f32_from_bits(r[11])).margin(3e-5f));
        const std::uint32_t key = std::bit_cast<std::uint32_t>(got.jerk_max_ne_msss);
        if (ff) {
            with_ff.insert(key);
        } else {
            without_ff.insert(key);
        }
    }
    REQUIRE(with_ff.size() > without_ff.size() * 2);
}

TEST_CASE("negative NE limits use magnitude", "[poscontrol][ne][limits]") {
    AttitudeCapability att{};
    const NeLimits negative = NeLimits::derive(-7.5f, -2.5f, 5.0f, att);
    const NeLimits positive = NeLimits::derive(7.5f, 2.5f, 5.0f, att);
    REQUIRE(negative.vel_max_ne_ms == Catch::Approx(positive.vel_max_ne_ms));
    REQUIRE(negative.accel_max_ne_mss == Catch::Approx(positive.accel_max_ne_mss));
    REQUIRE(negative.vel_max_ne_ms == Catch::Approx(7.5f));
    REQUIRE(negative.accel_max_ne_mss == Catch::Approx(2.5f));
}

TEST_CASE("offset timeout and controller active", "[poscontrol][ne][entry]") {
    REQUIRE(!offset_target_timed_out(3000, 0));
    REQUIRE(offset_target_timed_out(3001, 0));
    REQUIRE(offset_target_timed_out(100, 100u - 3001u));
    REQUIRE(!offset_target_timed_out(100, 100u - 3000u));

    REQUIRE(controller_is_active(10, 10));
    REQUIRE(controller_is_active(11, 10));
    REQUIRE(!controller_is_active(12, 10));
    REQUIRE(controller_is_active(0, UINT32_MAX));
}

static NeInitInputs ne_init_inputs() {
    NeInitInputs inp{};
    inp.estimates.pos_m = {10.0, 0.0};
    inp.estimates.vel_ms = {1.0f, 0.0f};
    inp.att_target_euler_rad = {0.1f, 0.0f, 0.2f};
    inp.lean_angle_max_rad = 0.8f;
    inp.now_ms = 2000;
    inp.ticks = 50;
    inp.last_update_ticks = 49;
    inp.ahrs_ekf_reset_ms = 7;
    inp.accel_target_mss = {0.3f, 0.0f};
    return inp;
}

TEST_CASE("NE init keeps accel when active", "[poscontrol][ne][init]") {
    PosControlNe ne{};
    NeOffsetState offsets{};
    offsets.target.pos_m = {2.0, 1.0};
    offsets.target.vel_ms = {0.25f, 0.0f};
    offsets.target_ms = 1000;
    AcPid2d vel_pid = AcPid2d::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    const NeInitOutput out = ne.init_controller(offsets, vel_pid, ne_init_inputs());

    REQUIRE(out.pos_target_m.x == Catch::Approx(10.0).margin(1e-5));
    REQUIRE(ne.pos_desired_m.x == Catch::Approx(8.0).margin(1e-5));
    REQUIRE(ne.vel_desired_ms.x == Catch::Approx(0.75f).margin(1e-5));
    REQUIRE(ne.accel_desired_mss.x == Catch::Approx(0.0f));
    REQUIRE(out.accel_target_mss.x == Catch::Approx(0.3f).margin(1e-5));
    REQUIRE(out.last_update_ticks == 50);
    REQUIRE(out.ekf_last_reset_ms == 7);
}

TEST_CASE("NE init seeds accel from lean when inactive", "[poscontrol][ne][init]") {
    PosControlNe ne{};
    NeOffsetState offsets{};
    AcPid2d vel_pid = AcPid2d::with_gains(1.0f, 0.0f, 0.0f, 0.25f, 10.0f, 0.0f, 0.0f);
    NeInitInputs inp = ne_init_inputs();
    inp.last_update_ticks = 0;
    inp.ticks = 10;
    inp.att_target_euler_rad = {0.0f, -0.2f, 1.2f};
    inp.ahrs_yaw = 1.5707963267948966f;
    inp.accel_target_mss = {9.0f, 9.0f};
    inp.lean_angle_max_rad = 1.0f;

    const NeInitOutput out = ne.init_controller(offsets, vel_pid, inp);
    fwcpp::math::Vector3f att = inp.att_target_euler_rad;
    att.z = inp.ahrs_yaw;
    const fwcpp::math::Vector3f expect = lean_angles_rad_to_accel_ned_mss(att);
    REQUIRE(out.accel_target_mss.x == Catch::Approx(expect.x).margin(1e-5f));
    REQUIRE(out.accel_target_mss.y == Catch::Approx(expect.y).margin(1e-5f));
}

TEST_CASE("NE input_accel advances desired state", "[poscontrol][ne][input]") {
    PosControlNe ne{};
    ne.accel_desired_mss = {1.0f, 0.0f};
    NeLimits limits = NeLimits::derive(5.0f, 2.0f, kPoscontrolJerkNeMsss, AttitudeCapability{});
    ne.input_accel({2.0f, 0.0f}, limits, 0.02f, {}, {});
    REQUIRE(ne.accel_desired_mss.x == Catch::Approx(1.1f).margin(1e-4f));
}

TEST_CASE("NE relax decays accel target", "[poscontrol][ne][relax]") {
    PosControlNe ne{};
    fwcpp::math::Vector2f accel{1.0f, 0.0f};
    ne.relax_velocity(accel, 0.02f);
    REQUIRE(accel.x == Catch::Approx(0.888888f).margin(1e-4f));
}

TEST_CASE("NE soften and stop stabilisation", "[poscontrol][ne][stop]") {
    NeStopTargets targets{};
    targets.pos_target_m = {0.0, 0.0};
    targets.offsets.pos_m = {1.0, 0.0};
    targets.pos_desired_m = {-1.0, 0.0};
    NeEstimates est{};
    est.pos_m = {10.0, 0.0};
    fwcpp::math::Vector2f limit{};
    fwcpp::math::Vector2f accel_target{0.5f, 0.0f};
    ne_soften_for_landing(targets, est, 0.02f, limit, accel_target);
    REQUIRE(targets.pos_target_m.x > 0.0);
    REQUIRE(limit.x == Catch::Approx(0.5f));

    ne_stop_pos_stabilisation(targets, est);
    REQUIRE(targets.pos_target_m.x == Catch::Approx(10.0).margin(1e-6));
    REQUIRE(targets.pos_desired_m.x == Catch::Approx(9.0).margin(1e-6));

    AcPid2d vel_pid = AcPid2d::with_gains(1.0f, 0.5f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    vel_pid.set_integrator({1.0f, 0.0f});
    targets.vel_target_ms = {5.0f, 0.0f};
    est.vel_ms = {1.0f, 0.0f};
    ne_stop_vel_stabilisation(targets, est, vel_pid);
    REQUIRE(targets.vel_target_ms.x == Catch::Approx(1.0f));
    REQUIRE(vel_pid.integrator().is_zero());
}
