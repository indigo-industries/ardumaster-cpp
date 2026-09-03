// Tests for fwcpp::nav::L1Control (CPP-017 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/nav/l1_control.hpp>

using namespace fwcpp::nav;
using fwcpp::Location;

namespace {
L1Control::Gains default_gains() {
    L1Control::Gains g;
    g.l1_period = 20.0f;
    g.l1_damping = 0.75f;
    g.l1_xtrack_i_gain = 0.0f; // disable integrator for predictable tests
    g.loiter_bank_limit = 45.0f;
    return g;
}
} // namespace

TEST_CASE("update_waypoint with invalid location marks data stale and does nothing else", "[l1]") {
    L1Control l1(default_gains());
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(1000.0f, 0.0f);

    L1Inputs in;
    in.location_valid = false;
    in.now_us = 1000;

    l1.update_waypoint(a, b, in);
    REQUIRE(l1.data_is_stale());
}

TEST_CASE("aircraft exactly on the track heading toward B has near-zero lateral accel demand", "[l1]") {
    L1Control l1(default_gains());
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(1000.0f, 0.0f); // B is 1000m north of A

    L1Inputs in;
    in.location_valid = true;
    in.current_loc = a;
    in.current_loc.offset(500.0f, 0.0f); // aircraft halfway along the track, exactly on line
    in.groundspeed_vector = fwcpp::math::Vector2f(15.0f, 0.0f); // flying north at 15 m/s
    in.yaw_rad = 0.0f; // facing north
    in.eas2tas = 1.0f;
    in.now_us = 1000;

    l1.update_waypoint(a, b, in);
    REQUIRE(std::fabs(l1.crosstrack_error()) < 0.5f); // essentially on track
    REQUIRE(std::fabs(l1.lateral_acceleration()) < 0.5f); // little correction needed
}

TEST_CASE("aircraft offset to one side of the track gets a lateral accel demand pulling it back", "[l1]") {
    L1Control l1(default_gains());
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(1000.0f, 0.0f); // track runs north

    L1Inputs in;
    in.location_valid = true;
    in.current_loc = a;
    in.current_loc.offset(500.0f, 50.0f); // 50m east of the track
    in.groundspeed_vector = fwcpp::math::Vector2f(15.0f, 0.0f); // still flying north
    in.yaw_rad = 0.0f;
    in.eas2tas = 1.0f;
    in.now_us = 1000;

    l1.update_waypoint(a, b, in);
    // crosstrack_error = a_air % ab = a_air.x*ab.y - a_air.y*ab.x (matches
    // Vector2::operator%, matches upstream's identical A_air % AB formula).
    // a_air ~= (500 north, 50 east), ab ~= (1, 0) (unit vector north) ->
    // 500*0 - 50*1 = -50: EAST of a north-bound track reads NEGATIVE in
    // this convention. Verified by computing the formula directly rather
    // than assumed - an earlier version of this test asserted the opposite
    // sign and failed against the real (correct) implementation.
    REQUIRE(l1.crosstrack_error() < -10.0f);
    REQUIRE(std::fabs(l1.lateral_acceleration()) > 0.01f); // meaningfully nonzero correction
}

TEST_CASE("target_bearing_cd points from the aircraft toward the next waypoint", "[l1]") {
    L1Control l1(default_gains());
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(0.0f, 1000.0f); // B is due east of A

    L1Inputs in;
    in.location_valid = true;
    in.current_loc = a;
    in.groundspeed_vector = fwcpp::math::Vector2f(0.0f, 15.0f);
    in.yaw_rad = static_cast<float>(M_PI_2); // facing east
    in.eas2tas = 1.0f;
    in.now_us = 1000;

    l1.update_waypoint(a, b, in);
    // Bearing to due east should be close to 9000 centidegrees (90 degrees).
    REQUIRE(l1.target_bearing_cd() == Catch::Approx(9000).margin(200));
}

TEST_CASE("nav_roll_cd is zero when lateral_acceleration demand is zero", "[l1]") {
    L1Control l1(default_gains());
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(1000.0f, 0.0f);

    L1Inputs in;
    in.location_valid = true;
    in.current_loc = a;
    in.current_loc.offset(500.0f, 0.0f); // on track
    in.groundspeed_vector = fwcpp::math::Vector2f(15.0f, 0.0f);
    in.yaw_rad = 0.0f;
    in.eas2tas = 1.0f;
    in.now_us = 1000;

    l1.update_waypoint(a, b, in);
    REQUIRE(std::abs(l1.nav_roll_cd(in)) < 200); // near-zero bank (< 2 degrees)
}

TEST_CASE("turn_distance scales with EAS2TAS squared and is capped by L1 distance", "[l1]") {
    L1Control l1(default_gains());
    L1Inputs in;
    in.eas2tas = 2.0f; // double true airspeed vs equivalent
    float d = l1.turn_distance(10.0f, in);
    REQUIRE(d <= 10.0f * 4.0f); // radius * eas2tas^2, but capped by L1 distance (0 before any update)
}

TEST_CASE("turn_distance with a turn_angle below 90 scales down linearly", "[l1]") {
    L1Control l1(default_gains());
    L1Inputs in;
    in.eas2tas = 1.0f;
    // Need a nonzero L1 distance to see the radius side of the MIN, so run
    // an update first to establish l1_dist_.
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(1000.0f, 0.0f);
    in.location_valid = true;
    in.current_loc = a;
    in.groundspeed_vector = fwcpp::math::Vector2f(15.0f, 0.0f);
    in.now_us = 1000;
    l1.update_waypoint(a, b, in);

    float full = l1.turn_distance(5.0f, 90.0f, in);
    float half = l1.turn_distance(5.0f, 45.0f, in);
    REQUIRE(half == Catch::Approx(full * 0.5f).margin(0.01f));
}

TEST_CASE("loiter_radius falls back to altitude scaling when bank limit or airspeed is zero", "[l1]") {
    L1Control::Gains g = default_gains();
    g.loiter_bank_limit = 0.0f; // disabled
    L1Control l1(g);
    L1Inputs in;
    in.eas2tas = 1.5f;
    in.target_airspeed = 15.0f;
    float r = l1.loiter_radius(100.0f, in);
    REQUIRE(r == Catch::Approx(100.0f * 1.5f * 1.5f)); // radius * eas2tas^2
}

TEST_CASE("reached_loiter_target is false immediately after update_waypoint", "[l1]") {
    // update_waypoint always sets _WPcircle = false (update_loiter can set
    // it true - see the [l1][loiter] tests below) - matches upstream
    // exactly.
    L1Control l1(default_gains());
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(1000.0f, 0.0f);
    L1Inputs in;
    in.location_valid = true;
    in.current_loc = a;
    in.groundspeed_vector = fwcpp::math::Vector2f(15.0f, 0.0f);
    in.now_us = 1000;
    l1.update_waypoint(a, b, in);
    REQUIRE_FALSE(l1.reached_loiter_target());
}

TEST_CASE("update_loiter with invalid location marks data stale and does nothing else", "[l1][loiter]") {
    L1Control l1(default_gains());
    Location center(0, 0, 0, Location::AltFrame::ABSOLUTE);
    L1Inputs in;
    in.location_valid = false;
    in.now_ms = 1000;
    l1.update_loiter(center, 200.0f, 1, in);
    REQUIRE(l1.data_is_stale());
}

TEST_CASE("update_loiter latches reached_loiter_target true once the aircraft is within the loiter radius", "[l1][loiter]") {
    // xtrack_err_circ = a_air.length() - radius. Whenever this is <= 0
    // (aircraft at or inside the circle), update_loiter's own
    // `xtrackErrCirc > 0.0f` switchover condition is false by construction,
    // so it unconditionally takes the "circle" branch (wp_circle_ = true) -
    // a deterministic case that doesn't depend on tuning a realistic
    // tangential velocity to hit the capture/circle crossover exactly.
    L1Control::Gains g = default_gains();
    g.loiter_bank_limit = 0.0f; // disable altitude scaling: loiter_radius(r) == r * eas2tas^2 exactly
    L1Control l1(g);
    Location center(0, 0, 0, Location::AltFrame::ABSOLUTE);

    L1Inputs in;
    in.location_valid = true;
    in.current_loc = center;
    in.current_loc.offset(190.0f, 0.0f); // 190m from center, inside a 200m loiter radius
    in.groundspeed_vector = fwcpp::math::Vector2f(0.0f, 15.0f);
    in.yaw_rad = static_cast<float>(M_PI_2);
    in.eas2tas = 1.0f;
    in.now_ms = 1000;

    l1.update_loiter(center, 200.0f, 1, in);
    REQUIRE(l1.reached_loiter_target());
    REQUIRE_FALSE(l1.data_is_stale());
}

TEST_CASE("update_loiter reports a positive crosstrack error whenever the aircraft is outside the loiter radius", "[l1][loiter]") {
    // crosstrack_error_ = xtrack_err_circ is assigned unconditionally
    // before the capture/circle branch - true regardless of which branch
    // ultimately runs, so this doesn't depend on the branch outcome.
    L1Control::Gains g = default_gains();
    g.loiter_bank_limit = 0.0f;
    L1Control l1(g);
    Location center(0, 0, 0, Location::AltFrame::ABSOLUTE);

    L1Inputs in;
    in.location_valid = true;
    in.current_loc = center;
    in.current_loc.offset(1000.0f, 0.0f); // well outside a 200m loiter radius
    in.groundspeed_vector = fwcpp::math::Vector2f(0.0f, 15.0f);
    in.yaw_rad = static_cast<float>(M_PI_2);
    in.eas2tas = 1.0f;
    in.now_ms = 1000;

    l1.update_loiter(center, 200.0f, 1, in);
    REQUIRE(l1.crosstrack_error() > 0.0f);
}

TEST_CASE("update_loiter entered near the center never demands a turn against the loiter direction while flying outbound", "[l1][loiter]") {
    // Upstream master 29d590b backport (see l1_control.hpp's clamp comment).
    // Deep inside the circle the P term (xtrack_err_circ * kx, ~ -27 m/s^2
    // at 5 m from the center of a 200 m circle) used to swamp the
    // centripetal term whenever the aircraft flew outbound with a positive
    // tangential component, so the net demand banked AGAINST the requested
    // direction. Sweep every outbound heading at several inside radii for
    // both directions and require the demand's sign never opposes
    // loiter_direction. (Before the backport this failed at 131 of the
    // inside-circle heading/radius/direction combinations.) Only strictly
    // OUTBOUND headings are swept (ltrack_vel_cap < 0: a northward velocity
    // component, the aircraft sitting due north of the center) because
    // that is the case the clamp covers; exactly-tangential headings
    // (90/270) are excluded since there ltrack_vel_cap == 0, the strict
    // `< 0` does not fire, and upstream master makes no sign guarantee
    // either.
    L1Control::Gains g = default_gains();
    g.loiter_bank_limit = 0.0f; // loiter_radius(r) == r exactly
    Location center(0, 0, 0, Location::AltFrame::ABSOLUTE);
    const float radius = 200.0f;
    const float dists[] = {5.0f, 50.0f, 100.0f, 150.0f, 190.0f};
    for (std::int8_t dir : {std::int8_t{-1}, std::int8_t{1}}) {
        for (float d : dists) {
            for (int h = -85; h <= 85; h += 5) {
                const float hdg = static_cast<float>(h) * static_cast<float>(M_PI) / 180.0f;
                L1Control l1(g);
                L1Inputs in;
                in.location_valid = true;
                in.current_loc = center;
                in.current_loc.offset(d, 0.0f); // due north of center
                in.groundspeed_vector = fwcpp::math::Vector2f(15.0f * std::cos(hdg), 15.0f * std::sin(hdg));
                in.yaw_rad = hdg;
                in.yaw_sensor_cd = static_cast<std::int32_t>(fwcpp::math::rad_to_cd(hdg));
                in.eas2tas = 1.0f;
                in.now_ms = 1000;
                l1.update_loiter(center, radius, dir, in);
                INFO("dir=" << int(dir) << " d=" << d << " hdg=" << h << " lat_acc=" << l1.lateral_acceleration());
                // Inside the circle the 'circle' branch is always taken, so
                // the aircraft is always reported as on-target...
                REQUIRE(l1.reached_loiter_target());
                // ...and the bank must be toward the loiter direction (or
                // level), never against it.
                REQUIRE(l1.lateral_acceleration() * static_cast<float>(dir) >= -1e-3f);
            }
        }
    }
}

TEST_CASE("update_loiter clamp still lets the PD term act when flying inbound or the right way round inside the circle", "[l1][loiter]") {
    // Guard against over-clamping: the new branch only fires while flying
    // OUTBOUND (ltrack_vel_cap < 0). Flying INBOUND from inside the circle
    // (toward the center) must still get the full, unclamped PD demand -
    // which for this geometry is a genuine wrong-direction-looking negative
    // number (P and D both pull the same way), because the aircraft really
    // is heading the wrong way and needs to be turned around. This is
    // upstream master's behaviour too.
    L1Control::Gains g = default_gains();
    g.loiter_bank_limit = 0.0f;
    Location center(0, 0, 0, Location::AltFrame::ABSOLUTE);
    L1Control l1(g);
    L1Inputs in;
    in.location_valid = true;
    in.current_loc = center;
    in.current_loc.offset(150.0f, 0.0f); // 150 m north, inside a 200 m circle
    in.groundspeed_vector = fwcpp::math::Vector2f(-15.0f, 0.0f); // flying south: straight inbound
    in.yaw_rad = static_cast<float>(M_PI);
    in.yaw_sensor_cd = 18000;
    in.eas2tas = 1.0f;
    in.now_ms = 1000;
    l1.update_loiter(center, 200.0f, 1, in);
    // omega = 2pi/20, kx = omega^2 ~ 0.0987, kv = 2*0.75*omega ~ 0.471.
    // xtrack_err = -50 -> P = -4.93; xtrack_vel_circ = -15 (inbound) -> D = -7.07;
    // vel_tangent = 0 -> ctr = 0. Unclamped PD = -12.0, dir=+1 -> -12.0.
    REQUIRE(l1.lateral_acceleration() == Catch::Approx(-12.0f).margin(0.2f));
}

TEST_CASE("update_heading_hold aligned with the commanded heading has zero lateral accel demand and zero crosstrack error", "[l1][heading_hold]") {
    L1Control l1(default_gains());
    L1Inputs in;
    in.yaw_sensor_cd = 9000; // facing east
    in.groundspeed_vector = fwcpp::math::Vector2f(0.0f, 15.0f);

    l1.update_heading_hold(9000, in); // commanded heading == current heading
    REQUIRE(l1.lateral_acceleration() == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(l1.crosstrack_error() == 0.0f);
    REQUIRE_FALSE(l1.reached_loiter_target());
    REQUIRE_FALSE(l1.data_is_stale());
}

TEST_CASE("update_heading_hold with a 90-degree heading error produces a meaningfully nonzero lateral accel demand", "[l1][heading_hold]") {
    L1Control l1(default_gains());
    L1Inputs in;
    in.yaw_sensor_cd = 0; // facing north
    in.groundspeed_vector = fwcpp::math::Vector2f(15.0f, 0.0f);

    l1.update_heading_hold(9000, in); // commanded heading == east, 90 degrees off
    REQUIRE(std::fabs(l1.lateral_acceleration()) > 0.1f);
}

TEST_CASE("update_level_flight copies yaw directly and zeroes all navigation demands", "[l1][level_flight]") {
    L1Control l1(default_gains());
    L1Inputs in;
    in.yaw_sensor_cd = 4500;
    in.yaw_rad = fwcpp::math::radians(45.0f);

    l1.update_level_flight(in);
    REQUIRE(l1.target_bearing_cd() == 4500);
    REQUIRE(l1.lateral_acceleration() == 0.0f);
    REQUIRE(l1.crosstrack_error() == 0.0f);
    REQUIRE_FALSE(l1.reached_loiter_target());
    REQUIRE_FALSE(l1.data_is_stale());
}
