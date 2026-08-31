// Tests for fwcpp::sim::SimPlane (CPP-030: STANDARD-config ground-truth
// fixed-wing flight dynamics, ported from upstream SITL::Plane; CPP-051:
// wind modeling - steady vector + turbulence gusts; CPP-094: load_coeffs()
// round-trip fidelity against a real upstream fixture, and the -heavy/-jet
// mass/thrust_scale frame overrides).

#include <cmath>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/sim/sim_plane.hpp>

using namespace fwcpp::sim;
using fwcpp::math::Vector3f;

namespace {

// Independently-transcribed reference formulas (not calling SimPlane's own
// liftCoeff/dragCoeff), matching ahrs_dcm_test.cpp's cross-check style:
// verify the class against a second, separately-written implementation of
// upstream's own math rather than reading the class's output back at itself.
double reference_lift_coeff(const Coefficients& c, float alpha) {
    const double alpha0 = c.alpha_stall;
    const double M = c.mcoeff;
    const double max_alpha_delta = 0.8;
    double a = alpha;
    if (a - alpha0 > max_alpha_delta) {
        a = alpha0 + max_alpha_delta;
    } else if (alpha0 - a > max_alpha_delta) {
        a = alpha0 - max_alpha_delta;
    }
    const double sigmoid = (1 + std::exp(-M * (a - alpha0)) + std::exp(M * (a + alpha0)))
                          / (1 + std::exp(-M * (a - alpha0))) / (1 + std::exp(M * (a + alpha0)));
    const double linear = (1.0 - sigmoid) * (c.c_lift_0 + c.c_lift_a * a);
    const double flatPlate = sigmoid * (2 * std::copysign(1.0, a) * std::pow(std::sin(a), 2) * std::cos(a));
    return linear + flatPlate;
}

double reference_drag_coeff(const Coefficients& c, float alpha) {
    const double AR = std::pow(static_cast<double>(c.b), 2) / c.s;
    return c.c_drag_p + std::pow(c.c_lift_0 + c.c_lift_a * alpha, 2) / (M_PI * c.oswald * AR);
}

// Independently-transcribed reference for the STEADY-wind half of upstream's
// Aircraft::update_wind (SIM_Aircraft.cpp:888) - built straight from degrees
// via M_PI/180, not by calling fwcpp::math::radians(), and including the
// real `wind_ef = -wind_ef` sign flip at the end - same cross-check style as
// reference_lift_coeff/reference_drag_coeff above. Turbulence is NOT
// modeled here (turbulence-free callers only).
Vector3f reference_steady_wind_ef(float speed, float direction_deg, float dir_z_deg) {
    const double dir = direction_deg * M_PI / 180.0;
    const double dz = dir_z_deg * M_PI / 180.0;
    Vector3f raw(static_cast<float>(std::cos(dir) * std::cos(dz)), static_cast<float>(std::sin(dir) * std::cos(dz)),
                 static_cast<float>(std::sin(dz)));
    raw = raw * speed;
    return -raw; // upstream's real final negation - see sim_plane.hpp's sign-convention note
}

} // namespace

TEST_CASE("liftCoeff matches an independently-computed reference across alpha", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;

    for (float alpha : {-0.6f, -0.2f, 0.0f, 0.1f, 0.3f, 0.6f, 1.2f, 2.5f}) {
        REQUIRE(plane.liftCoeff(alpha) == Catch::Approx(reference_lift_coeff(c, alpha)).margin(1e-4));
    }
}

TEST_CASE("liftCoeff rises through the linear regime then blends to the flat-plate stall model", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;

    // Small-alpha linear regime: lift increases with alpha, close to the
    // pure linear c_lift_0 + c_lift_a*alpha prediction (sigmoid ~ 0 far
    // below alpha_stall).
    const float lift_0 = plane.liftCoeff(0.0f);
    const float lift_small = plane.liftCoeff(0.2f);
    REQUIRE(lift_small > lift_0);
    REQUIRE(lift_0 == Catch::Approx(c.c_lift_0).margin(1e-3f));
    REQUIRE(lift_small == Catch::Approx(c.c_lift_0 + c.c_lift_a * 0.2f).margin(1e-2f));

    // Well past stall (alpha clamped to alpha_stall+0.8), the sigmoid-blended
    // flat-plate model must diverge substantially from naive linear
    // extrapolation - that divergence is the entire point of the model.
    const float alpha_deep_stall = 3.0f; // clamps to alpha_stall + 0.8
    const float lift_deep_stall = plane.liftCoeff(alpha_deep_stall);
    const float naive_linear = c.c_lift_0 + c.c_lift_a * alpha_deep_stall;
    REQUIRE(std::fabs(lift_deep_stall - naive_linear) > 1.0f);
}

TEST_CASE("dragCoeff matches an independently-computed reference and grows away from minimum-drag alpha", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;

    for (float alpha : {-0.5f, -0.08f, 0.0f, 0.3f, 0.8f}) {
        REQUIRE(plane.dragCoeff(alpha) == Catch::Approx(reference_drag_coeff(c, alpha)).margin(1e-5));
    }

    // Minimum induced drag is where c_lift_0 + c_lift_a*alpha == 0, i.e.
    // alpha ~ -c_lift_0/c_lift_a ~ -0.081 - moving away from it (either
    // direction) must increase drag.
    const float alpha_min_drag = -c.c_lift_0 / c.c_lift_a;
    const float drag_at_min = plane.dragCoeff(alpha_min_drag);
    REQUIRE(plane.dragCoeff(alpha_min_drag + 0.3f) > drag_at_min);
    REQUIRE(plane.dragCoeff(alpha_min_drag - 0.3f) > drag_at_min);
}

TEST_CASE("getForce returns exactly zero force when airspeed is exactly zero", "[sim_plane]") {
    SimPlane plane;
    const Vector3f force = plane.getForce(0.1f, 0.2f, -0.1f, 0.3f, 0.05f, 0.0f, Vector3f(0.1f, -0.2f, 0.05f), kSslAirDensity);
    REQUIRE(force == Vector3f(0.0f, 0.0f, 0.0f));
}

TEST_CASE("getTorque at zero airspeed reduces to exactly the CG-offset misalignment term", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;
    const Vector3f force(2.0f, -1.0f, 0.5f);

    const Vector3f torque = plane.getTorque(0.1f, 0.2f, -0.1f, 0.5f, force,
                                             0.3f, 0.0f, 0.05f, Vector3f(0.1f, -0.2f, 0.05f), kSslAirDensity);

    const float expected_la = c.cg_offset.y * force.z - c.cg_offset.z * force.y;
    const float expected_ma = -c.cg_offset.x * force.z + c.cg_offset.z * force.x;
    const float expected_na = -c.cg_offset.y * force.x + c.cg_offset.x * force.y;

    REQUIRE(torque.x == Catch::Approx(expected_la));
    REQUIRE(torque.y == Catch::Approx(expected_ma));
    REQUIRE(torque.z == Catch::Approx(expected_na));
}

TEST_CASE("analytically-derived trim angle of attack roughly balances weight in getForce", "[sim_plane]") {
    // Physical sanity check (see ticket): at a plausible cruise airspeed,
    // solve the LINEAR-region lift equation for the alpha that makes
    // lift ~= weight, then confirm the actual (sigmoid-blended) model
    // agrees with that estimate to within a generous margin - this is
    // ap-sim's own oracle, so this checks internal physical plausibility,
    // not an external ground truth.
    SimPlane plane;
    Coefficients c;

    const float cruise_airspeed = 15.0f; // m/s, plausible for a ~1.9m-span light UAV
    const float weight = plane.mass * kGravityMss;
    const float qbar = 0.5f * kSslAirDensity * cruise_airspeed * cruise_airspeed;
    const float required_cl = weight / (qbar * c.s);
    const float alpha_trim = (required_cl - c.c_lift_0) / c.c_lift_a;

    const Vector3f force = plane.getForce(0.0f, 0.0f, 0.0f, alpha_trim, 0.0f, cruise_airspeed, Vector3f(0.0f, 0.0f, 0.0f), kSslAirDensity);

    // force.z is body-frame "down"; lift opposes it, so it should be
    // negative and close in magnitude to weight for this small alpha.
    REQUIRE(force.z == Catch::Approx(-weight).margin(weight * 0.2f));
}

TEST_CASE("update() from rest under constant throttle accelerates forward without crashing", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -500.0f); // 500m up, well clear of the ground
    plane.dcm.identity();

    const float dt = 0.005f;
    const int steps = 200; // 1 second
    for (int i = 0; i < steps; ++i) {
        plane.update(0.0f, 0.0f, 0.0f, plane.hover_throttle, dt);
    }

    REQUIRE_FALSE(plane.dcm.is_nan());
    REQUIRE_FALSE(std::isnan(plane.velocity_ef.x));
    REQUIRE_FALSE(std::isnan(plane.position.z));

    // Accelerated meaningfully from rest.
    REQUIRE(plane.velocity_ef.length() > 3.0f);

    // Direct regression pin for a real bug caught in review: update()
    // must actually recompute airspeed from velocity_air_bf each tick
    // (update_dynamics's Aircraft::update_eas_airspeed()-equivalent). If
    // that assignment is ever dropped again, airspeed stays frozen at its
    // zero-initialized default, is_zero(airspeed) in getForce/getTorque
    // is permanently true, and the aircraft silently generates zero
    // aerodynamic force for its entire life (pure free-fall-under-
    // thrust). The velocity/position bounds below are too loose to catch
    // that on their own within a 1-second window, so pin it directly.
    REQUIRE(plane.airspeed > 3.0f);

    // Didn't fall out of the sky: even completely unpowered/liftless free
    // fall only loses ~4.9m in 1s (0.5*g*t^2); this plane has both thrust
    // and lift building as airspeed increases, so a generous 10m bound
    // catches a genuinely broken integrator without being sensitive to
    // the (uncontrolled, undamped) pitch transient a neutral-surface plane
    // naturally has.
    REQUIRE(plane.position.z < -490.0f);
}

TEST_CASE("update_dynamics ground-contact clamp prevents sinking through the floor", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, 0.0f); // on the ground
    plane.velocity_ef = Vector3f(0.0f, 0.0f, 5.0f); // moving down into the ground
    plane.dcm.identity();

    const float dt = 0.01f;
    for (int i = 0; i < 3; ++i) {
        plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), dt);
        // Bounded: one step's worth of overshoot at most, never a runaway sink.
        REQUIRE(plane.position.z < 0.1f);
        REQUIRE(plane.position.z >= 0.0f);
    }

    // Downward velocity has been clamped away, not merely reduced.
    REQUIRE(plane.velocity_ef.z == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("update_dynamics clamps an extreme rot_accel to the +-2000 deg/s gyro-rate limit", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f); // airborne, no ground interaction
    plane.dcm.identity();
    plane.gyro.zero();

    // Deliberately extreme rot_accel; a tiny dt keeps the resulting
    // dcm.rotate() step itself small even though the pre-clamp rate would
    // be enormous, isolating the clamp behavior from integrator blow-up.
    plane.update_dynamics(Vector3f(1.0e8f, 0.0f, 0.0f), 1.0e-6f);

    const float limit = fwcpp::math::radians(2000.0f);
    REQUIRE(plane.gyro.x == Catch::Approx(limit));
    REQUIRE(plane.gyro.y == Catch::Approx(0.0f).margin(1e-9f));
    REQUIRE(plane.gyro.z == Catch::Approx(0.0f).margin(1e-9f));
    REQUIRE_FALSE(plane.dcm.is_nan());
}

TEST_CASE("update_dynamics clamps an extreme body accel to +-64G", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f); // airborne, no ground clamp interaction
    plane.dcm.identity();
    plane.accel_body = Vector3f(1.0e8f, 0.0f, 0.0f); // deliberately extreme

    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.001f);

    // With dcm==identity and not on the ground, the final accelerometer-
    // equivalent accel_body (accel_earth + (0,0,-g)) equals the CLAMPED
    // input exactly - gravity is added then subtracted back out.
    const float limit = 64.0f * kGravityMss;
    REQUIRE(plane.accel_body.x == Catch::Approx(limit).margin(1e-2f));
    REQUIRE(plane.accel_body.y == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(plane.accel_body.z == Catch::Approx(0.0f).margin(1e-2f));
}

TEST_CASE("repeated update_dynamics keeps the DCM orthonormal over many integration steps", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    plane.dcm.identity();

    const Vector3f rot_accel(0.05f, -0.03f, 0.02f);
    const float dt = 0.001f;
    for (int i = 0; i < 5000; ++i) {
        plane.update_dynamics(rot_accel, dt);
    }

    REQUIRE_FALSE(plane.dcm.is_nan());
    REQUIRE(plane.dcm.a.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(plane.dcm.b.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(plane.dcm.c.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE((plane.dcm.a * plane.dcm.b) == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(plane.dcm.det() == Catch::Approx(1.0f).margin(1e-3f));
}

// --- CPP-051: wind modeling ---

TEST_CASE("update_wind reproduces the real steady wind vector and sign convention for zero turbulence", "[sim_plane][wind]") {
    SimPlane plane;
    plane.wind_config.turbulence = 0.0f;

    // Wind FROM due east (direction=90), horizontal (dir_z=0): physically
    // blows toward due WEST, i.e. wind_ef.y < 0 (NED: +y is East). Matches
    // the file banner's worked example exactly (direction=0 case), just
    // rotated 90 degrees.
    plane.wind_config.speed = 6.0f;
    plane.wind_config.direction = 90.0f;
    plane.wind_config.dir_z = 0.0f;
    plane.update_wind();
    Vector3f expected = reference_steady_wind_ef(6.0f, 90.0f, 0.0f);
    REQUIRE(plane.wind_ef.x == Catch::Approx(expected.x).margin(1e-5f));
    REQUIRE(plane.wind_ef.y == Catch::Approx(expected.y).margin(1e-5f));
    REQUIRE(plane.wind_ef.z == Catch::Approx(expected.z).margin(1e-5f));
    // Directly pin the physical direction too, not just cross-check against
    // the reference: wind FROM the east blows toward the west.
    REQUIRE(plane.wind_ef.y < -5.9f);
    REQUIRE(std::fabs(plane.wind_ef.x) < 1e-4f);

    // Wind FROM due south (direction=180): physically blows toward due
    // NORTH, i.e. wind_ef.x > 0 (a tailwind for a plane flying north).
    plane.wind_config.speed = 8.0f;
    plane.wind_config.direction = 180.0f;
    plane.wind_config.dir_z = 0.0f;
    plane.update_wind();
    expected = reference_steady_wind_ef(8.0f, 180.0f, 0.0f);
    REQUIRE(plane.wind_ef.x == Catch::Approx(expected.x).margin(1e-5f));
    REQUIRE(plane.wind_ef.y == Catch::Approx(expected.y).margin(1e-4f));
    REQUIRE(plane.wind_ef.x > 7.9f);

    // A nonzero vertical angle (dir_z) - exercises the cos(dir_z)/sin(dir_z)
    // terms the two horizontal-only cases above never touch.
    plane.wind_config.speed = 10.0f;
    plane.wind_config.direction = 0.0f;
    plane.wind_config.dir_z = 30.0f;
    plane.update_wind();
    expected = reference_steady_wind_ef(10.0f, 0.0f, 30.0f);
    REQUIRE(plane.wind_ef.x == Catch::Approx(expected.x).margin(1e-4f));
    REQUIRE(plane.wind_ef.y == Catch::Approx(expected.y).margin(1e-4f));
    REQUIRE(plane.wind_ef.z == Catch::Approx(expected.z).margin(1e-4f));
}

TEST_CASE("update_wind's turbulence gate matches upstream's wind_turb>0 && !on_ground() exactly", "[sim_plane][wind]") {
    // On the ground: turbulence state must never move, however large
    // wind_config.turbulence is - upstream's own `!on_ground()` gate.
    SimPlane on_ground_plane;
    on_ground_plane.position = Vector3f(0.0f, 0.0f, 0.0f); // on_ground() true
    on_ground_plane.wind_config.turbulence = 5.0f;
    for (int i = 0; i < 500; ++i) {
        on_ground_plane.update_wind();
    }
    REQUIRE(on_ground_plane.turbulence_horizontal_speed == 0.0f);
    REQUIRE(on_ground_plane.turbulence_vertical_speed == 0.0f);
    REQUIRE(on_ground_plane.turbulence_azimuth == 0.0f);

    // Airborne with turbulence==0: the `wind_turb > 0` half of the gate
    // must also hold it at zero.
    SimPlane no_turb_plane;
    no_turb_plane.position = Vector3f(0.0f, 0.0f, -100.0f); // airborne
    no_turb_plane.wind_config.turbulence = 0.0f;
    for (int i = 0; i < 500; ++i) {
        no_turb_plane.update_wind();
    }
    REQUIRE(no_turb_plane.turbulence_horizontal_speed == 0.0f);
    REQUIRE(no_turb_plane.turbulence_vertical_speed == 0.0f);

    // Airborne AND turbulence>0: the gust state must actually move.
    SimPlane airborne_plane;
    airborne_plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    airborne_plane.wind_config.turbulence = 5.0f;
    for (int i = 0; i < 500; ++i) {
        airborne_plane.update_wind();
    }
    REQUIRE((airborne_plane.turbulence_horizontal_speed != 0.0f || airborne_plane.turbulence_vertical_speed != 0.0f));
}

TEST_CASE("update_wind's turbulence settles to upstream's own IIR-filtered statistical distribution", "[sim_plane][wind]") {
    // Upstream's own comment (SIM_Aircraft.cpp:902-903, transcribed in this
    // class's update_wind()): "scale input.wind.turbulence to match
    // standard deviation when using iir_coef=0.98". This is a real,
    // checkable claim about the stationary distribution of the IIR-filtered
    // random walk: turbulence_horizontal_speed follows an AR(1) recurrence
    // h_n = iir_coef*h_{n-1} + (wind_turb*(1-iir_coef))*Z_n, Z_n ~ N(0,1),
    // wind_turb = turbulence*10 - whose stationary variance is
    // b^2/(1-a^2) with a=0.98, b=wind_turb*0.02, i.e. stddev ~= 1.005 *
    // turbulence. Verify that statistically, not by reproducing an exact
    // sequence (see file banner's RNG-substitution note - this class's RNG
    // is not bit-for-bit upstream's).
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f); // airborne throughout
    const float turbulence = 3.0f;
    plane.wind_config.turbulence = turbulence;

    const int burn_in = 1000; // let the AR(1) process reach its stationary distribution
    const int samples = 200000;
    double sum = 0.0;
    double sumsq = 0.0;
    for (int i = 0; i < burn_in + samples; ++i) {
        plane.update_wind();
        if (i >= burn_in) {
            sum += plane.turbulence_horizontal_speed;
            sumsq += static_cast<double>(plane.turbulence_horizontal_speed) * plane.turbulence_horizontal_speed;
        }
    }
    const double mean = sum / samples;
    const double variance = sumsq / samples - mean * mean;
    const double stddev = std::sqrt(variance);

    // Generous margins: the AR(1) process's ~50-tick correlation time means
    // 200000 samples give roughly a few thousand independent draws, not
    // 200000 - these margins are wide enough to be robust to that, while
    // still failing hard if the IIR recurrence, the wind_turb=turbulence*10
    // scale, or the iir_coef=0.98 constant is ever altered.
    REQUIRE(mean == Catch::Approx(0.0).margin(0.5));
    REQUIRE(stddev == Catch::Approx(turbulence).epsilon(0.25));
}

TEST_CASE("update_dynamics subtracts a real wind_ef when computing airmass-relative velocity", "[sim_plane][wind]") {
    // Direct, non-closed-loop check of SIM_Aircraft.cpp:762-766's real body
    // (`velocity_air_ef = velocity_ef - wind_ef; velocity_air_bf =
    // dcm.transposed() * velocity_air_ef;`), with wind_ef set directly
    // (bypassing update_wind()) to isolate this one recurrence.
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    plane.dcm.identity();
    plane.velocity_ef = Vector3f(20.0f, 0.0f, 0.0f);
    plane.wind_ef = Vector3f(5.0f, -2.0f, 0.5f);

    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.0f); // dt=0 isolates the wind subtraction from integration

    const Vector3f expected_air_ef = plane.velocity_ef - plane.wind_ef;
    REQUIRE(plane.velocity_air_ef.x == Catch::Approx(expected_air_ef.x));
    REQUIRE(plane.velocity_air_ef.y == Catch::Approx(expected_air_ef.y));
    REQUIRE(plane.velocity_air_ef.z == Catch::Approx(expected_air_ef.z));

    // dcm==identity, so velocity_air_bf == velocity_air_ef exactly here.
    REQUIRE(plane.velocity_air_bf.x == Catch::Approx(expected_air_ef.x));
    REQUIRE(plane.velocity_air_bf.y == Catch::Approx(expected_air_ef.y));
    REQUIRE(plane.velocity_air_bf.z == Catch::Approx(expected_air_ef.z));

    // Not simply equal to ground velocity - the whole point of this ticket.
    REQUIRE(plane.velocity_air_ef.x != Catch::Approx(plane.velocity_ef.x));
}

TEST_CASE("update_dynamics with wind_ef left at its zero default reproduces the pre-CPP-051 always-still-air behavior", "[sim_plane][wind]") {
    // Regression pin: a caller that never calls update_wind() (e.g. the
    // existing update_dynamics-only tests above) must see EXACTLY the same
    // velocity_air_bf == dcm.transposed()*velocity_ef this class always
    // produced before CPP-051 - wind_config/wind_ef default to all-zero.
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    plane.dcm.identity();
    plane.velocity_ef = Vector3f(12.0f, -3.0f, 1.0f);

    REQUIRE(plane.wind_ef == Vector3f(0.0f, 0.0f, 0.0f));
    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.0f);

    REQUIRE(plane.velocity_air_ef.x == Catch::Approx(plane.velocity_ef.x));
    REQUIRE(plane.velocity_air_ef.y == Catch::Approx(plane.velocity_ef.y));
    REQUIRE(plane.velocity_air_ef.z == Catch::Approx(plane.velocity_ef.z));
    REQUIRE(plane.velocity_air_bf.x == Catch::Approx(plane.velocity_ef.x));
    REQUIRE(plane.velocity_air_bf.y == Catch::Approx(plane.velocity_ef.y));
    REQUIRE(plane.velocity_air_bf.z == Catch::Approx(plane.velocity_ef.z));
}

TEST_CASE("a steady crosswind measurably shifts groundtrack/groundspeed relative to an otherwise-identical zero-wind run",
          "[sim_plane][wind]") {
    // The ticket's own required closed-loop proof: two SimPlane instances,
    // IDENTICAL initial state and IDENTICAL control-surface/throttle inputs
    // every tick, differing ONLY in wind_config. If SimPlane's physics
    // genuinely distinguishes airspeed (what the aerodynamic model reacts
    // to) from groundspeed (what actually gets integrated into position),
    // a real crosswind must measurably alter the trajectory even though
    // nothing else differs.
    SimPlane plane_no_wind;
    SimPlane plane_crosswind;

    plane_no_wind.position = Vector3f(0.0f, 0.0f, -100.0f); // 100m up
    plane_no_wind.dcm.identity();
    plane_no_wind.velocity_ef = Vector3f(18.0f, 0.0f, 0.0f); // flying north at 18 m/s groundspeed
    plane_crosswind.position = Vector3f(0.0f, 0.0f, -100.0f);
    plane_crosswind.dcm.identity();
    plane_crosswind.velocity_ef = Vector3f(18.0f, 0.0f, 0.0f);
    // Wind FROM due east: blows toward due west (wind_ef.y < 0), directly
    // crossing the plane's north-pointing initial track.
    plane_crosswind.wind_config.speed = 6.0f;
    plane_crosswind.wind_config.direction = 90.0f;
    plane_crosswind.wind_config.turbulence = 0.0f; // deterministic - isolates the steady-wind effect

    const float dt = 0.005f;
    const int steps = 400; // 2 seconds
    for (int i = 0; i < steps; ++i) {
        plane_no_wind.update(0.0f, 0.0f, 0.0f, plane_no_wind.hover_throttle, dt);
        plane_crosswind.update(0.0f, 0.0f, 0.0f, plane_crosswind.hover_throttle, dt);
    }

    REQUIRE_FALSE(plane_no_wind.dcm.is_nan());
    REQUIRE_FALSE(plane_crosswind.dcm.is_nan());

    // Baseline (no wind, no aileron/rudder, symmetric coefficients, zero
    // initial sideslip): the whole trajectory stays confined to the x-z
    // plane - lateral position/velocity stay at exactly zero.
    REQUIRE(plane_no_wind.position.y == Catch::Approx(0.0f).margin(1e-3f));
    REQUIRE(plane_no_wind.velocity_ef.y == Catch::Approx(0.0f).margin(1e-3f));

    // The crosswind run's GROUNDTRACK measurably diverges: real lateral
    // drift, in the physically-correct direction (toward -y, the direction
    // this wind blows toward), an order of magnitude past any integration
    // noise the zero-wind baseline shows.
    REQUIRE(plane_crosswind.position.y < -0.5f);
    REQUIRE(std::fabs(plane_crosswind.velocity_ef.y) > 0.2f);

    // GROUNDSPEED (magnitude of velocity_ef) measurably differs between the
    // two runs...
    const float groundspeed_no_wind = plane_no_wind.velocity_ef.length();
    const float groundspeed_crosswind = plane_crosswind.velocity_ef.length();
    REQUIRE(std::fabs(groundspeed_no_wind - groundspeed_crosswind) > 0.1f);

    // ...while AIRSPEED (what the shared aerodynamic model actually reacts
    // to) stays close between the two runs - both aircraft are flying the
    // same trimmed-ish condition relative to their OWN local air mass, they
    // just have different air masses to be still relative to. This is the
    // "distinguish airspeed from groundspeed" proof the ticket asks for:
    // same airspeed-driven physics, different resulting groundspeed.
    REQUIRE(std::fabs(plane_no_wind.airspeed - plane_crosswind.airspeed) < 0.5f);

    // And wind_ef on the crosswind plane really did hold the expected,
    // sign-verified value throughout (not just nonzero).
    REQUIRE(plane_crosswind.wind_ef.y == Catch::Approx(-6.0f).margin(1e-4f));
}

// --- CPP-094: load_coeffs() round-trip fidelity against real upstream
// content, and the -heavy/-jet mass/thrust_scale frame overrides ---

TEST_CASE("load_coeffs() loading the real upstream skywalker_2013.json fixture reproduces the hardcoded defaults exactly",
          "[sim_plane][load_coeffs]") {
    // tests/fixtures/skywalker_2013.json is a byte-for-byte copy of
    // upstream's OWN Tools/autotest/models/skywalker_2013.json
    // (Plane-4.7.0) - verified directly to be the ONLY real native-format
    // Plane coefficient file anywhere in the pinned upstream tree
    // (`grep -rl c_lift_a` across the whole pinned tree matches nothing
    // else). Callisto.json/freestyle.json (also under Tools/autotest/
    // models/) are MULTICOPTER frame-config formats (mass/battery/
    // motor-count fields, not aerodynamic coefficients) and
    // xplane_plane.json/xplane_heli.json are DREF mapping configs for the
    // unrelated X-Plane external-FDM backend - none of the three are valid
    // load_coeffs() inputs; see sim_plane.hpp's own file banner.
    //
    // Coefficients{}'s hardcoded defaults are THEMSELVES transcribed from
    // this exact file (see Coefficients's own doc comment, "from
    // last_letter skywalker_2013/aerodynamics.yaml"), so a correct
    // load_coeffs() must reproduce every one of them exactly - a genuine
    // round-trip fidelity check against real upstream content, not merely
    // a parser-mechanics smoke test against a synthetic fixture.
    SimPlane plane;
    const std::string path = std::string(FWCPP_SIM_PLANE_FIXTURES_DIR) + "/skywalker_2013.json";
    REQUIRE(plane.load_coeffs(path.c_str()));

    const Coefficients defaults;
    const Coefficients& loaded = plane.coefficient;
    REQUIRE(loaded.s == Catch::Approx(defaults.s));
    REQUIRE(loaded.b == Catch::Approx(defaults.b));
    REQUIRE(loaded.c == Catch::Approx(defaults.c));
    REQUIRE(loaded.c_lift_0 == Catch::Approx(defaults.c_lift_0));
    REQUIRE(loaded.c_lift_deltae == Catch::Approx(defaults.c_lift_deltae));
    REQUIRE(loaded.c_lift_a == Catch::Approx(defaults.c_lift_a));
    REQUIRE(loaded.c_lift_q == Catch::Approx(defaults.c_lift_q));
    REQUIRE(loaded.mcoeff == Catch::Approx(defaults.mcoeff));
    REQUIRE(loaded.oswald == Catch::Approx(defaults.oswald));
    REQUIRE(loaded.alpha_stall == Catch::Approx(defaults.alpha_stall));
    REQUIRE(loaded.c_drag_q == Catch::Approx(defaults.c_drag_q));
    REQUIRE(loaded.c_drag_deltae == Catch::Approx(defaults.c_drag_deltae));
    REQUIRE(loaded.c_drag_p == Catch::Approx(defaults.c_drag_p));
    REQUIRE(loaded.c_y_0 == Catch::Approx(defaults.c_y_0));
    REQUIRE(loaded.c_y_b == Catch::Approx(defaults.c_y_b));
    REQUIRE(loaded.c_y_p == Catch::Approx(defaults.c_y_p));
    REQUIRE(loaded.c_y_r == Catch::Approx(defaults.c_y_r));
    REQUIRE(loaded.c_y_deltaa == Catch::Approx(defaults.c_y_deltaa));
    REQUIRE(loaded.c_y_deltar == Catch::Approx(defaults.c_y_deltar));
    REQUIRE(loaded.c_l_0 == Catch::Approx(defaults.c_l_0));
    REQUIRE(loaded.c_l_p == Catch::Approx(defaults.c_l_p));
    REQUIRE(loaded.c_l_b == Catch::Approx(defaults.c_l_b));
    REQUIRE(loaded.c_l_r == Catch::Approx(defaults.c_l_r));
    REQUIRE(loaded.c_l_deltaa == Catch::Approx(defaults.c_l_deltaa));
    REQUIRE(loaded.c_l_deltar == Catch::Approx(defaults.c_l_deltar));
    REQUIRE(loaded.c_m_0 == Catch::Approx(defaults.c_m_0));
    REQUIRE(loaded.c_m_a == Catch::Approx(defaults.c_m_a));
    REQUIRE(loaded.c_m_q == Catch::Approx(defaults.c_m_q));
    REQUIRE(loaded.c_m_deltae == Catch::Approx(defaults.c_m_deltae));
    REQUIRE(loaded.c_n_0 == Catch::Approx(defaults.c_n_0));
    REQUIRE(loaded.c_n_b == Catch::Approx(defaults.c_n_b));
    REQUIRE(loaded.c_n_p == Catch::Approx(defaults.c_n_p));
    REQUIRE(loaded.c_n_r == Catch::Approx(defaults.c_n_r));
    REQUIRE(loaded.c_n_deltaa == Catch::Approx(defaults.c_n_deltaa));
    REQUIRE(loaded.c_n_deltar == Catch::Approx(defaults.c_n_deltar));
    REQUIRE(loaded.deltaa_max == Catch::Approx(defaults.deltaa_max));
    REQUIRE(loaded.deltae_max == Catch::Approx(defaults.deltae_max));
    REQUIRE(loaded.deltar_max == Catch::Approx(defaults.deltar_max));

    // The CG-offset vector - the field CPP-094 found was silently never
    // loading before this ticket (load_coeffs() read the key as "cg";
    // upstream's real key, SIM_Plane.cpp:191, is "CGOffset" - see
    // load_coeffs()'s own doc comment). Checked explicitly and separately
    // so a regression here can't hide behind the scalar-field checks above.
    REQUIRE(loaded.cg_offset.x == Catch::Approx(defaults.cg_offset.x));
    REQUIRE(loaded.cg_offset.y == Catch::Approx(defaults.cg_offset.y));
    REQUIRE(loaded.cg_offset.z == Catch::Approx(defaults.cg_offset.z));
}

TEST_CASE("load_coeffs() returns false and leaves coefficients untouched for a nonexistent path", "[sim_plane][load_coeffs]") {
    SimPlane plane;
    const Coefficients before = plane.coefficient;
    REQUIRE_FALSE(plane.load_coeffs("/does/not/exist/CPP-094-nonexistent.json"));
    REQUIRE(plane.coefficient.s == before.s);
    REQUIRE(plane.coefficient.cg_offset.x == before.cg_offset.x);
}

TEST_CASE("MassVariant::kStandard leaves mass and thrust_scale at the plain pre-CPP-094 defaults", "[sim_plane][mass_variant]") {
    SimPlane plane;
    REQUIRE(plane.mass == Catch::Approx(2.0f));
    const float expected = (2.0f * kGravityMss) / 0.7f;
    REQUIRE(plane.thrust_scale() == Catch::Approx(expected));
}

TEST_CASE("MassVariant::kHeavy sets mass=8 but leaves thrust_scale at the pre-heavy baseline - the real upstream asymmetry",
          "[sim_plane][mass_variant]") {
    // Upstream SIM_Plane.cpp:53-54: `if (strstr(frame_str, "-heavy")) { mass = 8; }`
    // - mass alone changes, thrust_scale is untouched. Re-verified directly
    // against the real source before writing this test (see
    // sim_plane.hpp's MassVariant doc comment) - deliberately NOT assumed
    // symmetric with kJet below.
    SimPlane plane(Coefficients{}, 2.0f, 0.7f, 20260827U, MassVariant::kHeavy);
    REQUIRE(plane.mass == Catch::Approx(8.0f));

    // thrust_scale is computed from the PRE-heavy baseline mass (2.0f, the
    // mass_kg passed to the constructor here), NOT the real post-heavy
    // 8kg mass.
    const float baseline_thrust_scale = (2.0f * kGravityMss) / 0.7f;
    REQUIRE(plane.thrust_scale() == Catch::Approx(baseline_thrust_scale));

    // Directly disproves the naive "thrust_scale follows mass" assumption
    // kJet's own test below confirms IS how kJet behaves: if kHeavy
    // recomputed the same way, thrust_scale would equal this larger value
    // instead.
    const float if_it_incorrectly_followed_mass = (8.0f * kGravityMss) / 0.7f;
    REQUIRE(plane.thrust_scale() != Catch::Approx(if_it_incorrectly_followed_mass));
}

TEST_CASE("MassVariant::kJet sets mass=22 and recomputes thrust_scale from the new mass", "[sim_plane][mass_variant]") {
    // Upstream SIM_Plane.cpp:56-58 ("a 22kg jet, level top speed is
    // 102m/s", comment transcribed verbatim in sim_plane.hpp): mass=22,
    // thrust_scale = (mass * GRAVITY_MSS) / hover_throttle, RECOMPUTED
    // from the new mass - the opposite of kHeavy's asymmetry above.
    SimPlane plane(Coefficients{}, 2.0f, 0.7f, 20260827U, MassVariant::kJet);
    REQUIRE(plane.mass == Catch::Approx(22.0f));

    // By-hand derivation (also stated in this ticket's commit message):
    // (22.0 * 9.80665) / 0.7 = 215.7463 / 0.7 = 308.209 N per unit
    // throttle. Sanity-check that derivation against a literal here before
    // using it as the expected value below.
    REQUIRE(((22.0f * kGravityMss) / 0.7f) == Catch::Approx(308.209f).margin(0.01f));

    const float expected_thrust_scale = (22.0f * kGravityMss) / 0.7f;
    REQUIRE(plane.thrust_scale() == Catch::Approx(expected_thrust_scale));

    // And explicitly distinct from kHeavy's own (unchanged) thrust_scale -
    // the two branches must not collapse to the same behavior.
    SimPlane heavy_plane(Coefficients{}, 2.0f, 0.7f, 20260827U, MassVariant::kHeavy);
    REQUIRE(plane.thrust_scale() != Catch::Approx(heavy_plane.thrust_scale()));
}
