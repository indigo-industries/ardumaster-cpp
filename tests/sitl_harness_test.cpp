// Tests for fwcpp::hal_sitl::SitlHarness (CPP-084).
//
// Style note: mirrors vehicle_test.cpp's own closed-loop integration tests
// (run_biased_closed_loop() etc.) - ap-sim (SimPlane) is a TEST-ONLY
// dependency of ap-hal-sitl's own test target here (ap-hal-sitl itself
// links ap-sim publicly since SitlHarness's constructor takes a SimPlane&,
// unlike ap-vehicle, which deliberately shares no code with ap-sim - see
// sim_plane.hpp's own file banner).
//
// THE ACCEPTANCE TEST THIS FILE EXISTS FOR: prove SitlHarness::step()
// genuinely produces the SAME tick-by-tick outcome as manually performing
// the same synthesis inline, not just "it compiles and runs without
// crashing". Below, TWO independent Plane/SimPlane pairs run the identical
// number of ticks with identical stick inputs and identical dt/now_ms
// progression - one side driven entirely through SitlHarness::step(), the
// other through a hand-rolled loop that inlines the exact same synthesis
// formulas SitlHarness::step() uses internally (see sitl_harness.hpp's own
// file banner for the citation behind each line reproduced here). Both
// SimPlane instances are default-constructed with the SAME default RNG
// seed (sim_plane.hpp's own wind_rng_seed default, 20260827U) and are
// driven through an identical call sequence, so the airspeed sensor's own
// per-tick noise draw (airspeed_sensor_differential_pressure()) advances
// identically on both sides too - nothing here is a source of nondeterminism
// between the two runs.

#include <cmath>
#include <cstdint>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/hal_sitl/sitl_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/baro/baro.hpp>
#include <fwcpp/sim/sim_baro.hpp>
#include <fwcpp/ahrs/ahrs_atmosphere.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

using namespace fwcpp::vehicle;
using fwcpp::hal_sitl::SitlHarness;

namespace {

// Same helper as vehicle_test.cpp's own set_sticks() (that file's copy is
// anonymous-namespace-local, not exported - reimplemented here identically
// rather than exposing it as shared test infrastructure this ticket's own
// scope does not call for).
void set_sticks(Plane& plane, std::uint16_t roll_pwm, std::uint16_t pitch_pwm, std::uint16_t throttle_pwm,
                 std::uint16_t rudder_pwm) {
    plane.hal.rc_input.set_channel(kChannelRoll, roll_pwm);
    plane.hal.rc_input.set_channel(kChannelPitch, pitch_pwm);
    plane.hal.rc_input.set_channel(kChannelThrottle, throttle_pwm);
    plane.hal.rc_input.set_channel(kChannelRudder, rudder_pwm);
    plane.rc_channels.read_input(plane.hal.rc_input);
}

constexpr float kTol = 1e-4f;

} // namespace

TEST_CASE("SitlHarness::step() is behavior-preserving relative to a hand-rolled equivalent using the same formulas",
          "[hal_sitl][sitl_harness][integration]") {
    constexpr float kDt = 0.02f; // 50Hz, matching every existing closed-loop test
    constexpr int kNumTicks = 500; // 10 simulated seconds - long enough for attitude/position/velocity to move
                                    // substantially and meaningfully diverge if the two synthesis paths ever
                                    // disagreed, short enough to run instantly under ctest.

    // "Harness" side - driven entirely through SitlHarness::step().
    Plane harness_plane;
    fwcpp::sim::SimPlane harness_sim;
    ModeFBWA harness_mode(harness_plane);
    harness_plane.control_mode = &harness_mode;
    harness_plane.armed = true;
    harness_plane.hal.rc_output.force_safety_off();
    SitlHarness harness(harness_plane, harness_sim);

    // "Manual" side - an INDEPENDENT Plane/SimPlane pair, driven by a
    // hand-rolled loop inlining SitlHarness::step()'s own exact formulas.
    Plane manual_plane;
    fwcpp::sim::SimPlane manual_sim;
    ModeFBWA manual_mode(manual_plane);
    manual_plane.control_mode = &manual_mode;
    manual_plane.armed = true;
    manual_plane.hal.rc_output.force_safety_off();

    std::uint32_t now_ms = 0;
    for (int i = 0; i < kNumTicks; ++i) {
        now_ms += 20;

        // Identical, fixed moderate right-roll-and-climb stick command on
        // both sides every tick - same precedent as vehicle_test.cpp's own
        // run_biased_closed_loop().
        set_sticks(harness_plane, 1650, 1500, 1700, 1500);
        set_sticks(manual_plane, 1650, 1500, 1700, 1500);

        harness.step(now_ms, kDt);

        // Manual equivalent of SitlHarness::step() - see sitl_harness.hpp's
        // own file banner for the citation behind each line below.
        StabilizeInputs in;
        in.dt = kDt;
        in.now_ms = now_ms;
        in.now_us = static_cast<std::uint64_t>(now_ms) * 1000ULL;

        const fwcpp::math::Vector3f measured_gyro = manual_sim.gyro; // zero bias, matches step()'s own default
        fwcpp::ahrs::GyroSample gyro_sample;
        gyro_sample.gyro = measured_gyro;
        gyro_sample.delta_angle = measured_gyro * kDt;
        gyro_sample.dangle_dt = kDt;

        in.accel_sample.accel = manual_sim.accel_body;
        in.accel_sample.delta_velocity = manual_sim.accel_body * kDt;
        in.accel_sample.delta_velocity_dt = kDt;
        in.accel_y = manual_sim.accel_body.y;

        in.compass_healthy = true;
        in.compass_field_bf = manual_plane.compass.rotate_earth_field_to_body(manual_sim.dcm);

        in.true_velocity_ned = manual_sim.velocity_ef;

        in.airspeed_sensor_enabled = true;
        in.airspeed_raw_pressure_pa = manual_sim.airspeed_sensor_differential_pressure();

        // wind_estimate left at StabilizeInputs' own zero default, matching
        // step()'s own choice.
        in.gps_use_enabled = true;
        in.position_ned = manual_sim.position;
        in.current_altitude_m = -manual_sim.position.z;
        // Mirror SitlHarness::step()'s baro feed. eas2tas is no longer set
        // here at all -- tick() derives it inside plane.baro from this
        // pressure/temperature pair, so mirroring the SENSOR input is what
        // keeps the two the same experiment.
        {
            const fwcpp::sim::SitlBaroSample baro = fwcpp::sim::sitl_baro_from_aircraft(manual_sim);
            in.baro_pressure_pa = baro.pressure_pa;
            in.baro_temperature_c = baro.temperature_k - fwcpp::baro::kCtoKelvin;
            in.baro_sensor_enabled = true;
        }

        tick(manual_plane, gyro_sample, in);

        const float aileron = manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) / kServoMax;
        const float elevator = manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator) / kServoMax;
        const float rudder = manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder) / kServoMax;
        const float throttle = manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) / 100.0f;
        manual_sim.update(aileron, elevator, rudder, throttle, kDt);
    }

    // --- Compare final SimPlane ground truth, exactly, field by field ---
    INFO("harness_sim.position = (" << harness_sim.position.x << "," << harness_sim.position.y << ","
                                     << harness_sim.position.z << "), manual_sim.position = ("
                                     << manual_sim.position.x << "," << manual_sim.position.y << ","
                                     << manual_sim.position.z << ")");
    REQUIRE(harness_sim.position == manual_sim.position);
    REQUIRE(harness_sim.velocity_ef == manual_sim.velocity_ef);
    REQUIRE(harness_sim.gyro == manual_sim.gyro);
    REQUIRE(harness_sim.accel_body == manual_sim.accel_body);
    REQUIRE(std::fabs(harness_sim.airspeed - manual_sim.airspeed) < kTol);

    float harness_true_roll = 0.0f, harness_true_pitch = 0.0f, harness_true_yaw = 0.0f;
    float manual_true_roll = 0.0f, manual_true_pitch = 0.0f, manual_true_yaw = 0.0f;
    harness_sim.dcm.to_euler(&harness_true_roll, &harness_true_pitch, &harness_true_yaw);
    manual_sim.dcm.to_euler(&manual_true_roll, &manual_true_pitch, &manual_true_yaw);
    REQUIRE(std::fabs(harness_true_roll - manual_true_roll) < kTol);
    REQUIRE(std::fabs(harness_true_pitch - manual_true_pitch) < kTol);
    REQUIRE(std::fabs(harness_true_yaw - manual_true_yaw) < kTol);

    // Sanity: 500 ticks of aggressive right-roll-and-climb stick input
    // actually moved the airframe substantially - a trivial "everything
    // stayed at zero" pass would not be a meaningful behavior-preservation
    // proof.
    REQUIRE(harness_sim.position.z != 0.0f);

    // --- Compare final Plane/AHRS estimated state ---
    REQUIRE(std::fabs(harness_plane.ahrs->get_roll() - manual_plane.ahrs->get_roll()) < kTol);
    REQUIRE(std::fabs(harness_plane.ahrs->get_pitch() - manual_plane.ahrs->get_pitch()) < kTol);
    REQUIRE(std::fabs(harness_plane.ahrs->get_yaw() - manual_plane.ahrs->get_yaw()) < kTol);

    // --- Compare final GPS/compass/airspeed sensor sample state ---
    REQUIRE(harness_plane.gps.sample().ground_speed_ms == manual_plane.gps.sample().ground_speed_ms);
    REQUIRE(harness_plane.gps.sample().ground_course_deg == manual_plane.gps.sample().ground_course_deg);
    REQUIRE(harness_plane.gps.sample().has_fix == manual_plane.gps.sample().has_fix);
    REQUIRE(harness_plane.compass.sample().field == manual_plane.compass.sample().field);
    REQUIRE(harness_plane.compass.sample().healthy == manual_plane.compass.sample().healthy);
    REQUIRE(harness_plane.airspeed_sensor.healthy() == manual_plane.airspeed_sensor.healthy());
    REQUIRE(std::fabs(harness_plane.airspeed_sensor.airspeed() - manual_plane.airspeed_sensor.airspeed()) < kTol);

    // --- Compare final commanded servo outputs (the last tick's, still
    //     cached on both plane.srv_channels after the loop above) ---
    REQUIRE(std::fabs(harness_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) -
                       manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron)) < kTol);
    REQUIRE(std::fabs(harness_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator) -
                       manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator)) < kTol);
    REQUIRE(std::fabs(harness_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder) -
                       manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder)) < kTol);
    REQUIRE(std::fabs(harness_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) -
                       manual_plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle)) < kTol);
}

TEST_CASE("SitlHarness::step() defaults to zero injected gyro bias, and a caller-supplied bias is honored",
          "[hal_sitl][sitl_harness]") {
    // Proves the OTHER documented design decision this ticket makes: the
    // harness itself defaults to feeding the true gyro rate (zero injected
    // bias), and a caller wanting to exercise drift correction against a
    // biased gyro (the exact scenario vehicle_test.cpp's own
    // run_biased_closed_loop() hand-rolls today) can do so through step()'s
    // own optional gyro_bias parameter, without reaching into SitlHarness's
    // internals.
    //
    // NOT tested via long-run attitude divergence (unlike
    // run_biased_closed_loop()'s own pair of tests): THAT test's dramatic
    // >30/>60deg divergence floor depends specifically on drift correction
    // being OFF (gps_use_enabled=false) in its "uncorrected" run.
    // SitlHarness::step() always wires the REAL, fully-corrected path
    // (compass_healthy=true, gps_use_enabled=true, unconditionally - see
    // sitl_harness.hpp's own file banner) - a constant gyro bias gets
    // actively, continuously corrected by drift_correction_yaw()/
    // drift_correction_accel() every tick here, so it does NOT accumulate
    // into a large long-run divergence the way it does with correction
    // disabled (verified directly: a 200-tick/4s run at this same bias
    // magnitude measured only ~0.12deg of final yaw divergence - correction
    // working as intended, not a sign the bias parameter is inert).
    //
    // Instead, this test checks the bias parameter's effect at the one
    // point in the pipeline where it is observable in isolation, BEFORE any
    // correction has had a chance to act: AhrsDcm::matrix_update()'s own
    // final line (ahrs_dcm.hpp) is `omega = sample.gyro + omega_i_;` - on
    // this vehicle's very FIRST tick, omega_i_ (the yaw-rate integrator
    // drift correction accumulates into) is still exactly zero (a
    // freshly-constructed AhrsDcm), so get_omega() immediately after tick 1
    // equals sample.gyro exactly - i.e. the measured (possibly biased) rate
    // step() built, with no correction yet applied to mask it.
    constexpr float kDt = 0.02f;
    const fwcpp::math::Vector3f bias(0.02f, -0.01f, 0.03f);

    Plane unbiased_plane;
    fwcpp::sim::SimPlane unbiased_sim;
    ModeFBWA unbiased_mode(unbiased_plane);
    unbiased_plane.control_mode = &unbiased_mode;
    unbiased_plane.armed = true;
    unbiased_plane.hal.rc_output.force_safety_off();
    SitlHarness unbiased_harness(unbiased_plane, unbiased_sim);

    Plane biased_plane;
    fwcpp::sim::SimPlane biased_sim;
    ModeFBWA biased_mode(biased_plane);
    biased_plane.control_mode = &biased_mode;
    biased_plane.armed = true;
    biased_plane.hal.rc_output.force_safety_off();
    SitlHarness biased_harness(biased_plane, biased_sim);

    // Both SimPlane instances start identically (default-constructed,
    // level, stationary) - true sim_plane.gyro is exactly zero entering
    // this very first tick on both sides, so unbiased_harness's measured
    // gyro is exactly zero and biased_harness's is exactly `bias`.
    set_sticks(unbiased_plane, 1650, 1500, 1700, 1500);
    set_sticks(biased_plane, 1650, 1500, 1700, 1500);
    unbiased_harness.step(20, kDt); // default gyro_bias = zero vector
    biased_harness.step(20, kDt, bias);

    const fwcpp::math::Vector3f unbiased_omega = unbiased_plane.ahrs->get_omega();
    const fwcpp::math::Vector3f biased_omega = biased_plane.ahrs->get_omega();
    const fwcpp::math::Vector3f observed_delta = biased_omega - unbiased_omega;

    INFO("unbiased get_omega() = (" << unbiased_omega.x << "," << unbiased_omega.y << "," << unbiased_omega.z
                                     << "), biased get_omega() = (" << biased_omega.x << "," << biased_omega.y << ","
                                     << biased_omega.z << ")");
    REQUIRE(std::fabs(observed_delta.x - bias.x) < kTol);
    REQUIRE(std::fabs(observed_delta.y - bias.y) < kTol);
    REQUIRE(std::fabs(observed_delta.z - bias.z) < kTol);
}

// ---------------------------------------------------------------------------
// Integration seam: harness baro model -> StabilizeInputs -> plane.baro.
//
// The model's own behaviour is covered in sim_baro_model_test.cpp and the
// barometer's arithmetic in baro_test.cpp. Neither proves they are actually
// CONNECTED -- the harness populates in.baro_* and tick() feeds plane.baro,
// and a broken seam there would leave both suites green while the vehicle flew
// on a barometer nobody was updating. Injecting a bias and watching it arrive
// at the far end is the cheapest thing that catches that.
// ---------------------------------------------------------------------------
TEST_CASE("a bias dialled into the harness baro model reaches plane.baro", "[sitl_harness][baro]") {
    // TWO harnesses stepped identically, differing only in the injected bias.
    //
    // Not one harness with a hand-set sim.location.alt: step() runs the plant,
    // and SimPlane::update() recomputes location from its NED position, so a
    // hand-set altitude is overwritten on the first step. (That overwrite is
    // itself the update_position() fix earlier in this branch -- before it,
    // location never moved and a test like this would have "worked" for the
    // wrong reason.) Two plants from the same deterministic seed evolve
    // identically, so any difference at the far end is the bias and nothing else.
    fwcpp::vehicle::Plane plane_clean;
    fwcpp::sim::SimPlane sim_clean;
    fwcpp::hal_sitl::SitlHarness clean(plane_clean, sim_clean);

    fwcpp::vehicle::Plane plane_biased;
    fwcpp::sim::SimPlane sim_biased;
    fwcpp::hal_sitl::SitlHarness biased(plane_biased, sim_biased);
    biased.baro_model().bias_m = 50.0f;   // reads 50 m high => LOWER pressure

    for (std::uint32_t t = 0; t <= 100; t += 20) {
        clean.step(t, 0.02f);
        biased.step(t, 0.02f);
    }

    REQUIRE(plane_clean.baro.healthy());
    REQUIRE(plane_biased.baro.healthy());
    INFO("clean " << plane_clean.baro.get_pressure() << " Pa vs biased "
         << plane_biased.baro.get_pressure() << " Pa");
    REQUIRE(plane_biased.baro.get_pressure() < plane_clean.baro.get_pressure());

    // Both calibrated on their own first reading, so the bias is baked into
    // each one's zero and altitude still reads near zero -- the bias is a
    // pressure offset, not an altitude the vehicle can see. Asserting that
    // explicitly, because it is the non-obvious half.
    INFO("clean alt " << plane_clean.baro.get_altitude()
         << " m, biased alt " << plane_biased.baro.get_altitude() << " m");
    REQUIRE(plane_biased.baro.get_altitude() ==
            Catch::Approx(plane_clean.baro.get_altitude()).margin(1.0f));
}

TEST_CASE("a disabled harness baro model stops updating plane.baro", "[sitl_harness][baro]") {
    fwcpp::vehicle::Plane plane;
    fwcpp::sim::SimPlane sim;
    fwcpp::hal_sitl::SitlHarness harness(plane, sim);

    harness.step(0, 0.02f);
    REQUIRE(plane.baro.healthy());
    const std::uint32_t last = plane.baro.last_update_ms();

    // With the sensor dead the harness must leave baro_sensor_enabled false, so
    // tick() never calls baro.update() and the reading goes stale rather than
    // silently tracking an aircraft the sensor can no longer see.
    harness.baro_model().disabled = true;
    // The plant keeps flying underneath it; a live baro would follow.
    for (std::uint32_t t = 20; t <= 200; t += 20) {
        harness.step(t, 0.02f);
    }
    REQUIRE(plane.baro.last_update_ms() == last);
}
