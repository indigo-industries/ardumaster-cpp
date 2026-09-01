// Tests for the barometer SENSOR model (modules/ap-sim/sim_baro.hpp).
//
// Deliberately separate from baro_test.cpp: that one asserts ap-baro's
// FORMULAS against clean inputs, this one asserts the ERROR MODEL. Upstream
// keeps the same split (AP_Baro vs the SITL backend) and merging them would
// let a bug in either hide behind the other.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

#include <fwcpp/sim/sim_baro.hpp>
#include <fwcpp/sim/sim_plane.hpp>

using fwcpp::sim::SitlBaro;
using fwcpp::sim::SitlBaroSample;

namespace {
// Set the only thing this model reads. SimPlane owns an RNG and so is
// non-copyable -- hence a mutator rather than a factory returning by value.
void set_amsl(fwcpp::sim::SimPlane& p, float amsl_m) {
    p.location.alt = static_cast<std::int32_t>(amsl_m * 100.0f);
}
float alt_of(const SitlBaroSample& s) { return s.altitude_amsl_m; }
}  // namespace

TEST_CASE("zero error by default -- identical to reading the aircraft directly", "[sim_baro]") {
    // This is the property that keeps every pre-existing caller and regression
    // baseline unchanged when the model was introduced. If it ever fails, the
    // model started editing readings nobody asked it to touch.
    fwcpp::sim::SimPlane plane;
    set_amsl(plane, 500.0f);
    SitlBaro m;
    SitlBaroSample got;
    REQUIRE(m.update(plane, 0, got));
    const SitlBaroSample ideal = fwcpp::sim::sitl_baro_from_aircraft(plane);
    REQUIRE(alt_of(got) == Catch::Approx(alt_of(ideal)));
    REQUIRE(got.pressure_pa == Catch::Approx(ideal.pressure_pa));
    REQUIRE(got.temperature_k == Catch::Approx(ideal.temperature_k));
}

TEST_CASE("a disabled barometer produces nothing at all", "[sim_baro]") {
    // Not "produces a stale reading" -- nothing. The caller must be able to
    // tell the difference, or a dead sensor looks like a frozen-but-healthy one.
    fwcpp::sim::SimPlane plane;
    set_amsl(plane, 100.0f);
    SitlBaro m;
    m.disabled = true;
    SitlBaroSample got;
    REQUIRE_FALSE(m.update(plane, 0, got));
    REQUIRE_FALSE(m.have_output());
}

TEST_CASE("bias is a fixed offset; glitch adds on top of it", "[sim_baro]") {
    fwcpp::sim::SimPlane plane;
    set_amsl(plane, 1000.0f);
    SitlBaro m;
    m.bias_m = 12.0f;
    SitlBaroSample got;
    REQUIRE(m.update(plane, 0, got));
    REQUIRE(alt_of(got) == Catch::Approx(1012.0f).margin(0.01));

    m.glitch_m = -30.0f;
    REQUIRE(m.update(plane, 10, got));
    REQUIRE(alt_of(got) == Catch::Approx(982.0f).margin(0.01));
}

TEST_CASE("drift accumulates in wall time, not per call", "[sim_baro]") {
    // The rate a caller happens to poll at must not change how much the sensor
    // has drifted after a given elapsed time -- otherwise the "error" is really
    // a scheduling artifact and would move whenever a loop rate changed.
    fwcpp::sim::SimPlane plane;
    set_amsl(plane, 0.0f);
    SitlBaroSample got;

    SitlBaro slow;
    slow.drift_mps = 0.5f;
    REQUIRE(slow.update(plane, 0, got));
    REQUIRE(slow.update(plane, 10000, got));      // one 10 s step

    SitlBaro fast;
    fast.drift_mps = 0.5f;
    REQUIRE(fast.update(plane, 0, got));
    for (std::uint32_t t = 100; t <= 10000; t += 100) {   // 100 steps, same 10 s
        REQUIRE(fast.update(plane, t, got));
    }

    INFO("slow " << slow.drift_accumulated_m() << " vs fast " << fast.drift_accumulated_m());
    REQUIRE(slow.drift_accumulated_m() == Catch::Approx(5.0f).margin(0.01));
    REQUIRE(fast.drift_accumulated_m() == Catch::Approx(slow.drift_accumulated_m()).margin(0.01));
}

TEST_CASE("noise is zero-mean and scales with the configured sigma", "[sim_baro]") {
    fwcpp::sim::SimPlane plane;
    set_amsl(plane, 200.0f);
    SitlBaro m;
    m.noise_m = 2.0f;
    SitlBaroSample got;

    double sum = 0.0;
    double sum_sq = 0.0;
    constexpr int kN = 4000;
    for (int i = 0; i < kN; ++i) {
        REQUIRE(m.update(plane, static_cast<std::uint32_t>(i), got));
        const double err = alt_of(got) - 200.0;
        sum += err;
        sum_sq += err * err;
    }
    const double mean = sum / kN;
    const double sigma = std::sqrt(sum_sq / kN - mean * mean);
    INFO("mean " << mean << " sigma " << sigma << " (configured 2.0)");
    REQUIRE(std::fabs(mean) < 0.15);                  // zero-mean, not a bias
    REQUIRE(sigma == Catch::Approx(2.0).epsilon(0.1)); // and the right magnitude
}

TEST_CASE("the same seed reproduces the same noise sequence", "[sim_baro]") {
    // Determinism is why this port owns its RNG instead of using libc rand():
    // a reproducible sensor is the difference between a regression test and a
    // coin flip.
    fwcpp::sim::SimPlane plane;
    set_amsl(plane, 300.0f);
    SitlBaroSample a{};
    SitlBaroSample b{};
    SitlBaro m1(12345U);
    SitlBaro m2(12345U);
    m1.noise_m = 1.0f;
    m2.noise_m = 1.0f;
    for (std::uint32_t i = 0; i < 50; ++i) {
        REQUIRE(m1.update(plane, i, a));
        REQUIRE(m2.update(plane, i, b));
        REQUIRE(alt_of(a) == Catch::Approx(alt_of(b)));
    }

    SitlBaro m3(999U);
    m3.noise_m = 1.0f;
    REQUIRE(m3.update(plane, 0, b));
    REQUIRE(alt_of(a) != Catch::Approx(alt_of(b)));   // a different seed really differs
}

TEST_CASE("transport lag withholds a reading until the delay is filled, then returns the old one",
          "[sim_baro]") {
    SitlBaro m;
    m.delay_ms = 200;
    SitlBaroSample got;

    // Climbing: each tick's true altitude differs, so a late sample is
    // detectable as the altitude it WAS, not the one it is now.
    fwcpp::sim::SimPlane climbing;
    set_amsl(climbing, 0.0f);
    REQUIRE_FALSE(m.update(climbing, 0, got));        // nothing yet -- delay unfilled
    REQUIRE_FALSE(m.have_output());

    for (std::uint32_t t = 50; t < 200; t += 50) {
        climbing.location.alt = static_cast<std::int32_t>(t * 100);  // 1 m per ms
        REQUIRE_FALSE(m.update(climbing, t, got));
    }

    climbing.location.alt = 20000;                    // now truly at 200 m
    REQUIRE(m.update(climbing, 200, got));            // 200 ms old sample arrives
    INFO("delayed reading " << alt_of(got) << " m while truly at 200 m");
    REQUIRE(alt_of(got) == Catch::Approx(0.0).margin(0.01));  // the t=0 sample
}

TEST_CASE("apply_upstream_defaults() matches upstream's own SITL parameters", "[sim_baro]") {
    // Upstream ships SIM_BARO_RND=0.2 and everything else zero. This port
    // defaults to fully zero instead (see the class comment), so the two must
    // be told apart deliberately rather than by accident.
    SitlBaro m;
    REQUIRE(m.noise_m == Catch::Approx(0.0));
    m.apply_upstream_defaults();
    REQUIRE(m.noise_m == Catch::Approx(0.2));
    REQUIRE(m.drift_mps == Catch::Approx(0.0));
    REQUIRE(m.delay_ms == 0);
    REQUIRE_FALSE(m.disabled);
}
