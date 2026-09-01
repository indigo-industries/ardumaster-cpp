// Unit tests for modules/ap-baro -- this port's AP_Baro subsystem.
//
// The most valuable assertion here is the CROSS-CHECK: Baro::get_eas2tas()
// works from measured pressure with a lapse-rate-extrapolated temperature
// (upstream AP_Baro::get_EAS2TAS), while fwcpp::atmosphere works from a full
// 1976 U.S. Standard Atmosphere table. Those are two independent derivations,
// so agreeing to a small tolerance is real evidence both are right -- and it
// is exactly the property that silently broke when the harness was feeding the
// plant's own eas2tas instead of an estimate.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <fwcpp/atmosphere.hpp>
#include <fwcpp/baro/baro.hpp>

using fwcpp::baro::Baro;

namespace {
// Feed a barometer as if the vehicle sat at `ground_amsl` and then flew to
// `alt_amsl`, both metres AMSL, using the standard atmosphere for the samples.
Baro baro_at(float ground_amsl, float alt_amsl) {
    Baro b;
    float p = 0.0f;
    float t_k = 0.0f;
    fwcpp::atmosphere::get_pressure_temperature_for_alt_amsl(ground_amsl, p, t_k);
    b.update(p, t_k - fwcpp::baro::kCtoKelvin, 0);
    b.calibrate();
    fwcpp::atmosphere::get_pressure_temperature_for_alt_amsl(alt_amsl, p, t_k);
    b.update(p, t_k - fwcpp::baro::kCtoKelvin, 100);
    return b;
}
}  // namespace

TEST_CASE("Baro is unhealthy and returns safe values before any sample", "[baro]") {
    Baro b;
    REQUIRE_FALSE(b.healthy());
    REQUIRE_FALSE(b.calibrated());
    // No pressure yet, so no altitude and no conversion -- must not divide by
    // the zero ground reference.
    REQUIRE(b.get_altitude() == Catch::Approx(0.0).margin(1e-6));
    REQUIRE(b.get_eas2tas() == Catch::Approx(1.0).margin(1e-6));
}

TEST_CASE("a rejected sample leaves the barometer unhealthy", "[baro]") {
    Baro b;
    b.update(0.0f, 15.0f, 10);      // non-positive pressure is not a reading
    REQUIRE_FALSE(b.healthy());
    b.update(-5.0f, 15.0f, 20);
    REQUIRE_FALSE(b.healthy());
    b.update(101325.0f, 15.0f, 30);
    REQUIRE(b.healthy());
    REQUIRE(b.last_update_ms() == 30);
}

TEST_CASE("the first good sample self-calibrates, so altitude starts at zero", "[baro]") {
    Baro b;
    b.update(101325.0f, 15.0f, 0);
    REQUIRE(b.calibrated());
    REQUIRE(b.get_ground_pressure() == Catch::Approx(101325.0));
    REQUIRE(b.get_altitude() == Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("calibrate() re-latches the ground reference at the current reading", "[baro]") {
    // Calibrating at 1000 m makes THAT the zero, which is the whole point of a
    // ground reference: altitude is relative to where you turned it on.
    Baro b = baro_at(0.0f, 1000.0f);
    REQUIRE(b.get_altitude() > 900.0f);
    b.calibrate();
    REQUIRE(b.get_altitude() == Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("pressure-derived altitude tracks the standard atmosphere", "[baro]") {
    // Upstream documents get_altitude_difference() as "within +-2.5m of the
    // standard atmosphere tables in the troposphere (up to 11,000 m amsl)".
    // Hold it to that claim rather than to a number this port invented.
    //
    // COMPARE AGAINST GEOPOTENTIAL ALTITUDE, not geometric. Upstream's closed
    // form is derived in geopotential altitude and returns it; the 1976 model
    // in fwcpp::atmosphere takes GEOMETRIC altitude and converts internally.
    // Comparing the two directly looks like formula error that grows with
    // height, and it is not: at 8000 m geometric the geopotential equivalent
    // is 8000 * R/(R+h) = 7989.9 m, which is the entire 10.2 m "discrepancy"
    // measured before this conversion was applied. Well inside +-2.5 m once
    // the two are expressed in the same units -- so upstream's claim holds,
    // and the naive comparison was the bug.
    for (const float alt : {100.0f, 500.0f, 1000.0f, 3000.0f, 8000.0f}) {
        const Baro b = baro_at(0.0f, alt);
        const float expected_geopotential = fwcpp::atmosphere::geometric_alt_to_geopotential(alt);
        INFO("alt " << alt << " m geometric (= " << expected_geopotential
             << " m geopotential) -> baro " << b.get_altitude() << " m");
        REQUIRE(b.get_altitude() == Catch::Approx(expected_geopotential).margin(2.5));
    }
}

TEST_CASE("eas2tas is 1.0 at sea level and rises with altitude", "[baro][eas2tas]") {
    const Baro ground = baro_at(0.0f, 0.0f);
    REQUIRE(ground.get_eas2tas() == Catch::Approx(1.0).margin(0.005));

    float previous = ground.get_eas2tas();
    for (const float alt : {500.0f, 1000.0f, 2000.0f, 5000.0f}) {
        const Baro b = baro_at(0.0f, alt);
        INFO("alt " << alt << " m -> eas2tas " << b.get_eas2tas());
        REQUIRE(b.get_eas2tas() > previous);   // strictly monotonic in altitude
        previous = b.get_eas2tas();
    }
    REQUIRE(previous > 1.25f);                 // ~1.29 at 5 km
}

TEST_CASE("baro eas2tas agrees with the independent 1976 atmosphere model", "[baro][eas2tas]") {
    // Two different derivations (measured pressure + lapse-rate temperature vs
    // full ISA table lookup) landing within 1% of each other across the
    // troposphere. This is the cross-check that would catch either one drifting.
    for (const float alt : {0.0f, 338.0f, 1000.0f, 3000.0f, 5000.0f}) {
        const Baro b = baro_at(0.0f, alt);
        const float model = fwcpp::atmosphere::get_eas2tas_for_alt_amsl(alt);
        INFO("alt " << alt << " m: baro " << b.get_eas2tas() << " vs model " << model);
        REQUIRE(b.get_eas2tas() == Catch::Approx(model).epsilon(0.01));
    }
}

TEST_CASE("a mis-set ground reference biases eas2tas -- the error a model cannot express",
          "[baro][eas2tas]") {
    // Calibrating on the ground at a 1500 m field and then flying to 3000 m
    // must give the same conversion as a barometer that saw the whole climb
    // from sea level: eas2tas depends on where you ARE, not where you started.
    // A standard-day model of "altitude above start" gets this wrong, which is
    // precisely why the estimate was replaced by a real barometer.
    const Baro from_sea_level = baro_at(0.0f, 3000.0f);
    const Baro from_high_field = baro_at(1500.0f, 3000.0f);
    INFO("sea-level-cal " << from_sea_level.get_eas2tas()
         << " vs high-field-cal " << from_high_field.get_eas2tas());
    REQUIRE(from_high_field.get_eas2tas() == Catch::Approx(from_sea_level.get_eas2tas()).epsilon(0.02));
}
