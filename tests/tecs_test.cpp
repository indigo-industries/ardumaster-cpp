// Tests for fwcpp::tecs::Tecs (CPP-029 slice 1).
//
// Style note: Tecs's control-law internals are private (mirroring
// upstream's own AP_TECS, and this port's L1Control precedent) - every
// test below drives the class through its public update_50hz()/
// update_pitch_throttle() entry points and reads back public accessors,
// several of which (energy_state(), get_hgt_dem(), get_tas_dem_adj(),
// pitch_limits_deg(), throttle_limits(), etc.) exist specifically to make
// this kind of white-box checking possible - see tecs.hpp's own banner.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tecs/tecs.hpp>

using namespace fwcpp::tecs;

namespace {

constexpr float kG = kGravityMss;

// A fresh TecsInputs with every field set to a plausible, level-flight,
// airspeed-available default. Individual tests override only what they
// need to change.
TecsInputs default_inputs(std::uint64_t now_us, std::uint32_t now_ms) {
    TecsInputs in;
    in.rotation_body_to_ned.identity();
    in.eas2tas = 1.0f;
    in.using_airspeed_sensor = true;
    in.airspeed_eas_valid = true;
    in.airspeed_eas = 12.0f; // matches FixedWingParams::airspeed_cruise default
    in.velocity_ned_valid = true;
    in.velocity_down_ms = 0.0f;
    in.now_us = now_us;
    in.now_ms = now_ms;
    return in;
}

} // namespace

// ---------------------------------------------------------------------
// _update_energies(): specific-energy/energy-balance math against
// hand-computed values.
// ---------------------------------------------------------------------

TEST_CASE("update_energies: SPE/SKE estimates match hand-computed values for a known height/speed state", "[tecs][energy]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    // First-ever update_50hz() call is deterministic regardless of filter
    // history: height_ = -relative_position_d_home_m (no filtering
    // involved), climb_rate_ = -velocity_down_ms (the velocity_ned_valid
    // branch overwrites the reset-tick's climb_rate_=0.0f unconditionally
    // - see tecs.hpp's update_50hz()), and tas_state_ = airspeed_eas
    // exactly (the reset branch of update_speed() sets it directly, no
    // filter lag) - see tecs.hpp's file banner note on this same fact.
    TecsInputs in = default_inputs(5'000'000, 1000);
    in.relative_position_d_home_m = -50.0f; // 50m above home
    in.velocity_down_ms = -2.0f;            // climbing at 2 m/s
    in.airspeed_eas = 15.0f;

    tecs.update_50hz(in);

    REQUIRE(tecs.get_height() == Catch::Approx(50.0f));
    REQUIRE(tecs.get_climb_rate() == Catch::Approx(2.0f));
    REQUIRE(tecs.get_tas_state() == Catch::Approx(15.0f));

    in.now_us += 20000;
    in.now_ms += 20;
    tecs.update_pitch_throttle(8000, 1600, 50.0f, 1.0f, in); // 80m / 16 m/s demand, hgt_afe matches actual height

    const auto e = tecs.energy_state();

    // Hand-computed: SPE_est = height * g, SKE_est = 0.5 * V^2.
    REQUIRE(e.spe_est == Catch::Approx(50.0f * kG));
    REQUIRE(e.ske_est == Catch::Approx(0.5f * 15.0f * 15.0f));
    // Hand-computed: SPEdot = climb_rate * g.
    REQUIRE(e.spedot == Catch::Approx(2.0f * kG));
    // On this (first, reset) tick vel_dot_/vel_dot_lpf_ are both still
    // exactly zero (update_speed()'s reset branch never touches vel_dot_,
    // only copies vel_dot_lpf_ from it) so SKEdot must be exactly zero too.
    REQUIRE(e.skedot == Catch::Approx(0.0f));

    // Algebraic identities: SPE_dem/SKE_dem are always exactly
    // hgt_dem*g / 0.5*tas_dem_adj^2 by construction of _update_energies(),
    // regardless of what the (separately-tested) demand-shaping filters
    // upstream of it produced this tick - checking the identity directly
    // verifies _update_energies() itself is wired correctly (right
    // formula, right sign, right constant).
    REQUIRE(e.spe_dem == Catch::Approx(tecs.get_hgt_dem() * kG));
    REQUIRE(e.ske_dem == Catch::Approx(0.5f * tecs.get_tas_dem_adj() * tecs.get_tas_dem_adj()));
}

TEST_CASE("update_energies: a second, independent height/speed/climb-rate scenario", "[tecs][energy]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    TecsInputs in = default_inputs(2'000'000, 2000);
    in.relative_position_d_home_m = -100.0f; // 100m above home
    in.velocity_down_ms = 3.0f;              // descending at 3 m/s (+ve down)
    in.airspeed_eas = 20.0f;

    tecs.update_50hz(in);

    REQUIRE(tecs.get_height() == Catch::Approx(100.0f));
    REQUIRE(tecs.get_climb_rate() == Catch::Approx(-3.0f));
    REQUIRE(tecs.get_tas_state() == Catch::Approx(20.0f));

    in.now_us += 20000;
    in.now_ms += 20;
    tecs.update_pitch_throttle(5000, 1800, 100.0f, 1.0f, in); // hgt_afe matches actual height

    const auto e = tecs.energy_state();
    REQUIRE(e.spe_est == Catch::Approx(100.0f * kG));
    REQUIRE(e.ske_est == Catch::Approx(0.5f * 20.0f * 20.0f));
    REQUIRE(e.spedot == Catch::Approx(-3.0f * kG));
    REQUIRE(e.spe_dem == Catch::Approx(tecs.get_hgt_dem() * kG));
    REQUIRE(e.ske_dem == Catch::Approx(0.5f * tecs.get_tas_dem_adj() * tecs.get_tas_dem_adj()));
}

// ---------------------------------------------------------------------
// _update_throttle_with_airspeed(): sensible throttle direction.
// ---------------------------------------------------------------------

namespace {

// Drives `ticks` calls of update_50hz()+update_pitch_throttle() with a
// CONSTANT actual state (height_m/airspeed_ms/climb_rate_ms, none of them
// affected by TECS's own output - there is no plant in this scenario) and
// a constant commanded (hgt_dem_cm, eas_dem_cm), returning the final
// get_throttle_demand(). Used to compare throttle direction between a
// "deficit" and "surplus" scenario once the demand-shaping filters have
// had time to settle.
float run_to_steady_throttle(Tecs& tecs, float height_m, float airspeed_ms, float climb_rate_ms,
                              std::int32_t hgt_dem_cm, std::int32_t eas_dem_cm, int ticks) {
    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.relative_position_d_home_m = -height_m;
    in.velocity_down_ms = -climb_rate_ms;
    in.airspeed_eas = airspeed_ms;
    tecs.update_50hz(in);

    float throttle = 0.0f;
    for (int i = 0; i < ticks; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        tecs.update_50hz(in);
        // hgt_afe = height_m: tells TECS the aircraft's demand-shaping
        // filters should initialise from its ACTUAL current height, not
        // from 0 - matching how a real vehicle always passes its own
        // current height estimate here (see tecs.hpp's _initialise_states
        // notes on hgt_afe).
        tecs.update_pitch_throttle(hgt_dem_cm, eas_dem_cm, height_m, 1.0f, in);
        throttle = tecs.get_throttle_demand();
    }
    return throttle;
}

} // namespace

TEST_CASE("update_throttle_with_airspeed: more throttle when below demanded height and speed, less when above", "[tecs][throttle]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;

    Tecs deficit(gains, aparm);
    // Actual state (40m, 10 m/s) below commanded (50m, 15 m/s): TECS
    // should push throttle up to close both gaps.
    const float throttle_deficit = run_to_steady_throttle(deficit, 40.0f, 10.0f, 0.0f, 5000, 1500, 100);

    Tecs surplus(gains, aparm);
    // Actual state (60m, 17 m/s) above commanded (50m, 12 m/s): TECS
    // should pull throttle down to bleed both off.
    const float throttle_surplus = run_to_steady_throttle(surplus, 60.0f, 17.0f, 0.0f, 5000, 1200, 100);

    REQUIRE(throttle_deficit > throttle_surplus);
    // The deficit scenario should demand more than trim throttle, the
    // surplus scenario less.
    REQUIRE(throttle_deficit > aparm.throttle_cruise);
    REQUIRE(throttle_surplus < aparm.throttle_cruise);
}

// ---------------------------------------------------------------------
// _update_pitch(): sensible pitch direction - nose-up when trading
// kinetic surplus for potential deficit, nose-down for the reverse.
// ---------------------------------------------------------------------

namespace {

float run_to_steady_pitch(Tecs& tecs, float height_m, float airspeed_ms, std::int32_t hgt_dem_cm,
                           std::int32_t eas_dem_cm, int ticks) {
    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.relative_position_d_home_m = -height_m;
    in.airspeed_eas = airspeed_ms;
    tecs.update_50hz(in);

    std::int32_t pitch_cd = 0;
    for (int i = 0; i < ticks; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        tecs.update_50hz(in);
        // hgt_afe = height_m - see run_to_steady_throttle's identical note.
        tecs.update_pitch_throttle(hgt_dem_cm, eas_dem_cm, height_m, 1.0f, in);
        pitch_cd = tecs.get_pitch_demand();
    }
    return static_cast<float>(pitch_cd);
}

} // namespace

TEST_CASE("update_pitch: nose-up when height is in deficit and speed is in surplus", "[tecs][pitch]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    // Actual: 50m / 20 m/s (fast, low). Demand: 150m / 12 m/s (climb,
    // slow down) - trading kinetic energy surplus for potential energy
    // deficit should demand a nose-up (positive) pitch.
    const float pitch_cd = run_to_steady_pitch(tecs, 50.0f, 20.0f, 15000, 1200, 100);
    REQUIRE(pitch_cd > 0.0f);
}

TEST_CASE("update_pitch: nose-down when height is in surplus and speed is in deficit", "[tecs][pitch]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    // Actual: 150m / 8 m/s (slow, high). Demand: 50m / 20 m/s (descend,
    // speed up) - trading potential energy surplus for kinetic energy
    // deficit should demand a nose-down (negative) pitch.
    const float pitch_cd = run_to_steady_pitch(tecs, 150.0f, 8.0f, 5000, 2000, 100);
    REQUIRE(pitch_cd < 0.0f);
}

// ---------------------------------------------------------------------
// _detect_underspeed() / _detect_bad_descent().
// ---------------------------------------------------------------------

TEST_CASE("detect_underspeed: triggers when far below min airspeed with throttle saturated, clears once recovered",
          "[tecs][underspeed]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm; // airspeed_min = 9.0f

    Tecs tecs(gains, aparm);
    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.airspeed_eas = 4.0f; // well below airspeed_min * 0.9 = 8.1
    tecs.update_50hz(in);

    bool triggered = false;
    for (int i = 0; i < 200 && !triggered; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        tecs.update_50hz(in);
        // Demand a big climb so the throttle law saturates near max while
        // genuinely underspeed - matches upstream's own
        // "tas_state < tas_min*0.9 && throttle >= thrmaxf*0.95" condition.
        // Throttle climbs toward that threshold gradually (THR_SLEWRATE
        // limits it to 100%/s by default), hence the generous tick budget.
        tecs.update_pitch_throttle(20000, 400, 0.0f, 1.0f, in);
        triggered = tecs.underspeed();
    }
    REQUIRE(triggered);

    // Recover: feed a healthy airspeed (>1.15x min) and a height demand
    // that matches actual height, then advance the clock past the 3
    // second hysteresis - matches upstream's own clearing condition.
    in.airspeed_eas = 15.0f;
    bool cleared = false;
    for (int i = 0; i < 250 && !cleared; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(static_cast<std::int32_t>(tecs.get_height() * 100.0f), 1500, 0.0f, 1.0f, in);
        cleared = !tecs.underspeed();
    }
    REQUIRE(cleared);
}

TEST_CASE("detect_bad_descent: triggers on a large total energy deficit with throttle saturated",
          "[tecs][bad_descent]") {
    // This scenario needs its own Gains/FixedWingParams tuning to isolate
    // _detect_bad_descent()'s trigger condition (STE_error > 200, actual
    // STEdot < 0, throttle >= 0.9*max) from two OTHER, separately-tested
    // pieces of upstream anti-windup/shaping machinery that would
    // otherwise mask it in a plausible number of ticks:
    //   - _update_height_demand()'s climb-rate-limited, 3-second-lagged
    //     height demand shaping (already covered by the "converges to
    //     sensible steady demands" test) makes a large SPE error take tens
    //     of seconds of simulated time to develop with default gains -
    //     max_climb_rate/hgt_dem_tconst are pushed to extreme values here
    //     purely to make the demand track its target almost immediately,
    //     so this test isolates the ENERGY-ERROR trigger, not the demand
    //     ramp's own (separately-tested) speed.
    //   - THR_SLEWRATE's throttle-slew limiting (already covered by
    //     _update_throttle_with_airspeed()'s own direction test) means a
    //     throttle demand that's genuinely >100% pre-clamp only reaches
    //     the required 0.9*max threshold after many seconds of ramping at
    //     its default 100%/s rate; disabled here (throttle_slewrate = 0)
    //     for the same isolation reason.
    Tecs::Gains gains;
    gains.max_climb_rate = 100.0f;
    gains.hgt_dem_tconst = 0.1f;
    Tecs::FixedWingParams aparm;
    aparm.throttle_slewrate = 0.0f;
    Tecs tecs(gains, aparm);

    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    // Actual airspeed held high (21.5 m/s, comfortably above tas_min so
    // underspeed - which suppresses bad_descent detection entirely, see
    // tecs.hpp's detect_bad_descent() - never triggers) with a small
    // genuine sink rate (so actual STEdot is truly negative, not just
    // flat), while height/speed demand asks for a climb to 50m at close
    // to max airspeed: with the actual state never responding (no plant),
    // the resulting large, sustained, unachievable energy deficit is
    // exactly upstream's own "demanded airspeed too high for the aircraft
    // to achieve" scenario.
    TecsInputs in = default_inputs(now_us, now_ms);
    in.airspeed_eas = 21.5f;
    in.relative_position_d_home_m = 0.0f;
    in.velocity_down_ms = 0.3f; // small genuine sink
    tecs.update_50hz(in);

    bool triggered = false;
    for (int i = 0; i < 60 && !triggered; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(5000, 2200, 0.0f, 1.0f, in); // demand 50m / 22 m/s
        triggered = tecs.bad_descent();
    }
    REQUIRE(triggered);
}

// ---------------------------------------------------------------------
// Pitch/throttle limit clamping.
// ---------------------------------------------------------------------

TEST_CASE("update_pitch_limits: PITCH_MAX/MIN=0 sentinel falls back to aparm pitch limits", "[tecs][limits]") {
    Tecs::Gains gains;
    gains.pitch_max = 0.0f; // sentinel -> use aparm.pitch_limit_max (20)
    gains.pitch_min = 0.0f; // sentinel -> use aparm.pitch_limit_min (-25)
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    TecsInputs in = default_inputs(5'000'000, 1000);
    tecs.update_50hz(in);
    in.now_us += 20000;
    in.now_ms += 20;
    tecs.update_pitch_throttle(1000, 1200, 0.0f, 1.0f, in);

    const auto lim = tecs.pitch_limits_deg();
    REQUIRE(lim.max_deg == Catch::Approx(20.0f));
    REQUIRE(lim.min_deg == Catch::Approx(-25.0f));
}

TEST_CASE("update_pitch_limits: non-zero PITCH_MAX/MIN gains override aparm limits", "[tecs][limits]") {
    Tecs::Gains gains; // default pitch_max=15, pitch_min=0(sentinel)
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    TecsInputs in = default_inputs(5'000'000, 1000);
    tecs.update_50hz(in);
    in.now_us += 20000;
    in.now_ms += 20;
    tecs.update_pitch_throttle(1000, 1200, 0.0f, 1.0f, in);

    const auto lim = tecs.pitch_limits_deg();
    REQUIRE(lim.max_deg == Catch::Approx(15.0f)); // gains.pitch_max, not aparm's 20
    REQUIRE(lim.min_deg == Catch::Approx(-25.0f)); // pitch_min sentinel still falls back to aparm
}

TEST_CASE("update_pitch_limits: set_pitch_max/set_pitch_min externally clamp for one cycle", "[tecs][limits]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    TecsInputs in = default_inputs(5'000'000, 1000);
    tecs.update_50hz(in);
    in.now_us += 20000;
    in.now_ms += 20;

    tecs.set_pitch_max(5.0f);
    tecs.set_pitch_min(-3.0f);
    tecs.update_pitch_throttle(1000, 1200, 0.0f, 1.0f, in);

    const auto lim = tecs.pitch_limits_deg();
    REQUIRE(lim.max_deg == Catch::Approx(5.0f));
    REQUIRE(lim.min_deg == Catch::Approx(-3.0f));

    // External limits are one-shot: the next cycle without re-calling
    // set_pitch_max/min should revert to the un-overridden limits.
    in.now_us += 20000;
    in.now_ms += 20;
    tecs.update_pitch_throttle(1000, 1200, 0.0f, 1.0f, in);
    const auto lim2 = tecs.pitch_limits_deg();
    REQUIRE(lim2.max_deg == Catch::Approx(15.0f));
}

TEST_CASE("update_throttle_limits: set_throttle_min/max clamp the resolved throttle range", "[tecs][limits]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    TecsInputs in = default_inputs(5'000'000, 1000);
    tecs.update_50hz(in);
    in.now_us += 20000;
    in.now_ms += 20;

    tecs.set_throttle_min(0.0f);
    tecs.set_throttle_max(0.8f);
    tecs.update_pitch_throttle(20000, 2000, 0.0f, 1.0f, in); // huge climb+speed demand, would want full throttle

    const auto lim = tecs.throttle_limits();
    REQUIRE(lim.min == Catch::Approx(0.0f));
    REQUIRE(lim.max == Catch::Approx(0.8f));
    REQUIRE(tecs.get_throttle_demand() <= 80.0f + 1e-3f);
    REQUIRE(tecs.get_throttle_demand() >= 0.0f - 1e-3f);
}

// ---------------------------------------------------------------------
// Full update_pitch_throttle() sequence: plausibility/convergence check
// for a "climb to altitude X at airspeed Y" scenario.
//
// There is no SimPlane (or other real 6DOF plant) wired into this test -
// out of scope for this slice's tests, same as the class itself has no
// plant. Instead this uses a small, self-contained, DELIBERATELY
// SIMPLIFIED point-mass energy model (climb_rate = V*sin(pitch), V_dot =
// kThrust*(throttle-trim) - g*sin(pitch) - the standard "pitch trades
// KE/PE, throttle controls total energy" equations of motion TECS itself
// is built on, not a TECS-specific reimplementation) purely so this test
// has a closed loop to converge in. This is a plausibility check on
// TECS's own control law stabilizing to sensible steady demands, not a
// claim that the toy plant is physically accurate - matching the task's
// own "no independent oracle for a full control-loop convergence test"
// framing (see sim_plane_test.cpp's own trim-check for the port's
// precedent on this style of test).
// ---------------------------------------------------------------------

TEST_CASE("update_pitch_throttle: a full tick sequence converges to sensible steady demands climbing to a target altitude/airspeed",
          "[tecs][convergence]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    constexpr float kDt = 0.02f;
    constexpr float kThrustGain = 6.0f; // m/s^2 of forward accel per unit throttle fraction above trim
    const float throttle_trim_frac = aparm.throttle_cruise * 0.01f;

    float height = 0.0f;
    float airspeed = aparm.airspeed_cruise;
    float climb_rate = 0.0f;
    float v_dot = 0.0f;

    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.relative_position_d_home_m = -height;
    in.airspeed_eas = airspeed;
    tecs.update_50hz(in);

    constexpr std::int32_t kHgtDemCm = 5000; // climb to 50m
    constexpr std::int32_t kEasDemCm = 1500; // at 15 m/s

    constexpr int kTicks = 1500; // 30 simulated seconds
    for (int i = 0; i < kTicks; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        in.relative_position_d_home_m = -height;
        in.velocity_ned_valid = true;
        in.velocity_down_ms = -climb_rate;
        in.airspeed_eas = airspeed;
        in.accel_body_x = v_dot;

        tecs.update_50hz(in);
        tecs.update_pitch_throttle(kHgtDemCm, kEasDemCm, height, 1.0f, in); // hgt_afe tracks actual height

        REQUIRE(std::isfinite(height));
        REQUIRE(std::isfinite(airspeed));
        const float throttle_pct = tecs.get_throttle_demand();
        const std::int32_t pitch_cd = tecs.get_pitch_demand();
        REQUIRE(throttle_pct >= -100.0f - 1e-3f);
        REQUIRE(throttle_pct <= 100.0f + 1e-3f);
        REQUIRE(pitch_cd >= -9000);
        REQUIRE(pitch_cd <= 9000);

        const float pitch_rad = static_cast<float>(pitch_cd) / 5729.5781f;
        const float throttle_frac = throttle_pct * 0.01f;

        climb_rate = airspeed * std::sin(pitch_rad);
        v_dot = kThrustGain * (throttle_frac - throttle_trim_frac) - kGravityMss * std::sin(pitch_rad);
        height += climb_rate * kDt;
        airspeed = std::max(airspeed + v_dot * kDt, 1.0f);
    }

    // Plausibility, not exact-number match: the toy plant should have
    // climbed substantially toward 50m and be flying near 15 m/s, not
    // diverged or stayed pinned at the start.
    REQUIRE(height > 30.0f);
    REQUIRE(height < 70.0f);
    REQUIRE(airspeed > 12.0f);
    REQUIRE(airspeed < 18.0f);

    // And the demand-shaping filters themselves should have made real,
    // substantial progress toward the commanded setpoint (independent of
    // the toy plant's own approximations, since these are TECS's own
    // internal states) - not an exact match: update_height_demand()'s own
    // max_climb_scaler_ anti-windup deliberately throttles back how fast
    // hgt_dem can approach a large step demand while the (toy, imprecise)
    // plant is still catching up, so full convergence legitimately takes
    // longer than this test's window - see tecs.hpp's update_height_demand().
    REQUIRE(tecs.get_hgt_dem() > 35.0f);
    REQUIRE(tecs.get_tas_dem_adj() == Catch::Approx(15.0f).margin(1.0f));
}

// ---------------------------------------------------------------------
// CPP-040: flare height-rate-demand blending. See tecs.hpp's file banner
// "CPP-040 ADDENDUM" for the exact upstream arithmetic (AP_TECS.cpp:
// 611-644) these tests check, and for why the gating bool is named
// `is_flaring` (matching the real upstream gate, `_landing.is_flaring()`)
// rather than the ticket's original "is_doing_auto_land" framing.
// ---------------------------------------------------------------------

TEST_CASE("update_pitch_throttle: non-flare path is byte-identical whether the landing parameter is omitted or "
          "explicitly non-flaring (CPP-040 regression)",
          "[tecs][flare][regression]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;

    Tecs tecs_default(gains, aparm);  // pre-CPP-040 5-argument update_pitch_throttle() call form
    Tecs tecs_explicit(gains, aparm); // explicitly passes a default-constructed (non-flaring) TecsLandingInputs

    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.relative_position_d_home_m = -40.0f;
    tecs_default.update_50hz(in);
    tecs_explicit.update_50hz(in);

    const TecsLandingInputs not_flaring{}; // is_flaring=false, distance_beyond_land_wp=0.0f

    for (int i = 0; i < 300; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;

        // A varying, non-trivial input trajectory - not just two idle
        // controllers agreeing by accident.
        const float t = static_cast<float>(i);
        const float height = 40.0f + 10.0f * std::sin(t * 0.01f);
        in.relative_position_d_home_m = -height;
        in.velocity_down_ms = -0.1f * std::cos(t * 0.01f);
        in.airspeed_eas = 12.0f + 2.0f * std::sin(t * 0.02f);

        tecs_default.update_50hz(in);
        tecs_explicit.update_50hz(in);

        tecs_default.update_pitch_throttle(6000, 1400, height, 1.0f, in);
        tecs_explicit.update_pitch_throttle(6000, 1400, height, 1.0f, in, not_flaring);
    }

    // Exact equality, not Catch::Approx: both call forms must run the
    // IDENTICAL arithmetic path (see tecs.hpp's update_height_demand() -
    // the non-flare arm is upstream's pre-CPP-040 code unchanged, merely
    // wrapped in `if (!landing.is_flaring)`), so their outputs must be
    // bit-for-bit identical, not merely close.
    REQUIRE(tecs_default.get_throttle_demand() == tecs_explicit.get_throttle_demand());
    REQUIRE(tecs_default.get_pitch_demand() == tecs_explicit.get_pitch_demand());
    REQUIRE(tecs_default.get_hgt_dem() == tecs_explicit.get_hgt_dem());
    REQUIRE(tecs_default.get_height_rate_demand() == tecs_explicit.get_height_rate_demand());
    REQUIRE(tecs_default.get_tas_dem_adj() == tecs_explicit.get_tas_dem_adj());

    const auto e_default = tecs_default.energy_state();
    const auto e_explicit = tecs_explicit.energy_state();
    REQUIRE(e_default.spe_dem == e_explicit.spe_dem);
    REQUIRE(e_default.ske_dem == e_explicit.ske_dem);
    REQUIRE(e_default.spedot == e_explicit.spedot);
    REQUIRE(e_default.skedot == e_explicit.skedot);
}

TEST_CASE("update_height_demand flare: hgt_rate_dem blends smoothly from the pre-flare rate toward -LAND_SINK as "
          "height drops to FLARE_HGT",
          "[tecs][flare]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    float height = 30.0f;
    in.relative_position_d_home_m = -height;
    tecs.update_50hz(in);

    // Settle into a real, non-zero descending hgt_rate_dem_ BEFORE flare
    // entry, by commanding a height demand well below the current height.
    for (int i = 0; i < 200; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        in.relative_position_d_home_m = -height;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(0, 1200, height, 1.0f, in); // hgt_dem_cm=0: descend toward 0m
    }
    const float pre_flare_rate = tecs.get_height_rate_demand();
    REQUIRE(pre_flare_rate < 0.0f); // must actually be descending going into flare

    TecsLandingInputs landing;
    landing.is_flaring = true;

    // hgt_afe starts well above FLARE_HGT (1.0m default) and is driven
    // down through it in small (0.05m) steps - this is the "final seconds
    // before touchdown" sequence the ticket asks for. The step is kept
    // small relative to the 7m (8.0-FLARE_HGT) flare range specifically so
    // the per-tick continuity check below is a meaningful test of the
    // blend formula rather than an artifact of a coarse height step.
    float hgt_afe = 8.0f;
    float prev_rate = pre_flare_rate;
    float first_flare_rate = 0.0f;
    float last_flare_rate = 0.0f;
    for (int i = 0; i < 170; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        in.relative_position_d_home_m = -hgt_afe;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(0, 1200, hgt_afe, 1.0f, in, landing);

        const float rate = tecs.get_height_rate_demand();
        if (i == 0) {
            first_flare_rate = rate;
        }
        last_flare_rate = rate;

        // No discontinuity tick-to-tick: p and hgt_rate_dem_ are both
        // continuous (in fact piecewise-linear) functions of hgt_afe,
        // which itself only moves a small, bounded step (0.05m) each
        // tick - a real bug (e.g. a sign error or an unseeded
        // hgt_rate_dem_at_flare_entry_) would produce a jump far larger
        // than this bound, not a violation by a hair.
        REQUIRE(std::fabs(rate - prev_rate) < 0.05f);
        prev_rate = rate;

        hgt_afe = std::max(hgt_afe - 0.05f, 0.0f);
    }

    // At the instant flare begins, hgt_at_start_of_flare_ == hgt_afe (p==0
    // exactly), so the demanded rate must equal the pre-flare rate exactly
    // - no jump at flare entry.
    REQUIRE(first_flare_rate == Catch::Approx(pre_flare_rate).margin(1e-4f));

    // Once height has dropped to (and through) FLARE_HGT, p==1 and the
    // demanded rate must equal exactly -LAND_SINK (LAND_SRC is 0 here, so
    // land_sink_rate_adj == gains.land_sink).
    REQUIRE(last_flare_rate == Catch::Approx(-gains.land_sink).margin(1e-4f));
}

TEST_CASE("update_height_demand flare: LAND_SRC increases the demanded sink rate as distance beyond the land wp "
          "grows",
          "[tecs][flare]") {
    Tecs::Gains gains;
    gains.land_sink_rate_change = 0.2f; // LAND_SRC
    Tecs::FixedWingParams aparm;

    // Runs a fresh Tecs through a full flare-entry-to-touchdown sequence at
    // a fixed `distance_beyond_land_wp`, returning the demanded height rate
    // once fully settled into flare (p==1).
    auto settled_flare_rate = [&](float distance_beyond_land_wp) {
        Tecs tecs(gains, aparm);
        std::uint64_t now_us = 5'000'000;
        std::uint32_t now_ms = 1000;
        TecsInputs in = default_inputs(now_us, now_ms);
        float hgt_afe = 5.0f;
        in.relative_position_d_home_m = -hgt_afe;
        tecs.update_50hz(in);

        TecsLandingInputs landing;
        landing.is_flaring = true;
        landing.distance_beyond_land_wp = distance_beyond_land_wp;

        for (int i = 0; i < 60; ++i) {
            now_us += 20000;
            now_ms += 20;
            in.now_us = now_us;
            in.now_ms = now_ms;
            in.relative_position_d_home_m = -hgt_afe;
            tecs.update_50hz(in);
            tecs.update_pitch_throttle(0, 1200, hgt_afe, 1.0f, in, landing);
            hgt_afe = std::max(hgt_afe - 0.2f, 0.0f);
        }
        return tecs.get_height_rate_demand();
    };

    const float rate_at_wp = settled_flare_rate(0.0f);
    const float rate_beyond_wp = settled_flare_rate(10.0f);

    // Hand-computed at full flare (p==1): hgt_rate_dem == -(LAND_SINK +
    // LAND_SRC*distance_beyond_land_wp).
    REQUIRE(rate_at_wp == Catch::Approx(-gains.land_sink).margin(1e-4f));
    REQUIRE(rate_beyond_wp ==
            Catch::Approx(-(gains.land_sink + gains.land_sink_rate_change * 10.0f)).margin(1e-4f));

    // Upstream's own LAND_SRC doc-comment: "A positive value will force
    // the plane to land sooner proportional to distance passed land
    // point" - a larger distance_beyond_land_wp (with LAND_SRC>0) must
    // produce a MORE NEGATIVE (faster) demanded sink rate.
    REQUIRE(rate_beyond_wp < rate_at_wp);
}

TEST_CASE("fresh TECS demand filters initialize at current height independent of boot time", "[tecs][initialization]") {
    for (const auto start_us : {20'000ULL, 100'000ULL, 180'000ULL, 200'000ULL,
                                220'000ULL, 400'000ULL, 1'400'000ULL}) {
        CAPTURE(start_us);
        Tecs tecs(Tecs::Gains{}, Tecs::FixedWingParams{});
        auto in = default_inputs(start_us, static_cast<std::uint32_t>(start_us / 1000));
        in.relative_position_d_home_m = -500.0f;
        in.baro_altitude_m = 500.0f;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(50'000, 1200, 500.0f, 1.0f, in);
        REQUIRE(tecs.get_hgt_dem() == Catch::Approx(500.0f).margin(.01f));
        for (int tick = 0; tick < 10; ++tick) {
            in.now_us += 20'000;
            in.now_ms += 20;
            tecs.update_50hz(in);
            tecs.update_pitch_throttle(50'000, 1200, 500.0f, 1.0f, in);
            REQUIRE(tecs.get_hgt_dem() == Catch::Approx(500.0f).margin(.01f));
        }
        // Explicit reset and stale-call reset must retain their existing behavior.
        tecs.reset();
        in.relative_position_d_home_m = -600.0f;
        in.baro_altitude_m = 600.0f;
        in.now_us += 20'000;
        in.now_ms += 20;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(60'000, 1200, 600.0f, 1.0f, in);
        REQUIRE(tecs.get_hgt_dem() == Catch::Approx(600.0f).margin(.01f));
        in.relative_position_d_home_m = -700.0f;
        in.baro_altitude_m = 700.0f;
        in.now_us += 2'000'000;
        in.now_ms += 2000;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(70'000, 1200, 700.0f, 1.0f, in);
        REQUIRE(tecs.get_hgt_dem() == Catch::Approx(700.0f).margin(.01f));
    }
}
