#pragma once

// Port of libraries/SITL/SIM_Baro.cpp + AP_Baro_SITL: the barometer SENSOR
// model -- pressure/temperature from Aircraft altitude, plus the error a real
// barometer actually exhibits.
//
// WHY THE ERROR LIVES HERE AND NOT IN ap-baro: upstream keeps sensor error in
// the SITL backend/driver, never in AP_Baro itself. AP_Baro consumes readings
// and does arithmetic on them; it has no business inventing noise. Keeping
// that split means ap-baro's tests assert its FORMULAS against clean inputs,
// and this file's tests assert the ERROR MODEL -- neither hides the other.
//
// ERROR IS APPLIED IN ALTITUDE, THEN CONVERTED, which is upstream's own order
// (AP_Baro_SITL::_timer() perturbs sim_alt and only then calls
// get_pressure_temperature_for_alt_amsl). It matters: pressure is non-linear
// in altitude, so N metres of error near the ground is a different number of
// pascals than N metres at altitude. Perturbing the pressure directly would
// quietly give the sensor an altitude-dependent error characteristic that no
// real barometer has.

#include <cstdint>
#include <random>
#include <vector>

#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>

namespace fwcpp::sim {

struct SitlBaroSample {
    float pressure_pa{101325.0f};
    float temperature_k{288.15f};
    float altitude_amsl_m{0.0f};
};

/** Ideal, noiseless reading straight off the aircraft. Kept as-is: it is the
 *  right thing for tests that want to assert something other than sensor
 *  error, and it is what SitlBaro perturbs. */
inline SitlBaroSample sitl_baro_from_aircraft(const Aircraft& aircraft) {
    SitlBaroSample s;
    s.altitude_amsl_m = aircraft.location.alt * 0.01f;
    get_pressure_temperature_for_alt_amsl(s.altitude_amsl_m, s.pressure_pa, s.temperature_k);
    return s;
}

/** Barometer sensor model. Upstream parameter names in brackets.
 *
 *  DEFAULTS ARE ZERO ERROR, deliberately, which is NOT upstream's default
 *  (SIM_BARO_RND is 0.2 m there). Every existing caller keeps the exact
 *  readings -- and the exact regression baselines -- it had before this class
 *  existed, and realism is opted into rather than switched on underneath
 *  people. apply_upstream_defaults() is the one-liner for upstream's own
 *  numbers. */
class SitlBaro {
public:
    explicit SitlBaro(std::uint32_t rng_seed = 20260827U) : rng_(rng_seed) {}

    float noise_m = 0.0f;          // [SIM_BARO_RND]     gaussian, 1-sigma metres
    float drift_mps = 0.0f;        // [SIM_BARO_DRIFT]   metres per second, accumulating
    float glitch_m = 0.0f;         // [SIM_BARO_GLITCH]  constant offset while set
    float bias_m = 0.0f;           // [SIM_BARO_ALT_ERR] fixed calibration error
    std::uint32_t delay_ms = 0;    // [SIM_BARO_DELAY]   transport lag
    bool disabled = false;         // [SIM_BARO_DISABLE] sensor produces nothing

    /** Upstream's own SITL defaults, for a caller that wants realism rather
     *  than reproducibility. Only noise is non-zero there. */
    void apply_upstream_defaults() {
        noise_m = 0.2f;
        drift_mps = 0.0f;
        glitch_m = 0.0f;
        bias_m = 0.0f;
        delay_ms = 0;
        disabled = false;
    }

    void reset() {
        history_.clear();
        drift_accum_m_ = 0.0f;
        last_ms_ = 0;
        seeded_ = false;
        have_output_ = false;
    }

    /** True when the model produced a reading this call. False only while
     *  `disabled`, or while a configured delay has not yet been filled --
     *  the caller must not feed plane.baro on a false return, exactly as a
     *  real driver reports no new sample rather than a stale one. */
    [[nodiscard]] bool update(const Aircraft& aircraft, std::uint32_t now_ms, SitlBaroSample& out) {
        if (disabled) {
            return false;
        }

        // Drift accumulates in wall time, so it is independent of call rate --
        // a caller polling at 100 Hz and one at 10 Hz must see the same drift
        // after the same elapsed time, or the "error" would really be a
        // scheduling artifact.
        if (!seeded_) {
            last_ms_ = now_ms;
            seeded_ = true;
        }
        const float dt_s = static_cast<float>(now_ms - last_ms_) * 0.001f;
        last_ms_ = now_ms;
        drift_accum_m_ += drift_mps * dt_s;

        float alt = aircraft.location.alt * 0.01f;
        alt += drift_accum_m_;
        alt += glitch_m;
        alt += bias_m;
        if (noise_m > 0.0f) {
            alt += noise_m * static_cast<float>(normal_(rng_));
        }

        SitlBaroSample s;
        s.altitude_amsl_m = alt;
        get_pressure_temperature_for_alt_amsl(alt, s.pressure_pa, s.temperature_k);

        if (delay_ms == 0) {
            out = s;
            have_output_ = true;
            return true;
        }

        // Transport lag: hand back the oldest sample that is now at least
        // delay_ms old. Stored rather than interpolated because a real driver
        // delivers whole samples late, it does not synthesise intermediate
        // ones.
        history_.push_back({now_ms, s});
        bool produced = false;
        while (!history_.empty() && (now_ms - history_.front().first) >= delay_ms) {
            out = history_.front().second;
            history_.erase(history_.begin());
            produced = true;
            have_output_ = true;
        }
        return produced;
    }

    [[nodiscard]] float drift_accumulated_m() const { return drift_accum_m_; }
    [[nodiscard]] bool have_output() const { return have_output_; }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
    std::vector<std::pair<std::uint32_t, SitlBaroSample>> history_;
    float drift_accum_m_ = 0.0f;
    std::uint32_t last_ms_ = 0;
    bool seeded_ = false;
    bool have_output_ = false;
};

}  // namespace fwcpp::sim
