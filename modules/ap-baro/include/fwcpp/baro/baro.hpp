#pragma once

// Port of libraries/AP_Baro/AP_Baro.cpp -- the barometer subsystem.
//
// Replaces this port's long-standing "BAROMETRIC ALTITUDE SUBSTITUTION"
// shortcut for the one thing that genuinely needed a barometer: EAS2TAS.
// Before this, ahrs_atmosphere.hpp derived the factor from the vehicle's TRUE
// altitude through a standard-day model. That is not what a barometer does --
// a barometer measures PRESSURE, and altitude is inferred from it against a
// ground reference latched at calibration. The distinction matters because it
// is where real-world error enters: a mis-set or drifting ground reference
// biases altitude and hence every airspeed demand, and a standard-day model
// cannot express that at all.
//
// WHAT IS MODELLED: pressure/temperature input, ground-reference calibration,
// pressure-derived altitude, and EAS2TAS from measured pressure with a
// lapse-rate-extrapolated temperature -- all following upstream's own
// formulas, cited per method below.
//
// WHAT IS NOT: no per-sensor noise, bias, drift or lag model, and no
// multi-instance/health-voting. The caller supplies one clean sample. Adding a
// noise/drift model is the natural next slice and belongs in update()'s
// caller (the SITL backend), not here -- upstream keeps sensor error in the
// AP_Baro_Backend drivers, not in AP_Baro itself.

#include <cmath>
#include <cstdint>

namespace fwcpp::baro {

// Upstream: AP_Baro.h
inline constexpr float kSslAirDensity = 1.225f;      // SSL_AIR_DENSITY
inline constexpr float kIsaGasConstant = 287.26f;    // ISA_GAS_CONSTANT
inline constexpr float kIsaLapseRate = 0.0065f;      // ISA_LAPSE_RATE, K/m
inline constexpr float kCtoKelvin = 273.15f;         // C_TO_KELVIN

class Baro {
public:
    /** Feed one sample. Upstream this is what the AP_Baro_Backend drivers
     *  push in; here the SITL harness supplies it from the plant's
     *  atmosphere. `now_ms` only drives healthy(). */
    void update(float pressure_pa, float temperature_c, std::uint32_t now_ms) {
        if (pressure_pa > 0.0f) {
            pressure_pa_ = pressure_pa;
            temperature_c_ = temperature_c;
            last_update_ms_ = now_ms;
            healthy_ = true;
        }
        // First good sample doubles as the ground reference until an explicit
        // calibrate(). Upstream calibrates during init before arming; a vehicle
        // that never calls it should still read sane altitudes rather than
        // divide by a zero reference.
        if (!calibrated_ && healthy_) {
            calibrate();
        }
    }

    /** Latch the current reading as the ground reference.
     *  Upstream: AP_Baro::calibrate(). */
    void calibrate() {
        ground_pressure_pa_ = pressure_pa_;
        ground_temperature_c_ = temperature_c_;
        calibrated_ = true;
    }

    [[nodiscard]] bool healthy() const { return healthy_; }
    [[nodiscard]] bool calibrated() const { return calibrated_; }
    [[nodiscard]] float get_pressure() const { return pressure_pa_; }
    [[nodiscard]] float get_temperature() const { return temperature_c_; }
    [[nodiscard]] float get_ground_pressure() const { return ground_pressure_pa_; }
    [[nodiscard]] float get_ground_temperature() const { return ground_temperature_c_; }
    [[nodiscard]] std::uint32_t last_update_ms() const { return last_update_ms_; }

    /** Altitude difference (m) between two pressures, using the ground
     *  temperature. Upstream: AP_Baro::get_altitude_difference() -- "an exact
     *  calculation that is within +-2.5m of the standard atmosphere tables in
     *  the troposphere (up to 11,000 m amsl)". Reproduced verbatim, including
     *  the 153.8462 / 0.190259 constants. */
    [[nodiscard]] float get_altitude_difference(float base_pressure, float pressure) const {
        if (base_pressure <= 0.0f || pressure <= 0.0f) {
            return 0.0f;
        }
        const float temp_k = ground_temperature_c_ + kCtoKelvin;
        const float scaling = pressure / base_pressure;
        return 153.8462f * temp_k * (1.0f - std::exp(0.190259f * std::log(scaling)));
    }

    /** Altitude (m) above the calibration point.
     *  Upstream: AP_Baro::get_altitude(). */
    [[nodiscard]] float get_altitude() const {
        if (!calibrated_) {
            return 0.0f;
        }
        return get_altitude_difference(ground_pressure_pa_, pressure_pa_);
    }

    /** EAS -> TAS factor. Upstream: AP_Baro::get_EAS2TAS().
     *
     *  Note what upstream deliberately does NOT do here: it does not look the
     *  temperature up in a full ISA model. It extrapolates from the GROUND
     *  temperature reading down the standard lapse rate, because that "provides
     *  a more consistent reading than trying to estimate a complete ISA model
     *  atmosphere" -- the ground reading is real data about the actual day,
     *  where the model is an assumption. Reproduced. */
    [[nodiscard]] float get_eas2tas() const {
        if (pressure_pa_ <= 0.0f) {
            return 1.0f;
        }
        const float temp_k =
            (ground_temperature_c_ + kCtoKelvin) - kIsaLapseRate * get_altitude();
        if (temp_k <= 0.0f) {
            return 1.0f;
        }
        const float density = pressure_pa_ / (kIsaGasConstant * temp_k);
        if (density <= 1.0e-5f) {
            return 1.0f;
        }
        return std::sqrt(kSslAirDensity / density);
    }

private:
    float pressure_pa_ = 0.0f;
    float temperature_c_ = 15.0f;
    float ground_pressure_pa_ = 0.0f;
    float ground_temperature_c_ = 15.0f;
    std::uint32_t last_update_ms_ = 0;
    bool healthy_ = false;
    bool calibrated_ = false;
};

}  // namespace fwcpp::baro
