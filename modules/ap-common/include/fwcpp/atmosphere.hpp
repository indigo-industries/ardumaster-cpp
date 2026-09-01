#pragma once

// 1976 U.S. Standard Atmosphere -- port of libraries/AP_Baro/
// AP_Baro_atmosphere.cpp.
//
// Lives in ap-common, NOT ap-sim, because upstream this is AP_Baro: firmware
// code the vehicle uses, not simulator code. It was originally added under
// fwcpp::sim because the plant was its only consumer; once the AHRS needed to
// ESTIMATE eas2tas (rather than the harness reading the plant's own field)
// keeping it sim-side would have meant the firmware depending on the
// simulator. fwcpp/sim/sim_atmosphere.hpp now forwards here, so every existing
// fwcpp::sim:: call site is unchanged.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::atmosphere {

inline constexpr float kSslAirDensity = 1.225f;
inline constexpr float kGravityMss = 9.80665f;
inline constexpr float kRadiusEarth = 6356.766e3f;
inline constexpr float kRSpecific = 287.053072f;

struct AtmosphereLayer {
    float amsl_m;
    float temp_k;
    float pressure_pa;
    float density;
    float temp_lapse;
};

inline constexpr AtmosphereLayer kAtmospheric1976[] = {
    {-5000.0f, 320.650f, 177687.0f, 1.930467f, -6.5e-3f},
    {11000.0f, 216.650f, 22632.1f, 0.363918f, 0.0f},
    {20000.0f, 216.650f, 5474.89f, 8.80349e-2f, 1e-3f},
    {32000.0f, 228.650f, 868.019f, 1.32250e-2f, 2.8e-3f},
    {47000.0f, 270.650f, 110.906f, 1.42753e-3f, 0.0f},
    {51000.0f, 270.650f, 66.9389f, 8.61606e-4f, -2.8e-3f},
    {71000.0f, 214.650f, 3.95642f, 6.42110e-5f, -2.0e-3f},
    {84852.0f, 186.946f, 0.37338f, 6.95788e-6f, 0.0f},
};

inline constexpr std::uint8_t kAtmospheric1976Size =
    static_cast<std::uint8_t>(sizeof(kAtmospheric1976) / sizeof(kAtmospheric1976[0]));

[[nodiscard]] inline float geometric_alt_to_geopotential(float alt) {
    return (kRadiusEarth * alt) / (kRadiusEarth + alt);
}

[[nodiscard]] inline float geopotential_alt_to_geometric(float alt) {
    return (kRadiusEarth * alt) / (kRadiusEarth - alt);
}

[[nodiscard]] inline std::uint8_t find_atmosphere_layer_by_altitude(float alt_m) {
    for (std::uint8_t idx = 1; idx < kAtmospheric1976Size; ++idx) {
        if (alt_m < kAtmospheric1976[idx].amsl_m) {
            return static_cast<std::uint8_t>(idx - 1);
        }
    }
    return static_cast<std::uint8_t>(kAtmospheric1976Size - 1);
}

[[nodiscard]] inline float get_temperature_by_altitude_layer(float alt, std::uint8_t idx) {
    if (math::is_zero(kAtmospheric1976[idx].temp_lapse)) {
        return kAtmospheric1976[idx].temp_k;
    }
    return kAtmospheric1976[idx].temp_k + kAtmospheric1976[idx].temp_lapse * (alt - kAtmospheric1976[idx].amsl_m);
}

// Upstream: AP_Baro::get_air_density_for_alt_amsl
[[nodiscard]] inline float get_air_density_for_alt_amsl(float alt_amsl) {
    alt_amsl = geometric_alt_to_geopotential(alt_amsl);
    const std::uint8_t idx = find_atmosphere_layer_by_altitude(alt_amsl);
    const float temp_slope = kAtmospheric1976[idx].temp_lapse;
    const float temp = get_temperature_by_altitude_layer(alt_amsl, idx);
    if (math::is_zero(temp_slope)) {
        const float fac =
            std::exp(-kGravityMss / (temp * kRSpecific) * (alt_amsl - kAtmospheric1976[idx].amsl_m));
        return kAtmospheric1976[idx].density * fac;
    }
    const float fac = kGravityMss / (temp_slope * kRSpecific);
    const float temp_ratio = temp / kAtmospheric1976[idx].temp_k;
    return kAtmospheric1976[idx].density * std::pow(temp_ratio, -(fac + 1.0f));
}

// Upstream: AP_Baro::get_EAS2TAS_for_alt_amsl
[[nodiscard]] inline float get_eas2tas_for_alt_amsl(float alt_amsl) {
    const float density = get_air_density_for_alt_amsl(alt_amsl);
    return std::sqrt(kSslAirDensity / std::fmax(1.0e-5f, density));
}

// Upstream: AP_Baro::get_pressure_temperature_for_alt_amsl
inline void get_pressure_temperature_for_alt_amsl(float alt_amsl, float& pressure, float& temperature_k) {
    alt_amsl = geometric_alt_to_geopotential(alt_amsl);
    const std::uint8_t idx = find_atmosphere_layer_by_altitude(alt_amsl);
    const float temp_slope = kAtmospheric1976[idx].temp_lapse;
    temperature_k = get_temperature_by_altitude_layer(alt_amsl, idx);
    if (math::is_zero(temp_slope)) {
        const float fac =
            std::exp(-kGravityMss / (temperature_k * kRSpecific) * (alt_amsl - kAtmospheric1976[idx].amsl_m));
        pressure = kAtmospheric1976[idx].pressure_pa * fac;
    } else {
        const float fac = kGravityMss / (temp_slope * kRSpecific);
        const float temp_ratio = temperature_k / kAtmospheric1976[idx].temp_k;
        pressure = kAtmospheric1976[idx].pressure_pa * std::pow(temp_ratio, -fac);
    }
}

}  // namespace fwcpp::atmosphere
