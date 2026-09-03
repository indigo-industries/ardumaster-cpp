#pragma once

// Port of libraries/AP_Declination/AP_Declination.cpp get_mag_field_ef.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_declination_tables.hpp>

namespace fwcpp::sim {

inline bool get_mag_field_ef(float latitude_deg, float longitude_deg, float& intensity_gauss,
                             float& declination_deg, float& inclination_deg) {
    using namespace declination;
    bool valid_input_data = true;
    float min_lat = static_cast<float>(static_cast<std::int32_t>(std::floor(latitude_deg / SAMPLING_RES) * SAMPLING_RES));
    float min_lon = static_cast<float>(static_cast<std::int32_t>(std::floor(longitude_deg / SAMPLING_RES) * SAMPLING_RES));

    if (latitude_deg <= SAMPLING_MIN_LAT) {
        min_lat = SAMPLING_MIN_LAT;
        valid_input_data = false;
    }
    if (latitude_deg >= SAMPLING_MAX_LAT) {
        min_lat = static_cast<float>(static_cast<std::int32_t>(latitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }
    if (longitude_deg <= SAMPLING_MIN_LON) {
        min_lon = SAMPLING_MIN_LON;
        valid_input_data = false;
    }
    if (longitude_deg >= SAMPLING_MAX_LON) {
        min_lon = static_cast<float>(static_cast<std::int32_t>(longitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }

    const std::uint32_t min_lat_index = static_cast<std::uint32_t>(
        math::constrain_value(static_cast<std::int32_t>((-(SAMPLING_MIN_LAT) + min_lat) / SAMPLING_RES), 0,
                              LAT_TABLE_SIZE - 2));
    const std::uint32_t min_lon_index = static_cast<std::uint32_t>(
        math::constrain_value(static_cast<std::int32_t>((-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES), 0,
                              LON_TABLE_SIZE - 2));

    auto bilinear = [&](const float table[LAT_TABLE_SIZE][LON_TABLE_SIZE]) {
        const float data_sw = table[min_lat_index][min_lon_index];
        const float data_se = table[min_lat_index][min_lon_index + 1];
        const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
        const float data_nw = table[min_lat_index + 1][min_lon_index];
        const float data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
        const float data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;
        return ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;
    };

    intensity_gauss = bilinear(intensity_table);
    declination_deg = bilinear(declination_table);
    inclination_deg = bilinear(inclination_table);
    return valid_input_data;
}

inline math::Vector3f earth_field_ga(float lat_deg, float lng_deg) {
    float declination_deg = 0.0f;
    float inclination_deg = 0.0f;
    float intensity_gauss = 0.0f;
    get_mag_field_ef(lat_deg, lng_deg, intensity_gauss, declination_deg, inclination_deg);
    math::Vector3f mag_ef(intensity_gauss, 0.0f, 0.0f);
    math::Matrix3f R;
    R.from_euler(0.0f, -math::radians(inclination_deg), math::radians(declination_deg));
    return R * mag_ef;
}

}  // namespace fwcpp::sim
