#pragma once

// Port of libraries/SITL/SIM_Gimbal.h/.cpp. Joint-limit rate conversion,
// dcm integration, delta angle/velocity. Uses Aircraft gyro/accel (no
// AP::ins singleton). ADR-0012: explicit now_us.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>

namespace fwcpp::sim {

class SimGimbal {
public:
    void set_demanded_rates(const math::Vector3f& rates) { demanded_angular_rate = rates; }

    void get_deltas(math::Vector3f& delta_angle_out, math::Vector3f& delta_velocity_out, std::uint32_t& delta_time_us,
                    std::uint32_t now_us) {
        delta_angle_out = delta_angle;
        delta_velocity_out = delta_velocity;
        delta_time_us = now_us - delta_start_us;
        delta_angle.zero();
        delta_velocity.zero();
        delta_start_us = now_us;
    }

    void get_joint_angles(math::Vector3f& angles) const { angles = joint_angles; }
    void get_dcm(math::Matrix3f& out) const { out = dcm; }
    void set_joint_limits(const math::Vector3f& lower_limits, const math::Vector3f& upper_limits) {
        lower_joint_limits = lower_limits;
        upper_joint_limits = upper_limits;
    }
    void set_dcm(const math::Matrix3f& m) {
        dcm = m;
        init_done = true;
    }

    void update(const Aircraft& aircraft, std::uint32_t now_us) {
        const float delta_t = (now_us - last_update_us) * 1.0e-6f;
        last_update_us = now_us;
        const math::Matrix3f& vehicle_dcm = aircraft.get_dcm();
        if (!init_done) {
            dcm = vehicle_dcm;
            init_done = true;
        }
        const math::Vector3f& vehicle_gyro = aircraft.get_gyro();
        const math::Vector3f& vehicle_accel_body = aircraft.accel_body;
        const math::Vector3f demRateRaw = demanded_angular_rate;
        const math::Vector3f copterAngRate_G = dcm.transposed() * vehicle_dcm * vehicle_gyro;
        math::Vector3f relativeGimbalRate = demanded_angular_rate - copterAngRate_G;
        math::Matrix3f rotmat_copter_gimbal = dcm.transposed() * vehicle_dcm;
        joint_angles = rotmat_copter_gimbal.transposed().to_euler312();
        const math::Vector3f upperRatelimit = -(joint_angles - upper_joint_limits) * travelLimitGain;
        const math::Vector3f lowerRatelimit = -(joint_angles - lower_joint_limits) * travelLimitGain;
        const float rollAngle = joint_angles.x;
        const float elevAngle = joint_angles.y;
        math::Matrix3f matrix(math::Vector3f(std::cos(elevAngle), 0.0f, std::sin(elevAngle)),
                              math::Vector3f(std::sin(elevAngle) * std::tan(rollAngle), 1.0f,
                                             -std::cos(elevAngle) * std::tan(rollAngle)),
                              math::Vector3f(-std::sin(elevAngle) / std::cos(rollAngle), 0.0f,
                                             std::cos(elevAngle) / std::cos(rollAngle)));
        math::Vector3f gimbalJointRates = matrix * relativeGimbalRate;
        gimbalJointRates.x = math::constrain_value(gimbalJointRates.x, lowerRatelimit.x, upperRatelimit.x);
        gimbalJointRates.y = math::constrain_value(gimbalJointRates.y, lowerRatelimit.y, upperRatelimit.y);
        gimbalJointRates.z = math::constrain_value(gimbalJointRates.z, lowerRatelimit.z, upperRatelimit.z);
        matrix = math::Matrix3f(math::Vector3f(std::cos(elevAngle), 0.0f, -std::cos(rollAngle) * std::sin(elevAngle)),
                                math::Vector3f(0.0f, 1.0f, std::sin(rollAngle)),
                                math::Vector3f(std::sin(elevAngle), 0.0f, std::cos(elevAngle) * std::cos(rollAngle)));
        relativeGimbalRate = matrix * gimbalJointRates;
        gimbal_angular_rate = demRateRaw;
        dcm.rotate(gimbal_angular_rate * delta_t);
        dcm.normalize();
        rotmat_copter_gimbal = dcm.transposed() * vehicle_dcm;
        joint_angles = rotmat_copter_gimbal.transposed().to_euler312();
        gyro = gimbal_angular_rate + true_gyro_bias;
        delta_angle += gyro * delta_t;
        const math::Vector3f copter_accel_earth = vehicle_dcm * vehicle_accel_body;
        const math::Vector3f accel = dcm.transposed() * copter_accel_earth;
        delta_velocity += accel * delta_t;
    }

private:
    math::Matrix3f dcm{};
    bool init_done{false};
    std::uint32_t last_update_us{0};
    math::Vector3f gimbal_angular_rate{};
    math::Vector3f gyro{};
    math::Vector3f joint_angles{};
    math::Vector3f lower_joint_limits{math::radians(-40.0f), math::radians(-135.0f), math::radians(-7.5f)};
    math::Vector3f upper_joint_limits{math::radians(40.0f), math::radians(45.0f), math::radians(7.5f)};
    const float travelLimitGain{20.0f};
    math::Vector3f true_gyro_bias{};
    std::uint32_t delta_start_us{0};
    math::Vector3f delta_angle{};
    math::Vector3f delta_velocity{};
    math::Vector3f demanded_angular_rate{};
};

}  // namespace fwcpp::sim
