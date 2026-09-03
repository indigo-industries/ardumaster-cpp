#pragma once

// Port of libraries/SITL/SIM_SingleCopter.h/.cpp.

#include <cstring>

#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_motor.hpp>

namespace fwcpp::sim {

class SimSingleCopter : public Aircraft {
public:
    enum FrameType : std::uint8_t { kSingle = 0, kCoax = 1 };

    explicit SimSingleCopter(const char* frame_str = "singlecopter") {
        mass = 2.0f;
        frame_type_ = (std::strstr(frame_str, "coax") != nullptr) ? kCoax : kSingle;
        thrust_scale_ = (mass * kGravityMss) / hover_throttle_;
        frame_height = 0.1f;
        ground_behavior = GroundBehavior::kNoMovement;
    }

    void update(const SitlInput& input, float dt) {
        update_wind(input);
        float actuator[4];
        for (std::uint8_t i = 0; i < 4; ++i) {
            actuator[i] = math::constrain_value((input.servos[i] - 1500) / 500.0f, -1.0f, 1.0f);
        }
        float thrust;
        float yaw_thrust;
        float roll_thrust;
        float pitch_thrust;
        if (frame_type_ == kSingle) {
            thrust = math::constrain_value((input.servos[4] - 1000) / 1000.0f, 0.0f, 1.0f);
            yaw_thrust = -(actuator[0] + actuator[1] + actuator[2] + actuator[3]) * 0.25f * thrust +
                         thrust * rotor_rot_accel_;
            roll_thrust = (actuator[0] - actuator[2]) * 0.5f * thrust;
            pitch_thrust = (actuator[1] - actuator[3]) * 0.5f * thrust;
        } else {
            const float motor1 = math::constrain_value((input.servos[4] - 1000) / 1000.0f, 0.0f, 1.0f);
            const float motor2 = math::constrain_value((input.servos[5] - 1000) / 1000.0f, 0.0f, 1.0f);
            thrust = 0.5f * (motor1 + motor2);
            yaw_thrust = -(actuator[0] + actuator[1] + actuator[2] + actuator[3]) * 0.25f * thrust +
                         (motor2 - motor1) * rotor_rot_accel_;
            roll_thrust = (actuator[0] - actuator[2]) * 0.5f * thrust;
            pitch_thrust = (actuator[1] - actuator[3]) * 0.5f * thrust;
        }
        math::Vector3f rot_accel(roll_thrust * roll_rate_max_, pitch_thrust * pitch_rate_max_,
                                 yaw_thrust * yaw_rate_max_);
        rot_accel.x -= gyro.x * math::radians(5000.0f) / terminal_rotation_rate_;
        rot_accel.y -= gyro.y * math::radians(5000.0f) / terminal_rotation_rate_;
        rot_accel.z -= gyro.z * math::radians(400.0f) / terminal_rotation_rate_;
        const math::Vector3f air_resistance = velocity_air_ef * -(kGravityMss / terminal_velocity_) / eas2tas;
        thrust *= thrust_scale_;
        accel_body = math::Vector3f(0.0f, 0.0f, -thrust / mass);
        accel_body += dcm.transposed() * air_resistance;
        update_dynamics(rot_accel, dt);
        time_advance(dt);
        update_position();
        update_mag_field_bf();
    }

private:
    FrameType frame_type_{kSingle};
    float terminal_rotation_rate_{4.0f * math::radians(360.0f)};
    float hover_throttle_{0.65f};
    float terminal_velocity_{40.0f};
    float rotor_rot_accel_{math::radians(20.0f) * kMotorsYawFactorCw};
    float roll_rate_max_{math::radians(700.0f)};
    float pitch_rate_max_{math::radians(700.0f)};
    float yaw_rate_max_{math::radians(700.0f)};
    float thrust_scale_{0.0f};
};

}  // namespace fwcpp::sim
