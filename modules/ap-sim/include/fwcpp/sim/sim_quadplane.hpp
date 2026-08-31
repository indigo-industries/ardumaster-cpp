#pragma once

// Port of libraries/SITL/SIM_QuadPlane.h/.cpp. Plane aero + Frame motors,
// motor_offset, tilt/firefly/tailsitter variants.

#include <cstring>

#include <fwcpp/math/rotations.hpp>
#include <fwcpp/sim/sim_frame.hpp>
#include <fwcpp/sim/sim_plane.hpp>

namespace fwcpp::sim {

class SimQuadPlane : public SimPlane {
public:
    explicit SimQuadPlane(const char* frame_str = "quadplane") : SimPlane() {
        const char* frame_type = "x";
        std::uint8_t motor_offset = 4;
        ground_behavior = GroundBehavior::kNoMovement;

        if (std::strstr(frame_str, "-octa-quad-cor") != nullptr) {
            frame_type = "octa-quad-cor";
        } else if (std::strstr(frame_str, "-octa-quad-cw-cor") != nullptr) {
            frame_type = "octa-quad-cw-cor";
        } else if (std::strstr(frame_str, "-octa-quad") != nullptr || std::strstr(frame_str, "-octaquad") != nullptr) {
            frame_type = "octa-quad";
        } else if (std::strstr(frame_str, "-octa") != nullptr) {
            frame_type = "octa";
        } else if (std::strstr(frame_str, "-hexax") != nullptr) {
            frame_type = "hexax";
        } else if (std::strstr(frame_str, "-hexa") != nullptr) {
            frame_type = "hexa";
        } else if (std::strstr(frame_str, "-plus") != nullptr) {
            frame_type = "+";
        } else if (std::strstr(frame_str, "-y6") != nullptr) {
            frame_type = "y6";
        } else if (std::strstr(frame_str, "-tri") != nullptr) {
            frame_type = "tri";
        } else if (std::strstr(frame_str, "-tilttrivec") != nullptr) {
            frame_type = "tilttrivec";
            thrust_scale_zero_ = true;
        } else if (std::strstr(frame_str, "-tilthvec") != nullptr) {
            frame_type = "tilthvec";
        } else if (std::strstr(frame_str, "-tilttri") != nullptr) {
            frame_type = "tilttri";
            thrust_scale_zero_ = true;
        } else if (std::strstr(frame_str, "firefly") != nullptr) {
            frame_type = "firefly";
            frame_config.mix = AirframeMix::kElevons;
            thrust_scale_zero_ = true;
            motor_offset = 2;
        } else if (std::strstr(frame_str, "-tilt") != nullptr) {
            frame_type = "tilt";
            thrust_scale_zero_ = true;
        } else if (std::strstr(frame_str, "cl84") != nullptr) {
            frame_type = "tilttri";
            thrust_scale_zero_ = true;
        } else if (std::strstr(frame_str, "-copter_tailsitter") != nullptr) {
            frame_type = "+";
            copter_tailsitter_ = true;
            ground_behavior = GroundBehavior::kTailsitter;
        }

        frame_ = Frame::create_frame(frame_type);
        if (!frame_.valid()) {
            frame_ = Frame::create_frame("x");
        }
        if (std::strstr(frame_str, "cl84") != nullptr) {
            frame_.motors[0].servo_type = Motor::kServoRetract;
            frame_.motors[0].servo_rate = 7.0f * 60.0f / 90.0f;
            frame_.motors[1].servo_type = Motor::kServoRetract;
            frame_.motors[1].servo_rate = 7.0f * 60.0f / 90.0f;
        }
        frame_.motor_offset = motor_offset;
        frame_.set_mass_scale(1.5f);
        frame_.set_sitl(&sitl_params);
        frame_.rpm_out = rpm;
        frame_.init(frame_str);
        mass = frame_.get_mass();
        battery.setup(frame_.get_model_batt_capacity_ah(), frame_.get_model_batt_resistance_ohm(),
                      frame_.get_model_batt_max_voltage(), 25.0f);
        battery_voltage = battery.get_voltage();
        frame_.set_battery_voltage(battery_voltage);
    }

    [[nodiscard]] Frame& frame() { return frame_; }
    [[nodiscard]] const Frame& frame() const { return frame_; }

    // VCP-012: test-only accessor exposing the tailsitter frame-string flag
    // (real upstream `copter_tailsitter`, SIM_QuadPlane.cpp line 82) so tests
    // can assert the frame-string parser alone, independent of flight dynamics.
    [[nodiscard]] bool copter_tailsitter() const { return copter_tailsitter_; }

    void update(const SitlInput& input, float dt) {
        mass = frame_.get_mass();
        update_wind_from_input(input);

        math::Vector3f rot_accel;
        const float aileron = servo_norm(input.servos[0]);
        const float elevator = servo_norm(input.servos[1]);
        float throttle = (input.servos[2] - 1000) / 1000.0f;
        throttle = math::constrain_value(throttle, 0.0f, 1.0f);
        if (thrust_scale_zero_) {
            throttle = 0.0f;
        }
        const float rudder = servo_norm(input.servos[3]);
        const SurfaceDeflections mixed = mix_surfaces(aileron, elevator, rudder, throttle);
        angle_of_attack = std::atan2(velocity_air_bf.z, velocity_air_bf.x);
        beta = std::atan2(velocity_air_bf.y, velocity_air_bf.x);
        const math::Vector3f force =
            getForce(mixed.aileron, mixed.elevator, mixed.rudder, angle_of_attack, beta, airspeed, gyro, air_density);
        rot_accel = getTorque(mixed.aileron, mixed.elevator, mixed.rudder, mixed.throttle, force, angle_of_attack,
                              airspeed, beta, gyro, air_density);
        const float thrust_scale = (mass * kGravityMss) / hover_throttle;
        const float thrust_newtons = mixed.throttle * thrust_scale;
        accel_body = math::Vector3f(thrust_newtons, 0.0f, 0.0f) + force;
        accel_body = accel_body / mass;

        math::Vector3f quad_rot;
        math::Vector3f quad_accel;
        const float alt_amsl = location.alt * 0.01f;
        frame_.set_battery_voltage(battery_voltage);
        frame_.calculate_forces(dcm, velocity_air_ef, gyro, alt_amsl, input, quad_rot, quad_accel, gross_mass(), true,
                                time_now_us);
        if (copter_tailsitter_) {
            // ROTATION_PITCH_270 on quad forces
            math::Matrix3f r;
            r.from_euler(0.0f, math::radians(270.0f), 0.0f);
            quad_rot = r * quad_rot;
            quad_accel = r * quad_accel;
        }

        if (frame_.battery_changed()) {
            battery.setup(frame_.get_model_batt_capacity_ah(), frame_.get_model_batt_resistance_ohm(),
                          frame_.get_model_batt_max_voltage(), 25.0f);
        }
        battery.maybe_reset(sitl_params.batt_voltage, sitl_params.batt_capacity_ah, sitl_params.batt_resistance);
        battery_voltage = battery.get_voltage();
        battery_current = frame_.get_current_amp();
        battery_temperature_degC = battery.get_temperature_degC();
        battery.consume_energy(battery_current, time_now_us);
        battery_current += 20.0f * std::fabs(throttle);
        frame_.set_battery_voltage(battery_voltage);

        rot_accel += quad_rot;
        accel_body += quad_accel;
        update_dynamics(rot_accel, dt);
        time_advance(dt);
        update_position();
        update_mag_field_bf();
    }

    using SimPlane::update;

private:
    static float servo_norm(std::uint16_t pwm) {
        return math::constrain_value((static_cast<float>(pwm) - 1500.0f) / 500.0f, -1.0f, 1.0f);
    }

    void update_wind_from_input(const SitlInput& input) {
        wind_config.speed = input.wind.speed;
        wind_config.direction = input.wind.direction;
        wind_config.turbulence = input.wind.turbulence;
        wind_config.dir_z = input.wind.dir_z;
        Aircraft::update_wind(input);
    }

    Frame frame_{};
    bool thrust_scale_zero_{false};
    bool copter_tailsitter_{false};
};

}  // namespace fwcpp::sim
