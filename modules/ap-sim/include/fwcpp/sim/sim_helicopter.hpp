#pragma once

// Port of libraries/SITL/SIM_Helicopter.h/.cpp. All original frame types
// (conventional, DDVP/DDFP tail, blade360, dual, compound), rotor dynamics,
// RPM model, servo delay buffer, battery. ADR-0012: explicit dt / time_us.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>

namespace fwcpp::sim {

class SimHelicopter : public Aircraft {
public:
    enum FrameType : std::uint8_t {
        kConventional = 0,
        kConventionalDdvpTail,
        kConventionalDdfpTail,
        kDual,
        kCompound,
        kBlade360,
    };

    explicit SimHelicopter(const char* frame_str = "heli") {
        mass = 4.54f;
        if (std::strstr(frame_str, "-dual") != nullptr) {
            frame_type_ = kDual;
            time_delay_ms_ = 30;
            nominal_rpm_ = 1300.0f;
            mass = 9.08f;
            iyy_ = 5.0f;
        } else if (std::strstr(frame_str, "-compound") != nullptr) {
            frame_type_ = kCompound;
            time_delay_ms_ = 50;
            nominal_rpm_ = 1500.0f;
        } else if (std::strstr(frame_str, "-blade360") != nullptr) {
            frame_type_ = kBlade360;
            time_delay_ms_ = 40;
            nominal_rpm_ = 2100.0f;
        } else if (std::strstr(frame_str, "-ddvptail") != nullptr) {
            frame_type_ = kConventionalDdvpTail;
            time_delay_ms_ = 50;
            nominal_rpm_ = 1500.0f;
        } else if (std::strstr(frame_str, "-ddfptail") != nullptr) {
            frame_type_ = kConventionalDdfpTail;
            time_delay_ms_ = 50;
            nominal_rpm_ = 1500.0f;
        } else {
            frame_type_ = kConventional;
            time_delay_ms_ = 50;
            nominal_rpm_ = 1500.0f;
        }

        const float omega_n = nominal_rpm_ * 2.0f * static_cast<float>(M_PI) / 60.0f;
        const float omega_n2 = omega_n * omega_n;
        thrust_scale_ = (mass * kGravityMss) / (hover_coll_ * omega_n2);
        torque_mpog_ = 1.08f / omega_n2;
        torque_max_ = ((mass * kGravityMss * std::sin(math::radians(hover_lean_)) * tr_dist_ -
                        torque_mpog_ * omega_n2) *
                           std::pow(2.0f, 1.5f) +
                       torque_mpog_ * omega_n2) /
                      omega_n2;
        torque_scale_ = (torque_max_ - torque_mpog_) / std::pow(10.0f, 1.5f);
        frame_height = 0.1f;
        gas_heli_ = std::strstr(frame_str, "-gas") != nullptr;
        ground_behavior = GroundBehavior::kNoMovement;
        battery.setup(sitl_params.batt_capacity_ah, default_battery_resistance_ohm_, sitl_params.batt_voltage, 25.0f);
    }

    [[nodiscard]] FrameType frame_type() const { return frame_type_; }

    void update(const SitlInput& input, float dt) {
        update_wind(input);
        motor_interlock_ = input.servos[7] > 1400;
        const float rsc = math::constrain_value((input.servos[7] - 1000) / 1000.0f, 0.0f, 1.0f);
        power_consumption_watts_ = gas_heli_ ? 1.0f : 1000.0f;
        update_battery();

        if (time_delay_ms_ == 0) {
            for (std::uint8_t i = 0; i < 6; ++i) {
                servos_delayed_[i] = input.servos[i];
            }
        } else if (!buffer_inited_) {
            const int n = static_cast<int>(math::constrain_value(static_cast<float>(time_delay_ms_), 1.0f, 100.0f) *
                                           0.001f / std::fmax(dt, 1.0e-6f));
            buffer_size_ = static_cast<std::size_t>(std::max(n, 1));
            for (std::size_t i = 0; i < buffer_size_; ++i) {
                push_to_buffer(input);
            }
            for (std::uint8_t i = 0; i < 6; ++i) {
                servos_delayed_[i] = input.servos[i];
            }
            buffer_inited_ = true;
        } else {
            pull_from_buffer();
            push_to_buffer(input);
        }

        const float swash1 = (servos_delayed_[0] - 1000) / 1000.0f;
        const float swash2 = (servos_delayed_[1] - 1000) / 1000.0f;
        const float swash3 = (servos_delayed_[2] - 1000) / 1000.0f;
        math::Vector3f rot_accel;

        switch (frame_type_) {
        case kConventional:
            update_conventional(input, swash1, swash2, swash3, rsc, dt, rot_accel, /*tailrsc*/ 1.0f, /*ddfp*/ false);
            break;
        case kConventionalDdvpTail: {
            const float tailrsc = math::constrain_value((input.servos[6] - 1000) / 1000.0f, 0.0f, 1.0f);
            update_conventional(input, swash1, swash2, swash3, rsc, dt, rot_accel, tailrsc, false);
            break;
        }
        case kConventionalDdfpTail:
            update_conventional(input, swash1, swash2, swash3, rsc, dt, rot_accel, 1.0f, true);
            break;
        case kBlade360:
            update_blade360(swash1, swash2, swash3, rsc, dt, rot_accel);
            break;
        case kDual:
            update_dual(swash1, swash2, swash3, rsc, dt, rot_accel);
            break;
        case kCompound:
            update_compound(swash1, swash2, swash3, rsc, dt, rot_accel);
            break;
        }

        update_dynamics(rot_accel, dt);
        add_shove_forces(rot_accel, accel_body, static_cast<std::uint32_t>(time_now_us / 1000U));
        add_twist_forces(rot_accel, static_cast<std::uint32_t>(time_now_us / 1000U));
        add_external_forces(accel_body);
        update_position();
        time_advance(dt);
        update_mag_field_bf();
    }

private:
    void update_conventional(const SitlInput& input, float swash1, float swash2, float swash3, float rsc, float dt,
                             math::Vector3f& rot_accel, float tailrsc, bool ddfp) {
        constexpr float Ma1s = 617.5f;
        constexpr float Lb1s = 3588.6f;
        constexpr float Mu = 0.003f;
        constexpr float Lv = -0.006f;
        constexpr float Xu = -0.125f;
        constexpr float Yv = -0.375f;
        constexpr float Zw = -2.25f;
        float tail_rotor;
        if (ddfp) {
            tail_rotor = (input.servos[6] - 1000) / 1000.0f;
        } else {
            tail_rotor = (servos_delayed_[3] - 1000) / 1000.0f;
        }
        const float coll = 50.0f * (swash1 + swash2 + swash3) / 3.0f - 25.0f;
        float eng_torque = 0.0f;
        const float thrust = (rpm[0] / nominal_rpm_) * thrust_scale_ * sq(nominal_rpm_ * 0.104667f) * coll;
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, coll, dt);
        const float roll_cyclic = 1.283f * (swash1 - swash2) / cyclic_scalar_;
        const float pitch_cyclic = 1.48f * ((swash1 + swash2) / 2.0f - swash3) / cyclic_scalar_;
        math::Vector2f ctrl_pos(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, tpp_angle_, dt);
        float tail_rotor_torque;
        if (ddfp) {
            tail_rotor_torque = (21.6f * 2.96f - 2.96f * gyro.z) *
                                sq(tail_rotor * battery_voltage / std::fmax(sitl_params.batt_voltage, 0.1f));
        } else {
            const float yaw_cmd = 2.0f * tail_rotor - 1.0f;
            tail_rotor_torque = (21.6f * 2.96f * yaw_cmd - 2.96f * gyro.z) * tailrsc * sq(rpm[0] / nominal_rpm_);
        }
        const float tail_rotor_thrust = -1.0f * tail_rotor_torque * izz_ / tr_dist_;
        rot_accel.x = tpp_angle_.x * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = tpp_angle_.y * Ma1s + Mu * velocity_air_bf.x;
        rot_accel.z = tail_rotor_torque - eng_torque;
        const float lateral_y_thrust = tail_rotor_thrust / mass + kGravityMss * tpp_angle_.x + Yv * velocity_air_bf.y;
        const float lateral_x_thrust = -1.0f * kGravityMss * tpp_angle_.y + Xu * velocity_air_bf.x;
        accel_body = math::Vector3f(lateral_x_thrust, lateral_y_thrust, -thrust / mass + velocity_air_bf.z * Zw);
    }

    void update_blade360(float swash1, float swash2, float swash3, float rsc, float dt, math::Vector3f& rot_accel) {
        constexpr float Ma1s = 796.7f;
        constexpr float Lb1s = 5115.2f;
        constexpr float Mu = 2.7501f;
        constexpr float Mv = -2.3039f;
        constexpr float Lu = -28.7796f;
        constexpr float Lv = -5.5376f;
        constexpr float Xu = -0.2270f;
        constexpr float Yv = -0.1852f;
        constexpr float Yp = 0.2303f;
        constexpr float Zw = -0.5910f;
        constexpr float Nr = -2.0131f;
        constexpr float Nw = 5.7574f;
        constexpr float Nv = 1.7258f;
        constexpr float Ncol = -32.4616f;
        constexpr float Nped = 63.0040f;
        constexpr float Zcol = -22.3239f;
        const float tail_rotor = (servos_delayed_[3] - 1000) / 1000.0f;
        const float coll = 3.51f * ((swash1 + swash2 + swash3) / 3.0f - 0.5f);
        float eng_torque = 0.0f;
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, coll, dt);
        const float roll_cyclic = 1.283f * (swash1 - swash2);
        const float pitch_cyclic = 1.48f * ((swash1 + swash2) / 2.0f - swash3);
        math::Vector2f ctrl_pos(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, tpp_angle_, dt);
        const float yaw_cmd = 1.45f * (2.0f * tail_rotor - 1.0f);
        rot_accel.x = tpp_angle_.x * Lb1s + Lu * velocity_air_bf.x + Lv * velocity_air_bf.y;
        rot_accel.y = tpp_angle_.y * Ma1s + Mu * velocity_air_bf.x + Mv * velocity_air_bf.y;
        rot_accel.z = Nv * velocity_air_bf.y + Nr * gyro.z + sq(rpm[0] / nominal_rpm_) * Nped * yaw_cmd +
                      Nw * velocity_air_bf.z + sq(rpm[0] / nominal_rpm_) * Ncol * (coll - 0.5f);
        const float lateral_y_thrust =
            kGravityMss * tpp_angle_.x + Yv * velocity_air_bf.y + Yp * gyro.x - 3.2f * 0.01745f * kGravityMss;
        const float lateral_x_thrust = -1.0f * kGravityMss * tpp_angle_.y + Xu * velocity_air_bf.x;
        const float vertical_thrust = Zcol * coll * sq(rpm[0] / nominal_rpm_) + velocity_air_bf.z * Zw;
        accel_body = math::Vector3f(lateral_x_thrust, lateral_y_thrust, vertical_thrust);
        (void)eng_torque;
    }

    void update_dual(float swash1, float swash2, float swash3, float rsc, float dt, math::Vector3f& rot_accel) {
        constexpr float Ma1s = 617.5f / 5.0f;
        constexpr float Lb1s = 3588.6f;
        constexpr float Mu = 0.003f;
        constexpr float Lv = -0.006f;
        constexpr float Xu = -0.125f;
        constexpr float Yv = -0.375f;
        constexpr float Zw = -2.25f;
        constexpr float hub_dist = 1.8f;
        const float swash4 = (servos_delayed_[3] - 1000) / 1000.0f;
        const float swash5 = (servos_delayed_[4] - 1000) / 1000.0f;
        const float swash6 = (servos_delayed_[5] - 1000) / 1000.0f;
        const float coll_1 = 50.0f * (swash1 + swash2 + swash3) / 3.0f - 25.0f;
        const float roll_cyclic_1 = 1.283f * (swash1 - swash2) / cyclic_scalar_;
        const float pitch_cyclic_1 = 1.48f * ((swash1 + swash2) / 2.0f - swash3) / cyclic_scalar_;
        math::Vector2f ctrl_pos_1(roll_cyclic_1, pitch_cyclic_1);
        update_rotor_dynamics(gyro, ctrl_pos_1, tpp_angle_1_, dt);
        const float coll_2 = 50.0f * (swash4 + swash5 + swash6) / 3.0f - 25.0f;
        const float roll_cyclic_2 = 1.283f * (swash4 - swash5) / cyclic_scalar_;
        const float pitch_cyclic_2 = 1.48f * ((swash4 + swash5) / 2.0f - swash6) / cyclic_scalar_;
        math::Vector2f ctrl_pos_2(roll_cyclic_2, pitch_cyclic_2);
        update_rotor_dynamics(gyro, ctrl_pos_2, tpp_angle_2_, dt);
        float eng_torque = 0.0f;
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, (coll_1 + coll_2) * 0.5f, dt);
        const float thrust_1 = 0.5f * thrust_scale_ * sq(rpm[0] * 0.104667f) * coll_1;
        const float thrust_2 = 0.5f * thrust_scale_ * sq(rpm[0] * 0.104667f) * coll_2;
        rot_accel.x = (tpp_angle_1_.x + tpp_angle_2_.x) * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = (tpp_angle_1_.y + tpp_angle_2_.y) * Ma1s + (thrust_1 - thrust_2) * hub_dist / iyy_ +
                      Mu * velocity_air_bf.x;
        rot_accel.z = (tpp_angle_1_.x * thrust_1 - tpp_angle_2_.x * thrust_2) * hub_dist / (iyy_ * 2.0f) - 0.5f * gyro.z;
        const float lateral_y_thrust = kGravityMss * (tpp_angle_1_.x + tpp_angle_2_.x) + Yv * velocity_air_bf.y;
        const float lateral_x_thrust = -1.0f * kGravityMss * (tpp_angle_1_.y + tpp_angle_2_.y) + Xu * velocity_air_bf.x;
        accel_body =
            math::Vector3f(lateral_x_thrust, lateral_y_thrust, -(thrust_1 + thrust_2) / mass + velocity_air_bf.z * Zw);
        (void)eng_torque;
    }

    void update_compound(float swash1, float swash2, float swash3, float rsc, float dt, math::Vector3f& rot_accel) {
        constexpr float Ma1s = 617.5f;
        constexpr float Lb1s = 3588.6f;
        constexpr float Mu = 0.003f;
        constexpr float Lv = -0.006f;
        constexpr float Xu = -0.125f;
        constexpr float Yv = -0.375f;
        constexpr float Zw = -2.25f;
        const float coll = 50.0f * (swash1 + swash2 + swash3) / 3.0f - 25.0f;
        const float thrust = thrust_scale_ * sq(rpm[0] * 0.104667f) * coll;
        float eng_torque = 0.0f;
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, coll, dt);
        const float roll_cyclic = 1.283f * (swash1 - swash2) / cyclic_scalar_;
        const float pitch_cyclic = 1.48f * ((swash1 + swash2) / 2.0f - swash3) / cyclic_scalar_;
        math::Vector2f ctrl_pos(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, tpp_angle_, dt);
        const float right_thruster_cmd = 2.0f * (servos_delayed_[3] - 1000) / 1000.0f - 1.0f;
        const float left_thruster_cmd = 2.0f * (servos_delayed_[4] - 1000) / 1000.0f - 1.0f;
        const float right_thruster_torque =
            (-0.5f * 21.6f * 2.96f * right_thruster_cmd - 2.96f * gyro.z) * sq(rpm[0] / nominal_rpm_);
        const float left_thruster_torque =
            (0.5f * 21.6f * 2.96f * left_thruster_cmd - 2.96f * gyro.z) * sq(rpm[0] / nominal_rpm_);
        const float right_thruster_force = -1.0f * right_thruster_torque * izz_ / (0.5f * tr_dist_);
        const float left_thruster_force = left_thruster_torque * izz_ / (0.5f * tr_dist_);
        rot_accel.x = tpp_angle_.x * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = tpp_angle_.y * Ma1s + Mu * velocity_air_bf.x;
        rot_accel.z = right_thruster_torque + left_thruster_torque - eng_torque;
        const float lateral_y_thrust = kGravityMss * tpp_angle_.x + Yv * velocity_air_bf.y;
        const float lateral_x_thrust =
            (right_thruster_force + left_thruster_force) / mass - kGravityMss * tpp_angle_.y + Xu * velocity_air_bf.x;
        accel_body = math::Vector3f(lateral_x_thrust, lateral_y_thrust, -thrust / mass + velocity_air_bf.z * Zw);
    }

    void update_rotor_dynamics(const math::Vector3f& gyros, const math::Vector2f& ctrl_pos, math::Vector2f& tpp_angle,
                               float dt) {
        float tf_inv, Lfa1s, Mfb1s, Lflt, Lflg, Mflt, Mflg;
        if (frame_type_ == kBlade360) {
            tf_inv = 1.0f / 0.0353f;
            Lfa1s = 1.0477f;
            Mfb1s = -1.0057f;
            Lflt = 0.2375f;
            Lflg = -0.0286f;
            Mflt = 0.0344f;
            Mflg = 0.2292f;
        } else if (frame_type_ == kDual) {
            tf_inv = 1.0f / 0.068232f;
            Lfa1s = 0.0f;
            Mfb1s = 0.0f;
            Lflt = 1.7635f;
            Lflg = 0.0f;
            Mflt = 0.0f;
            Mflg = 1.9432f;
        } else {
            tf_inv = 1.0f / 0.068232f;
            Lfa1s = 1.2963f;
            Mfb1s = -1.3402f;
            Lflt = 1.7635f;
            Lflg = -0.61171f;
            Mflt = 0.52454f;
            Mflg = 1.9432f;
        }
        const float b1s_dot =
            -1.0f * gyros.x - tf_inv * tpp_angle.x + tf_inv * (Lfa1s * tpp_angle.y + Lflt * ctrl_pos.x + Lflg * ctrl_pos.y);
        const float a1s_dot =
            -1.0f * gyros.y - tf_inv * tpp_angle.y + tf_inv * (Mfb1s * tpp_angle.x + Mflt * ctrl_pos.x + Mflg * ctrl_pos.y);
        tpp_angle.x += b1s_dot * dt;
        tpp_angle.y += a1s_dot * dt;
    }

    float update_rpm(float curr_rpm, float throttle, float& engine_torque, float collective, float dt) {
        float accel_scale = 100.0f;
        const float rotor_torque =
            (sq(curr_rpm * 0.104667f) * (torque_mpog_ + torque_scale_ * std::pow(std::fabs(collective), 1.5f))) / izz_;
        const float auto_ss_torque =
            sq(nominal_rpm_ * 0.104667f) * (torque_mpog_ + torque_scale_ * std::pow(1.0f, 1.5f)) / izz_;
        float descent_torque = 0.0f;
        if (math::is_positive(velocity_air_bf.z)) {
            descent_torque = (velocity_air_bf.z - 5.3f) * auto_ss_torque / 5.3f + auto_ss_torque;
        }
        const float engine_torque_max = sq(nominal_rpm_ * 0.104667f) * torque_max_ / izz_;
        float rpm_dot = 0.0f;
        if (gas_heli_) {
            engine_torque = 1.20f * throttle * engine_torque_max;
            float input_torque = 0.0f;
            const float rpm_engine = nominal_rpm_ * throttle / 0.3f;
            if (throttle >= 0.15f && rpm_engine > curr_rpm) {
                input_torque = engine_torque;
            }
            if (throttle <= 0.15f && curr_rpm < 300.0f) {
                rpm_dot = -40.0f;
                if (curr_rpm <= 0.0f) {
                    rpm_dot = 0.0f;
                    curr_rpm = 0.0f;
                }
            } else {
                rpm_dot = accel_scale * (input_torque + descent_torque - rotor_torque);
                if (curr_rpm <= 0.0f && !math::is_positive(rpm_dot)) {
                    rpm_dot = 0.0f;
                    curr_rpm = 0.0f;
                }
            }
        } else {
            if (throttle > 0.25f) {
                motor_status_ = 3;
            } else if (motor_status_ == 3 && throttle <= 0.25f && throttle > 0.15f) {
                motor_status_ = 2;
            } else if (throttle <= 0.15f) {
                motor_status_ = 1;
            }
            if (battery_is_empty()) {
                motor_status_ = 1;
            }
            float runup_time = 8.0f;
            if (motor_status_ == 2) {
                runup_time = 2.0f;
            }
            const float runup_increment = dt / runup_time;
            if (motor_status_ > 2) {
                if (rotor_runup_output_ < 1.0f) {
                    rotor_runup_output_ += runup_increment;
                } else {
                    rotor_runup_output_ = 1.0f;
                }
                if (curr_rpm < nominal_rpm_ - 25.0f) {
                    accel_scale = 2000.0f / runup_time;
                }
            } else {
                if (rotor_runup_output_ > 0.0f) {
                    rotor_runup_output_ -= runup_increment * 10.0f;
                } else {
                    rotor_runup_output_ = 0.0f;
                }
            }
            engine_torque = 0.333f * rotor_runup_output_ * engine_torque_max;
            float input_torque;
            if (rotor_runup_output_ >= 1.0f && curr_rpm > nominal_rpm_ - 100.0f) {
                input_torque = rotor_torque * sq(nominal_rpm_ / curr_rpm);
            } else if (rotor_runup_output_ <= 0.0f) {
                input_torque = descent_torque;
            } else {
                input_torque = engine_torque + descent_torque;
            }
            if (rotor_runup_output_ <= 0.0f && curr_rpm < 300.0f) {
                rpm_dot = -40.0f;
                if (curr_rpm <= 0.0f) {
                    rpm_dot = 0.0f;
                    curr_rpm = 0.0f;
                }
            } else {
                rpm_dot = accel_scale * (input_torque - rotor_torque);
            }
            engine_torque = input_torque;
        }
        curr_rpm += rpm_dot * dt;
        return curr_rpm;
    }

    void update_battery() {
        battery.maybe_reset(sitl_params.batt_voltage, sitl_params.batt_capacity_ah);
        battery_voltage = battery.get_voltage();
        float power_watt = 1.0f;
        if (motor_interlock_) {
            power_watt = power_consumption_watts_;
        }
        battery_current = power_watt / std::fmax(battery_voltage, 0.1f);
        battery.consume_energy(battery_current, time_now_us);
    }

    [[nodiscard]] bool battery_is_empty() const { return battery_voltage < 0.5f; }

    static float sq(float x) { return x * x; }

    struct ServosStored {
        std::uint16_t s[6]{};
    };

    void push_to_buffer(const SitlInput& input) {
        ServosStored sample;
        for (int i = 0; i < 6; ++i) {
            sample.s[i] = input.servos[i];
        }
        servo_buf_.push_back(sample);
        while (servo_buf_.size() > buffer_size_) {
            servo_buf_.pop_front();
        }
    }

    void pull_from_buffer() {
        if (servo_buf_.empty()) {
            return;
        }
        const ServosStored sample = servo_buf_.front();
        servo_buf_.pop_front();
        for (int i = 0; i < 6; ++i) {
            servos_delayed_[i] = sample.s[i];
        }
    }

    FrameType frame_type_{kConventional};
    float hover_lean_{3.2f};
    float izz_{0.2f};
    float iyy_{1.0f};
    float tr_dist_{0.85f};
    float cyclic_scalar_{7.2f};
    float thrust_scale_{0.0f};
    math::Vector2f tpp_angle_{};
    math::Vector2f tpp_angle_1_{};
    math::Vector2f tpp_angle_2_{};
    float torque_scale_{0.0f};
    float torque_mpog_{0.0f};
    float torque_max_{0.0f};
    float hover_coll_{5.0f};
    bool motor_interlock_{false};
    std::uint8_t time_delay_ms_{50};
    bool gas_heli_{false};
    float nominal_rpm_{1500.0f};
    float power_consumption_watts_{1000.0f};
    float default_battery_resistance_ohm_{0.01f};
    std::uint16_t servos_delayed_[6]{};
    std::deque<ServosStored> servo_buf_{};
    std::size_t buffer_size_{1};
    bool buffer_inited_{false};
    float rotor_runup_output_{0.0f};
    std::uint8_t motor_status_{0};
};

}  // namespace fwcpp::sim
