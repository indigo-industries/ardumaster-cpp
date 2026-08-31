#pragma once

// Port of libraries/AP_Motors/AP_MotorsHeli_Swash.cpp calculate_roll_pitch_collective_factors
// and calculate()/rc_write so SIM_Helicopter can be fed real CCPM mixing PWM
// rather than raw cyclic. RSC/Dual/Quad mixing helpers included for Single.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::motors {

enum class SwashPlateType : std::uint8_t {
    H3 = 0,
    H1 = 1,
    H3_140 = 2,
    H3_120 = 3,
    H4_90 = 4,
    H4_45 = 5,
};

enum class CollectiveDirection : std::uint8_t { NORMAL = 0, REVERSED = 1 };

class MotorsHeliSwash {
public:
    static constexpr std::uint8_t kMaxServos = 4;
    SwashPlateType _swash_type = SwashPlateType::H3_120;
    CollectiveDirection _collective_direction = CollectiveDirection::NORMAL;
    bool _make_servo_linear = false;
    float _servo1_pos = -60;
    float _servo2_pos = 60;
    float _servo3_pos = 180;
    float _phase_angle = 0;
    bool _enabled[kMaxServos]{};
    float _rollFactor[kMaxServos]{};
    float _pitchFactor[kMaxServos]{};
    float _collectiveFactor[kMaxServos]{};
    float _output[kMaxServos]{};
    float _roll_input = 0;
    float _pitch_input = 0;
    float _collective_input_scaled = 0;

    void add_servo_raw(std::uint8_t num, float roll, float pitch, float collective) {
        if (num >= kMaxServos) {
            return;
        }
        _enabled[num] = true;
        _rollFactor[num] = roll * 0.45f;
        _pitchFactor[num] = pitch * 0.45f;
        _collectiveFactor[num] = collective;
    }
    void add_servo_angle(std::uint8_t num, float angle, float collective) {
        add_servo_raw(num, std::cos(math::radians(angle + 90)), std::cos(math::radians(angle)), collective);
    }
    void calculate_roll_pitch_collective_factors() {
        for (std::uint8_t i = 0; i < kMaxServos; i++) {
            _enabled[i] = false;
            _rollFactor[i] = _pitchFactor[i] = _collectiveFactor[i] = 0;
        }
        switch (_swash_type) {
        case SwashPlateType::H3:
            add_servo_angle(0, _servo1_pos - _phase_angle, 1.0f);
            add_servo_angle(1, _servo2_pos - _phase_angle, 1.0f);
            add_servo_angle(2, _servo3_pos - _phase_angle, 1.0f);
            break;
        case SwashPlateType::H1:
            add_servo_raw(0, 1.0f, 0.0f, 0.0f);
            add_servo_raw(1, 0.0f, 1.0f, 0.0f);
            add_servo_raw(2, 0.0f, 0.0f, 1.0f);
            break;
        case SwashPlateType::H3_140:
            add_servo_raw(0, 1.0f, 1.0f, 1.0f);
            add_servo_raw(1, -1.0f, 1.0f, 1.0f);
            add_servo_raw(2, 0.0f, -1.0f, 1.0f);
            break;
        case SwashPlateType::H3_120:
            add_servo_angle(0, -60.0f, 1.0f);
            add_servo_angle(1, 60.0f, 1.0f);
            add_servo_angle(2, 180.0f, 1.0f);
            break;
        case SwashPlateType::H4_90:
            add_servo_angle(0, -90.0f, 1.0f);
            add_servo_angle(1, 90.0f, 1.0f);
            add_servo_angle(2, 180.0f, 1.0f);
            add_servo_angle(3, 0.0f, 1.0f);
            break;
        case SwashPlateType::H4_45:
            add_servo_angle(0, -45.0f, 1.0f);
            add_servo_angle(1, 45.0f, 1.0f);
            add_servo_angle(2, -135.0f, 1.0f);
            add_servo_angle(3, 135.0f, 1.0f);
            break;
        }
    }
    void configure() { calculate_roll_pitch_collective_factors(); }
    static float get_linear_servo_output(float input) {
        input = math::constrain_value(input, -1.0f, 1.0f);
        return math::safe_asin(0.766044f * input) * 1.145916f;
    }
    void calculate(float roll, float pitch, float collective) {
        _roll_input = roll;
        _pitch_input = pitch;
        _collective_input_scaled = collective;
        if (_collective_direction == CollectiveDirection::REVERSED) {
            collective = 1 - collective;
        }
        for (std::uint8_t i = 0; i < kMaxServos; i++) {
            if (!_enabled[i]) {
                continue;
            }
            _output[i] = (_rollFactor[i] * roll) + (_pitchFactor[i] * pitch) + _collectiveFactor[i] * collective;
            if (_swash_type == SwashPlateType::H1 && (i == 0 || i == 1)) {
                _output[i] += 0.5f;
            }
            _output[i] = 2.0f * _output[i] - 1.0f;
            if (_make_servo_linear) {
                _output[i] = get_linear_servo_output(_output[i]);
            }
        }
    }
    static std::uint16_t rc_write_pwm(float swash_in) {
        return static_cast<std::uint16_t>(1500 + 500 * swash_in);
    }
    void write_servos(std::uint16_t* servos, float rsc_pwm = 1600) const {
        for (std::uint8_t i = 0; i < kMaxServos; i++) {
            if (_enabled[i]) {
                servos[i] = rc_write_pwm(_output[i]);
            }
        }
        servos[7] = static_cast<std::uint16_t>(rsc_pwm);
    }
};

class MotorsHeliRSC {
public:
    float desired_rotor_speed = 0;
    float rotor_ramp = 0;
    float ramp_time = 1.0f;
    float runup_time = 10.0f;
    float rotor_runup = 0;
    void set_desired_rotor_speed(float s) { desired_rotor_speed = s; }
    void update_rotor_ramp(float dt) {
        const float step = (ramp_time > 0) ? dt / ramp_time : 1.0f;
        if (rotor_ramp < desired_rotor_speed) {
            rotor_ramp = std::min(desired_rotor_speed, rotor_ramp + step);
        } else {
            rotor_ramp = std::max(desired_rotor_speed, rotor_ramp - step);
        }
        const float run_step = (runup_time > 0) ? dt / runup_time : 1.0f;
        if (rotor_runup < rotor_ramp) {
            rotor_runup = std::min(rotor_ramp, rotor_runup + run_step);
        } else {
            rotor_runup = std::max(rotor_ramp, rotor_runup - run_step);
        }
    }
    float get_rotor_speed() const { return rotor_runup; }
    std::uint16_t output_pwm() const { return static_cast<std::uint16_t>(1000 + 1000 * rotor_ramp); }
};

enum class HeliTailType : std::uint8_t {
    SERVO = 0,
    SERVO_EXTGYRO = 1,
    DDVP = 2,
    DDFP_CW = 3,
    DDFP_CCW = 4,
};
enum class HeliDualMode : std::uint8_t { TANDEM = 0, TRANSVERSE = 1, INTERMESHING = 2 };

class MotorsHeliSingle {
public:
    MotorsHeliSwash swash;
    MotorsHeliRSC rsc;
    MotorsHeliRSC tail_rsc;
    HeliTailType tail_type = HeliTailType::SERVO;
    bool flybar_mode = false;
    float cyclic_max = 4500;
    float collective_min = 1250;
    float collective_max = 1750;
    float collective_land_min = 0;
    float collective_zero_thrust_pct = 0.5f;
    float collective_yaw_scale = 0;
    float yaw_trim = 0;
    float ext_gyro_gain_std = 350;
    float ext_gyro_gain_acro = 0;
    float ddvp_tailspeed = 1.0f;
    float servo4_out = 0;
    bool acro_tail = false;
    bool rotor_active = true;

    void configure() { swash.configure(); }

    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_in) {
        // cyclic scaler: original divides by cyclic_max/4500
        const float scaler = cyclic_max > 0 ? 4500.0f / cyclic_max : 1.0f;
        float roll = math::constrain_value(roll_out * scaler, -1.0f, 1.0f);
        float pitch = math::constrain_value(pitch_out * scaler, -1.0f, 1.0f);
        float collective = math::constrain_value(coll_in, 0.0f, 1.0f);
        if (!rotor_active) {
            collective = std::max(collective, collective_land_min);
        }
        float yaw_offset = 0;
        if (tail_type != HeliTailType::SERVO_EXTGYRO && rotor_active) {
            yaw_offset = collective_yaw_scale * std::pow(std::fabs(collective - collective_zero_thrust_pct), 1.5f);
            if (tail_type == HeliTailType::DDFP_CW || tail_type == HeliTailType::DDFP_CCW) {
                yaw_offset += yaw_trim;
            }
        }
        const float collective_scalar = (collective_max - collective_min) * 0.001f;
        const float collective_out_scaled = collective * collective_scalar + (collective_min - 1000.0f) * 0.001f;
        swash.calculate(roll, pitch, collective_out_scaled);
        servo4_out = math::constrain_value(yaw_in + yaw_offset, -1.0f, 1.0f);
        rsc.set_desired_rotor_speed(rotor_active ? 1.0f : 0.0f);
        if (tail_type == HeliTailType::DDVP) {
            tail_rsc.set_desired_rotor_speed(rotor_active ? ddvp_tailspeed : 0.0f);
        }
    }

    void write_servos(std::uint16_t* servos, float dt = 0.01f) {
        rsc.update_rotor_ramp(dt);
        tail_rsc.update_rotor_ramp(dt);
        swash.write_servos(servos, rsc.output_pwm());
        float yaw = servo4_out;
        if (tail_type == HeliTailType::DDFP_CCW) {
            yaw = -yaw;
        }
        if (tail_type == HeliTailType::DDFP_CW || tail_type == HeliTailType::DDFP_CCW) {
            servos[3] = static_cast<std::uint16_t>(1000 + 1000 * math::constrain_value(yaw, 0.0f, 1.0f));
        } else {
            servos[3] = static_cast<std::uint16_t>(1500 + 500 * yaw);
        }
        if (tail_type == HeliTailType::SERVO_EXTGYRO) {
            const float gain = (acro_tail && ext_gyro_gain_acro > 0) ? ext_gyro_gain_acro : ext_gyro_gain_std;
            servos[6] = static_cast<std::uint16_t>(1000 + gain);
        }
        if (tail_type == HeliTailType::DDVP) {
            servos[7] = rsc.output_pwm();
            servos[8] = tail_rsc.output_pwm();
        }
    }
};

class MotorsHeliDual {
public:
    MotorsHeliSwash swash1;
    MotorsHeliSwash swash2;
    MotorsHeliRSC rsc;
    HeliDualMode dual_mode = HeliDualMode::TANDEM;
    float dcp_scaler = 0.25f;
    float yaw_scaler = 1.0f;
    float collective_min = 1250;
    float collective_max = 1750;

    void configure() {
        swash1.configure();
        swash2.configure();
    }
    void mix_tandem(float roll, float pitch, float yaw, float coll, float& r1, float& p1, float& r2, float& p2) {
        r1 = roll - yaw * yaw_scaler;
        r2 = roll + yaw * yaw_scaler;
        p1 = pitch + roll * dcp_scaler;
        p2 = pitch - roll * dcp_scaler;
        (void)coll;
    }
    void mix_transverse(float roll, float pitch, float yaw, float coll, float& r1, float& p1, float& r2, float& p2) {
        p1 = pitch + yaw * yaw_scaler;
        p2 = pitch - yaw * yaw_scaler;
        r1 = roll + pitch * dcp_scaler;
        r2 = roll - pitch * dcp_scaler;
        (void)coll;
    }
    void mix_intermeshing(float roll, float pitch, float yaw, float coll, float& r1, float& p1, float& r2, float& p2) {
        p1 = pitch + yaw * yaw_scaler;
        p2 = pitch - yaw * yaw_scaler;
        r1 = roll;
        r2 = -roll;
        (void)coll;
    }
    void move_actuators(float roll, float pitch, float coll, float yaw) {
        const float collective_scalar = (collective_max - collective_min) * 0.001f;
        const float coll_scaled = math::constrain_value(coll, 0.0f, 1.0f) * collective_scalar + (collective_min - 1000.0f) * 0.001f;
        float r1, p1, r2, p2;
        if (dual_mode == HeliDualMode::TANDEM) {
            mix_tandem(roll, pitch, yaw, coll, r1, p1, r2, p2);
        } else if (dual_mode == HeliDualMode::TRANSVERSE) {
            mix_transverse(roll, pitch, yaw, coll, r1, p1, r2, p2);
        } else {
            mix_intermeshing(roll, pitch, yaw, coll, r1, p1, r2, p2);
        }
        swash1.calculate(r1, p1, coll_scaled);
        swash2.calculate(r2, p2, coll_scaled);
        rsc.set_desired_rotor_speed(1.0f);
    }
    void write_servos(std::uint16_t* servos, float dt = 0.01f) {
        rsc.update_rotor_ramp(dt);
        for (std::uint8_t i = 0; i < MotorsHeliSwash::kMaxServos; i++) {
            if (swash1._enabled[i]) {
                servos[i] = MotorsHeliSwash::rc_write_pwm(swash1._output[i]);
            }
            if (swash2._enabled[i]) {
                servos[3 + i] = MotorsHeliSwash::rc_write_pwm(swash2._output[i]);
            }
        }
        servos[7] = rsc.output_pwm();
    }
};

class MotorsHeliQuad {
public:
    MotorsHeliRSC rsc;
    float collective_min = 1250;
    float collective_max = 1750;
    float out[4]{};
    void move_actuators(float roll, float pitch, float coll, float yaw) {
        const float collective_scalar = (collective_max - collective_min) * 0.001f;
        const float coll_scaled = math::constrain_value(coll, 0.0f, 1.0f) * collective_scalar + (collective_min - 1000.0f) * 0.001f;
        static constexpr float angles[4] = {45, 225, 315, 135};
        static constexpr bool clockwise[4] = {false, false, true, true};
        const float c45 = std::cos(math::radians(45.0f));
        for (int i = 0; i < 4; i++) {
            const float r = -0.25f * std::sin(math::radians(angles[i])) / c45 * roll;
            const float p = 0.25f * std::cos(math::radians(angles[i])) / c45 * pitch;
            float y = 0.25f * yaw;
            if (clockwise[i]) {
                y = -y;
            }
            out[i] = r + p + y + coll_scaled;
        }
        rsc.set_desired_rotor_speed(1.0f);
    }
    void write_servos(std::uint16_t* servos, float dt = 0.01f) {
        rsc.update_rotor_ramp(dt);
        for (int i = 0; i < 4; i++) {
            servos[i] = MotorsHeliSwash::rc_write_pwm(2.0f * out[i] - 1.0f);
        }
        servos[7] = rsc.output_pwm();
    }
};

}  // namespace fwcpp::motors
