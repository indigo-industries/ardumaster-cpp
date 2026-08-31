#pragma once

// CCP-043: SitlCopterHarness - Copter analogue of SitlHarness (CPP-084).
// CCP-045: motor PWM -> SimMulticopter Frame/Motor plant.
// CCP-065: PWM mixing is MotorsMatrix from AC_PosControl NE lean + D throttle.
// leftover_apply_collective is not the XY path.
//
// Upstream ROLE: AP_HAL_SITL SITL_State sensor synthesis for ArduCopter.
// Not a port of AP_HAL_SITL source (ADR-0012). Mirrors SitlHarness:
// sensors from sim truth, vehicle tick, then servo/PWM feedback into the
// plant. Copter plant is SimMulticopter (SIM_Multicopter Frame/Motor),
// not SimPlane.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <fwcpp/compass/compass.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/motors/motors_matrix.hpp>
#include <fwcpp/sim/sim_baro.hpp>
#include <fwcpp/sim/sim_gps.hpp>
#include <fwcpp/sim/sim_motor.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

namespace fwcpp::hal_sitl {

class SitlCopterHarness {
public:
    // CCP-066: mixer_ used to be a function-local `static` in
    // apply_motor_pwm, shared by every SitlCopterHarness instance in the
    // process (a real multi-vehicle/multi-test state-sharing bug, and
    // -fno-threadsafe-statics meant even its one-time init wasn't
    // guarded). It is now a genuine per-instance member, initialized once
    // per instance right here in the constructor.
    SitlCopterHarness(copter::LeftoverCopter& copter, sim::SimMulticopter& sim)
        : copter_(copter), sim_(sim) {
        mixer_.setup_motors(motors::MotorsMatrix::FrameClass::Quad, motors::MotorsMatrix::FrameType::X);
        mixer_.normalise_rpy_factors();
        mixer_.set_throttle_thrust_max(1.0f);
    }

    // Synthesize gyro/accel/baro/GPS/compass from sim_ into leftover
    // sensor buffers, inject arm/spool/attitude-hold smoke flags, tick
    // leftover Copter, mix MotorsMatrix PWM from PosControl NE lean + D
    // throttle, then feed motor_pwm into SimMulticopter::update.
    void step(float dt) {
        copter_.loop_dt = dt;
        copter_.gyro_buffer = sim_.gyro;
        copter_.accel_buffer = sim_.accel_body;
        copter_.gyro_injected = true;
        copter_.accel_injected = true;

        sim_.update_position();
        sim_.update_mag_field_bf();
        const auto baro = sim::sitl_baro_from_aircraft(sim_);
        copter_.baro_altitude_m = baro.altitude_amsl_m - sim_.home.alt * 0.01f;
        copter_.baro_injected = true;

        const auto gps = sim::sitl_gps_from_aircraft(sim_);
        copter_.gps_lat = gps.lat;
        copter_.gps_lng = gps.lng;
        copter_.gps_injected = true;

        copter_.compass_field_bf = sim_.get_mag_field_bf();
        copter_.compass_injected = true;

        copter_.motors_armed_injected = true;
        if (copter_.motors_armed) {
            copter_.spool_state = copter::SpoolState::THROTTLE_UNLIMITED;
            copter_.attitude_hold = true;
        } else {
            copter_.spool_state = copter::SpoolState::SHUT_DOWN;
            copter_.attitude_hold = false;
        }
        copter_.spool_injected = true;
        copter_.attitude_hold_injected = true;

        copter::leftover_copter_tick(copter_);
        apply_motor_pwm(dt);

        sim::SitlInput input;
        for (std::uint8_t i = 0; i < sim::kSitlServoChannels; ++i) {
            input.servos[i] = copter_.motor_pwm[i];
        }
        sim_.update(input, dt);
    }

    [[nodiscard]] copter::LeftoverCopter& copter() { return copter_; }
    [[nodiscard]] sim::SimMulticopter& sim() { return sim_; }
    [[nodiscard]] sim::SimMulticopter& sim_plane() { return sim_; }
    [[nodiscard]] const compass::Compass& compass() const { return compass_; }
    [[nodiscard]] std::uint32_t tick_count() const { return copter_.tick_count; }

    // CCP-066: test-only accessor proving mixer_ is a genuine per-instance
    // member (distinct address per SitlCopterHarness) rather than the
    // function-local `static` it used to be (one shared address for every
    // instance in the process, regardless of `this`).
    [[nodiscard]] const motors::MotorsMatrix& mixer_for_test() const { return mixer_; }

private:
    void apply_motor_pwm(float dt) {
        if (!copter_.motors_armed) {
            for (std::uint8_t i = 0; i < sim::kSitlServoChannels; ++i) {
                copter_.motor_pwm[i] = 0;
            }
            return;
        }
        // CCP-045 tests inject differential motor_pwm without PosControl.
        // Mix from NE lean + D throttle only when the mission actually
        // commanded collective/lean this tick.
        if (copter_.throttle_out <= 1.0e-6f && std::fabs(copter_.roll_target_rad) <= 1.0e-6f &&
            std::fabs(copter_.pitch_target_rad) <= 1.0e-6f) {
            return;
        }
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        sim_.dcm.to_euler(&roll, &pitch, &yaw);
        constexpr float kAttP = 2.0f;
        const float roll_in =
            math::constrain_value(kAttP * (copter_.roll_target_rad - roll), -1.0f, 1.0f);
        const float pitch_in =
            math::constrain_value(kAttP * (copter_.pitch_target_rad - pitch), -1.0f, 1.0f);
        const float yaw_in = math::constrain_value(-0.2f * sim_.gyro.z, -1.0f, 1.0f);
        const float command = math::constrain_value(copter_.throttle_out, 0.0f, 1.0f);
        bool lr = false, lp = false, ly = false, ll = false, lu = false;
        mixer_.output_armed_stabilizing(roll_in, 0.0f, pitch_in, 0.0f, yaw_in, 0.0f, command, command, 0.0f, 1.0f, dt,
                                        lr, lp, ly, ll, lu);
        mixer_.set_spool_state(motors::MotorsMatrix::SpoolState::ThrottleUnlimited);
        motors::ThrustLinParams params;
        params.curve_expo = 0.0f;
        params.spin_min = 0.0f;
        params.spin_max = 1.0f;
        mixer_.output_to_motors(true, false, 0.0f, 0.0f, 0.0f, params, dt, 1000, 2000);
        const auto& frame = sim_.frame();
        for (std::uint8_t i = 0; i < frame.num_motors; ++i) {
            copter_.motor_pwm[frame.motor_offset + frame.motors[i].servo] =
                static_cast<std::uint16_t>(mixer_.pwm_out(i));
        }
    }

    copter::LeftoverCopter& copter_;
    sim::SimMulticopter& sim_;
    compass::Compass compass_{};
    motors::MotorsMatrix mixer_{};
};

namespace sitl_copter {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"SitlCopterHarness scaffold", PortStatus::kThisSlice,
     "refs LeftoverCopter + SimMulticopter; step sensor inject + leftover_copter_tick"},
    {"leftover_copter_tick", PortStatus::kThisSlice,
     "CCP-064: leftover_copter_loop = Copter::loop leftover scheduler + update_flight_mode"},
    {"leftover_copter_loop", PortStatus::kThisSlice,
     "FAST_TASK rate/motors/AHRS/inertia/ekf/mode + SCHED_TASK rc/throttle/nav"},
    {"gyro/accel synthesis", PortStatus::kThisSlice,
     "SimMulticopter::gyro / accel_body -> leftover buffers + inject flags"},
    {"baro synthesis", PortStatus::kThisSlice,
     "SimMulticopter altitude (-position.z) -> leftover baro_altitude_m + flag"},
    {"GPS synthesis", PortStatus::kThisSlice,
     "home lat/lng + SimMulticopter NED north/east -> leftover gps_lat/gps_lng + flag"},
    {"compass synthesis", PortStatus::kThisSlice,
     "Compass earth field via SimMulticopter::dcm -> compass_field_bf + flag"},
    {"closed-loop arm/spool/hold", PortStatus::kThisSlice,
     "step injects motors_armed + spool + attitude_hold smoke"},
    {"motor PWM to SimMulticopter", PortStatus::kThisSlice,
     "CCP-065: PosControl NE lean + D throttle -> MotorsMatrix PWM -> Frame/Motor; leftover_apply_collective is not XY"},
    {"SitlHarness Plane path (CPP-084)", PortStatus::kOnMain,
     "sitl_harness.hpp; Plane+SimPlane closed loop"},
    {"CCP-035 update_flight_mode", PortStatus::kOnMain,
     "update_flight_mode.hpp; harness wires via leftover_copter_tick"},
    {"SIM_Multicopter Frame/Motor plant", PortStatus::kOnMain,
     "sim_multicopter.hpp / sim_frame.hpp / sim_motor.hpp (CCP-045)"},
    {"AP:: / HAL SITL singletons", PortStatus::kOutOfScope, "ADR-0012 explicit refs"},
    {"Rust copter-sitl", PortStatus::kOutOfScope, "Do not copy Rust"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace sitl_copter

}  // namespace fwcpp::hal_sitl
