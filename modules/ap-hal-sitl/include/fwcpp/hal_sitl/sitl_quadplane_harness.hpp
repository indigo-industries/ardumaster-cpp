#pragma once

// VCP-011: SitlQuadPlaneHarness — QuadPlane analogue of SitlHarness (CPP-084).
// RC/sensors from SimQuadPlane truth -> Plane::tick + QuadPlane::update ->
// FW servos + VTOL motors into original-source SIM_QuadPlane (VCP-010).
//
// Matches sitl/main.cpp wiring (Plane + plant via reusable harness). Plane
// tick does not own QuadPlane (disclosed fw-cpp gap); this harness ticks
// both in the SITL HAL role. VTOL PWM is leftover MotorsMatrix Quad-X at
// Frame::motor_offset (same leftover_apply_collective path as copter),
// not a new mixer.

#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/motors/motors_matrix.hpp>
#include <fwcpp/q_modes/mode_qhover.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_pilot_input.hpp>
#include <fwcpp/quadplane/quadplane_update.hpp>
#include <fwcpp/sim/sim_quadplane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

namespace fwcpp::hal_sitl {

// roll_dem_rad / pitch_dem_rad (WOPR bridge phase 3, 2026-08-31): attitude
// DEMANDS for the leveler. Default 0 preserves the original behavior exactly
// (level hover); a caller running a position/velocity cascade above this
// (e.g. wopr_bridge's qhover position hold / NAV_LAND-at-coords) tilts the
// frame by demanding non-zero attitude here. The P leveler is unchanged —
// it just targets the demand instead of zero.
inline void leftover_apply_vtol_motors(sim::SitlInput& input, const sim::SimQuadPlane& sim, float command,
                                       bool armed, float dt = 0.0025f,
                                       float roll_dem_rad = 0.0f, float pitch_dem_rad = 0.0f) {
    static motors::MotorsMatrix mixer;
    static bool inited = false;
    if (!inited) {
        mixer.setup_motors(motors::MotorsMatrix::FrameClass::Quad, motors::MotorsMatrix::FrameType::X);
        mixer.normalise_rpy_factors();
        mixer.set_throttle_thrust_max(1.0f);
        inited = true;
    }
    if (!armed) {
        return;
    }
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    sim.dcm.to_euler(&roll, &pitch, &yaw);
    const float roll_in = math::constrain_value(-0.5f * (roll - roll_dem_rad), -1.0f, 1.0f);
    const float pitch_in = math::constrain_value(-0.5f * (pitch - pitch_dem_rad), -1.0f, 1.0f);
    const float yaw_in = math::constrain_value(-0.2f * sim.gyro.z, -1.0f, 1.0f);
    bool lr = false, lp = false, ly = false, ll = false, lu = false;
    mixer.output_armed_stabilizing(roll_in, 0.0f, pitch_in, 0.0f, yaw_in, 0.0f, command, command, 0.0f, 1.0f, dt, lr,
                                   lp, ly, ll, lu);
    mixer.set_spool_state(motors::MotorsMatrix::SpoolState::ThrottleUnlimited);
    motors::ThrustLinParams params;
    params.curve_expo = 0.0f;
    params.spin_min = 0.0f;
    params.spin_max = 1.0f;
    mixer.output_to_motors(true, false, 0.0f, 0.0f, 0.0f, params, dt, 1000, 2000);
    const auto& frame = sim.frame();
    for (std::uint8_t i = 0; i < frame.num_motors; ++i) {
        input.servos[frame.motor_offset + frame.motors[i].servo] = static_cast<std::uint16_t>(mixer.pwm_out(i));
    }
}

class SitlQuadPlaneHarness {
public:
    SitlQuadPlaneHarness(vehicle::Plane& plane, quadplane::QuadPlane& quadplane, sim::SimQuadPlane& sim)
        : plane_(plane), quadplane_(quadplane), sim_(sim) {}

    // Sensors from SimQuadPlane (SitlHarness pattern), Plane::tick, QuadPlane
    // update + QHover leftover, then FW+VTOL SitlInput into SIM_QuadPlane.
    // roll/pitch demands (radians) feed the VTOL leveler — see
    // leftover_apply_vtol_motors; 0 = level hover (original behavior).
    void step(std::uint32_t now_ms, float dt, float vtol_command, bool vtol_armed,
              const math::Vector3f& gyro_bias = math::Vector3f{},
              float vtol_roll_dem_rad = 0.0f, float vtol_pitch_dem_rad = 0.0f) {
        vehicle::StabilizeInputs in;
        in.dt = dt;
        in.now_ms = now_ms;
        in.now_us = static_cast<std::uint64_t>(now_ms) * 1000ULL;

        const math::Vector3f measured_gyro = sim_.gyro + gyro_bias;
        ahrs::GyroSample gyro_sample;
        gyro_sample.gyro = measured_gyro;
        gyro_sample.delta_angle = measured_gyro * dt;
        gyro_sample.dangle_dt = dt;

        in.accel_sample.accel = sim_.accel_body;
        in.accel_sample.delta_velocity = sim_.accel_body * dt;
        in.accel_sample.delta_velocity_dt = dt;
        in.accel_y = sim_.accel_body.y;

        in.compass_healthy = true;
        in.compass_field_bf = plane_.compass.rotate_earth_field_to_body(sim_.dcm);
        in.true_velocity_ned = sim_.velocity_ef;
        in.airspeed_sensor_enabled = true;
        in.airspeed_raw_pressure_pa = sim_.airspeed_sensor_differential_pressure();
        in.gps_use_enabled = true;
        in.position_ned = sim_.position;
        in.current_altitude_m = -sim_.position.z;

        vehicle::tick(plane_, gyro_sample, in);

        quadplane::QuadPlaneUpdateView view;
        view.now_ms = now_ms;
        view.armed_and_safety_off = plane_.armed;
        view.in_vtol_mode = true;
        view.have_airspeed = true;
        view.airspeed_ms = sim_.airspeed;
        last_qp_tick_ = quadplane_.update(view);

        q_modes::QHoverRunInputs qh;
        qh.throttle_wait = !vtol_armed;
        last_qhover_ = q_modes::qhover_run(qh);
        if (vtol_armed && last_qhover_.actions.hold_hover) {
            quadplane::DesiredYawRateInputs yaw{};
            last_hold_ = quadplane_.hold_hover(last_qhover_.actions.hold_hover_climb_rate_cms, yaw);
        }

        quadplane::MotorsOutputView mo;
        mo.armed_and_safety_off = plane_.armed && vtol_armed;
        mo.now_ms = now_ms;
        mo.motors_throttle = vtol_command;
        last_motors_ = quadplane::run_motors_output(mo, quadplane_.options(), false, motors_state_);

        sim::SitlInput input{};
        const float aileron = plane_.srv_channels.get_output_scaled(srv::Function::kAileron) / vehicle::kServoMax;
        const float elevator = plane_.srv_channels.get_output_scaled(srv::Function::kElevator) / vehicle::kServoMax;
        const float rudder = plane_.srv_channels.get_output_scaled(srv::Function::kRudder) / vehicle::kServoMax;
        const float throttle = plane_.srv_channels.get_output_scaled(srv::Function::kThrottle) / 100.0f;
        input.servos[0] = static_cast<std::uint16_t>(math::constrain_value(1500.0f + aileron * 500.0f, 1000.0f, 2000.0f));
        input.servos[1] = static_cast<std::uint16_t>(math::constrain_value(1500.0f + elevator * 500.0f, 1000.0f, 2000.0f));
        input.servos[2] = static_cast<std::uint16_t>(math::constrain_value(1000.0f + throttle * 1000.0f, 1000.0f, 2000.0f));
        input.servos[3] = static_cast<std::uint16_t>(math::constrain_value(1500.0f + rudder * 500.0f, 1000.0f, 2000.0f));
        leftover_apply_vtol_motors(input, sim_, vtol_command, vtol_armed && last_motors_.action == quadplane::MotorsOutputAction::kOutput, dt,
                                   vtol_roll_dem_rad, vtol_pitch_dem_rad);
        last_input_ = input;
        sim_.update(input, dt);
        ++tick_count_;
    }

    [[nodiscard]] vehicle::Plane& plane() { return plane_; }
    [[nodiscard]] quadplane::QuadPlane& quadplane() { return quadplane_; }
    [[nodiscard]] sim::SimQuadPlane& sim() { return sim_; }
    [[nodiscard]] std::uint32_t tick_count() const { return tick_count_; }
    [[nodiscard]] const quadplane::QuadPlaneUpdateTick& last_qp_tick() const { return last_qp_tick_; }
    [[nodiscard]] const q_modes::QHoverRunResult& last_qhover() const { return last_qhover_; }
    [[nodiscard]] const sim::SitlInput& last_input() const { return last_input_; }

private:
    vehicle::Plane& plane_;
    quadplane::QuadPlane& quadplane_;
    sim::SimQuadPlane& sim_;
    quadplane::MotorsOutputState motors_state_{};
    quadplane::QuadPlaneUpdateTick last_qp_tick_{};
    q_modes::QHoverRunResult last_qhover_{};
    quadplane::HoldHoverTick last_hold_{};
    quadplane::MotorsOutputTick last_motors_{};
    sim::SitlInput last_input_{};
    std::uint32_t tick_count_{0};
};

namespace sitl_quadplane {

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
    {"SitlQuadPlaneHarness scaffold", PortStatus::kThisSlice,
     "Plane + QuadPlane + SimQuadPlane; sensors then tick then SitlInput"},
    {"sensor synthesis", PortStatus::kThisSlice, "SitlHarness pattern from SimQuadPlane truth"},
    {"Plane::tick", PortStatus::kThisSlice, "vehicle::tick like sitl/main.cpp"},
    {"QuadPlane::update", PortStatus::kThisSlice, "run_quadplane_update leftover"},
    {"QHover leftover", PortStatus::kThisSlice, "qhover_run + QuadPlane::hold_hover"},
    {"VTOL motors leftover", PortStatus::kThisSlice,
     "MotorsMatrix Quad-X at Frame::motor_offset into SitlInput"},
    {"SIM_QuadPlane plant", PortStatus::kOnMain, "sim_quadplane.hpp VCP-010 original-source"},
    {"SitlHarness Plane path (CPP-084)", PortStatus::kOnMain, "sitl_harness.hpp"},
    {"GCS / MAVLink / interactive run", PortStatus::kOutOfScope, "bounded duration like CPP-085"},
    {"AP:: / HAL SITL singletons", PortStatus::kOutOfScope, "ADR-0012 explicit refs"},
    {"Rust quadplane-sitl", PortStatus::kOutOfScope, "Do not copy Rust"},
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

[[nodiscard]] inline constexpr std::size_t on_main_count() { return count_status(PortStatus::kOnMain); }
[[nodiscard]] inline constexpr std::size_t this_slice_count() { return count_status(PortStatus::kThisSlice); }
[[nodiscard]] inline constexpr std::size_t remaining_count() { return count_status(PortStatus::kRemaining); }
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() { return count_status(PortStatus::kOutOfScope); }

}  // namespace sitl_quadplane

}  // namespace fwcpp::hal_sitl
