#pragma once

// CCP-027 slice 8: AC_PosControl class shell — aggregated state with injected
// deps (AHRS estimates, attitude capability, motors throttle limits). No
// AP_Param, AP::ahrs(), or HAL logging (OutOfScope per pos_control_leftover.hpp).

#include <cstdint>

#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/pid/ac_pid_2d.hpp>
#include <fwcpp/pid/ac_pid_basic.hpp>
#include <fwcpp/poscontrol/pos_control_accessors.hpp>
#include <fwcpp/poscontrol/pos_control_d.hpp>
#include <fwcpp/poscontrol/pos_control_defaults.hpp>
#include <fwcpp/poscontrol/pos_control_ne.hpp>
#include <fwcpp/poscontrol/pos_control_path.hpp>

namespace fwcpp::poscontrol {

struct PosControlConfig {
    float lean_angle_max_deg = 0.0f;
    float shaping_jerk_ne_msss = kPoscontrolJerkNeMsss;
    float shaping_jerk_d_msss = kPoscontrolJerkDMsss;
    float ne_pos_p = kNePosP;
};

struct PosControlInjectedDeps {
    AttitudeCapability attitude{};
    float attitude_lean_angle_max_rad = 0.0f;
    float angle_max_override_rad = 0.0f;
    float ahrs_control_scale_xy = 1.0f;
    float ahrs_control_scale_z = 1.0f;
    float cos_yaw = 1.0f;
    float sin_yaw = 0.0f;
    float att_yaw_target_rad = 0.0f;
    math::Vector3f att_target_euler_rad{};
    float throttle_hover = 0.5f;
    float throttle_in = 0.5f;
    bool throttle_lower = false;
    bool throttle_upper = false;
    float estimated_accel_d_mss = 0.0f;
    bool vibe_comp_enabled = false;
    std::uint32_t now_ms = 0;
    std::uint32_t ticks = 0;
    std::uint32_t ahrs_ekf_reset_ms = 0;
    std::uint16_t position_d_reset_count = 0;
};

class PosControl {
public:
    PosControl() = default;
    PosControl(PosControlInjectedDeps deps, PosControlConfig config = {})
        : deps_(deps), config_(config) {
        p_pos_ne_ = pid::AcP2d::with_kp(config_.ne_pos_p);
    }

    void set_injected_deps(PosControlInjectedDeps deps) { deps_ = deps; }
    [[nodiscard]] const PosControlInjectedDeps& injected_deps() const { return deps_; }

    void set_config(PosControlConfig config) {
        config_ = config;
        p_pos_ne_.kp = config_.ne_pos_p;
    }
    [[nodiscard]] const PosControlConfig& config() const { return config_; }

    void set_dt_s(float dt) { dt_s_ = dt; }
    [[nodiscard]] float get_dt_s() const { return dt_s_; }

    [[nodiscard]] float get_shaping_jerk_ne_msss() const { return config_.shaping_jerk_ne_msss; }

    void update_estimates(const AhrsPosControlEstimateInputs& ahrs) {
        estimates_ = ::fwcpp::poscontrol::update_estimates(ahrs, estimates_);
    }

    [[nodiscard]] const NedEstimates& get_estimates() const { return estimates_; }

    [[nodiscard]] float get_lean_angle_max_rad() const {
        LeanAngleMaxConfig lean{};
        lean.lean_angle_max_deg = config_.lean_angle_max_deg;
        lean.angle_max_override_rad = deps_.angle_max_override_rad;
        lean.attitude_lean_angle_max_rad = deps_.attitude_lean_angle_max_rad;
        return fwcpp::poscontrol::get_lean_angle_max_rad(lean);
    }

    void ne_set_max_speed_accel_m(float speed_ne_ms, float accel_ne_mss) {
        ne_limits_ = fwcpp::poscontrol::ne_set_max_speed_accel_m(speed_ne_ms, accel_ne_mss,
                                              config_.shaping_jerk_ne_msss, deps_.attitude);
    }

    void ne_set_max_speed_accel_cm(float speed_ne_cms, float accel_ne_cmss) {
        ne_set_max_speed_accel_m(speed_ne_cms * 0.01f, accel_ne_cmss * 0.01f);
    }

    void ne_set_correction_speed_accel_m(float speed_ne_ms, float accel_ne_mss) {
        fwcpp::poscontrol::ne_set_correction_speed_accel_m(p_pos_ne_, speed_ne_ms, accel_ne_mss);
    }

    [[nodiscard]] float ne_get_max_speed_ms() const { return ne_limits_.vel_max_ne_ms; }
    [[nodiscard]] float ne_get_max_accel_mss() const { return ne_limits_.accel_max_ne_mss; }

    void d_set_max_speed_accel_m(float descent_ms, float climb_ms, float accel_d_mss) {
        d_limits_ = fwcpp::poscontrol::d_set_max_speed_accel_m(d_limits_, descent_ms, climb_ms, accel_d_mss, config_.shaping_jerk_d_msss, pid_accel_d_);
    }

    [[nodiscard]] NeUpdateOutput ne_update_controller() {
        NeUpdateInputs inp{};
        inp.dt = dt_s_;
        inp.ahrs_control_scale_xy = deps_.ahrs_control_scale_xy;
        inp.ne_control_scale_factor = ne_control_scale_factor_;
        ne_control_scale_factor_ = 1.0f;
        inp.vel_max_ne_ms = ne_limits_.vel_max_ne_ms;
        inp.estimates.pos_m = {estimates_.pos_m.x, estimates_.pos_m.y};
        inp.estimates.vel_ms = {estimates_.vel_ms.x, estimates_.vel_ms.y};
        inp.offsets = ne_offsets_.current;
        inp.lean_angle_max_rad = get_lean_angle_max_rad();
        inp.cos_yaw = deps_.cos_yaw;
        inp.sin_yaw = deps_.sin_yaw;
        inp.att_yaw_target_rad = deps_.att_yaw_target_rad;

        const NeUpdateOutput out =
            ne_.update_controller(p_pos_ne_, pid_vel_ne_, inp, ne_disturb_);

        last_update_ne_ticks_ = deps_.ticks;
        roll_target_rad_ = out.roll_target_rad;
        pitch_target_rad_ = out.pitch_target_rad;
        yaw_target_rad_ = out.yaw_target_rad;
        yaw_rate_target_rads_ = out.yaw_rate_target_rads;
        vel_target_ned_ms_.x = out.vel_target_ms.x;
        vel_target_ned_ms_.y = out.vel_target_ms.y;
        accel_target_ned_mss_.x = out.accel_target_mss.x;
        accel_target_ned_mss_.y = out.accel_target_mss.y;
        pos_target_ned_m_.x = out.pos_target_m.x;
        pos_target_ned_m_.y = out.pos_target_m.y;
        return out;
    }

    [[nodiscard]] DUpdateOutput d_update_controller() {
        DUpdateInputs inp{};
        inp.dt = dt_s_;
        inp.now_ms = deps_.now_ms;
        inp.ahrs_control_scale_z = deps_.ahrs_control_scale_z;
        inp.estimates.pos_m = estimates_.pos_m.z;
        inp.estimates.vel_ms = estimates_.vel_ms.z;
        inp.offsets = d_offsets_.current;
        inp.terrain = terrain_;
        inp.estimated_accel_d_mss = deps_.estimated_accel_d_mss;
        inp.throttle_lower = deps_.throttle_lower;
        inp.throttle_upper = deps_.throttle_upper;
        inp.throttle_hover = deps_.throttle_hover;
        inp.vibe_comp_enabled = deps_.vibe_comp_enabled;
        inp.vel_max_down_ms = d_limits_.vel_max_down_ms;

        const DUpdateOutput out =
            d_.update_controller(p_pos_d_, pid_vel_d_, pid_accel_d_, inp);

        last_update_d_ticks_ = deps_.ticks;
        vel_target_ned_ms_.z = out.vel_target_ms;
        accel_target_ned_mss_.z = out.accel_target_mss;
        pos_target_ned_m_.z = out.pos_target_m;
        return out;
    }

    [[nodiscard]] bool ne_is_active() const {
        return controller_is_active(deps_.ticks, last_update_ne_ticks_);
    }

    [[nodiscard]] bool d_is_active() const {
        return controller_is_active(deps_.ticks, last_update_d_ticks_);
    }

    void ne_set_control_scale_factor(float factor) { ne_control_scale_factor_ = factor; }

    void input_pos_ned_m(math::Vector3<math::postype_t>& pos_ned_m, float pos_terrain_target_d_m,
                         float terrain_margin_m) {
        InputPosNedPathContext ctx{};
        ctx.dt = dt_s_;
        ctx.pos_estimate_d_m = static_cast<float>(estimates_.pos_m.z);
        ctx.pos_target_d_m = static_cast<float>(pos_target_ned_m_.z);
        ctx.vel_max_ne_ms = ne_limits_.vel_max_ne_ms;
        ctx.ne_limits = ne_limits_;
        ctx.d_limits = d_limits_;
        ::fwcpp::poscontrol::input_pos_ned_m(pos_ned_m, pos_terrain_target_d_m, terrain_margin_m, ne_, d_, terrain_, ctx);
    }

    [[nodiscard]] PosControlNe& ne() { return ne_; }
    [[nodiscard]] PosControlD& d() { return d_; }
    [[nodiscard]] DTerrain& terrain() { return terrain_; }

private:
    PosControlInjectedDeps deps_{};
    PosControlConfig config_{};
    float dt_s_ = 0.0f;

    NedEstimates estimates_{};
    PosControlNe ne_{};
    PosControlD d_{};
    NeLimits ne_limits_{};
    DLimits d_limits_{};
    NeOffsetState ne_offsets_{};
    DOffsetState d_offsets_{};
    DTerrain terrain_{};

    pid::AcP2d p_pos_ne_{};
    pid::AcP1d p_pos_d_{};
    pid::AcPid2d pid_vel_ne_{};
    pid::AcPidBasic pid_vel_d_{};
    pid::AcPid pid_accel_d_{pid::AcPid::Gains{}};
    NeDisturbance ne_disturb_{};

    float ne_control_scale_factor_ = 1.0f;
    std::uint32_t last_update_ne_ticks_ = 0;
    std::uint32_t last_update_d_ticks_ = 0;

    math::Vector3<math::postype_t> pos_target_ned_m_{};
    math::Vector3f vel_target_ned_ms_{};
    math::Vector3f accel_target_ned_mss_{};
    float roll_target_rad_ = 0.0f;
    float pitch_target_rad_ = 0.0f;
    float yaw_target_rad_ = 0.0f;
    float yaw_rate_target_rads_ = 0.0f;
};

}  // namespace fwcpp::poscontrol
