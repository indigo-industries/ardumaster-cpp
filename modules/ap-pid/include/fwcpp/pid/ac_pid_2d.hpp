#pragma once

// Port of AC_PID_2D — CCP-027 slice 2 (NE velocity loop). Rust: ap-pid/pid_2d.rs.

#include <cmath>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>

namespace fwcpp::pid {

inline constexpr float kNeVelP = 1.0f;
inline constexpr float kNeVelI = 0.5f;
inline constexpr float kNeVelD = 0.0f;
inline constexpr float kNeVelImax = 10.0f;
inline constexpr float kNeVelFiltHz = 5.0f;
inline constexpr float kNeVelFiltDHz = 5.0f;

class AcPid2d {
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float kff = 0.0f;
    float imax = 0.0f;
    float filt_e_hz = 0.0f;
    float filt_d_hz = 0.0f;

    [[nodiscard]] static AcPid2d with_gains(float kp_in, float ki_in, float kd_in, float kff_in,
                                            float imax_in, float filt_e_hz_in, float filt_d_hz_in) {
        AcPid2d p;
        p.kp = kp_in;
        p.ki = ki_in;
        p.kd = kd_in;
        p.kff = kff_in;
        p.imax = std::fabs(imax_in);
        p.filt_e_hz = filt_e_hz_in;
        p.filt_d_hz = filt_d_hz_in;
        p.reset_filter_ = true;
        return p;
    }

    [[nodiscard]] static AcPid2d ne_velocity() {
        return with_gains(kNeVelP, kNeVelI, kNeVelD, 0.0f, kNeVelImax, kNeVelFiltHz, kNeVelFiltDHz);
    }

    [[nodiscard]] math::Vector2f error() const { return error_; }
    [[nodiscard]] math::Vector2f integrator() const { return integrator_; }

    void reset_filter() { reset_filter_ = true; }
    void reset_i() { integrator_.zero(); }

    void set_integrator(math::Vector2f i) {
        integrator_ = i;
        integrator_.limit_length(imax);
    }

    [[nodiscard]] math::Vector2f update_all(math::Vector2f target, math::Vector2f measurement,
                                            float dt, math::Vector2f limit) {
        if (target.is_nan() || target.is_inf() || measurement.is_nan() || measurement.is_inf()) {
            return math::Vector2f{};
        }

        target_ = target;

        if (reset_filter_) {
            reset_filter_ = false;
            error_ = target_ - measurement;
            derivative_.zero();
        } else {
            const math::Vector2f error_last = error_;
            error_ += ((target_ - measurement) - error_) * filt_e_alpha(dt);
            if (math::is_positive(dt)) {
                const math::Vector2f derivative = (error_ - error_last) / dt;
                derivative_ += (derivative - derivative_) * filt_d_alpha(dt);
            }
        }

        update_i(dt, limit);

        return error_ * kp + integrator_ + derivative_ * kd + target_ * kff;
    }

private:
    math::Vector2f target_{};
    math::Vector2f error_{};
    math::Vector2f derivative_{};
    math::Vector2f integrator_{};
    bool reset_filter_ = true;

    void update_i(float dt, math::Vector2f limit) {
        const math::Vector2f delta_integrator = (error_ * ki) * dt;
        const float integrator_length = integrator_.length();
        integrator_ += delta_integrator;
        if (math::is_positive(delta_integrator * limit)) {
            integrator_.limit_length(integrator_length);
        }
        integrator_.limit_length(imax);
    }

    [[nodiscard]] float filt_e_alpha(float dt) const {
        return math::calc_lowpass_alpha_dt(dt, filt_e_hz);
    }

    [[nodiscard]] float filt_d_alpha(float dt) const {
        return math::calc_lowpass_alpha_dt(dt, filt_d_hz);
    }
};

}  // namespace fwcpp::pid
