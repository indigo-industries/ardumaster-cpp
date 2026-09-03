#pragma once

// CCP-027: vector half helpers for NE position control (Rust ap-math/control.rs).
// Upstream AP_Math/control.cpp — sqrt_controller_xy, limit_accel_xy (Rust parity).

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>

namespace fwcpp::math {

[[nodiscard]] inline Vector2f sqrt_controller_xy(Vector2f error, float p, float second_ord_lim, float dt) {
    const float error_length = error.length();
    if (!is_positive(error_length)) {
        return Vector2f{};
    }
    const float correction_length = sqrt_controller(error_length, p, second_ord_lim, dt);
    return error * (correction_length / error_length);
}

[[nodiscard]] inline bool limit_accel_xy(Vector2f vel, Vector2f& accel, float accel_max) {
    if (!is_positive(accel_max)) {
        return false;
    }
    if (accel.length_squared() <= accel_max * accel_max) {
        return false;
    }

    if (vel.is_zero()) {
        accel.limit_length(accel_max);
        return true;
    }

    const Vector2f vel_unit = vel.normalized();
    float accel_dir = vel_unit * accel;
    Vector2f accel_cross = accel - vel_unit * accel_dir;

    if (accel_cross.limit_length(accel_max)) {
        accel_dir = 0.0f;
    } else {
        const float accel_max_dir = safe_sqrt(accel_max * accel_max - accel_cross.length_squared());
        accel_dir = constrain_value(accel_dir, -accel_max_dir, accel_max_dir);
    }

    accel = accel_cross + vel_unit * accel_dir;
    return true;
}

}  // namespace fwcpp::math
