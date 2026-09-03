#pragma once

// Port of libraries/SITL/SIM_Tether.h/.cpp get_forces_on_vehicle.
// Ground-anchored tether: tension when the vehicle exceeds tether length.

#include <cmath>

#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>

namespace fwcpp::sim {

class Tether {
public:
    bool enabled{false};
    float length{10.0f};
    float stiffness{100.0f};
    float damping{10.0f};
    math::Vector3f anchor_ef{};  // NED from origin

    void get_forces_on_vehicle(const math::Vector3f& vehicle_pos_ef, const math::Vector3f& vehicle_vel_ef,
                               math::Vector3f& force_ef) const {
        force_ef.zero();
        if (!enabled || length <= 0.0f) {
            return;
        }
        const math::Vector3f rel = vehicle_pos_ef - anchor_ef;
        const float dist = rel.length();
        if (dist <= length || dist < 1.0e-4f) {
            return;
        }
        const math::Vector3f dir = rel / dist;
        const float stretch = dist - length;
        const float closing = vehicle_vel_ef * dir;
        force_ef = dir * -(stiffness * stretch + damping * closing);
    }
};

}  // namespace fwcpp::sim
