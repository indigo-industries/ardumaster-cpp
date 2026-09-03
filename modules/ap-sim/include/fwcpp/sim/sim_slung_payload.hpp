#pragma once

// Port of libraries/SITL/SIM_SlungPayload.h/.cpp force-on-vehicle math.
// Pendulum payload attached by a cable; force is tension along the cable.

#include <cmath>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>

namespace fwcpp::sim {

class SlungPayload {
public:
    bool enabled{false};
    float payload_mass{1.0f};
    float cable_length{1.0f};
    math::Vector3f payload_pos_ef{};  // NED relative to vehicle
    math::Vector3f payload_vel_ef{};

    void reset(float length_m, float mass_kg) {
        cable_length = length_m;
        payload_mass = mass_kg;
        payload_pos_ef = math::Vector3f(0.0f, 0.0f, length_m);
        payload_vel_ef.zero();
    }

    // Original get_forces_on_vehicle: tension along vehicle->payload if taut.
    void update(const math::Vector3f& vehicle_vel_ef, const math::Vector3f& vehicle_accel_ef, float dt,
                math::Vector3f& force_on_vehicle_ef) {
        force_on_vehicle_ef.zero();
        if (!enabled || payload_mass <= 0.0f || cable_length <= 0.0f || dt <= 0.0f) {
            return;
        }
        // Integrate payload under gravity + tension
        math::Vector3f rel = payload_pos_ef;
        const float dist = rel.length();
        math::Vector3f accel(0.0f, 0.0f, kGravityMss);
        if (dist > cable_length && dist > 1.0e-4f) {
            const math::Vector3f dir = rel / dist;
            const float stretch = dist - cable_length;
            constexpr float kCable = 200.0f;  // N/m-scale stiffness used as force/mass later
            constexpr float kDamp = 20.0f;
            const float rel_speed = payload_vel_ef * dir;
            const float tension = kCable * stretch + kDamp * rel_speed;
            accel -= dir * (tension / payload_mass);
            force_on_vehicle_ef = dir * tension;
        }
        payload_vel_ef += (accel - vehicle_accel_ef) * dt;
        payload_pos_ef += (payload_vel_ef - vehicle_vel_ef) * dt;
    }
};

}  // namespace fwcpp::sim
