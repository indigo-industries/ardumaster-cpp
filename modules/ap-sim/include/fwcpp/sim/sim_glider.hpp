#pragma once

// Port of libraries/SITL/SIM_Glider.h/.cpp (high-altitude balloon drop).
#include <cmath>
#include <cstdint>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class SimGlider : public Aircraft {
public:
    struct Model {
        float mass = 9.07441f;
        float Sref = 0.92762f;
        float refSpan = 1.827411f;
        float refChord = 0.507614f;
        float IXX = 0.234f;
        float IYY = 1.85f;
        float IZZ = 2.04f;
        float CN2 = -0.5771f, CN1 = 3.9496f, CN0 = 0;
        float CA2 = -1.6809f, CA1 = -0.0057f, CA0 = 0.0150f;
        float CY2 = -3.342f, CY1 = 0.0227f, CY0 = -0.4608f;
        float Cl2 = 0.2888f, Cl1 = -0.8518f, Cl0 = -0.0491f;
        float Cm2 = 0.099f, Cm1 = -0.6506f, Cm0 = -0.0005f;
        float Cn2 = 0.0057f, Cn1 = -0.0101f, Cn0 = 0.1744f;
        float Cmq = -6.1866f;
        float Clp2 = 0.156f, Clp1 = 0.0129f, Clp0 = -0.315f;
        float Clr2 = -0.0284f, Clr1 = 0.2641f, Clr0 = 0.0343f;
        float Cnp2 = 0.0199f, Cnp1 = -0.315f, Cnp0 = -0.013f;
        float Cnr2 = 0.1297f, Cnr1 = 0.0343f, Cnr0 = -0.264f;
        float elevatorDeflectionLimitDeg = -12.5f;
        float deltaCNperRadianElev = -0.7f;
        float deltaCAperRadianElev = 0.12f;
        float deltaCmperRadianElev = 1.39f;
        float deltaCYperRadianElev = 0;
        float deltaClperRadianElev = 0;
        float deltaCnperRadianElev = 0;
        float rudderDeflectionLimitDeg = 18.0f;
        float deltaCNperRadianRud = 0;
        float deltaCAperRadianRud = 0.058f;
        float deltaCmperRadianRud = 0;
        float deltaCYperRadianRud = 0.31f;
        float deltaClperRadianRud = 0.038f;
        float deltaCnperRadianRud = -0.174f;
        float aileronDeflectionLimitDeg = 15.5f;
        float deltaCNperRadianAil = 0;
        float deltaCAperRadianAil = 0.016f;
        float deltaCmperRadianAil = 0;
        float deltaCYperRadianAil = -0.015f;
        float deltaClperRadianAil0 = 0.09191f;
        float deltaClperRadianAil1 = 0.0001f;
        float deltaClperRadianAil2 = -0.08645f;
        float deltaCnperRadianAil0 = 0.00789f;
        float deltaCnperRadianAil1 = 0.00773f;
        float deltaCnperRadianAil2 = -0.01162f;
        float alphaRadMax = 0.209f;
        float betaRadMax = 0.209f;
        float tetherLength = 50.0f;
        float tetherPogoFreq = 2.0f;
    } model;

    enum class CarriageState : std::uint8_t {
        kNone = 0, kWaitingForPickup = 1, kWaitingForRelease = 2, kPreRelease = 3, kReleased = 4
    };

    explicit SimGlider(const char* = "glider") {
        mass = model.mass;
        ground_behavior = GroundBehavior::kNoMovement;
        carriage_state = CarriageState::kWaitingForPickup;
        balloon_position = math::Vector3f(0.0f, 0.0f, -45.0f);
    }

    float balloon_burst_amsl{30000.0f};
    float balloon_rate{5.5f};
    CarriageState carriage_state{CarriageState::kWaitingForPickup};
    bool armed{true};

    [[nodiscard]] bool on_ground() const {
        switch (carriage_state) {
        case CarriageState::kNone:
        case CarriageState::kReleased:
            return hagl() <= 0.001f;
        default:
            return false;
        }
    }

    void update(const SitlInput& input, float dt) {
        math::Vector3f rot_accel;
        update_wind(input);
        calculate_forces(input, rot_accel, accel_body, dt);
        if (carriage_state == CarriageState::kWaitingForPickup) {
            accel_body = dcm.transposed() * math::Vector3f(0.0f, 0.0f, -kGravityMss);
            velocity_ef.zero();
            gyro.zero();
            dcm.from_euler(0.0f, math::radians(-80.0f), math::radians(home_yaw));
        } else {
            update_dynamics(rot_accel, dt);
        }
        add_external_forces(accel_body);
        update_position();
        time_advance(dt);
        update_mag_field_bf();
        battery.consume_energy(battery_current, time_now_us);
    }

private:
    static float servo_angle(std::uint16_t pwm) {
        return math::constrain_value((static_cast<float>(pwm) - 1500.0f) / 400.0f, -1.0f, 1.0f);
    }
    static float servo_range(std::uint16_t pwm) {
        return math::constrain_value((static_cast<float>(pwm) - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    }
    static float sq(float x) { return x * x; }

    float alpharad{0.0f};
    float betarad{0.0f};
    math::Vector3f balloon_velocity{};
    math::Vector3f balloon_position{0.0f, 0.0f, -45.0f};

    math::Vector3f getTorque(float ia, float ie, float ir) const;
    math::Vector3f getForce(float ia, float ie, float ir);
    bool update_balloon(float balloon, math::Vector3f& force, math::Vector3f& rot_accel);
    void calculate_forces(const SitlInput& input, math::Vector3f& rot_accel, math::Vector3f& body_accel, float dt);
};

inline math::Vector3f SimGlider::getTorque(float inputAileron, float inputElevator, float inputRudder) const {
    const auto& m = model;
    const double qPa = 0.5 * air_density * sq(velocity_air_bf.length());
    const float aileron_rad = inputAileron * math::radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * math::radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * math::radians(m.rudderDeflectionLimitDeg);
    const float tas = std::fmax(airspeed * eas2tas, 1.0f);
    float Cl = (m.Cl2 * sq(alpharad) + m.Cl1 * alpharad + m.Cl0) * betarad;
    float Cm = m.Cm2 * sq(alpharad) + m.Cm1 * alpharad + m.Cm0;
    float Cn = (m.Cn2 * sq(alpharad) + m.Cn1 * alpharad + m.Cn0) * betarad;
    Cl += m.deltaClperRadianElev * elevator_rad; Cm += m.deltaCmperRadianElev * elevator_rad; Cn += m.deltaCnperRadianElev * elevator_rad;
    Cl += m.deltaClperRadianRud * rudder_rad; Cm += m.deltaCmperRadianRud * rudder_rad; Cn += m.deltaCnperRadianRud * rudder_rad;
    Cl += (m.deltaClperRadianAil2 * sq(alpharad) + m.deltaClperRadianAil1 * alpharad + m.deltaClperRadianAil0) * aileron_rad;
    Cm += m.deltaCmperRadianAil * aileron_rad;
    Cn += (m.deltaCnperRadianAil2 * sq(alpharad) + m.deltaCnperRadianAil1 * alpharad + m.deltaCnperRadianAil0) * aileron_rad;
    const float Clp = m.Clp2 * sq(alpharad) + m.Clp1 * alpharad + m.Clp0;
    const float Clr = m.Clr2 * sq(alpharad) + m.Clr1 * alpharad + m.Clr0;
    const float Cnp = m.Cnp2 * sq(alpharad) + m.Cnp1 * alpharad + m.Cnp0;
    const float Cnr = m.Cnr2 * sq(alpharad) + m.Cnr1 * alpharad + m.Cnr0;
    math::Vector3f pqr_norm = gyro;
    pqr_norm.x *= 0.5f * m.refSpan / tas;
    pqr_norm.y *= 0.5f * m.refChord / tas;
    pqr_norm.z *= 0.5f * m.refSpan / tas;
    Cl += pqr_norm.x * Clp; Cl += pqr_norm.z * Clr; Cn += pqr_norm.x * Cnp; Cn += pqr_norm.z * Cnr; Cm += pqr_norm.y * m.Cmq;
    return math::Vector3f(static_cast<float>(Cl * qPa * m.Sref * m.refSpan) / m.IXX,
                          static_cast<float>(Cm * qPa * m.Sref * m.refChord) / m.IYY,
                          static_cast<float>(Cn * qPa * m.Sref * m.refSpan) / m.IZZ);
}

inline math::Vector3f SimGlider::getForce(float inputAileron, float inputElevator, float inputRudder) {
    const auto& m = model;
    const float aileron_rad = inputAileron * math::radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * math::radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * math::radians(m.rudderDeflectionLimitDeg);
    const double qPa = 0.5 * air_density * sq(velocity_air_bf.length());
    float CA = m.CA2 * sq(alpharad) + m.CA1 * alpharad + m.CA0;
    float CY = (m.CY2 * sq(alpharad) + m.CY1 * alpharad + m.CY0) * betarad;
    float CN = m.CN2 * sq(alpharad) + m.CN1 * alpharad + m.CN0;
    CN += m.deltaCNperRadianElev * elevator_rad; CA += m.deltaCAperRadianElev * elevator_rad; CY += m.deltaCYperRadianElev * elevator_rad;
    CN += m.deltaCNperRadianRud * rudder_rad; CA += m.deltaCAperRadianRud * rudder_rad; CY += m.deltaCYperRadianRud * rudder_rad;
    CN += m.deltaCNperRadianAil * aileron_rad; CA += m.deltaCAperRadianAil * aileron_rad; CY += m.deltaCYperRadianAil * aileron_rad;
    return math::Vector3f(static_cast<float>(-CA * qPa * m.Sref), static_cast<float>(CY * qPa * m.Sref),
                          static_cast<float>(-CN * qPa * m.Sref));
}

inline bool SimGlider::update_balloon(float balloon, math::Vector3f& force, math::Vector3f& rot_accel) {
    if (balloon_rate < 0.0f) { carriage_state = CarriageState::kReleased; }
    if (!armed) { return false; }
    if (carriage_state == CarriageState::kNone || carriage_state == CarriageState::kReleased) { return false; }
    const math::Vector3f tether_pos_bf(-1.0f, 0.0f, 0.0f);
    const float omega = model.tetherPogoFreq * 2.0f * static_cast<float>(M_PI);
    const float zeta = 0.7f;
    const float tether_stiffness = model.mass * omega * omega;
    const float tether_damping = 2.0f * zeta * omega / model.mass;
    const math::Vector3f relative_position = balloon_position - (position + (dcm * tether_pos_bf));
    const float separation_distance = relative_position.length();
    if (separation_distance < 1.0e-6f) {
        rot_accel.zero();
        force = dcm.transposed() * math::Vector3f(0.0f, 0.0f, -kGravityMss * model.mass);
        return true;
    }
    const math::Vector3f tether_unit_vec_ef = relative_position.normalized();
    const math::Vector3f attachment_velocity_ef = velocity_ef + dcm * (gyro % tether_pos_bf);
    const math::Vector3f relative_velocity = balloon_velocity - attachment_velocity_ef;
    const float separation_speed = relative_velocity * tether_unit_vec_ef;
    float tension_force = std::fmax(0.0f, (separation_distance - model.tetherLength) * tether_stiffness);
    if (tension_force > 0.0f) {
        tension_force += math::constrain_value(separation_speed * tether_damping, 0.0f, 0.05f * tension_force);
    }
    if (carriage_state == CarriageState::kWaitingForPickup && tension_force > 1.2f * model.mass * kGravityMss && balloon > 0.01f) {
        carriage_state = CarriageState::kWaitingForRelease;
    }
    if (carriage_state == CarriageState::kWaitingForRelease || carriage_state == CarriageState::kPreRelease) {
        const math::Vector3f tension_force_vector_bf = dcm.transposed() * (tether_unit_vec_ef * tension_force);
        force = tension_force_vector_bf;
        math::Vector3f aero_force_bf(0.0f, 0.2f * velocity_air_bf.y * std::fabs(velocity_air_bf.y),
                                     velocity_air_bf.z * std::fabs(velocity_air_bf.z));
        aero_force_bf *= air_density * model.Sref;
        force -= aero_force_bf;
        const math::Vector3f tm = tether_pos_bf % tension_force_vector_bf;
        rot_accel = math::Vector3f(tm.x / model.IXX, tm.y / model.IYY, tm.z / model.IZZ);
        rot_accel -= gyro * (0.5f * air_density);
    } else {
        rot_accel.zero();
        force = dcm.transposed() * math::Vector3f(0.0f, 0.0f, -kGravityMss * model.mass);
    }
    return true;
}

inline void SimGlider::calculate_forces(const SitlInput& input, math::Vector3f& rot_accel, math::Vector3f& body_accel, float dt) {
    const float aileron = 0.5f * (servo_angle(input.servos[1]) + servo_angle(input.servos[4]));
    const float elevator = servo_angle(input.servos[2]);
    const float rudder = servo_angle(input.servos[3]);
    const float balloon = std::fmax(0.0f, servo_range(input.servos[5]));
    const float balloon_cut = servo_range(input.servos[9]);
    if (carriage_state == CarriageState::kWaitingForRelease) {
        balloon_velocity = math::Vector3f(-wind_ef.x, -wind_ef.y, -wind_ef.z - balloon_rate * balloon);
        balloon_position += balloon_velocity * dt;
        const float height_amsl = 0.01f * static_cast<float>(home.alt) - position.z;
        if (height_amsl > balloon_burst_amsl || balloon_cut > 0.8f) { carriage_state = CarriageState::kPreRelease; }
    } else if (carriage_state == CarriageState::kPreRelease) {
        balloon_velocity *= 0.999f;
        balloon_position += balloon_velocity * dt;
        if (balloon_velocity.length() < 0.5f) { carriage_state = CarriageState::kReleased; }
    } else if (carriage_state == CarriageState::kWaitingForPickup) {
        balloon_velocity = math::Vector3f(0.0f, 0.0f, -balloon_rate * balloon);
        balloon_position += balloon_velocity * dt;
    }
    alpharad = math::constrain_value(std::atan2(velocity_air_bf.z, velocity_air_bf.x), -model.alphaRadMax, model.alphaRadMax);
    betarad = math::constrain_value(std::atan2(velocity_air_bf.y, velocity_air_bf.x), -model.betaRadMax, model.betaRadMax);
    math::Vector3f force;
    if (!update_balloon(balloon, force, rot_accel)) {
        force = getForce(aileron, elevator, rudder);
        rot_accel = getTorque(aileron, elevator, rudder);
    }
    accel_body = force / model.mass;
    if (on_ground()) {
        const math::Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
    accel_body.x = math::constrain_value(accel_body.x, -16.0f * kGravityMss, 16.0f * kGravityMss);
    accel_body.y = math::constrain_value(accel_body.y, -16.0f * kGravityMss, 16.0f * kGravityMss);
    accel_body.z = math::constrain_value(accel_body.z, -16.0f * kGravityMss, 16.0f * kGravityMss);
    body_accel = accel_body;
}

}  // namespace fwcpp::sim
