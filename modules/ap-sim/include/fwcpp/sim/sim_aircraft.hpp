#pragma once

// Port of libraries/SITL/SIM_Aircraft.h + SIM_Aircraft.cpp core plant
// methods: update_dynamics, update_wind, update_position, update_mag_field_bf,
// ground_behavior, get_air_density, shove/twist/external/clamp, eas airspeed.
// Plane and Copter inherit this class. ADR-0012: explicit dt / time_us.

#include <cmath>
#include <cstdint>
#include <random>

#include <fwcpp/location.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_battery.hpp>
#include <fwcpp/sim/sim_declination.hpp>
#include <fwcpp/sim/sim_sitl.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>
#include <fwcpp/sim/sim_slung_payload.hpp>
#include <fwcpp/sim/sim_tether.hpp>

namespace fwcpp::sim {

enum class GroundBehavior : std::uint8_t {
    kNone = 0,
    kNoMovement = 1,
    kFwdOnly = 2,
    kTailsitter = 3,
};

struct ExternalForceSource {
    math::Vector3f force_ef{};
    float mass_kg{0.0f};
    bool enabled{false};
};

class Aircraft {
public:
    explicit Aircraft(std::uint32_t wind_rng_seed = 20260827U) : wind_rng_(wind_rng_seed) {
        dcm.identity();
        home.lat = -353632621;
        home.lng = 1491652374;
        home.alt = 0;
        origin = home;
        location = home;
        ground_level = 0.0f;
        sitl_params.batt_voltage = 12.6f;
        battery.setup(0.0f, 0.01f, 12.6f, 25.0f);
        battery_voltage = 12.6f;
    }

    void set_start_location(const Location& start_loc, float start_yaw_deg) {
        home = start_loc;
        origin = home;
        position.x = 0.0f;
        position.y = 0.0f;
        home_yaw = start_yaw_deg;
        home_is_set = true;
        location = home;
        ground_level = home.alt * 0.01f;
        dcm.from_euler(0.0f, 0.0f, math::radians(home_yaw));
    }

    [[nodiscard]] float gross_mass() const { return mass + external_payload_mass; }
    [[nodiscard]] float get_air_density(float alt_amsl) const { return get_air_density_for_alt_amsl(alt_amsl); }
    /** TRUE airspeed (m/s). `airspeed` is EQUIVALENT airspeed -- see
     *  update_eas_airspeed(), which divides the airmass-relative speed by
     *  eas2tas -- so anything wanting TAS must multiply it back. Provided
     *  because that distinction is invisible at sea level and silently wrong
     *  above it: eas2tas was pinned at 1.0 for SimPlane until update_position()
     *  was added to its epilogue, and a consumer feeding `airspeed` straight
     *  into an API documented as taking TAS (EkfCore::fuse_airspeed) went
     *  unnoticed the whole time. Prefer this over `airspeed * eas2tas` at call
     *  sites so the intent is stated rather than reconstructed. */
    [[nodiscard]] float true_airspeed() const { return airspeed * eas2tas; }

    [[nodiscard]] const Location& get_location() const { return location; }
    [[nodiscard]] const Location& get_home() const { return home; }
    [[nodiscard]] const Location& get_origin() const { return origin; }
    [[nodiscard]] const math::Matrix3f& get_dcm() const { return dcm; }
    [[nodiscard]] const math::Vector3f& get_gyro() const { return gyro; }
    [[nodiscard]] const math::Vector3f& get_velocity_ef() const { return velocity_ef; }
    [[nodiscard]] const math::Vector3f& get_velocity_air_ef() const { return velocity_air_ef; }
    [[nodiscard]] const math::Vector3f& get_mag_field_bf() const { return mag_bf; }
    [[nodiscard]] float get_battery_voltage() const { return battery_voltage; }
    [[nodiscard]] float get_battery_current() const { return battery_current; }
    [[nodiscard]] std::uint64_t time_us() const { return time_now_us; }

    [[nodiscard]] float hagl() const {
        return (-position.z) + home.alt * 0.01f - ground_level - frame_height;
    }

    [[nodiscard]] bool on_ground() const { return hagl() <= 0.001f; }

    math::Vector3f get_position_relhome() const {
        // NED metres from home, including the origin-shift correction.
        math::Vector3f p = position;
        // home vs origin is accounted by location; for short SITL runs origin==home.
        return p;
    }

    void update_position() {
        location = origin;
        location.offset(position.x, position.y);
        location.alt = static_cast<std::int32_t>(home.alt - position.z * 100.0f);
    }

    void update_mag_field_bf() {
        float intensity = 0.0f;
        float declination = 0.0f;
        float inclination = 0.0f;
        get_mag_field_ef(location.lat * 1.0e-7f, location.lng * 1.0e-7f, intensity, declination, inclination);
        math::Vector3f mag_ef(1.0e3f * intensity, 0.0f, 0.0f);
        math::Matrix3f R;
        R.from_euler(0.0f, -math::radians(inclination), math::radians(declination));
        mag_ef = R * mag_ef;

        const float frame_height_agl = std::fmax((-position.z) + home.alt * 0.01f - ground_level, 0.0f);
        float anomaly_scaler = (sitl_params.mag_anomaly_hgt / (frame_height_agl + sitl_params.mag_anomaly_hgt));
        anomaly_scaler = anomaly_scaler * anomaly_scaler * anomaly_scaler;
        mag_ef += sitl_params.mag_anomaly_ned * anomaly_scaler;
        mag_bf = dcm.transposed() * mag_ef;
        mag_bf += sitl_params.mag_mot * battery_current;
    }

    double rand_normal(double mean, double stddev) {
        return mean + stddev * wind_normal_dist_(wind_rng_);
    }

    void update_wind(const SitlInput& input) {
        const float speed = input.wind.speed;
        const float dir = input.wind.direction;
        const float dir_z = input.wind.dir_z;
        wind_ef = math::Vector3f(std::cos(math::radians(dir)) * std::cos(math::radians(dir_z)),
                                 std::sin(math::radians(dir)) * std::cos(math::radians(dir_z)),
                                 std::sin(math::radians(dir_z))) *
                  speed;

        const float wind_turb = input.wind.turbulence * 10.0f;
        const float iir_coef = 0.98f;
        if (wind_turb > 0.0f && !on_ground()) {
            turbulence_azimuth = turbulence_azimuth + (2.0f * static_cast<float>(wind_azimuth_step_dist_(wind_rng_)));
            turbulence_horizontal_speed = static_cast<float>(
                turbulence_horizontal_speed * iir_coef + wind_turb * rand_normal(0, 1) * (1.0 - iir_coef));
            turbulence_vertical_speed = static_cast<float>(
                turbulence_vertical_speed * iir_coef + wind_turb * rand_normal(0, 1) * (1.0 - iir_coef));
            wind_ef += math::Vector3f(std::cos(math::radians(turbulence_azimuth)) * turbulence_horizontal_speed,
                                      std::sin(math::radians(turbulence_azimuth)) * turbulence_horizontal_speed,
                                      turbulence_vertical_speed);
        }
        wind_ef = math::Vector3f(-wind_ef.x, -wind_ef.y, -wind_ef.z);
    }

    void update_eas_airspeed() {
        airspeed = velocity_air_ef.length() / eas2tas;
        airspeed_pitot = airspeed;
        const float pitot_aoa =
            std::atan2(std::sqrt(velocity_air_bf.y * velocity_air_bf.y + velocity_air_bf.z * velocity_air_bf.z),
                       velocity_air_bf.x);
        const float max_pitot_aoa = math::radians(20.0f);
        if (pitot_aoa > math::radians(90.0f)) {
            airspeed_pitot = 0.0f;
        } else if (pitot_aoa > max_pitot_aoa) {
            const float gain_factor = (0.5f * math::pi_constant()) / (math::radians(90.0f) - max_pitot_aoa);
            airspeed_pitot *= std::cos((pitot_aoa - max_pitot_aoa) * gain_factor);
        }
    }

    void apply_ground_behavior(float dt) {
        if (!on_ground()) {
            return;
        }
        position.z = -(ground_level + frame_height - home.alt * 0.01f);
        switch (ground_behavior) {
        case GroundBehavior::kNone:
            break;
        case GroundBehavior::kNoMovement: {
            float r = 0.0f;
            float p = 0.0f;
            float y = 0.0f;
            dcm.to_euler(&r, &p, &y);
            dcm.from_euler(0.0f, 0.0f, y);
            velocity_ef.x = 0.0f;
            velocity_ef.y = 0.0f;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            break;
        }
        case GroundBehavior::kFwdOnly: {
            float r = 0.0f;
            float p = 0.0f;
            float y = 0.0f;
            dcm.to_euler(&r, &p, &y);
            if (velocity_ef.length() < 5.0f) {
                p = 0.0f;
            } else {
                p = std::fmax(p, 0.0f);
            }
            dcm.from_euler(0.0f, p, y);
            math::Vector3f v_bf = dcm.transposed() * velocity_ef;
            v_bf.y = 0.0f;
            if (v_bf.x < 0.0f) {
                v_bf.x = 0.0f;
            }
            velocity_ef = dcm * v_bf;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            break;
        }
        case GroundBehavior::kTailsitter: {
            math::Matrix3f rot;
            rot.from_euler(0.0f, math::radians(270.0f), 0.0f);
            float r = 0.0f;
            float p = 0.0f;
            float y = 0.0f;
            math::Matrix3f tmp = dcm * rot;
            tmp.to_euler(&r, &p, &y);
            dcm.from_euler(0.0f, 0.0f, y);
            rot.from_euler(0.0f, math::radians(90.0f), 0.0f);
            dcm = dcm * rot;
            velocity_ef.x = 0.0f;
            velocity_ef.y = 0.0f;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            break;
        }
        }
        (void)dt;
    }

    void update_dynamics(const math::Vector3f& rot_accel, float dt) {
        eas2tas = get_eas2tas_for_alt_amsl(location.alt * 0.01f);
        air_density = get_air_density_for_alt_amsl(location.alt * 0.01f);

        gyro += rot_accel * dt;
        gyro.x = math::constrain_value(gyro.x, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.y = math::constrain_value(gyro.y, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.z = math::constrain_value(gyro.z, -math::radians(2000.0f), math::radians(2000.0f));

        const float accel_limit = 64.0f * kGravityMss;
        accel_body.x = math::constrain_value(accel_body.x, -accel_limit, accel_limit);
        accel_body.y = math::constrain_value(accel_body.y, -accel_limit, accel_limit);
        accel_body.z = math::constrain_value(accel_body.z, -accel_limit, accel_limit);

        dcm.rotate(gyro * dt);
        dcm.normalize();

        math::Vector3f accel_earth = dcm * accel_body;
        accel_earth += math::Vector3f(0.0f, 0.0f, kGravityMss);

        if (on_ground() && accel_earth.z > 0.0f) {
            accel_earth.z = 0.0f;
        }

        accel_body = dcm.transposed() * (accel_earth + math::Vector3f(0.0f, 0.0f, -kGravityMss));

        velocity_ef += accel_earth * dt;
        const bool was_on_ground = on_ground();
        position += velocity_ef * dt;

        velocity_air_ef = velocity_ef - wind_ef;
        velocity_air_bf = dcm.transposed() * velocity_air_ef;
        update_eas_airspeed();

        if (on_ground()) {
            if (!was_on_ground) {
                last_ground_contact_ms = static_cast<std::uint32_t>(time_now_us / 1000U);
            }
            apply_ground_behavior(dt);
            if (on_ground() && velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
        }
    }

    void add_shove_forces(math::Vector3f& /*rot_accel*/, math::Vector3f& body_accel, std::uint32_t now_ms) {
        if (sitl_params.shove.t == 0) {
            return;
        }
        if (sitl_params.shove.start_ms == 0) {
            sitl_params.shove.start_ms = now_ms;
        }
        if (now_ms - sitl_params.shove.start_ms < sitl_params.shove.t) {
            body_accel.x += sitl_params.shove.x;
            body_accel.y += sitl_params.shove.y;
            body_accel.z += sitl_params.shove.z;
        } else {
            sitl_params.shove.start_ms = 0;
            sitl_params.shove.t = 0;
        }
    }

    void add_twist_forces(math::Vector3f& rot_accel, std::uint32_t now_ms) {
        if (sitl_params.gnd_behav != -1) {
            ground_behavior = static_cast<GroundBehavior>(sitl_params.gnd_behav);
        }
        if (sitl_params.twist.t == 0) {
            return;
        }
        if (sitl_params.twist.start_ms == 0) {
            sitl_params.twist.start_ms = now_ms;
        }
        if (now_ms - sitl_params.twist.start_ms < sitl_params.twist.t) {
            rot_accel.x += sitl_params.twist.x;
            rot_accel.y += sitl_params.twist.y;
            rot_accel.z += sitl_params.twist.z;
        } else {
            sitl_params.twist.start_ms = 0;
            sitl_params.twist.t = 0;
        }
    }

    void add_external_forces(math::Vector3f& body_accel) {
        math::Vector3f total_force;
        if (slung_payload.enabled) {
            math::Vector3f fv;
            slung_payload.update(velocity_ef, math::Vector3f(0.0f, 0.0f, 0.0f), 0.0025f, fv);
            total_force += fv;
            external_payload_mass = slung_payload.payload_mass;
        }
        if (tether.enabled) {
            math::Vector3f fv;
            tether.get_forces_on_vehicle(position, velocity_ef, fv);
            total_force += fv;
        }
        if (mass > 0.0f) {
            body_accel += dcm.transposed() * total_force / mass;
        }
    }

    bool clamp_active(const SitlInput& input) {
        if (sitl_params.clamp_ch < 1) {
            return currently_clamped_;
        }
        const std::uint32_t clamp_idx = static_cast<std::uint32_t>(sitl_params.clamp_ch - 1);
        if (clamp_idx >= kSitlServoChannels) {
            return currently_clamped_;
        }
        const std::uint16_t servo_pos = input.servos[clamp_idx];
        bool new_clamped = currently_clamped_;
        if (servo_pos == 0) {
            // invalid
        } else if (servo_pos < 1200) {
            new_clamped = false;
            grab_attempted_ = false;
        } else if (servo_pos > 1800 && !grab_attempted_) {
            const float distance_from_home = get_position_relhome().length();
            if (distance_from_home < 0.5f) {
                new_clamped = true;
            }
            grab_attempted_ = true;
        }
        currently_clamped_ = new_clamped;
        return currently_clamped_;
    }

    void time_advance(float dt) {
        time_now_us += static_cast<std::uint64_t>(dt * 1.0e6f + 0.5f);
    }

    math::Matrix3f dcm{};
    math::Vector3f gyro{};
    math::Vector3f accel_body{};
    math::Vector3f velocity_ef{};
    math::Vector3f velocity_air_ef{};
    math::Vector3f velocity_air_bf{};
    math::Vector3f position{};
    math::Vector3f wind_ef{};
    math::Vector3f mag_bf{};

    float airspeed{0.0f};
    float airspeed_pitot{0.0f};
    float air_density{kSslAirDensity};
    float eas2tas{1.0f};
    float mass{1.0f};
    float external_payload_mass{0.0f};
    float battery_voltage{12.6f};
    float battery_current{0.0f};
    float battery_temperature_degC{0.0f};
    float ground_level{0.0f};
    float frame_height{0.0f};
    float home_yaw{0.0f};
    bool home_is_set{false};
    GroundBehavior ground_behavior{GroundBehavior::kNone};

    float turbulence_azimuth{0.0f};
    float turbulence_horizontal_speed{0.0f};
    float turbulence_vertical_speed{0.0f};

    Location origin{};
    Location home{};
    Location location{};

    SitlParams sitl_params{};
    Battery battery{};
    SlungPayload slung_payload{};
    Tether tether{};
    float rpm[32]{};

    std::uint64_t time_now_us{0};

    std::mt19937 wind_rng_;
    std::normal_distribution<double> wind_normal_dist_{0.0, 1.0};
    std::uniform_real_distribution<float> wind_azimuth_step_dist_{0.0f, 360.0f};

private:
    bool currently_clamped_{false};
    bool grab_attempted_{false};
    std::uint32_t last_ground_contact_ms{0};
};

}  // namespace fwcpp::sim
