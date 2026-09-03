#pragma once

// Port of AP_L1_Control/AP_L1_Control.h + AP_L1_Control.cpp. CPP-017,
// slice 1. Written by Brandon Jones 2013, modified by Paul Riseborough.
//
// AP_AHRS&/AP_TECS* REPLACED WITH AN EXPLICIT L1Inputs STRUCT: upstream
// reaches into a stored `AP_AHRS&` (get_location, groundspeed_vector,
// get_yaw_rad, get_pitch_rad, yaw_sensor, get_EAS2TAS) and an optional
// `const AP_TECS*` (get_target_airspeed) on every update. Neither AP_AHRS
// nor AP_TECS exist in this port yet - building either is a substantial
// sub-effort of its own, and L1 does not need to wait on them: it only
// needs their OUTPUT values, not their internals. Matches this port's
// standing pattern (constrain_value's InternalError*, SlewLimiter's/
// AC_PID's now_ms) of taking external state as an explicit parameter
// instead of reaching for it - here scaled up to a small struct because
// there are several such values instead of one. `target_airspeed` and
// `location_valid` fold in what upstream gets from a possibly-null _tecs
// pointer and a possibly-failing _ahrs.get_location() call respectively.
//
// AP_HAL::micros() folded into L1Inputs.now_us for the same reason.
//
// AP_Float REPLACED WITH PLAIN float for the tunable gains (L1_period,
// L1_damping, L1_xtrack_i_gain, loiter_bank_limit) - same precedent as
// AC_PID's Gains struct, no AP_Param in this port yet.
//
// SLICE 2 adds update_loiter, update_heading_hold, update_level_flight -
// the three update_* variants deferred from slice 1. update_loiter needed
// Location::same_loc_as (added to CPP-011's Location alongside this) for
// its "keep _WPcircle latched" check, and math::cd_to_rad (added to
// CPP-004's scalar module alongside this) for update_heading_hold's
// centidegrees-to-radians conversion - upstream had both already; this
// port didn't need them until now.
//
// L1Inputs gained now_ms (upstream: AP_HAL::millis(), used only by
// update_loiter's 200ms latch window) alongside the pre-existing now_us
// (AP_HAL::micros(), used by update_waypoint's dt calc) - same explicit-
// parameter treatment, not derived from one another since upstream itself
// reads two independent clocks.
//
// SLICE BOUNDARY (now closed): nav_roll_cd, lateral_acceleration,
// nav_bearing_cd, bearing_error_cd, target_bearing_cd, turn_distance (both
// overloads), loiter_radius, reached_loiter_target, update_waypoint,
// update_loiter, update_heading_hold, update_level_flight - every public
// member of upstream AP_L1_Control is now ported.
//
// LITERAL SAFETY: GRAVITY_MSS (9.80665f) and every other literal touched
// in this slice are already explicitly float-suffixed upstream - nothing
// here needed the compiled-.cpp treatment scalar.cpp's wrap_* family or
// Location::get_bearing needed.
//
// POST-PIN BACKPORT (update_loiter): the loiter PD wrong-way clamp carries
// upstream master 29d590b ("limit loiter PD when inside radius"), which is
// NOT in Plane-4.7.0. See the comment at the clamp itself for the failure
// it fixes and the measurement behind it. Everything else in this file
// still matches the 4.7.0 tag line-for-line (re-verified against both the
// tag and master when the backport was made).
//
// CPP-048 ADDENDUM (bottom of this file, after the L1Control class): a
// real top-level AP_Param Info[] table for L1Control::Gains, phase 2e of
// the AP_Param vehicle-integration effort CPP-043 started. See that
// addendum's own banner for the full design rationale and upstream
// citations. CPP-048 also found a pre-existing port bug in this file's
// own Gains::l1_period default (25.0f instead of upstream's real 17.0f)
// - CPP-050 fixed it (see Gains::l1_period below and that addendum's own
// updated note).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/param/group_info.hpp> // param::Info (CPP-048)
#include <fwcpp/param/native_value.hpp> // set_native_value/native_cast_to_float (CPP-048)
#include <fwcpp/param/param.hpp> // param::VarType, ParamHeader, set_key (CPP-048)
#include <fwcpp/param/persistence.hpp> // type_size/load_raw/save_raw/should_skip_save (CPP-048)
#include <fwcpp/param/storage.hpp> // StorageAccess (CPP-048)

namespace fwcpp::nav {

inline constexpr float kGravityMss = 9.80665f;

// Everything L1 needs from the AHRS/TECS for one update - see file banner.
struct L1Inputs {
    Location current_loc;
    bool location_valid = false; // upstream: _ahrs.get_location() return value
    math::Vector2f groundspeed_vector;
    float yaw_rad = 0.0f;         // upstream: _ahrs.get_yaw_rad()
    std::int32_t yaw_sensor_cd = 0; // upstream: _ahrs.yaw_sensor
    float pitch_rad = 0.0f;       // upstream: _ahrs.get_pitch_rad()
    float eas2tas = 1.0f;         // upstream: _ahrs.get_EAS2TAS()
    float target_airspeed = 0.0f; // upstream: _tecs->get_target_airspeed(), 0 if _tecs is null
    std::uint32_t now_us = 0;     // upstream: AP_HAL::micros(), used by update_waypoint's dt
    std::uint32_t now_ms = 0;     // upstream: AP_HAL::millis(), used by update_loiter's latch window
};

class L1Control {
public:
    struct Gains {
        float l1_period = 17.0f;    // upstream NAVL1_PERIOD default (CPP-050: was wrongly 25.0f
                                     // since CPP-017; real default verified directly against
                                     // AP_L1_Control.cpp's var_info[] AP_GROUPINFO("PERIOD", ...,
                                     // 17) at the pinned plane-4.7.0 tag)
        float l1_damping = 0.75f;   // upstream NAVL1_DAMPING default
        float l1_xtrack_i_gain = 0.02f; // upstream NAVL1_XTRACK_I default (fixed-wing)
        float loiter_bank_limit = 0.0f; // upstream NAVL1_LIM_BANK default
    };

    explicit L1Control(const Gains& g)
        : l1_period_(g.l1_period), l1_damping_(g.l1_damping),
          l1_xtrack_i_gain_(g.l1_xtrack_i_gain), loiter_bank_limit_(g.loiter_bank_limit) {}

    L1Control(const L1Control&) = delete;
    L1Control& operator=(const L1Control&) = delete;

    void set_reverse(bool reverse) { reverse_ = reverse; }
    void set_default_period(float period) { l1_period_ = period; }
    void set_data_is_stale() { data_is_stale_ = true; }
    [[nodiscard]] bool data_is_stale() const { return data_is_stale_; }

    [[nodiscard]] float crosstrack_error() const { return crosstrack_error_; }
    [[nodiscard]] float crosstrack_error_integrator() const { return l1_xtrack_i_; }
    [[nodiscard]] float lateral_acceleration() const { return lat_acc_dem_; }
    [[nodiscard]] bool reached_loiter_target() const { return wp_circle_; }

    [[nodiscard]] std::int32_t nav_bearing_cd() const {
        return math::wrap_180_cd(math::rad_to_cd(nav_bearing_));
    }
    [[nodiscard]] std::int32_t bearing_error_cd() const { return math::rad_to_cd(bearing_error_); }
    [[nodiscard]] std::int32_t target_bearing_cd() const { return math::wrap_180_cd(target_bearing_cd_); }

    // Bank angle (centidegrees) to achieve the lateral acceleration demand
    // from the last update_*() call.
    [[nodiscard]] std::int32_t nav_roll_cd(const L1Inputs& in) const {
        const float pitch_lim = math::radians(60.0f);
        const float pitch = math::constrain_value(in.pitch_rad, -pitch_lim, pitch_lim);
        float ret = math::degrees(std::atan(lat_acc_dem_ * (1.0f / (kGravityMss * std::cos(pitch))))) * 100.0f;
        ret = math::constrain_value(ret, -9000.0f, 9000.0f);
        return static_cast<std::int32_t>(ret);
    }

    [[nodiscard]] float turn_distance(float wp_radius, const L1Inputs& in) const {
        wp_radius *= in.eas2tas * in.eas2tas;
        return std::min(wp_radius, l1_dist_);
    }

    [[nodiscard]] float turn_distance(float wp_radius, float turn_angle, const L1Inputs& in) const {
        const float distance_90 = turn_distance(wp_radius, in);
        turn_angle = std::fabs(turn_angle);
        if (turn_angle >= 90.0f) {
            return distance_90;
        }
        return distance_90 * turn_angle / 90.0f;
    }

    [[nodiscard]] float loiter_radius(float radius, const L1Inputs& in) const {
        const float sanitized_bank_limit = math::constrain_value(loiter_bank_limit_, 0.0f, 89.0f);
        const float lateral_accel_sea_level = std::tan(math::radians(sanitized_bank_limit)) * kGravityMss;
        const float nominal_velocity_sea_level = in.target_airspeed;
        const float eas2tas_sq = in.eas2tas * in.eas2tas;

        if (math::is_zero(sanitized_bank_limit) || math::is_zero(nominal_velocity_sea_level)
            || math::is_zero(lateral_accel_sea_level)) {
            return radius * eas2tas_sq;
        }
        const float sea_level_radius = (nominal_velocity_sea_level * nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            return radius * eas2tas_sq;
        }
        return std::max(sea_level_radius * eas2tas_sq, radius);
    }

    // The core L1 waypoint-tracking update. Computes lateral_acceleration()
    // and nav_bearing_cd() for the caller to read afterward, matching
    // upstream's side-effecting update_*() shape.
    void update_waypoint(const Location& prev_wp, const Location& next_wp, const L1Inputs& in, float dist_min = 0.0f) {
        float dt = static_cast<float>(in.now_us - last_update_waypoint_us_) * 1.0e-6f;
        if (dt > 1.0f) {
            l1_xtrack_i_ = 0.0f;
        }
        if (dt > 0.1f) {
            dt = 0.1f;
        }
        last_update_waypoint_us_ = in.now_us;

        const float k_l1 = 4.0f * l1_damping_ * l1_damping_;

        if (!in.location_valid) {
            data_is_stale_ = true;
            return;
        }
        const Location& current_loc = in.current_loc;
        math::Vector2f groundspeed_vector = in.groundspeed_vector;

        target_bearing_cd_ = current_loc.get_bearing_to(next_wp);

        float ground_speed = groundspeed_vector.length();

        const bool moving_forwards =
            std::fabs(math::wrap_PI(groundspeed_vector.angle() - get_yaw(in))) < static_cast<float>(M_PI_2);

        if (ground_speed < 0.1f || !moving_forwards) {
            ground_speed = 0.1f;
            groundspeed_vector = math::Vector2f(std::cos(get_yaw(in)), std::sin(get_yaw(in))) * ground_speed;
        }

        l1_dist_ = std::max(0.3183099f * l1_damping_ * l1_period_ * ground_speed, dist_min);

        math::Vector2f ab = prev_wp.get_distance_NE(next_wp);
        const float ab_length = ab.length();

        if (ab.length() < 1.0e-6f) {
            ab = current_loc.get_distance_NE(next_wp);
            if (ab.length() < 1.0e-6f) {
                ab = math::Vector2f(std::cos(get_yaw(in)), std::sin(get_yaw(in)));
            }
        }
        ab.normalize();

        const math::Vector2f a_air = prev_wp.get_distance_NE(current_loc);
        crosstrack_error_ = a_air % ab;

        const float wp_a_dist = a_air.length();
        const float along_track_dist = a_air * ab;

        float nu;
        if (wp_a_dist > l1_dist_ && along_track_dist / std::max(wp_a_dist, 1.0f) < -0.7071f) {
            const math::Vector2f a_air_unit = a_air.normalized();
            const float xtrack_vel = groundspeed_vector % (-a_air_unit);
            const float ltrack_vel = groundspeed_vector * (-a_air_unit);
            nu = std::atan2(xtrack_vel, ltrack_vel);
            nav_bearing_ = std::atan2(-a_air_unit.y, -a_air_unit.x);
        } else if (along_track_dist > ab_length + ground_speed * 3.0f) {
            const math::Vector2f b_air = next_wp.get_distance_NE(current_loc);
            const math::Vector2f b_air_unit = b_air.normalized();
            const float xtrack_vel = groundspeed_vector % (-b_air_unit);
            const float ltrack_vel = groundspeed_vector * (-b_air_unit);
            nu = std::atan2(xtrack_vel, ltrack_vel);
            nav_bearing_ = std::atan2(-b_air_unit.y, -b_air_unit.x);
        } else {
            const float xtrack_vel = groundspeed_vector % ab;
            const float ltrack_vel = groundspeed_vector * ab;
            const float nu2 = std::atan2(xtrack_vel, ltrack_vel);

            float sine_nu1 = crosstrack_error_ / std::max(l1_dist_, 0.1f);
            sine_nu1 = math::constrain_value(sine_nu1, -0.7071f, 0.7071f);
            const float nu1_base = std::asin(sine_nu1);
            float nu1 = nu1_base;

            if (l1_xtrack_i_gain_ <= 0.0f || !math::is_equal(l1_xtrack_i_gain_, l1_xtrack_i_gain_prev_)) {
                l1_xtrack_i_ = 0.0f;
                l1_xtrack_i_gain_prev_ = l1_xtrack_i_gain_;
            } else if (std::fabs(nu1) < math::radians(5.0f)) {
                l1_xtrack_i_ += nu1 * l1_xtrack_i_gain_ * dt;
                l1_xtrack_i_ = math::constrain_value(l1_xtrack_i_, -0.1f, 0.1f);
            }

            nu1 += l1_xtrack_i_;
            nu = nu1 + nu2;
            nav_bearing_ = math::wrap_PI(std::atan2(ab.y, ab.x) + nu1);
        }

        prevent_indecision(nu, in);
        last_nu_ = nu;

        nu = math::constrain_value(nu, -1.5708f, 1.5708f);
        lat_acc_dem_ = k_l1 * ground_speed * ground_speed / l1_dist_ * std::sin(nu);

        wp_circle_ = false;
        last_loiter_.reached_loiter_target_ms = 0;

        bearing_error_ = nu;
        data_is_stale_ = false;
    }

    // L1-guided circular loiter around center_wp. Blends a "capture" law
    // (same L1 guidance as update_waypoint, pulling the aircraft toward
    // the circle) with a "circle" law (PD + centripetal, holding it on the
    // circle), switching between them at the point the two demands cross
    // so the transition is seamless rather than a mode-switch discontinuity.
    void update_loiter(const Location& center_wp, float radius, std::int8_t loiter_direction, const L1Inputs& in) {
        const float radius_unscaled = radius;
        radius = loiter_radius(std::fabs(radius), in);

        const float omega = 6.2832f / l1_period_;
        const float kx = omega * omega;
        const float kv = 2.0f * l1_damping_ * omega;
        const float k_l1 = 4.0f * l1_damping_ * l1_damping_;

        if (!in.location_valid) {
            data_is_stale_ = true;
            return;
        }
        const Location& current_loc = in.current_loc;
        const math::Vector2f& groundspeed_vector = in.groundspeed_vector;

        const float ground_speed = std::max(groundspeed_vector.length(), 1.0f);

        target_bearing_cd_ = current_loc.get_bearing_to(center_wp);

        l1_dist_ = 0.3183099f * l1_damping_ * l1_period_ * ground_speed;

        const math::Vector2f a_air = center_wp.get_distance_NE(current_loc);

        math::Vector2f a_air_unit;
        if (a_air.length() > 0.1f) {
            a_air_unit = a_air.normalized();
        } else if (groundspeed_vector.length() < 0.1f) {
            a_air_unit = math::Vector2f(std::cos(get_yaw(in)), std::sin(get_yaw(in)));
        } else {
            a_air_unit = groundspeed_vector.normalized();
        }

        const float xtrack_vel_cap = a_air_unit % groundspeed_vector;
        const float ltrack_vel_cap = -(groundspeed_vector * a_air_unit);
        float nu = std::atan2(xtrack_vel_cap, ltrack_vel_cap);

        prevent_indecision(nu, in);
        last_nu_ = nu;

        nu = math::constrain_value(nu, -1.5708f, 1.5708f);

        const float lat_acc_dem_cap = k_l1 * ground_speed * ground_speed / l1_dist_ * std::sin(nu);

        const float xtrack_vel_circ = -ltrack_vel_cap;
        const float xtrack_err_circ = a_air.length() - radius;

        crosstrack_error_ = xtrack_err_circ;

        float lat_acc_dem_circ_pd = xtrack_err_circ * kx + xtrack_vel_circ * kv;

        const float vel_tangent = xtrack_vel_cap * static_cast<float>(loiter_direction);

        // Centripetal demand is computed BEFORE the PD clamp below because
        // the inside-radius branch of that clamp needs it (upstream master
        // 29d590b reordered these two lines for the same reason).
        const float lat_acc_dem_circ_ctr = vel_tangent * vel_tangent / std::max(0.5f * radius, radius + xtrack_err_circ);

        // Prevent the PD demand from turning the wrong way, either when
        // flying the wrong way round or while flying OUT from inside the
        // loiter radius. DELIBERATE DEPARTURE FROM THE Plane-4.7.0 PIN:
        // this is upstream master commit 29d590b ("AP_L1_Control: limit
        // loiter PD when inside radius", rubenp02/tridge, 2026-05-26),
        // which landed after 4.7.0. At the pin the clamp only caught
        // `vel_tangent < 0`, so a loiter entered near its own center (this
        // port's ModeLOITER orbits the entry point; RTL orbits home) saw the
        // P term (xtrack_err_circ * kx, large and negative deep inside the
        // circle) swamp the centripetal term and command a bank AGAINST the
        // requested loiter direction until the aircraft neared the ring
        // (measured: up to 24 m/s^2 the wrong way 5 m from center). Bounding
        // the PD term at -ctr makes the net circle demand >= 0 in the loiter
        // direction while outbound-inside, exactly as master does.
        if (ltrack_vel_cap < 0.0f) {
            if (vel_tangent < 0.0f) {
                lat_acc_dem_circ_pd = std::max(lat_acc_dem_circ_pd, 0.0f);
            } else if (xtrack_err_circ < 0.0f) {
                lat_acc_dem_circ_pd = std::max(lat_acc_dem_circ_pd, -lat_acc_dem_circ_ctr);
            }
        }

        const float lat_acc_dem_circ = static_cast<float>(loiter_direction) * (lat_acc_dem_circ_pd + lat_acc_dem_circ_ctr);

        const std::uint32_t now_ms = in.now_ms;
        if (xtrack_err_circ > 0.0f
            && static_cast<float>(loiter_direction) * lat_acc_dem_cap < static_cast<float>(loiter_direction) * lat_acc_dem_circ) {
            lat_acc_dem_ = lat_acc_dem_cap;

            // See file banner / Location::same_loc_as: keeps _WPcircle
            // (wp_circle_) latched true across brief capture-mode blips
            // (a wind gust, an unachievable radius) rather than letting
            // reached_loiter_target() flicker false and back.
            if (wp_circle_ && last_loiter_.reached_loiter_target_ms != 0
                && now_ms - last_loiter_.reached_loiter_target_ms < 200U
                && loiter_direction == last_loiter_.direction
                && math::is_equal(radius_unscaled, last_loiter_.radius)
                && center_wp.same_loc_as(last_loiter_.center_wp)) {
                last_loiter_.reached_loiter_target_ms = now_ms;
            } else {
                wp_circle_ = false;
                last_loiter_.reached_loiter_target_ms = 0;
            }

            bearing_error_ = nu;
            nav_bearing_ = std::atan2(-a_air_unit.y, -a_air_unit.x);
        } else {
            lat_acc_dem_ = lat_acc_dem_circ;
            wp_circle_ = true;
            last_loiter_.reached_loiter_target_ms = now_ms;
            bearing_error_ = 0.0f;
            nav_bearing_ = std::atan2(-a_air_unit.y, -a_air_unit.x);
        }

        last_loiter_.radius = radius_unscaled;
        last_loiter_.direction = loiter_direction;
        last_loiter_.center_wp = center_wp;

        data_is_stale_ = false;
    }

    // Heading-hold navigation: track a commanded heading directly rather
    // than a waypoint line. Unlike update_waypoint/update_loiter, this
    // reads yaw_sensor_cd directly (NOT through get_yaw_sensor(in)'s
    // reverse-aware wrapper) - matching upstream's own choice to bypass
    // the reverse flag here.
    void update_heading_hold(std::int32_t navigation_heading_cd, const L1Inputs& in) {
        const float omega_a = 4.4428f / l1_period_; // sqrt(2)*pi/period

        target_bearing_cd_ = math::wrap_180_cd(navigation_heading_cd);
        nav_bearing_ = math::cd_to_rad(static_cast<float>(navigation_heading_cd));

        std::int32_t nu_cd = target_bearing_cd_ - math::wrap_180_cd(in.yaw_sensor_cd);
        nu_cd = math::wrap_180_cd(nu_cd);
        float nu = math::cd_to_rad(static_cast<float>(nu_cd));

        const float ground_speed = in.groundspeed_vector.length();

        l1_dist_ = ground_speed / omega_a;
        const float v_omega_a = ground_speed * omega_a;

        wp_circle_ = false;
        last_loiter_.reached_loiter_target_ms = 0;

        crosstrack_error_ = 0.0f;
        bearing_error_ = nu;

        nu = math::constrain_value(nu, -1.5708f, 1.5708f);
        lat_acc_dem_ = 2.0f * std::sin(nu) * v_omega_a;

        data_is_stale_ = false;
    }

    // Level flight on the current heading - no navigation demand at all.
    // Also bypasses the reverse-aware get_yaw()/get_yaw_sensor() helpers,
    // matching upstream's direct AHRS reads here.
    void update_level_flight(const L1Inputs& in) {
        target_bearing_cd_ = in.yaw_sensor_cd;
        nav_bearing_ = in.yaw_rad;
        bearing_error_ = 0.0f;
        crosstrack_error_ = 0.0f;

        wp_circle_ = false;
        last_loiter_.reached_loiter_target_ms = 0;

        lat_acc_dem_ = 0.0f;

        data_is_stale_ = false;
    }

private:
    [[nodiscard]] float get_yaw(const L1Inputs& in) const {
        if (reverse_) {
            return math::wrap_PI(static_cast<float>(M_PI) + in.yaw_rad);
        }
        return in.yaw_rad;
    }

    [[nodiscard]] std::int32_t get_yaw_sensor(const L1Inputs& in) const {
        if (reverse_) {
            return math::wrap_180_cd(18000 + in.yaw_sensor_cd);
        }
        return in.yaw_sensor_cd;
    }

    void prevent_indecision(float& nu, const L1Inputs& in) {
        constexpr float kNuLimit = 0.9f * static_cast<float>(M_PI);
        if (std::fabs(nu) > kNuLimit && std::fabs(last_nu_) > kNuLimit
            && std::abs(math::wrap_180_cd(target_bearing_cd_ - get_yaw_sensor(in))) > 12000
            && nu * last_nu_ < 0.0f) {
            nu = last_nu_;
        }
    }

    float l1_period_;
    float l1_damping_;
    float l1_xtrack_i_gain_;
    float loiter_bank_limit_;

    float lat_acc_dem_ = 0.0f;
    float l1_dist_ = 0.0f;
    bool wp_circle_ = false;
    float nav_bearing_ = 0.0f;
    float bearing_error_ = 0.0f;
    float crosstrack_error_ = 0.0f;
    std::int32_t target_bearing_cd_ = 0;
    float last_nu_ = 0.0f;
    float l1_xtrack_i_ = 0.0f;
    float l1_xtrack_i_gain_prev_ = 0.0f;
    std::uint32_t last_update_waypoint_us_ = 0;
    bool data_is_stale_ = true;

    // Remembers the last update_loiter() decision, for the "keep _WPcircle
    // latched" check in update_loiter (see its body) - matches upstream's
    // anonymous _last_loiter struct.
    struct LastLoiter {
        std::uint32_t reached_loiter_target_ms = 0;
        float radius = 0.0f;
        std::int8_t direction = 1;
        Location center_wp;
    };
    LastLoiter last_loiter_;

    bool reverse_ = false;
};

// =====================================================================
// CPP-048 ADDENDUM: real top-level AP_Param Info[] table for
// L1Control::Gains (NAVL1_ prefix), phase 2e of the AP_Param vehicle-
// integration effort CPP-043 started (phase 1: Plane::aparm). See
// CPP-043's own commit message and plane.hpp's "CPP-043 ADDENDUM" for
// the pattern this follows: a real Info[]-building free function, a
// native-value read/write bridge, and load/save free functions reusing
// should_skip_save/load_raw/save_raw/scan/type_size/get_base (CPP-022)
// unchanged.
//
// REAL UPSTREAM SOURCE (libraries/AP_L1_Control/AP_L1_Control.cpp, read
// in full): var_info[] has exactly FOUR AP_GROUPINFO entries, and ALL
// FOUR are genuinely backed 1:1 by this port's Gains struct (unlike
// CPP-043's aparm, where only 13 of ~50 FixedWingTunables fields turned
// out to be real) - there are no internal-only tuning constants in
// Gains to exclude here:
//   AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period,        17)
//   AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping,     0.75f)
//   AP_GROUPINFO("XTRACK_I",  2, AP_L1_Control, _L1_xtrack_i_gain, 0.02)
//   AP_GROUPINFO("LIM_BANK",  3, AP_L1_Control, _loiter_bank_limit, 0.0f)
// -> Gains::l1_period / l1_damping / l1_xtrack_i_gain / loiter_bank_limit.
// All four are upstream AP_Float, matching this port's own plain
// `float` fields exactly - unlike aparm's seven Int8/Int16-narrowed
// fields (CPP-043 finding #3), there is NO on-storage width divergence
// to register here.
//
// REAL GROUP REGISTRATION, VERIFIED (not assumed): ArduPlane/
// Parameters.cpp:868 registers the whole object as a real GOBJECT:
//   GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),
// - a genuine two-level upstream structure (a vehicle-level Info entry
// of type Group named "NAVL1_", pointing at AP_L1_Control's own
// var_info[] GroupInfo table above with per-field idx 0-3), UNLIKE
// aparm's flat, individually-top-level-keyed fields (CPP-043 finding
// #1). That said, per this ticket's own scope, there is still no real
// Plane-wide vehicle table to root a nested GROUP Info entry in - wiring
// Plane-level GOBJECT-style tables across CPP-044 through CPP-049 is
// explicitly a LATER, separate integration ticket (this ticket's own
// "Explicitly out of scope", and this file may not touch plane.hpp to
// build one). l1_param_info() below therefore builds a FLAT top-level
// table - matching CPP-043's aparm_param_info() shape exactly, as this
// ticket's own text requires - with each entry's name PRE-CONCATENATED
// to its real full upstream name ("NAVL1_PERIOD", not bare "PERIOD") so
// a name lookup against this table alone still resolves the same
// string a real GCS would send.
//
// REGISTERED DIVERGENCE (structural, not byte-width like CPP-043's): a
// real upstream save of these four fields stores them keyed under
// k_param_L1_controller with a NONZERO group_element (idx 0-3 encoded
// via group_id, see group_info.hpp), not as four independently-
// top-level-keyed scalars with group_element=0. This table's own
// load/save round-trip below uses its OWN newly-allocated top-level
// keys (L1ParamKey) with group_element=0 throughout: internally
// self-consistent and correctly round-trips through this port's own
// storage, but NOT byte-compatible with what a real upstream vehicle
// would produce for "NAVL1_*". A later integration ticket that builds a
// real Plane-wide GOBJECT table can replace this flat shape with a true
// nested GroupInfo-based one without changing anything this ticket's
// own round-trip test observes about VALUES.
//
// A SEPARATE, PRE-EXISTING PORT BUG FOUND HERE BY CPP-048, FIXED BY
// CPP-050: Gains::l1_period's own in-class default used to be 25.0f,
// independently (and also wrongly) re-asserted by plane.hpp's old
// "CPP-043 ADDENDUM" comment as "upstream's real NAVL1_PERIOD ...
// default" ("CHECKED, NOT RE-DERIVED"). Reading the REAL AP_GROUPINFO
// line above directly shows the raw default is 17, not 25.
// AP_L1_Control::set_default_period() (AP_L1_Control.h:59-61) is the
// only mechanism that could legitimately override that raw default -
// grepping the ENTIRE pinned plane-4.7.0 tree for a caller of it found
// none; it is declared but never invoked anywhere in this tag, so 17 is
// genuinely ArduPlane's live shipped default. (ArduPlane/
// ReleaseNotes.txt:5234's own "NAVL1_PERIOD from 20 to 17" entry
// independently confirms 17, not 25 or 20, is this version's default.)
// CPP-050 changed Gains::l1_period's in-class default above to 17.0f -
// it now agrees with l1_param_info()'s own def_value for PERIOD below,
// which was already correctly 17.0f even while Gains' own in-class
// default was still wrong (see that entry) - and corrected plane.hpp's
// stale "CHECKED, NOT RE-DERIVED" comment. CPP-050 re-ran the full
// tests/vehicle_test.cpp suite by hand against the corrected period:
// every closed-loop convergence assertion and every nav_roll_cd()/
// crosstrack_error() check there already used generous, qualitative
// margins (sign/direction, or "converges within N meters"), not exact
// values tied to the specific 25.0f response shape, so NONE needed
// re-tuning - only this file's own l1_control_param_test.cpp (which
// explicitly asserted the bug's signature: Gains{}'s default diverging
// from the table's real default) needed updating. See CPP-050's own
// commit message for the full accounting.
//
// PLAIN NATIVE FIELDS, NOT ParamValue<T>: Gains' four fields are plain
// `float` (this file's own pre-existing banner above: "AP_Float
// REPLACED WITH PLAIN float ... no AP_Param in this port yet"), so this
// addendum reuses native_value.hpp's memcpy-based set_native_value/
// native_cast_to_float (CPP-043), NOT CPP-022 slice 6/7's set_value/
// cast_to_float (valid only when an actual ParamFloat/ParamInt8 object
// lives at the target address) - same reasoning as native_value.hpp's
// own banner and CPP-043 finding #4.
//
// This ticket's own top-level key allocation for L1's four real fields
// - independent of both upstream's real k_param_* enum (an EEPROM-
// migration-ordering detail, ADR-0013) and every other phase-2e
// module's own independent per-module enum (CPP-044 through CPP-049 -
// no shared vehicle-wide key space exists yet, matching CPP-043's own
// "no full vehicle-wide key space yet" starting point; a later
// integration ticket reconciles all of them into one real table).
enum class L1ParamKey : std::uint16_t {
    kPeriod = 1,
    kDamping = 2,
    kXtrackIGain = 3,
    kLoiterBankLimit = 4,
};

// Builds a fresh top-level param::Info[] table (4 real scalar entries +
// a VarType::None sentinel, matching every other table in this port's
// AP_Param module) addressing `gains`'s fields DIRECTLY (info.ptr =
// &gains.field). Built per-call rather than a shared `static` table -
// same reasoning as aparm_param_info (CPP-043): this port allows more
// than one live Gains object (this ticket's own round-trip test
// constructs two), so there is no single fixed address to bake in at
// compile time.
[[nodiscard]] inline std::array<param::Info, 5> l1_param_info(L1Control::Gains& gains) {
    using param::Info;
    using param::VarType;
    auto entry = [](const char* name, const void* ptr, float def_value, L1ParamKey key, VarType type) {
        Info info{};
        info.name = name;
        info.ptr = ptr;
        info.def_value = def_value;
        info.flags = 0;
        info.key = static_cast<std::uint16_t>(key);
        info.type = static_cast<std::uint8_t>(type);
        return info;
    };
    return {{
        entry("NAVL1_PERIOD", &gains.l1_period, 17.0f, L1ParamKey::kPeriod, VarType::Float),
        entry("NAVL1_DAMPING", &gains.l1_damping, 0.75f, L1ParamKey::kDamping, VarType::Float),
        entry("NAVL1_XTRACK_I", &gains.l1_xtrack_i_gain, 0.02f, L1ParamKey::kXtrackIGain, VarType::Float),
        entry("NAVL1_LIM_BANK", &gains.loiter_bank_limit, 0.0f, L1ParamKey::kLoiterBankLimit, VarType::Float),
        Info{}, // sentinel: type == VarType::None (0) via zero-init, matching every other table in this module
    }};
}

// Applies every entry's own AP_Param-table default (the REAL upstream
// values found above, all matching Gains' own in-class defaults since
// CPP-050 - notably 17.0f for PERIOD) directly into `gains`'s live
// fields. Explicit, not implicit - matches CPP-043's
// apply_aparm_defaults (not called from any constructor; L1Control's
// own constructor still reads whatever Gains it's handed, unchanged by
// this addendum).
inline void apply_l1_defaults(L1Control::Gains& gains) {
    const std::array<param::Info, 5> table = l1_param_info(gains);
    for (const param::Info& info : table) {
        if (info.type == static_cast<std::uint8_t>(param::VarType::None)) {
            break;
        }
        param::set_native_value(static_cast<param::VarType>(info.type), const_cast<void*>(info.ptr), info.def_value);
    }
}

// Port of AP_Param::load()'s real not-found-then-default behavior
// (AP_Param.cpp ~line 1310, read in full - same citation as CPP-043),
// specialized to this ticket's own flat top-level table (group_element
// always 0 - see this addendum's "REGISTERED DIVERGENCE" note above).
// Reuses load_raw (CPP-022 slice 5, unchanged: a plain memcpy, safe to
// target a Gains field's live address) and set_native_value (CPP-043)
// for the not-found/default case.
inline void load_l1_parameters(const storage::StorageAccess& storage, L1Control::Gains& gains) {
    const std::array<param::Info, 5> table = l1_param_info(gains);
    for (const param::Info& info : table) {
        if (info.type == static_cast<std::uint8_t>(param::VarType::None)) {
            break;
        }
        const auto type = static_cast<param::VarType>(info.type);
        param::ParamHeader phdr{};
        phdr.type = info.type;
        param::set_key(phdr, info.key);
        phdr.group_element = 0;
        void* field_ptr = const_cast<void*>(info.ptr);
        if (!param::load_raw(storage, phdr, field_ptr, param::type_size(type))) {
            param::set_native_value(type, field_ptr, info.def_value);
        }
    }
}

// Port of AP_Param::save_sync's default-skip-then-write path
// (AP_Param.cpp ~line 1138, read in full - same citation as CPP-043),
// specialized the same way load_l1_parameters is above. Reuses
// should_skip_save (CPP-022 slice 7, persistence.hpp) COMPLETELY
// UNCHANGED - pure float arithmetic with no pointer casting, so it
// applies here exactly as it does to the ParamValue<T>-based case this
// port already had it working for. `force_save` matches upstream's own
// save_sync(force_save, ...) parameter.
inline void save_l1_parameters(storage::StorageAccess& storage, L1Control::Gains& gains, bool force_save = false) {
    const std::array<param::Info, 5> table = l1_param_info(gains);
    for (const param::Info& info : table) {
        if (info.type == static_cast<std::uint8_t>(param::VarType::None)) {
            break;
        }
        const auto type = static_cast<param::VarType>(info.type);
        const void* field_ptr = info.ptr;
        const float current = param::native_cast_to_float(type, field_ptr);
        if (param::should_skip_save(type, current, info.def_value, force_save)) {
            continue;
        }
        param::ParamHeader phdr{};
        phdr.type = info.type;
        param::set_key(phdr, info.key);
        phdr.group_element = 0;
        (void)param::save_raw(storage, phdr, field_ptr, param::type_size(type));
    }
}

} // namespace fwcpp::nav
