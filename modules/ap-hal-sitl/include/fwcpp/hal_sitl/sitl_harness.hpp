#pragma once

// CPP-084: SitlHarness - a real, reusable (non-test) class that generalizes
// the "synthesize every real sensor input Plane::tick() needs from
// SimPlane's own physics truth, then feed servo outputs back into
// SimPlane's own physics update" pattern that several tests/vehicle_test.cpp
// closed-loop tests (run_biased_closed_loop(),
// run_stationary_yaw_bias_closed_loop() - see those functions directly)
// already hand-roll per-test. Phase 1 of a two-phase effort: CPP-085
// (a separate, later ticket that DEPENDS on this one) builds the actual
// standalone SITL executable around this class - this ticket only builds
// the harness itself and proves it, in isolation, against a hand-rolled
// equivalent.
//
// UPSTREAM COUNTERPART: AP_HAL_SITL's SITL_State / sitl_*.cpp files -
// upstream's own real "synthesize sensor readings from SITL::SIM's own
// physics truth for the real HAL/vehicle code to consume" glue layer. This
// module is named ap-hal-sitl (namespace fwcpp::hal_sitl) to mirror that
// exactly, matching this port's own established convention of naming each
// module after the upstream library it plays the same structural role for
// (ap-gps ~ AP_GPS, ap-tecs ~ AP_TECS, ...) - NOT a port of AP_HAL_SITL's
// actual source (that class is deeply tangled with upstream's HAL
// singleton/scheduler machinery this port doesn't have - ADR-0012), but the
// same real ROLE, built from this port's own existing precedent instead.
//
// EVERY FIELD SitlHarness::step() SYNTHESIZES WAS FOUND BY DIRECTLY READING
// mode.hpp's tick(Plane&, const GyroSample&, StabilizeInputs) function IN
// FULL (not from any pre-enumerated list) - see that function's own body
// and this class's own step() comments below for the exact citation behind
// each field. Summary, matched to tick()'s real read sites:
//
//   - GyroSample::gyro/delta_angle/dangle_dt - fed from SimPlane::gyro,
//     EXACTLY the pattern tests/vehicle_test.cpp's run_biased_closed_loop()
//     and run_stationary_yaw_bias_closed_loop() already establish
//     (measured_gyro = sim_plane.gyro + bias; delta_angle = measured_gyro *
//     dt; dangle_dt = dt). step()'s own `gyro_bias` parameter defaults to
//     the zero vector - a caller wanting to exercise drift correction
//     against a biased gyro (the exact scenario those two test helpers
//     drive) passes a nonzero bias explicitly; the harness itself always
//     feeds the TRUE rate with zero injected bias by default, matching the
//     ticket's own explicit instruction.
//   - StabilizeInputs::accel_sample (accel/delta_velocity/delta_velocity_dt)
//     - fed from SimPlane::accel_body, symmetrically to gyro above, exactly
//     matching run_biased_closed_loop()'s own `with_correction` branch
//     (accel_sample.accel = sim_plane.accel_body; delta_velocity =
//     accel_body * dt; delta_velocity_dt = dt). Always populated (unlike
//     that test's own with_correction=false branch, which deliberately
//     withholds it to test the uncorrected case) - see this class's own
//     "WHY ALWAYS-ON, UNLIKE THE TEST HELPERS" note below for why that
//     divergence from the test helpers is deliberate, not an oversight.
//   - StabilizeInputs::accel_y - NOT explicitly called out in the ticket's
//     own field list, found by direct reading: plane.hpp's file banner
//     (search "AP::ins()'s bias-corrected lateral accel") documents this as
//     genuinely real, upstream-matching "inert until wired" state (fed into
//     YawCoordinationInputs::accel_y by stabilize_yaw(), mode.hpp, but
//     YawController::Gains's real k_d default of 0.0f makes it provably
//     inert for an untuned vehicle either way). Since step() already has
//     SimPlane's true accel_body on hand for accel_sample above, feeding
//     its y-component here too is the natural, real synthesis (the exact
//     same true body-frame lateral accel a real accelerometer would read)
//     rather than leaving genuinely-available truth at a stale zero.
//   - StabilizeInputs::current_altitude_m - ALSO not in the ticket's own
//     field list, and NOT read directly by tick() itself, but read by
//     ModeFBWB/ModeCRUISE/ModeTAKEOFF's update()/navigate() (dispatched
//     from inside tick() via mode.update()/mode.navigate()) - found by
//     direct reading of mode.hpp's per-mode bodies. plane.hpp's own file
//     banner (search "current_loc's own design rationale") gives this
//     harness's EXACT formula verbatim as the established convention every
//     closed-loop test already maintains: `in.position_ned =
//     sim_plane.position; in.current_altitude_m = -sim_plane.position.z;`
//     (position_ned.z is NED-down, so altitude above the fixed start point
//     is its negation) - reproduced here unchanged.
//   - StabilizeInputs::position_ned - fed from SimPlane::position (NED,
//     relative to the vehicle's own fixed start point) - see the
//     current_altitude_m note just above for the shared derivation.
//   - Compass: plane.compass.rotate_earth_field_to_body(sim_plane.dcm) -
//     re-verified directly (compass.hpp): a real, existing, const method,
//     "true_dcm.transposed() * mag_ef_" - upstream's exact
//     update_mag_field_bf() line, built for exactly this use (its own doc
//     comment names it "TEST/SITL-INTEGRATION-HARNESS HELPER"). compass_
//     healthy is unconditionally true every tick (per the ticket's own
//     instruction - this port has no compass failure-injection to model
//     yet), matching run_stationary_yaw_bias_closed_loop()'s own
//     with_compass=true branch.
//   - GPS: StabilizeInputs::true_velocity_ned is fed from
//     SimPlane::velocity_ef every tick; tick() ITSELF calls
//     plane.gps.update(in.true_velocity_ned, in.now_ms) internally (mode.hpp
//     step 2) - re-verified directly, this is NOT a call step() needs to
//     make - and that real Gps::update() internally rate-limits to 5Hz
//     (200ms) on its own, exactly like a real backend, so most calls here
//     are a no-op. tick() also reads plane.gps.sample() directly (mode.hpp,
//     "const ahrs::GpsSample& gps_sample = plane.gps.sample();") - not
//     through StabilizeInputs at all - re-verified directly, matching the
//     ticket's own instruction to check this call site.
//   - Airspeed (CPP-082 pipeline): StabilizeInputs::airspeed_raw_pressure_pa
//     is fed from SimPlane::airspeed_sensor_differential_pressure() (CPP-082,
//     sim_plane.hpp) every tick, with airspeed_sensor_enabled=true - this
//     class is the FIRST real, non-test, end-to-end caller of that whole
//     pipeline (tick() itself then internally calls
//     plane.airspeed_sensor.update() and overrides airspeed_valid/
//     airspeed_eas from its output - re-verified directly in mode.hpp).
//   - wind_estimate - DELIBERATELY LEFT AT StabilizeInputs' OWN ZERO-VECTOR
//     DEFAULT. Re-verified directly (ahrs_dcm.hpp's file banner, the
//     "CPP-051 RE-EXAMINATION" note on drift_correction_accel()'s own
//     parameter list): wind_estimate is a genuine CALLER-SUPPLIED INPUT
//     AhrsDcm reads (not state AhrsDcm computes/owns) - upstream's real
//     `_wind` is itself computed by an EKF/DCM wind-ESTIMATION algorithm
//     fusing airspeed+GPS+heading, never by reading simulator ground truth
//     directly. This port has no such estimator ported (a disclosed gap,
//     same file banner, unchanged since CPP-051 gave SimPlane its own real
//     ground-truth wind model). Feeding SimPlane's true wind_ef here
//     instead of zero was considered and REJECTED: the file banner itself
//     names the exact failure mode - "would hand the estimator oracle
//     knowledge no real flight controller has" - which would make this
//     harness's drift-correction behavior artificially better than a real
//     vehicle's, not a faithful stand-in for one. Zero matches this port's
//     existing, real, disclosed behavior (every existing caller that never
//     touches this field already gets zero) - not a shortcut invented for
//     this ticket.
//   - StabilizeInputs::eas2tas - left at its own real default (1.0f),
//     matching SimPlane's own airspeed_sensor_differential_pressure() doc
//     comment ("eas2tas == 1.0 - see doc comment above") and this port's
//     "no AP_Baro" exclusion (sim_plane.hpp file banner) - EAS and TAS are
//     the same quantity throughout this port, on both sides of the
//     synthesis, so there is nothing for step() to compute here.
//   - StabilizeInputs::gps_use_enabled - set true, matching upstream's real
//     AHRS_GPS_USE default (GPSUse::Enable) - the harness's whole point is
//     to exercise the REAL, fully-wired drift-correction path, not the
//     "GPS disabled" path run_biased_closed_loop(false, ...) deliberately
//     tests as a separate, narrower scenario.
//
// WHY ALWAYS-ON, UNLIKE THE TEST HELPERS: run_biased_closed_loop()'s
// with_correction=false branch and run_stationary_yaw_bias_closed_loop()'s
// with_compass=false branch, and the latter's own choice to NEVER feed
// tick()'s computed servos back into SimPlane, are deliberate, narrow,
// single-purpose test knobs - each exists to isolate ONE specific drift-
// correction code path from another for a specific pass/fail discriminator
// (see those functions' own file banners). SitlHarness is meant to be
// genuinely reusable PRODUCTION code (CPP-085's future real executable, not
// just this ticket's own test) - a real vehicle has no equivalent of
// "pretend the compass doesn't exist this tick" or "compute servo outputs
// but don't actually apply them to the airframe" toggles, so step() does
// not expose them. A caller that genuinely needs the test helpers'
// narrower, single-feature-isolating behavior still has those helpers
// available unchanged (see this ticket's own commit message for why they
// were NOT migrated onto this class).
//
// SERVO OUTPUT FEEDBACK: after tick() returns, step() reads
// plane.srv_channels.get_output_scaled(...) for aileron/elevator/rudder/
// throttle and feeds them into sim_plane.update(...), reproducing
// run_biased_closed_loop()'s own exact scaling (aileron/elevator/rudder
// divided by fwcpp::vehicle::kServoMax == 4500.0f; throttle divided by
// 100.0f) - re-verified directly against that helper's own source.
//
// RC INPUT IS DELIBERATELY NOT SYNTHESIZED HERE: tick()'s own step 1 reads
// plane.rc_channels.read_input(plane.hal.rc_input) - a real RC receiver
// frame (or, in every existing test, set_sticks()) - not anything derivable
// from SimPlane's physics truth. A SitlHarness caller is expected to drive
// plane.hal.rc_input itself (directly, or via a helper like the existing
// tests' set_sticks()) before calling step(), exactly as every existing
// closed-loop test already does before calling tick() directly.

#include <cstdint>

#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/baro/baro.hpp>
#include <fwcpp/sim/sim_baro.hpp>
#include <fwcpp/ahrs/ahrs_atmosphere.hpp>
#include <fwcpp/vehicle/plane.hpp>

namespace fwcpp::hal_sitl {

class SitlHarness {
public:
    // Does not own Plane/SimPlane - a caller constructs and configures both
    // itself (arm the vehicle, force safety off, pick a control_mode, call
    // any one-time per-mode setup like set_target_altitude_current()/
    // set_home() the chosen mode needs) exactly as every existing
    // vehicle_test.cpp closed-loop test already does before its own
    // hand-rolled per-tick loop - this class only generalizes the PER-TICK
    // synthesis, not vehicle setup.
    SitlHarness(vehicle::Plane& plane, sim::SimPlane& sim_plane) : plane_(plane), sim_plane_(sim_plane) {}

    // Performs exactly the sequence this class's own file banner documents:
    // synthesize every real input tick() needs from sim_plane_'s CURRENT
    // truth (i.e. the state left by the PREVIOUS step()'s own sim_plane_
    // .update() call, or the caller's initial state on the first call),
    // call tick(), read tick()'s computed servo outputs back, then advance
    // sim_plane_ by dt.
    //
    // gyro_bias defaults to the zero vector, per the ticket's own explicit
    // instruction: this harness always feeds the true gyro rate by default;
    // a caller wanting to exercise drift correction against a biased gyro
    // (exactly like run_biased_closed_loop()/
    // run_stationary_yaw_bias_closed_loop() do today) passes a nonzero bias
    // here instead of hand-rolling its own tick()-driving loop.
    void step(std::uint32_t now_ms, float dt, const math::Vector3f& gyro_bias = math::Vector3f{}) {
        vehicle::StabilizeInputs in;
        in.dt = dt;
        in.now_ms = now_ms;
        in.now_us = static_cast<std::uint64_t>(now_ms) * 1000ULL;

        // Gyro/accel - see file banner's own citation.
        const math::Vector3f measured_gyro = sim_plane_.gyro + gyro_bias;
        ahrs::GyroSample gyro_sample;
        gyro_sample.gyro = measured_gyro;
        gyro_sample.delta_angle = measured_gyro * dt;
        gyro_sample.dangle_dt = dt;

        in.accel_sample.accel = sim_plane_.accel_body;
        in.accel_sample.delta_velocity = sim_plane_.accel_body * dt;
        in.accel_sample.delta_velocity_dt = dt;
        in.accel_y = sim_plane_.accel_body.y;

        // Compass - always healthy, per the ticket's own instruction (no
        // compass failure-injection model in this port yet).
        in.compass_healthy = true;
        in.compass_field_bf = plane_.compass.rotate_earth_field_to_body(sim_plane_.dcm);

        // GPS - tick() itself calls plane.gps.update(in.true_velocity_ned,
        // in.now_ms) internally (mode.hpp step 2); step() only needs to
        // supply the true velocity, not call update()/sample() itself.
        in.true_velocity_ned = sim_plane_.velocity_ef;

        // Airspeed (CPP-082 pipeline) - see file banner's own citation.
        in.airspeed_sensor_enabled = true;
        in.airspeed_raw_pressure_pa = sim_plane_.airspeed_sensor_differential_pressure();

        // wind_estimate deliberately left at StabilizeInputs' own zero-
        // vector default - see this file's own banner for the full
        // reasoning (a real, disclosed "no wind-estimation algorithm
        // exists yet" gap, not a shortcut).

        in.gps_use_enabled = true; // exercise the real, fully-wired drift-correction path

        // current_altitude_m/position_ned - see file banner's own citation
        // for the exact convention this reproduces (plane.hpp file banner).
        in.position_ned = sim_plane_.position;
        in.current_altitude_m = -sim_plane_.position.z;
        // BARO. Feed the plant's pressure/temperature through plane.baro and
        // let tick() derive eas2tas from the MEASURED pressure against the
        // barometer's own calibrated ground reference (mode.hpp). This retires
        // the ahrs_atmosphere.hpp standard-day estimate that stood in for a
        // barometer until ap-baro existed -- and with it the last place the
        // vehicle read a conversion factor straight off the plant.
        //
        // sitl_baro_from_aircraft() reads aircraft.location.alt, which is only
        // correct because SimPlane::update() now refreshes Location; before
        // that fix this sample was frozen at the start altitude too.
        {
            const sim::SitlBaroSample baro = sim::sitl_baro_from_aircraft(sim_plane_);
            in.baro_pressure_pa = baro.pressure_pa;
            in.baro_temperature_c = baro.temperature_k - baro::kCtoKelvin;
            in.baro_sensor_enabled = true;
        }

        vehicle::tick(plane_, gyro_sample, in);

        // Servo output feedback - see file banner's own citation for the
        // exact scaling this reproduces from run_biased_closed_loop().
        const float aileron = plane_.srv_channels.get_output_scaled(srv::Function::kAileron) / vehicle::kServoMax;
        const float elevator = plane_.srv_channels.get_output_scaled(srv::Function::kElevator) / vehicle::kServoMax;
        const float rudder = plane_.srv_channels.get_output_scaled(srv::Function::kRudder) / vehicle::kServoMax;
        const float throttle = plane_.srv_channels.get_output_scaled(srv::Function::kThrottle) / 100.0f;
        sim_plane_.update(aileron, elevator, rudder, throttle, dt);
    }

    [[nodiscard]] vehicle::Plane& plane() { return plane_; }
    [[nodiscard]] sim::SimPlane& sim_plane() { return sim_plane_; }

    /** AMSL elevation of the vehicle's fixed start point, so
     *  current_altitude_m (which is above THAT point, not AMSL) can be
     *  turned into the AMSL altitude the atmosphere model needs.
     *  Upstream a barometer is calibrated at boot and reports AMSL
     *  directly; this is the equivalent one-time survey figure. Default
     *  0 keeps every existing caller at sea level, i.e. eas2tas ~ 1. */
    void set_origin_amsl_m(float amsl_m) { origin_amsl_m_ = amsl_m; }
    [[nodiscard]] float origin_amsl_m() const { return origin_amsl_m_; }

private:
    float origin_amsl_m_ = 0.0f;
    vehicle::Plane& plane_;
    sim::SimPlane& sim_plane_;
};

} // namespace fwcpp::hal_sitl
