#pragma once

// Port of upstream SITL's "Plane" ground-truth flight-dynamics model:
// STANDARD fixed-wing configuration only. CPP-030.
//
// Upstream sources (Plane-4.7.0, read directly from the pinned worktree):
//   - libraries/SITL/SIM_Plane.h (141 lines, read in full) - the
//     Coefficients struct and its default_coefficients values (a real,
//     named airframe: last_letter's skywalker_2013/aerodynamics.yaml,
//     credited there to Georacer), mass=2.0f, hover_throttle=0.7f.
//   - libraries/SITL/SIM_Plane.cpp:
//       Plane::liftCoeff (line 235), Plane::dragCoeff (line 257),
//       Plane::getTorque (line 273), Plane::getForce (line 344),
//       Plane::calculate_forces (line 398), Plane::update (line 522).
//   - libraries/SITL/SIM_Aircraft.cpp:
//       Aircraft::hagl (line 145), Aircraft::on_ground (line 153),
//       Aircraft::update_dynamics (line 709).
//
// THIS IS NOT A PORT OF FLIGHT-CONTROL CODE. It is upstream's own PHYSICS
// ORACLE - the thing that stands in for a real aircraft during SITL - so
// this port's control loops (L1Control, AC_PID, AhrsDcm) can be tested
// end-to-end against a known-correct trajectory. SimPlane propagates truth
// directly and deliberately SHARES NO CODE with the estimator it is used to
// check: its own attitude integration (dcm.rotate + dcm.normalize) reuses
// ap-math's Matrix3, the same primitive AhrsDcm is built on, but never
// AhrsDcm itself - using the same integrator class for both the truth model
// and the thing being tested against it would make the check circular.
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching this port's
// standing pattern (AcPid's Gains, L1Control's L1Inputs, AhrsDcm's
// GyroSample):
//   - Upstream's Aircraft/Plane reach into AP::ahrs(), AP_HAL::millis(), a
//     global SITL::SIM* singleton, AP_JSON model-file loading, and
//     battery/RPM/GCS simulation hooks. None of that exists here. SimPlane
//     owns all its own state directly (coefficients, mass, attitude,
//     gyro, velocity, position) and every update() call takes control
//     surface deflections, throttle, and dt explicitly.
//   - `struct sitl_input` (16-channel PWM array + per-vehicle decode via
//     filtered_servo_angle/filtered_servo_range) is replaced by plain
//     aileron/elevator/rudder/throttle float parameters, already in the
//     -1..1 (surfaces) / 0..1 (throttle) ranges filtered_servo_angle/
//     filtered_servo_range would have produced - servo PWM decoding is
//     ap-srv-channel's job, not this class's.
//
// SCOPE: STANDARD FIXED-WING CONFIGURATION ONLY. The following upstream
// Plane/Aircraft members, branches, and constructor frame-string flags are
// DELIBERATELY NOT PORTED in this slice:
//   - All airframe config variants and their "-suffix" frame-string
//     parsing: elevons, vtail, dspoilers, redundant, reverse_thrust,
//     reverse_elevator_rudder, tailsitter, aerobatic, copter_tailsitter,
//     have_launcher, have_steering, ice_engine (and ICEngine itself).
//     calculate_forces' surface-mixing if/else-if chain (elevons/vtail/
//     dspoilers/redundant) and its reverse_elevator_rudder negation are
//     skipped entirely - aileron/elevator/rudder/throttle are used exactly
//     as given. getTorque's `if (tailsitter || aerobatic)` effective-
//     airspeed/alpha adjustment (SIM_Plane.cpp lines ~281-289) is excluded
//     too - alpha and airspeed are used as passed in, unmodified.
//     ("-heavy"/"-jet" mass/thrust_scale overrides used to be listed here
//     too - CPP-094 ported them, see MassVariant below.)
//   - load_coeffs() (AP_JSON model-file loading) IS PORTED (real line ~398
//     below) - this bullet used to claim otherwise; corrected by CPP-094,
//     which also gave it its first real test: a byte-for-byte copy of
//     upstream's own `Tools/autotest/models/skywalker_2013.json`
//     (tests/fixtures/skywalker_2013.json) loaded via load_coeffs() and
//     checked to reproduce Coefficients{}'s hardcoded defaults exactly -
//     the genuine round-trip proof the previous synthetic-fixture-only
//     parser test never gave. skywalker_2013.json is the ONLY real
//     native-format Plane coefficient file anywhere in the pinned upstream
//     tree (verified directly: `grep -rl c_lift_a` across the whole pinned
//     tree matches nothing else) - `Callisto.json`/`freestyle.json` (also
//     under Tools/autotest/models/) are MULTICOPTER frame-config formats
//     (mass/battery/motor-count fields, not aerodynamic coefficients), and
//     `xplane_plane.json`/`xplane_heli.json` are DREF mapping configs for
//     the unrelated X-Plane external-FDM backend - none of the three are
//     valid load_coeffs() inputs; a caller wanting a real, different
//     upstream-format plane has exactly the one file. Getting that one
//     real file to actually round-trip surfaced two genuine, previously-
//     silent bugs in this port, both fixed by CPP-094: (1) load_coeffs()
//     itself read the CG-offset key as "cg" - upstream's real key
//     (SIM_Plane.cpp:191) is "CGOffset" - so the field silently never
//     loaded from any real file (masked before now because the
//     constructor's own default cg_offset already equals
//     skywalker_2013.json's value); (2) sim_json.hpp's parser only
//     recognized "//" comments - skywalker_2013.json uses "#" (upstream's
//     own real AP_JSON.cpp strips "#" to end-of-line, see sim_json.hpp's
//     banner) - so a byte-for-byte copy of the real file failed to parse
//     at all until "#"-comment support was added.
//   - Atmosphere/air density model (AP_Baro::get_air_density_for_alt_amsl,
//     eas2tas from altitude). air_density defaults to SSL_AIR_DENSITY
//     (1.225f kg/m^3, AP_Math/definitions.h) and is held constant - a
//     caller may override it, but there is no altitude-driven model here
//     since this port has no AP_Baro yet.
//   - Sensor/GCS simulation: fill_fdm, smooth_sensors, add_noise,
//     update_mag_field_bf, battery/RPM simulation (battery_voltage,
//     battery_current, rpm[]), extrapolate_sensors - no sensor subsystems
//     (beyond AnalogIn) exist in this port to feed.
//   - The launcher (have_launcher / launch_accel / launch_time /
//     launch_start_ms) and ground-steering (have_steering, the
//     gyro.z += steering * ... nose-wheel hack in Plane::update) hooks -
//     test/debug airframe features, not core physics.
//   - Frame-rate/timing bookkeeping: frame_time_us, time_advance,
//     setup_frame_time, adjust_frame_time, sync_frame_time,
//     lock_step_scheduled. This class takes an explicit dt per update()
//     call, driven by whatever test/harness calls it, not a real-time
//     scheduler matching a target loop rate.
//   - set_pose / external pose injection, precision-landing hooks, Clamp,
//     shove/twist/external forces, ship/tether/slung-payload hooks
//     (AP_SIM_SHIP_ENABLED / AP_SIM_SLUNGPAYLOAD_ENABLED / AP_SIM_TETHER_
//     ENABLED) - test/debug/companion-computer features, not core physics.
//   - Aircraft::update_dynamics' entire `switch (ground_behavior)` block
//     (GROUND_BEHAVIOR_NO_MOVEMENT / FWD_ONLY / TAILSITTER) - upstream's
//     taxi/takeoff-roll simulation for various airframe configs (zeroing
//     roll/pitch, forcing forward-only body velocity, rotating the DCM for
//     tailsitter ground attitude). This slice's on_ground() handling is
//     the flat "don't sink through the floor" clamp described below,
//     nothing more.
//   - update_eas_airspeed's airspeed_pitot (120 m/s pitot-tube clamp) -
//     airspeed_pitot has no consumer in this port (no airspeed sensor
//     model yet); the plain `airspeed` member upstream itself uses for all
//     physics (as opposed to sensor simulation) is kept.
//
// WIND MODELING (CPP-051) - closes the "wind_ef treated as always zero"
// gap CPP-030 originally disclosed. Upstream: Aircraft::update_wind()
// (SIM_Aircraft.cpp:888, read in full), Aircraft::velocity_air_ef /
// velocity_air_bf recomputation (SIM_Aircraft.cpp:762-766, in
// update_dynamics()), `struct sitl_input`'s nested `wind` struct
// (SITL_Input.h, read in full: speed/direction/turbulence/dir_z, exactly
// that field order).
//   - WindConfig (below) transcribes sitl_input.wind field-for-field: a
//     real, but compile-time-or-test-supplied, input a caller sets on
//     SimPlane::wind_config directly - the same "no AP_Param/GCS live-
//     tunable path exists in this port" precedent Coefficients above
//     already establishes. There is no SIM_WIND_* MAVLink/AP_Param
//     runtime-set path here (no GCS at all in this port) - verified by
//     checking there is no other AP_Param consumer of sitl->wind_*
//     anywhere update_wind() reads besides input.wind itself.
//   - update_wind() reproduces upstream's real body exactly: the steady
//     wind vector `Vector3f(cos(direction)*cos(dir_z), sin(direction)*
//     cos(dir_z), sin(dir_z)) * speed`, the turbulence gust IIR-filtered
//     random walk (iir_coef=0.98, wind_turb = turbulence*10.0f, gated on
//     `wind_turb > 0 && !on_ground()`, upstream's own comment on the
//     10.0f scale transcribed verbatim), and the final `wind_ef =
//     -wind_ef` sign flip. That flip is REAL and was verified algebraically
//     against upstream's OWN two real call sites, not guessed: upstream's
//     velocity_air_ef = velocity_ef - wind_ef (the member, i.e.
//     POST-negation) is the standard physics identity "airmass-relative
//     velocity = ground velocity - true wind velocity" only if the
//     POST-negation wind_ef is the actual physical earth-frame velocity of
//     the moving air mass. Direct substitution shows the PRE-negation
//     vector this class builds points in the "wind is coming FROM this
//     compass heading" sense (standard meteorological convention for
//     input.wind.direction) - e.g. direction=0 (from due north) yields a
//     pre-negation vector of (+speed,0,0) (pointing north), which negates
//     to (-speed,0,0) (pointing south) - a physical air mass correctly
//     moving away from the north it's blowing from. Reproduced here with
//     the identical negate-the-whole-vector-at-the-end structure (steady
//     term and turbulence term negated together, matching upstream's
//     statement order).
//   - get_local_updraft()'s terrain-relief thermal/updraft model
//     (SIM_Aircraft.cpp:1241, read in full) is EXCLUDED: every non-zero
//     `sitl->thermal_scenario` case hard-codes specific thermal
//     positions/radii against a terrain-relative position query
//     (`position + home.get_distance_NED_double(origin)`) this port has no
//     Location/home/terrain subsystem to support (same absence the GROUND
//     MODEL note below already establishes); the real upstream default
//     (THML_SCENARI=0, verified against SITL.cpp's own AP_GROUPINFO table)
//     unconditionally `return 0` before reaching any of that, so omitting
//     the whole feature changes nothing for any wind configuration this
//     port can express. wind_ef.z therefore has no updraft term added -
//     a real, disclosed exclusion, not a silent drop.
//   - rand_normal()-equivalent: this port has NO existing normal-random
//     helper anywhere (checked every other SITL-adjacent module - ap-gps,
//     ap-compass - both explicitly disclose "no noise model" instead of
//     having one to reuse), so this is the first one. Upstream's own
//     Aircraft::rand_normal (SIM_Aircraft.cpp:343) hand-rolls a Marsaglia-
//     polar Box-Muller transform over the PROCESS-GLOBAL libc rand()/
//     RAND_MAX, with its second sample cached in a FUNCTION-STATIC shared
//     across every call site in the whole SITL binary (gyro/accel sensor
//     noise elsewhere in Aircraft, ADSB/AIS/Vicon position noise, and this
//     turbulence model, all drawing from the same global stream). ADR-0012
//     rules out exactly this kind of hidden shared mutable state. This
//     class instead owns its own explicitly-seedable std::mt19937 +
//     std::normal_distribution<double>(0,1) member (wind_rng_ /
//     wind_normal_dist_, seeded via the constructor's wind_rng_seed
//     parameter) - statistically equivalent (mean 0, unit-variance
//     Gaussian samples feeding the identical IIR-filtered random walk
//     upstream uses) but per-instance and deterministic-per-seed rather
//     than a process-global stream. This is NOT a bit-exact RNG-sequence
//     match to upstream - neither achievable (libc rand()'s sequence is
//     implementation-defined) nor required: turbulence is a stochastic
//     gust MODEL, verified by this ticket's tests via its statistical/
//     settling properties (mean/stddev of the IIR-filtered output), not by
//     reproducing a specific pseudo-random sequence.
//   - turbulence_azimuth's per-tick re-randomization
//     (`turbulence_azimuth = turbulence_azimuth + (2 * rand())`,
//     SIM_Aircraft.cpp:903) only matters modulo 360 degrees once run
//     through cosf/sinf - its purpose is purely "give the horizontal gust
//     a fresh, uncorrelated direction every tick", not to accumulate any
//     meaningful angle. Reproduced with an explicit
//     std::uniform_real_distribution<float>(0, 360) draw from the SAME
//     wind_rng_ member instead of libc rand()/RAND_MAX - identical effect
//     (full re-randomization every tick, no persistent directional
//     memory), same RNG-substitution rationale as rand_normal() above.
//
// GROUND MODEL - A DELIBERATE FLAT-EARTH SIMPLIFICATION, not upstream's:
// upstream's on_ground() is `hagl() <= 0.001f`, where hagl() itself is
// `(-position.z) + home.alt*0.01f - ground_level - frame_height -
// ground_height_difference()` - i.e. height above a possibly-sloped,
// possibly-offset-from-origin terrain model this port has none of (no
// Location/home/terrain subsystem wired to this class). This slice defines
// on_ground() as simply `position.z >= 0.0f`: a flat earth in NED
// coordinates, ground at z=0, with the caller responsible for initializing
// position.z to -initial_altitude. Likewise, upstream unconditionally snaps
// `position.z = -(ground_level + frame_height - home.alt*0.01f +
// ground_height_difference())` whenever on_ground() (SIM_Aircraft.cpp,
// just before the ground_behavior switch) - this class has no such terrain
// value to snap to, so ground contact is instead limited to exactly the two
// clamps upstream's OWN core update_dynamics already does independently of
// ground_behavior: (1) accel_earth.z clamped to <=0 while on the ground
// (upstream's real clamp, ported as-is), and (2) velocity_ef.z clamped to
// <=0 once integration lands on/through the ground plane. No position snap,
// no taxi/roll simulation - "don't sink through the floor", not a landing-
// gear model.
//
// LITERAL SAFETY / double-precision transcription: this header has no
// compiled .cpp of its own (no fwcpp_upstream_flags target to link), so
// nothing here needs the compiled-.cpp treatment scalar.cpp's wrap_*
// family needed - see scalar.hpp's own file banner for why that split
// exists at all. liftCoeff/dragCoeff/getForce/getTorque deliberately keep
// upstream's OWN `double` locals (sigmoid, linear, flatPlate, AR, c_drag_a,
// qbar, ax/ay/az, la/ma/na, p/q/r) exactly as upstream typed them, even
// though the surrounding Coefficients fields and parameters are float -
// ADR-0012's stance is against AMBIGUOUS bare literals under
// -fsingle-precision-constant, not against double itself, and upstream
// chose double precision for these specific intermediates on purpose.
// M_PI is not used directly (unlike upstream's dragCoeff, which multiplies
// by upstream's bare M_PI macro); fwcpp::math::pi_constant() is used
// instead - the same double-precision PI this port's own scalar.cpp
// already exposes for exactly this "rare caller needs the bare double
// constant" case, rather than adding a second hardcoded copy of M_PI here.

#include <cmath>
#include <cstdint>
#include <random>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_json.hpp>

namespace fwcpp::sim {

// Upstream: AP_Math/definitions.h's GRAVITY_MSS (9.80665f) - reproduced as
// a local named constant rather than a shared ap-math export, matching
// this port's existing precedent (ap-nav's l1_control.hpp kGravityMss).
// CPP-082 - defaults for airspeed_sensor_differential_pressure() below.
// Real upstream has TWO independent parameters here: SITL's own
// simulated-sensor ratio (SITL_Airspeed.cpp's AP_GROUPINFO("RATIO", 7,
// AirspeedParm, ratio, 1.99) - ARSPD_RATIO under the SIM_ prefix,
// modeling manufacturing tolerance in the simulated sensor itself) and
// the vehicle's own calibrated ratio it divides back out on read
// (AP_Airspeed_Params.cpp's AP_GROUPINFO("RATIO", 4, AP_Airspeed_Params,
// ratio, 2) - the real ARSPD_RATIO a user calibrates). This port has no
// calibration-error/ratio-mismatch modeling (see modules/ap-airspeed's
// own file banner "OUT OF SCOPE" list - the interactive ratio-
// calibration state machine is real, separate, deferred upstream scope)
// so phase 1 deliberately reuses ONE shared value for both sides -
// upstream's own AP_Airspeed_Params.cpp default (2.0, not SITL's 1.99),
// verified directly, since that is the value this port's AirspeedSensor
// (ap-airspeed) itself also defaults to (kDefaultRatio, airspeed_
// sensor.hpp) - a caller wanting to exercise a genuine ratio mismatch
// can still pass different values to each side explicitly.
inline constexpr float kDefaultAirspeedSensorRatio = 2.0f;

// Upstream: SITL_Airspeed.cpp's AP_GROUPINFO("RND", 1, AirspeedParm,
// noise, 2.0) - ARSPD_RND under the SIM_ prefix, the simulated pressure
// noise amplitude (Pa) `_update_airspeed()` scales its rand_float() draw
// by. No vehicle-side counterpart (this is purely a simulated-sensor
// characteristic) - verified directly, not invented.
inline constexpr float kDefaultAirspeedNoisePa = 2.0f;

// Upstream: SIM_Plane.h's nested `struct Coefficients` and its
// `default_coefficients` member - reproduced here as a plain, caller-
// overridable aggregate (no AP_JSON load_coeffs() in this port; a caller
// wanting a different airframe just constructs one and assigns fields).
// Comment and every numeric value transcribed exactly from upstream.
struct Coefficients {
    // from last_letter skywalker_2013/aerodynamics.yaml
    // thanks to Georacer!
    float s = 0.45f;
    float b = 1.88f;
    float c = 0.24f;
    float c_lift_0 = 0.56f;
    float c_lift_deltae = 0.0f;
    float c_lift_a = 6.9f;
    float c_lift_q = 0.0f;
    float mcoeff = 50.0f;
    float oswald = 0.9f;
    float alpha_stall = 0.4712f;
    float c_drag_q = 0.0f;
    float c_drag_deltae = 0.0f;
    float c_drag_p = 0.1f;
    float c_y_0 = 0.0f;
    float c_y_b = -0.98f;
    float c_y_p = 0.0f;
    float c_y_r = 0.0f;
    float c_y_deltaa = 0.0f;
    float c_y_deltar = -0.2f;
    float c_l_0 = 0.0f;
    float c_l_p = -1.0f;
    float c_l_b = -0.12f;
    float c_l_r = 0.14f;
    float c_l_deltaa = 0.25f;
    float c_l_deltar = -0.037f;
    float c_m_0 = 0.045f;
    float c_m_a = -0.7f;
    float c_m_q = -20.0f;
    float c_m_deltae = 1.0f;
    float c_n_0 = 0.0f;
    float c_n_b = 0.25f;
    float c_n_p = 0.022f;
    float c_n_r = -1.0f;
    float c_n_deltaa = 0.0f;
    float c_n_deltar = 0.1f;
    float deltaa_max = 0.3491f;
    float deltae_max = 0.3491f;
    float deltar_max = 0.3491f;
    // the X CoG offset should be -0.02, but that makes the plane too tail heavy
    // in manual flight. Adjusted to -0.15 gives reasonable flight
    math::Vector3f cg_offset{-0.15f, 0.0f, -0.05f};
};

// Upstream: `struct sitl_input`'s nested `wind` struct (SITL_Input.h, read
// in full) - transcribed field-for-field, same names/units/order. See file
// banner's "WIND MODELING" note for why this is a caller-set aggregate
// rather than an AP_Param/MAVLink-tunable (no GCS in this port at all).
// direction/dir_z use upstream's own meteorological "wind is coming FROM
// this heading" convention - see update_wind()'s own sign-convention note.
struct WindConfig {
    float speed = 0.0f;       // m/s
    float direction = 0.0f;   // deg, 0..360, compass bearing wind blows FROM
    float turbulence = 0.0f;  // turbulence intensity (upstream's own SIM_WIND_TURB-equivalent units)
    float dir_z = 0.0f;       // deg, -90..90, vertical wind angle
};

// CPP-030 leftover closer: Aircraft::ground_behavior (SIM_Aircraft.h:308).
// Upstream Plane defaults to GROUND_BEHAVIOR_FWD_ONLY (SIM_Plane.cpp:50);
// this class defaults to kNone so the already-landed "don't sink through
// the floor" clamp stays the no-config path (existing sim_plane_test).
// kTailsitter is named for catalog completeness and is a documented no-op
// (fw-cpp is fixed-wing only).

// CPP-030 leftover closer: Plane constructor frame-string mix flags
// (SIM_Plane.cpp:61-74). kDspoilers/kRedundant need extra servo channels
// this port's four-float update() does not carry; those mixes live on
// mix_dspoilers()/mix_redundant() instead of the update() path.
enum class AirframeMix : std::uint8_t {
    kStandard = 0,
    kElevons = 1,
    kVtail = 2,
    kDspoilers = 3,
    kRedundant = 4,
};

// CPP-094: real upstream `-heavy`/`-jet` frame-string mass/thrust_scale
// overrides (SIM_Plane.cpp:53-59), reproduced as an explicit constructor
// enum - same ADR-0012 shape as AirframeMix above - rather than raw
// frame_str string parsing, which this class has no other trace of.
// Verified asymmetry (re-checked directly against the real source, not
// assumed symmetric): `-heavy` is `mass = 8;` ALONE; `-jet` is `mass = 22;
// thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;` - jet recomputes
// thrust_scale from its own new mass, heavy does not. See the SimPlane
// constructor's own doc comment for how that asymmetry is expressed here
// (SimPlane::heavy_frozen_mass_kg_) and for why kHeavy+kJet don't need a
// combinable bitmask: upstream applies both real `if`s unconditionally in
// sequence, so a frame_str carrying both suffixes ends up behaviorally
// IDENTICAL to kJet alone (jet's unconditional mass/thrust_scale
// overwrite leaves nothing of heavy's effect observable) - a mutually
// exclusive 3-value enum loses no real distinguishable upstream behavior.
enum class MassVariant : std::uint8_t {
    kStandard = 0,
    kHeavy = 1,
    kJet = 2,
};

struct FrameConfig {
    AirframeMix mix = AirframeMix::kStandard;
    bool reverse_elevator_rudder = false;
    bool reverse_thrust = false;
};

struct SurfaceDeflections {
    float aileron = 0.0f;
    float elevator = 0.0f;
    float rudder = 0.0f;
    float throttle = 0.0f;
};

struct DspoilerInputs {
    float dspoiler1_left = 0.0f;
    float dspoiler1_right = 0.0f;
    float dspoiler2_left = 0.0f;
    float dspoiler2_right = 0.0f;
};

struct RedundantInputs {
    float aileron_left = 0.0f;
    float aileron_right = 0.0f;
    float elevator_left = 0.0f;
    float elevator_right = 0.0f;
    float rudder_top = 0.0f;
    float rudder_bottom = 0.0f;
};

// Ground-truth fixed-wing flight dynamics model - upstream: SITL::Plane
// (STANDARD configuration only; see file banner for every excluded
// variant). Owns the aircraft's true attitude/velocity/position state and
// advances it one `update()` call at a time given control-surface
// deflections and throttle.
class SimPlane : public Aircraft {
public:
    // Upstream: Plane::Plane() sets coefficient = default_coefficients,
    // mass = 2.0f, hover_throttle is a const 0.7f member. All three are
    // constructor-overridable here since nothing in this port's JSON-model
    // sense applies, but the defaults reproduce upstream's own standard
    // "skywalker_2013"-equivalent plane with zero configuration.
    //
    // wind_rng_seed has no upstream counterpart (upstream's rand_normal()
    // draws from a process-global libc rand() stream with no seed a
    // caller controls) - see file banner's "rand_normal()-equivalent"
    // note. Defaulted to a fixed, arbitrary constant rather than a
    // time/random_device seed so `SimPlane plane;` stays fully
    // deterministic and reproducible in tests that never touch wind_config
    // (the overwhelming majority of existing sim_plane_test.cpp cases) -
    // a caller exercising turbulence and wanting a specific sequence
    // passes their own seed explicitly.
    //
    // mass_variant (CPP-094, default kStandard - identical behavior to
    // before this ticket) applies the real upstream `-heavy`/`-jet`
    // frame-string overrides - see MassVariant's own doc comment above for
    // the full derivation and real line citations. Applied AFTER mass_kg
    // so mass_kg still names the pre-override baseline mass (matching
    // upstream's own real "mass = 2.0f, THEN maybe overwritten" order,
    // SIM_Plane.cpp:41 vs 53-59).
    explicit SimPlane(const Coefficients& coeffs = Coefficients{}, float mass_kg = 2.0f, float hover_throttle = 0.7f,
                       std::uint32_t wind_rng_seed = 20260827U, MassVariant mass_variant = MassVariant::kStandard)
        : Aircraft(wind_rng_seed), coefficient(coeffs), hover_throttle(hover_throttle) {
        mass = mass_kg;
        switch (mass_variant) {
        case MassVariant::kStandard:
            break;
        case MassVariant::kHeavy:
            // Upstream SIM_Plane.cpp:53-54: `if (strstr(frame_str,
            // "-heavy")) { mass = 8; }` - mass alone changes. thrust_scale
            // must NOT follow: freeze update()'s thrust-scale calculation
            // at the PRE-heavy mass_kg (2.0f by default here, matching
            // upstream's real mass=2.0f baseline at the point this branch
            // runs) via heavy_frozen_mass_kg_ - see that field's own doc
            // comment and update()'s own thrust_scale line.
            heavy_frozen_mass_kg_ = mass_kg;
            mass = 8.0f;
            break;
        case MassVariant::kJet:
            // Upstream SIM_Plane.cpp:56-58 ("a 22kg jet, level top speed is
            // 102m/s", comment transcribed verbatim): mass=22, AND
            // thrust_scale recomputed from that new mass. No freeze
            // needed: this port's update() already recomputes thrust_scale
            // from the live `mass` field every call by default (see
            // update()'s own comment on why) - kJet's real "recompute from
            // the new mass" behavior falls out for free.
            mass = 22.0f;
            break;
        }
        dcm.identity();
        ground_behavior = GroundBehavior::kNone;
    }

    bool load_coeffs(const char* model_json) {
        JsonValue obj;
        std::string err;
        if (!load_json_file(model_json, obj, err)) {
            return false;
        }
        json_get_float(obj, "s", coefficient.s);
        json_get_float(obj, "b", coefficient.b);
        json_get_float(obj, "c", coefficient.c);
        json_get_float(obj, "c_lift_0", coefficient.c_lift_0);
        json_get_float(obj, "c_lift_deltae", coefficient.c_lift_deltae);
        json_get_float(obj, "c_lift_a", coefficient.c_lift_a);
        json_get_float(obj, "c_lift_q", coefficient.c_lift_q);
        json_get_float(obj, "mcoeff", coefficient.mcoeff);
        json_get_float(obj, "oswald", coefficient.oswald);
        json_get_float(obj, "alpha_stall", coefficient.alpha_stall);
        json_get_float(obj, "c_drag_q", coefficient.c_drag_q);
        json_get_float(obj, "c_drag_deltae", coefficient.c_drag_deltae);
        json_get_float(obj, "c_drag_p", coefficient.c_drag_p);
        json_get_float(obj, "c_y_0", coefficient.c_y_0);
        json_get_float(obj, "c_y_b", coefficient.c_y_b);
        json_get_float(obj, "c_y_p", coefficient.c_y_p);
        json_get_float(obj, "c_y_r", coefficient.c_y_r);
        json_get_float(obj, "c_y_deltaa", coefficient.c_y_deltaa);
        json_get_float(obj, "c_y_deltar", coefficient.c_y_deltar);
        json_get_float(obj, "c_l_0", coefficient.c_l_0);
        json_get_float(obj, "c_l_p", coefficient.c_l_p);
        json_get_float(obj, "c_l_b", coefficient.c_l_b);
        json_get_float(obj, "c_l_r", coefficient.c_l_r);
        json_get_float(obj, "c_l_deltaa", coefficient.c_l_deltaa);
        json_get_float(obj, "c_l_deltar", coefficient.c_l_deltar);
        json_get_float(obj, "c_m_0", coefficient.c_m_0);
        json_get_float(obj, "c_m_a", coefficient.c_m_a);
        json_get_float(obj, "c_m_q", coefficient.c_m_q);
        json_get_float(obj, "c_m_deltae", coefficient.c_m_deltae);
        json_get_float(obj, "c_n_0", coefficient.c_n_0);
        json_get_float(obj, "c_n_b", coefficient.c_n_b);
        json_get_float(obj, "c_n_p", coefficient.c_n_p);
        json_get_float(obj, "c_n_r", coefficient.c_n_r);
        json_get_float(obj, "c_n_deltaa", coefficient.c_n_deltaa);
        json_get_float(obj, "c_n_deltar", coefficient.c_n_deltar);
        json_get_float(obj, "deltaa_max", coefficient.deltaa_max);
        json_get_float(obj, "deltae_max", coefficient.deltae_max);
        json_get_float(obj, "deltar_max", coefficient.deltar_max);
        // CPP-094: this key was "cg" until this ticket - upstream's real
        // key (SIM_Plane.cpp:191, `{ "CGOffset", &coefficient.CGOffset,
        // VarType::VECTOR3F }`) is "CGOffset". A real, silent inherited bug:
        // every real-world model file (including skywalker_2013.json
        // itself) uses "CGOffset", so the old key never matched anything
        // and coefficient.cg_offset silently stayed at whatever the
        // constructor default already was - undetected because that
        // default happens to equal skywalker_2013.json's own CGOffset
        // value, so no prior (synthetic-fixture) test could have caught it.
        json_get_vector3(obj, "CGOffset", coefficient.cg_offset);
        return true;
    }

    // Upstream: Plane::liftCoeff (SIM_Plane.cpp:235) - from last_letter,
    // https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp,
    // thanks to Georacer! Sigmoid-blended stall model: a linear small-alpha
    // lift term smoothly blended into a flat-plate beyond-stall term as
    // alpha approaches/exceeds alpha_stall. alpha is clamped to
    // alpha_stall +/- 0.8 before use, to avoid exp() overflow in the
    // sigmoid - ported exactly, including the clamp.
    [[nodiscard]] float liftCoeff(float alpha) const {
        const float alpha0 = coefficient.alpha_stall;
        const float M = coefficient.mcoeff;
        const float c_lift_0 = coefficient.c_lift_0;
        const float c_lift_a0 = coefficient.c_lift_a;

        // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
        const float max_alpha_delta = 0.8f;
        if (alpha - alpha0 > max_alpha_delta) {
            alpha = alpha0 + max_alpha_delta;
        } else if (alpha0 - alpha > max_alpha_delta) {
            alpha = alpha0 - max_alpha_delta;
        }

        const double sigmoid = (1 + std::exp(-M * (alpha - alpha0)) + std::exp(M * (alpha + alpha0)))
                              / (1 + std::exp(-M * (alpha - alpha0))) / (1 + std::exp(M * (alpha + alpha0)));
        const double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a0 * alpha); // Lift at small AoA
        const double flatPlate = sigmoid * (2 * std::copysign(1.0, alpha) * std::pow(std::sin(alpha), 2) * std::cos(alpha)); // Lift beyond stall

        const float result = static_cast<float>(linear + flatPlate);
        return result;
    }

    // Upstream: Plane::dragCoeff (SIM_Plane.cpp:257) - simple
    // aspect-ratio-based induced drag model, same last_letter source.
    [[nodiscard]] float dragCoeff(float alpha) const {
        const float b = coefficient.b;
        const float s = coefficient.s;
        const float c_drag_p = coefficient.c_drag_p;
        const float c_lift_0 = coefficient.c_lift_0;
        const float c_lift_a0 = coefficient.c_lift_a;
        const float oswald = coefficient.oswald;

        const double AR = std::pow(b, 2) / s;
        const double c_drag_a = c_drag_p + std::pow(c_lift_0 + c_lift_a0 * alpha, 2) / (math::pi_constant() * oswald * AR);

        return static_cast<float>(c_drag_a);
    }

    // Upstream: Plane::getForce (SIM_Plane.cpp:344). alpha/beta/airspeed/
    // gyro/air_density are upstream member state (angle_of_attack, beta,
    // airspeed, gyro, air_density) taken here as explicit parameters
    // instead - see file banner's "no singletons" note. Zero-force guard
    // (is_zero(airspeed)) ported exactly.
    [[nodiscard]] math::Vector3f getForce(float inputAileron, float inputElevator, float inputRudder,
                                           float alpha, float beta, float airspeed,
                                           const math::Vector3f& gyro, float air_density) const {
        const float c_drag_q = coefficient.c_drag_q;
        const float c_lift_q = coefficient.c_lift_q;
        const float s = coefficient.s;
        const float c = coefficient.c;
        const float b = coefficient.b;
        const float c_drag_deltae = coefficient.c_drag_deltae;
        const float c_lift_deltae = coefficient.c_lift_deltae;
        const float c_y_0 = coefficient.c_y_0;
        const float c_y_b = coefficient.c_y_b;
        const float c_y_p = coefficient.c_y_p;
        const float c_y_r = coefficient.c_y_r;
        const float c_y_deltaa = coefficient.c_y_deltaa;
        const float c_y_deltar = coefficient.c_y_deltar;

        const float rho = air_density;

        // request lift and drag alpha-coefficients from the corresponding functions
        const double c_lift_a = liftCoeff(alpha);
        const double c_drag_a = dragCoeff(alpha);

        // convert coefficients to the body frame
        const double c_x_a = -c_drag_a * std::cos(alpha) + c_lift_a * std::sin(alpha);
        const double c_x_q = -c_drag_q * std::cos(alpha) + c_lift_q * std::sin(alpha);
        const double c_z_a = -c_drag_a * std::sin(alpha) - c_lift_a * std::cos(alpha);
        const double c_z_q = -c_drag_q * std::sin(alpha) - c_lift_q * std::cos(alpha);

        // read angular rates
        const double p = gyro.x;
        const double q = gyro.y;
        const double r = gyro.z;

        // calculate aerodynamic force
        const double qbar = 1.0 / 2.0 * rho * std::pow(airspeed, 2) * s; // Calculate dynamic pressure
        double ax, ay, az;
        if (math::is_zero(airspeed)) {
            ax = 0;
            ay = 0;
            az = 0;
        } else {
            ax = qbar * (c_x_a + c_x_q * c * q / (2 * airspeed) - c_drag_deltae * std::cos(alpha) * std::fabs(inputElevator)
                         + c_lift_deltae * std::sin(alpha) * inputElevator);
            // split c_x_deltae to include "abs" term
            ay = qbar * (c_y_0 + c_y_b * beta + c_y_p * b * p / (2 * airspeed) + c_y_r * b * r / (2 * airspeed)
                         + c_y_deltaa * inputAileron + c_y_deltar * inputRudder);
            az = qbar * (c_z_a + c_z_q * c * q / (2 * airspeed) - c_drag_deltae * std::sin(alpha) * std::fabs(inputElevator)
                         - c_lift_deltae * std::cos(alpha) * inputElevator);
            // split c_z_deltae to include "abs" term
        }
        return math::Vector3f(static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az));
    }

    // Upstream: Plane::getTorque (SIM_Plane.cpp:273). The
    // `if (tailsitter || aerobatic)` effective-airspeed/alpha adjustment at
    // the top of upstream's version is EXCLUDED - see file banner - so
    // alpha/airspeed are used exactly as passed in. inputThrust is kept in
    // the signature to match upstream's own parameter list even though,
    // with that branch excluded, this slice never reads it - matches
    // upstream's exact interface shape for a future slice that re-adds
    // tailsitter/aerobatic support.
    [[nodiscard]] math::Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder,
                                            [[maybe_unused]] float inputThrust, const math::Vector3f& force,
                                            float alpha, float airspeed, float beta,
                                            const math::Vector3f& gyro, float air_density) const {
        // calculate aerodynamic torque
        const float effective_airspeed = airspeed;

        const float s = coefficient.s;
        const float c = coefficient.c;
        const float b = coefficient.b;
        const float c_l_0 = coefficient.c_l_0;
        const float c_l_b = coefficient.c_l_b;
        const float c_l_p = coefficient.c_l_p;
        const float c_l_r = coefficient.c_l_r;
        const float c_l_deltaa = coefficient.c_l_deltaa;
        const float c_l_deltar = coefficient.c_l_deltar;
        const float c_m_0 = coefficient.c_m_0;
        const float c_m_a = coefficient.c_m_a;
        const float c_m_q = coefficient.c_m_q;
        const float c_m_deltae = coefficient.c_m_deltae;
        const float c_n_0 = coefficient.c_n_0;
        const float c_n_b = coefficient.c_n_b;
        const float c_n_p = coefficient.c_n_p;
        const float c_n_r = coefficient.c_n_r;
        const float c_n_deltaa = coefficient.c_n_deltaa;
        const float c_n_deltar = coefficient.c_n_deltar;
        const math::Vector3f& cg_offset = coefficient.cg_offset;

        const float rho = air_density;

        // read angular rates
        const double p = gyro.x;
        const double q = gyro.y;
        const double r = gyro.z;

        const double qbar = 1.0 / 2.0 * rho * std::pow(effective_airspeed, 2) * s; // Calculate dynamic pressure
        double la, na, ma;
        if (math::is_zero(effective_airspeed)) {
            la = 0;
            ma = 0;
            na = 0;
        } else {
            la = qbar * b
                 * (c_l_0 + c_l_b * beta + c_l_p * b * p / (2 * effective_airspeed) + c_l_r * b * r / (2 * effective_airspeed)
                    + c_l_deltaa * inputAileron + c_l_deltar * inputRudder);
            ma = qbar * c * (c_m_0 + c_m_a * alpha + c_m_q * c * q / (2 * effective_airspeed) + c_m_deltae * inputElevator);
            na = qbar * b
                 * (c_n_0 + c_n_b * beta + c_n_p * b * p / (2 * effective_airspeed) + c_n_r * b * r / (2 * effective_airspeed)
                    + c_n_deltaa * inputAileron + c_n_deltar * inputRudder);
        }

        // Add torque to force misalignment with CG
        // r x F, where r is the distance from CoG to CoL
        la += cg_offset.y * force.z - cg_offset.z * force.y;
        ma += -cg_offset.x * force.z + cg_offset.z * force.x;
        na += -cg_offset.y * force.x + cg_offset.x * force.y;

        return math::Vector3f(static_cast<float>(la), static_cast<float>(ma), static_cast<float>(na));
    }

    // Upstream: Aircraft::rand_normal(0, 1) (SIM_Aircraft.cpp:343), always
    // called with mean=0/stddev=1 at every real call site update_wind()
    // uses - see file banner's "rand_normal()-equivalent" note for the
    // std::mt19937/std::normal_distribution substitution rationale.
    [[nodiscard]] double rand_normal() { return wind_normal_dist_(wind_rng_); }

    // Upstream: Aircraft::update_wind (SIM_Aircraft.cpp:888, read in full)
    // - see file banner's "WIND MODELING" note for the full trace of the
    // sign convention, the get_local_updraft() exclusion, and the RNG
    // substitutions. Takes no dt: upstream's own version doesn't either -
    // the turbulence IIR filter's implicit timestep is baked into
    // iir_coef=0.98 as a constant, tied to upstream's per-tick call
    // cadence, not to an explicit dt argument.
    void update_wind() {
        SitlInput in;
        in.wind.speed = wind_config.speed;
        in.wind.direction = wind_config.direction;
        in.wind.turbulence = wind_config.turbulence;
        in.wind.dir_z = wind_config.dir_z;
        Aircraft::update_wind(in);
        return;
        wind_ef = math::Vector3f(std::cos(math::radians(wind_config.direction)) * std::cos(math::radians(wind_config.dir_z)),
                                  std::sin(math::radians(wind_config.direction)) * std::cos(math::radians(wind_config.dir_z)),
                                  std::sin(math::radians(wind_config.dir_z)))
                  * wind_config.speed;

        // get_local_updraft() term EXCLUDED here - see file banner.

        // scale input.wind.turbulence to match standard deviation when
        // using iir_coef=0.98 - upstream's own comment, transcribed
        // verbatim (SIM_Aircraft.cpp:902-903).
        const float wind_turb = wind_config.turbulence * 10.0f;
        const float iir_coef = 0.98f;

        if (wind_turb > 0.0f && !on_ground()) {
            // re-randomize gust direction every tick - see file banner's
            // turbulence_azimuth note for the RNG substitution.
            turbulence_azimuth = std::fmod(turbulence_azimuth + wind_azimuth_step_dist_(wind_rng_), 360.0f);

            turbulence_horizontal_speed = static_cast<float>(turbulence_horizontal_speed * iir_coef
                                                               + wind_turb * rand_normal() * (1.0 - iir_coef));
            turbulence_vertical_speed = static_cast<float>(turbulence_vertical_speed * iir_coef
                                                             + wind_turb * rand_normal() * (1.0 - iir_coef));

            wind_ef += math::Vector3f(std::cos(math::radians(turbulence_azimuth)) * turbulence_horizontal_speed,
                                       std::sin(math::radians(turbulence_azimuth)) * turbulence_horizontal_speed,
                                       turbulence_vertical_speed);
        }

        // "the AHRS wants wind with opposite sense" - upstream's own
        // comment (SIM_Aircraft.cpp:915), transcribed verbatim. Negates
        // the ENTIRE vector built above (steady + turbulence together),
        // converting the meteorological "FROM heading" construction into
        // the physical earth-frame air-mass velocity this class's
        // velocity_air_ef = velocity_ef - wind_ef then consumes - see
        // file banner's algebraic verification of this sign flip.
        wind_ef = -wind_ef;
    }

    // Upstream: Aircraft::hagl() (SIM_Aircraft.cpp:145) / on_ground()
    // (SIM_Aircraft.cpp:153) - replaced by a flat-earth simplification, see
    // file banner's "GROUND MODEL" note. position.z follows NED convention
    // (down positive); the caller initializes position.z = -initial_altitude,
    // so position.z >= 0 means "at or below the starting ground plane".
    [[nodiscard]] bool on_ground() const { return Aircraft::on_ground(); }

    // CPP-030 leftover closer: Plane::calculate_forces surface mix
    // (SIM_Plane.cpp:405-447). reverse_elevator_rudder runs first (same
    // order as upstream). elevons/vtail are the real four-channel mixes.
    // kDspoilers/kRedundant on this four-channel path leave surfaces as
    // given; callers with extra channels use mix_dspoilers/mix_redundant.
    // reverse_thrust: this port has no PWM decode (throttle is already a
    // float); the flag is a stored leftover surface and throttle is used
    // as given (signed allowed).
    [[nodiscard]] SurfaceDeflections mix_surfaces(float aileron, float elevator, float rudder, float throttle) const {
        if (frame_config.reverse_elevator_rudder) {
            elevator = -elevator;
            rudder = -rudder;
        }
        SurfaceDeflections out;
        out.throttle = throttle;
        switch (frame_config.mix) {
        case AirframeMix::kElevons: {
            const float ch1 = aileron;
            const float ch2 = elevator;
            out.aileron = (ch2 - ch1) / 2.0f;
            out.elevator = -(ch2 + ch1) / 2.0f;
            out.rudder = 0.0f;
            break;
        }
        case AirframeMix::kVtail: {
            const float ch1 = elevator;
            const float ch2 = rudder;
            out.aileron = aileron;
            out.elevator = (ch2 - ch1) / 2.0f;
            out.rudder = (ch2 + ch1) / 2.0f;
            break;
        }
        case AirframeMix::kDspoilers:
        case AirframeMix::kRedundant:
        case AirframeMix::kStandard:
            out.aileron = aileron;
            out.elevator = elevator;
            out.rudder = rudder;
            break;
        }
        return out;
    }

    // Upstream: SIM_Plane.cpp:426-436 (channels 1/2/4/5 as dspoilers).
    [[nodiscard]] static SurfaceDeflections mix_dspoilers(const DspoilerInputs& in) {
        const float elevon_left = (in.dspoiler1_left + in.dspoiler2_left) / 2.0f;
        const float elevon_right = (in.dspoiler1_right + in.dspoiler2_right) / 2.0f;
        SurfaceDeflections out;
        out.aileron = (elevon_right - elevon_left) / 2.0f;
        out.elevator = (elevon_left + elevon_right) / 2.0f;
        out.rudder = std::fabs(in.dspoiler1_right - in.dspoiler2_right) / 2.0f
                     - std::fabs(in.dspoiler1_left - in.dspoiler2_left) / 2.0f;
        return out;
    }

    // Upstream: SIM_Plane.cpp:437-446 (paired leftover channels averaged).
    [[nodiscard]] static SurfaceDeflections mix_redundant(const RedundantInputs& in) {
        SurfaceDeflections out;
        out.aileron = (in.aileron_left + in.aileron_right) / 2.0f;
        out.elevator = (in.elevator_left + in.elevator_right) / 2.0f;
        out.rudder = (in.rudder_top + in.rudder_bottom) / 2.0f;
        return out;
    }

    // CPP-030 leftover closer: Aircraft::update_dynamics switch
    // (SIM_Aircraft.cpp:787-868). Ship/tether gnd_movement is always
    // zero (no ship sim). kTailsitter is a no-op (out of scope).
    void apply_ground_behavior([[maybe_unused]] float dt) {
        if (!on_ground()) {
            return;
        }
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
                p = (p > 0.0f) ? p : 0.0f;
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
        case GroundBehavior::kTailsitter:
            break;
        }
    }

    // Upstream: thrust_scale = (mass * GRAVITY_MSS) / hover_throttle
    // (SIM_Plane.cpp:47), computed once in Plane::Plane(); computed
    // per-call here instead since mass/hover_throttle are plain public
    // fields a caller may change between calls (a port-specific
    // convenience upstream's own single-shot constructor computation
    // doesn't offer). CPP-094: exposed as its own accessor (rather than
    // an update()-local variable) so heavy_frozen_mass_kg_'s real
    // MassVariant::kHeavy asymmetry - mass changes, thrust_scale does NOT
    // follow - is directly testable without stepping full update()/
    // update_dynamics() physics. Reads heavy_frozen_mass_kg_ instead of
    // `mass` directly whenever it's set (>= 0.0f, only ever true for
    // kHeavy); for every kStandard/kJet caller (including every
    // pre-CPP-094 caller, since heavy_frozen_mass_kg_ defaults to the
    // sentinel) this reduces to `(mass * kGravityMss) / hover_throttle`,
    // byte-for-byte unchanged from before CPP-094.
    [[nodiscard]] float thrust_scale() const {
        const float mass_for_thrust = (heavy_frozen_mass_kg_ >= 0.0f) ? heavy_frozen_mass_kg_ : mass;
        return (mass_for_thrust * kGravityMss) / hover_throttle;
    }

    // Upstream: Plane::calculate_forces (SIM_Plane.cpp:398) + the thrust-
    // scaling/ground-friction tail of it, folded together with
    // Aircraft::update_dynamics (SIM_Aircraft.cpp:709) into one per-tick
    // entry point - upstream's own Plane::update (SIM_Plane.cpp:522) does
    // the same two-call sequence (calculate_forces then update_dynamics),
    // just via a `struct sitl_input` this port has no equivalent of. Every
    // STANDARD-config branch is reproduced; every config-variant branch
    // (elevon/vtail/dspoilers/redundant/reverse_thrust/
    // reverse_elevator_rudder/tailsitter/aerobatic/launcher) is skipped -
    // see file banner.
    void update(float aileron, float elevator, float rudder, float throttle, float dt) {
        // Upstream: Plane::update calls update_wind(input) FIRST, before
        // calculate_forces/update_dynamics (SIM_Plane.cpp:526) - reproduced
        // in the same order. Uses THIS tick's pre-integration on_ground()
        // state (matching upstream exactly, since upstream's update_wind()
        // call also precedes the position integration inside
        // update_dynamics()).
        update_wind();

        // CPP-030 leftover closer: Plane::calculate_forces surface-mixing
        // (SIM_Plane.cpp:405-447). Default FrameConfig is identity.
        const SurfaceDeflections mixed = mix_surfaces(aileron, elevator, rudder, throttle);

        // calculate angle of attack (upstream: Plane::calculate_forces,
        // reading the PREVIOUS tick's velocity_air_bf - exactly reproduced:
        // velocity_air_bf here is only ever written by update_dynamics(),
        // at the end of the previous update() call, or left at its
        // zero-initialized default on the very first call).
        angle_of_attack = std::atan2(velocity_air_bf.z, velocity_air_bf.x);
        beta = std::atan2(velocity_air_bf.y, velocity_air_bf.x);

        const math::Vector3f force = getForce(mixed.aileron, mixed.elevator, mixed.rudder, angle_of_attack, beta, airspeed, gyro, air_density);
        math::Vector3f rot_accel = getTorque(mixed.aileron, mixed.elevator, mixed.rudder, mixed.throttle, force, angle_of_attack, airspeed, beta, gyro, air_density);

        // DELIBERATE, DEFAULT-PRESERVING DIVERGENCE (WOPR bridge, 2026-08-30):
        // upstream Plane::calculate_forces uses getTorque()'s raw aerodynamic
        // MOMENT (N*m) directly as rot_accel (rad/s^2) - an implicit 1 kg*m^2
        // inertia on every axis. Tolerable for the 2 kg skywalker default;
        // catastrophically wrong for a JSON-loaded heavy airframe, whose
        // moments scale with qbar*S*b (~1000x) while the implicit inertia
        // stays 1 (observed: instant ground tumble on a 65 t model). The
        // default {1,1,1} reproduces upstream bit-identically; a model file
        // supplies its real per-axis inertia (kg*m^2) via the WOPR bridge's
        // "moment_inertia" key.
        rot_accel.x /= std::fmax(moment_inertia.x, 1.0e-6f);
        rot_accel.y /= std::fmax(moment_inertia.y, 1.0e-6f);
        rot_accel.z /= std::fmax(moment_inertia.z, 1.0e-6f);

        // scale thrust to newtons - see thrust_scale()'s own doc comment
        // above for the upstream citation and the CPP-094
        // heavy_frozen_mass_kg_ handling.
        const float thrust_newtons = mixed.throttle * thrust_scale();

        accel_body = math::Vector3f(thrust_newtons, 0.0f, 0.0f) + force;
        accel_body = accel_body / mass;

        if (on_ground()) {
            // add some ground friction — upstream's constant 0.3f, promoted to
            // a field for the same default-preserving reason as moment_inertia:
            // a speed-proportional 0.3/s drag equilibrates the ground roll at
            // thrust_accel/0.3 m/s, which for a realistic T/W (0.2-0.25) is
            // ~7 m/s — below any real rotate speed. Wheeled JSON models set a
            // rolling-resistance-scale value (~0.02) via "ground_friction".
            const math::Vector3f vel_body = dcm.transposed() * velocity_ef;
            accel_body.x -= vel_body.x * ground_friction;
        }

        update_dynamics(rot_accel, dt);
        // Refresh the geodetic Location from the freshly integrated NED
        // position. Every sibling plant ends its update() with this same
        // epilogue slot (SimMulticopter / SimQuadPlane / SimRover / ... :
        // update_dynamics, time_advance, update_position, update_mag_field_bf)
        // and so do upstream's Aircraft subclasses; SimPlane was the one that
        // omitted it. Ordered AFTER update_dynamics to match them exactly, so
        // the density used by step N is the position at the end of step N-1.
        //
        // It went unnoticed because nothing in the fixed-wing CONTROL path
        // reads location -- SitlHarness feeds the controller from NED state
        // and the compass from rotate_earth_field_to_body(dcm). Two consumers
        // do read it and were wrong for an entire flight while it stayed
        // pinned at the start fix:
        //   - Aircraft::update_dynamics recomputes eas2tas and air_density
        //     from location.alt, so air density was frozen at the takeoff
        //     altitude however high the aircraft climbed;
        //   - wopr_bridge's MAVLink GLOBAL_POSITION_INT reports location.lat/
        //     lng, so a connected GCS drew the aircraft parked on the runway.
        update_position();
    }

    // CPP-093: this override now fully delegates to the base class.
    // Upstream: Aircraft::update_dynamics (SIM_Aircraft.cpp:709) - the
    // rigid-body integrator (gyro integration + clamp, accel clamp, DCM
    // rotate+normalize, body->earth accel rotation plus gravity, ground
    // accel/velocity clamps, velocity/position integration, wind-relative
    // velocity, and update_eas_airspeed()) plus the real, altitude-dependent
    // eas2tas/air_density recompute and the on-ground apply_ground_behavior()
    // call - all already correctly implemented in Aircraft::update_dynamics
    // (this file's base class, see sim_aircraft.hpp). A prior hand-rolled
    // copy of this same integration lived here as dead code below an
    // unconditional early `return` (a real eas2tas=1.0 simplification of
    // the base class's own real altitude-dependent value) - it was verified
    // during CPP-093 to be fully superseded (the base class computes a real
    // eas2tas rather than assuming 1.0, and SimPlane::apply_ground_behavior
    // already no-ops when off the ground, so the base class's on-ground-only
    // call site is behaviorally identical to the dead code's unconditional
    // one) and was removed rather than left unreachable.
    void update_dynamics(const math::Vector3f& rot_accel, float dt) {
        Aircraft::update_dynamics(rot_accel, dt);
    }

    // CPP-082 - upstream: HALSITL::SITL_State::_update_airspeed(true_
    // airspeed) (AP_HAL_SITL/sitl_airspeed.cpp, read in full). Produces
    // the raw differential pressure (Pa) a real airspeed pitot sensor
    // would report for this aircraft's CURRENT true airspeed (the
    // `airspeed` member above, already EAS==TAS at this port's own
    // established eas2tas=1.0 simplification - see that field's own doc
    // comment and this ticket's own "EAS2TAS" note) - the value a caller
    // feeds into fwcpp::airspeed::AirspeedSensor::update() (ap-airspeed)
    // to drive the vehicle's real sensor model end to end.
    //
    // WITHOUT the real fail/fail_pressure/signflip branches
    // (sitl_airspeed.cpp's own arspd.fail/arspd.fail_pressure/
    // arspd.signflip - sensor failure injection, explicitly deferred to
    // a future ticket, see modules/ap-airspeed's own file banner for the
    // fuller exclusion list) and WITHOUT the SITL-side arspd.offset
    // additive term real upstream's own _update_airspeed() computes -
    // VERIFIED DIRECTLY against AP_Airspeed_SITL.cpp's real
    // get_differential_pressure() (the backend AP_Airspeed::init()
    // actually selects for CONFIG_HAL_BOARD==HAL_BOARD_SITL builds, NOT
    // the legacy analog-pin path _update_airspeed()'s own
    // airspeed_pin_voltage/PASCAL_TO_VOLTS machinery exists to serve):
    // get_differential_pressure() returns `_sitl->state.airspeed_raw_
    // pressure[i]`, which _update_airspeed() assigns BEFORE adding
    // arspd.offset (`_sitl->state.airspeed_raw_pressure[i] =
    // airspeed_pressure;` textually precedes `airspeed_raw =
    // airspeed_pressure + arspd.offset;` - only airspeed_raw/
    // airspeed_pin_voltage, read solely by the unported analog backend,
    // ever sees that offset term). So the REAL modern SITL backend this
    // port models never applies that additive term at all - reproduced
    // faithfully here by omitting it, not by silently dropping a real
    // behavior the ticket's own summary (written before this direct
    // re-verification) implied existed.
    //
    // Formula (sitl_airspeed.cpp), ratio/noise_amplitude passed
    // explicitly - see kDefaultAirspeedSensorRatio/kDefaultAirspeedNoisePa
    // above for why phase 1 reuses one shared ratio rather than
    // upstream's two independent real parameters:
    //   eas = true_airspeed / eas2tas;             // eas2tas == 1.0 here, so eas == true_airspeed == this->airspeed
    //   diff_pressure = eas^2 / ratio;
    //   eas_noisy = sqrt(|ratio * (diff_pressure + noise_amplitude * rand_float())|);
    //   airspeed_pressure = eas_noisy^2 / ratio;    // == AP_Airspeed_SITL::get_differential_pressure()'s real return
    // eas_noisy^2/ratio is algebraically identical to
    // |diff_pressure + noise_amplitude*rand_float()| for any ratio > 0
    // (a real, always-positive calibration constant) - the sqrt-then-
    // square round trip is kept literal anyway, matching upstream's own
    // statement-for-statement structure exactly rather than an
    // algebraically-equivalent but harder-to-audit rewrite (same
    // "byte-for-byte" fidelity precedent as compass.hpp's from_euler
    // reproduction).
    //
    // rand_float() is upstream's own UNIFORM [-1,1] draw (AP_Math.cpp's
    // real rand_float(), verified directly - NOT Gaussian, despite this
    // class's OWN wind-turbulence model using a normal distribution for
    // a different, unrelated real upstream quantity). Reproduced with
    // airspeed_noise_dist_ (std::uniform_real_distribution<float>(-1,1))
    // drawn from this SAME instance's wind_rng_ engine - reusing the one
    // per-instance, explicitly-seeded engine already established by
    // WindConfig's turbulence model (see wind_rng_seed's own doc comment
    // below) rather than adding a second, separately-seeded engine for
    // this second, unrelated noise source. Determinism note: this means
    // a caller who also drives update_wind() interleaves both noise
    // streams from the one engine - acceptable for a test harness (both
    // are still fully deterministic for a given seed and call sequence)
    // and matches upstream's own real behavior of every SITL noise
    // source sharing ONE process-global libc rand() stream.
    [[nodiscard]] float airspeed_sensor_differential_pressure(float ratio = kDefaultAirspeedSensorRatio,
                                                               float noise_amplitude = kDefaultAirspeedNoisePa) {
        const float eas = airspeed; // eas2tas == 1.0 - see doc comment above
        const float diff_pressure = (eas * eas) / ratio;
        const float eas_noisy =
            std::sqrt(std::fabs(ratio * (diff_pressure + noise_amplitude * airspeed_noise_dist_(wind_rng_))));
        return (eas_noisy * eas_noisy) / ratio;
    }

    // Aerodynamic/mass model - upstream: Plane::coefficient (assigned from
    // default_coefficients, or JSON-loaded - see file banner), Plane::mass
    // (Aircraft::mass, 2.0f), Plane::hover_throttle (const 0.7f).
    Coefficients coefficient;
    float hover_throttle;
    // Per-axis moment of inertia (kg*m^2) dividing getTorque()'s moment into
    // rot_accel. {1,1,1} = upstream's own implicit-unit-inertia behavior,
    // bit-identical - see the update() divergence note.
    math::Vector3f moment_inertia{1.0f, 1.0f, 1.0f};
    // Speed-proportional ground drag (1/s). 0.3f = upstream's own constant,
    // bit-identical - see the on_ground() note in update().
    float ground_friction = 0.3f;
    std::uniform_real_distribution<float> airspeed_noise_dist_{-1.0f, 1.0f};
    WindConfig wind_config;
    FrameConfig frame_config;
    float angle_of_attack = 0.0f;
    float beta = 0.0f;

    // CPP-094: sentinel < 0.0f means "not frozen" - update()'s thrust_scale
    // calculation follows the live `mass` field every call, exactly as it
    // did before this ticket (MassVariant::kStandard/kJet). Only
    // MassVariant::kHeavy's constructor branch ever sets this, to the
    // PRE-heavy mass_kg, reproducing the real upstream asymmetry where
    // `-heavy` changes mass alone and thrust_scale does not follow - see
    // the constructor's own doc comment and MassVariant's doc comment
    // above the enum for the full derivation.
    float heavy_frozen_mass_kg_ = -1.0f;
};

} // namespace fwcpp::sim

// LEFTOVER CLOSER (CPP-030): ground_behavior taxi/takeoff variants
// (kNone / kNoMovement / kFwdOnly) and FW airframe mix (elevons / vtail
// plus dspoilers / redundant / reverse_* leftover surfaces) are stubbed
// on this class. tailsitter (airframe + GROUND_BEHAVIOR_TAILSITTER) is
// formally out of scope — fw-cpp is fixed-wing only. See
// fwcpp/sim/sim_leftover.hpp for the leftover-complete catalog.
