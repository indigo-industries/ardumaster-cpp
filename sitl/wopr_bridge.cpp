// WOPR-BRIDGE: a lockstep UDP bridge around SitlHarness (CPP-084) so an
// external host (the WOPR5000 Unreal sim) can use this port's real Plane
// flight stack + SimPlane plant as an external autopilot process.
//
// DESIGN, in this port's own terms:
//   - This is the "future real work" CPP-085's banner deferred ("an
//     indefinite/interactive run mode"): the same Plane + SimPlane +
//     SitlHarness triple sitl_run's main() drives, but stepped on demand
//     by a remote clock instead of a bounded local for-loop.
//   - The HOST OWNS TIME. Nothing here reads a wall clock (ADR-0012's
//     no-hidden-clock stance extends naturally to the wire): each STEP
//     request carries how many fixed 20ms ticks to run, the bridge runs
//     exactly that many harness.step() calls, and replies with the
//     resulting state. Bit-deterministic, lockstep, host-paced.
//   - ONE FRAME CONVENTION ON THE WIRE: NED metres relative to the
//     vehicle's fixed start point — the exact frame SimPlane::position /
//     Plane::update_current_loc() already agree on (current_loc is
//     Location() offset by position_ned; mission Locations are built the
//     same way here). The host does all true geodesy; the real start
//     lat/lng/alt are given to the SIM only (set_start_location) so the
//     mag-field model and air density are geographically honest.
//   - UDP request/reply, one datagram each way, loopback by default. The
//     reply to EVERY request type is the same STATE packet (echoing the
//     request's seq), so the host needs exactly one decoder and any lost
//     datagram is recovered by the host simply re-sending.
//
// WIRE FORMAT: packed little-endian structs (both ends are x86-64; the
// static_asserts below pin the layout). All floats are IEEE-754 binary32.
//
// Caller responsibilities mirrored from the modes' own class banners:
//   - FBWB/CRUISE/LOITER never set target_altitude_cm in enter() (their
//     banners: "A CALLER MUST CALL set_target_altitude_current") — MODE
//     handling here does it for them from the sim's current truth.
//   - RTL/AUTO need home — INIT sets plane home to the frame origin
//     (Location(), alt 0 ABSOLUTE = the start point itself).
//   - Arming uses the same primitives sitl_run's main() uses (armed=true
//     + force_safety_off), for the same rc-gate reason its banner cites.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <span>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
// wingdi.h (dragged in by windows.h) defines ABSOLUTE/RELATIVE as macros,
// colliding with fwcpp::Location::AltFrame::ABSOLUTE.
#undef ABSOLUTE
#undef RELATIVE
using socklen_type = int;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using socklen_type = socklen_t;
#endif

#include <chrono>

#include <fwcpp/gcs/framing.hpp>
#include <fwcpp/hal_sitl/sitl_harness.hpp>
#include <fwcpp/hal_sitl/sitl_quadplane_harness.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/q_modes/mode_qhover.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/sim/sim_quadplane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

#ifdef _WIN32
using sock_type = SOCKET;
#else
using sock_type = int;
#endif

namespace {

constexpr std::uint32_t kMagic = 0x57534231; // 'WSB1'

// sim_json has no string getter (its frame/coeff schemas are all numeric);
// the bridge's model schema carries two string keys, read with the same
// JsonValue idiom json_get_float uses.
inline bool json_get_string(const fwcpp::sim::JsonValue& obj, const char* key, std::string& dest) {
    const fwcpp::sim::JsonValue* v = obj.get(key);
    if (v == nullptr || v->type != fwcpp::sim::JsonValue::Type::kString) {
        return false;
    }
    dest = v->str;
    return true;
}

// Directory of a path (both separators), WITH the trailing separator; empty
// when the path has none.
inline std::string path_dirname(const char* path) {
    const std::string p(path);
    const std::size_t cut = p.find_last_of("/\\");
    return (cut == std::string::npos) ? std::string() : p.substr(0, cut + 1);
}
constexpr float kDt = 0.02f;                 // this port's established Plane tick rate (50Hz)
constexpr std::uint16_t kDefaultPort = 9101;
constexpr std::uint16_t kMaxStepsPerRequest = 64; // 1.28 sim-seconds; caps a host dt spiral

// --- requests -------------------------------------------------------------

enum class MsgType : std::uint8_t {
    kInit = 1,
    kStep = 2,
    kMode = 3,
    kMission = 4,
    kArm = 5,
};

#pragma pack(push, 1)
struct RequestHeader {
    std::uint32_t magic;
    std::uint8_t type;
    std::uint8_t pad[3];
    std::uint32_t seq;
};
static_assert(sizeof(RequestHeader) == 12);

struct InitPayload {
    std::int32_t lat_1e7;   // true start latitude, 1e-7 deg (sim-side geodesy only)
    std::int32_t lng_1e7;   // true start longitude, 1e-7 deg
    float alt_m;            // true start altitude, metres (sim ground level / air density)
    float yaw_deg;          // initial true heading
    std::uint8_t in_air;    // 0 = on ground at rest; 1 = warm-start from the state below
    std::uint8_t pad[3];
    float pos_ned_m[3];     // in_air only: NED metres from start (z negative = above start)
    float vel_ned_mps[3];   // in_air only
    float rpy_rad[3];       // in_air only: roll/pitch/yaw
    // Optional airframe model file (NUL-terminated path; empty = the built-in
    // skywalker_2013 defaults). One JSON carries BOTH halves of an airframe:
    // the SimPlane aero set load_coeffs() reads (upstream's own plane:*.json
    // last_letter schema — s/b/c, c_lift_*, c_drag_*, c_m_*, ..., "cg"), plus
    // bridge-applied extras: "mass" (kg), "hover_throttle" (full-throttle
    // T/W = 1/hover_throttle), and the FLIGHT-CODE envelope "airspeed_min"/
    // "airspeed_cruise"/"airspeed_max"/"scaling_speed" (m/s → plane.aparm) —
    // a heavy plant with the default 12 m/s TECS target would just stall.
    char model_json[260];
};
static_assert(sizeof(InitPayload) == 316);

struct StepPayload {
    std::uint16_t n_steps;  // number of 20ms ticks to run (clamped to kMaxStepsPerRequest)
    std::uint16_t rc_pwm[4]; // roll/pitch/throttle/rudder microseconds; 0 = leave unchanged
};
static_assert(sizeof(StepPayload) == 10);

// Optional STEP tail appended by hosts that can sample real terrain under the
// aircraft (WOPR ticket c0faaf38): little-endian float terrain HAE metres +
// uint8 valid flag. Parsed manually (memcpy at fixed offsets) so StepPayload's
// 10-byte layout stays byte-stable — an old host omits the tail, an old bridge
// ignores it, and both keep flying on the flat start-fix ground plane.
constexpr int kStepTerrainTailLen = 5;

struct ModePayload {
    std::uint8_t mode; // BridgeMode below
};

struct ArmPayload {
    std::uint8_t armed;
};

struct MissionItemWire {
    std::uint8_t cmd; // 0 = Waypoint, 1 = Takeoff, 2 = Land
    std::uint8_t pad[3];
    float north_m;
    float east_m;
    float up_m; // metres above the start point
    float acceptance_radius_m;
    float takeoff_pitch_deg;
};
static_assert(sizeof(MissionItemWire) == 24);

struct MissionPayloadHeader {
    std::uint16_t count;
    std::uint16_t pad;
};

// --- reply ----------------------------------------------------------------

enum class Status : std::uint8_t {
    kOk = 0,
    kBadPacket = 1,
    kNotInitialized = 2,
    kBadMode = 3,
    kBadMission = 4,
    kModeRefused = 5,
    kBadModel = 6, // model_json given but unreadable/unparseable — REFUSED, never silently skywalker
};

struct StateReply {
    std::uint32_t magic;
    std::uint8_t type; // 0x81
    std::uint8_t status;
    std::uint8_t mode;      // BridgeMode currently active
    std::uint8_t on_ground; // sim truth
    std::uint32_t seq;      // echoed from the request
    float pos_ned_m[3];
    float vel_ned_mps[3];
    float rpy_rad[3]; // TRUE attitude from SimPlane's dcm (not the AHRS estimate)
    float gyro_rps[3];
    float airspeed_mps; // true airspeed from the plant
    float hagl_m;
    float servo_norm[4]; // aileron/elevator/rudder in [-1,1]; throttle in [0,1]
    float vtol_lift;     // lift-motor collective actually applied [0,1]; 0 on fixed-wing sessions
    float vtol_hover_cmd; // the frame's own hover collective (Frame::hover_command());
                          // 0 on fixed-wing sessions. The host normalizes lift
                          // against THIS, never a hard-coded constant.
    std::uint16_t mission_index;
    std::uint16_t mission_count;
    std::uint32_t sim_time_ms;
    std::uint8_t armed;
    std::uint8_t initialized;
    std::uint8_t pad[2];
};
static_assert(sizeof(StateReply) == 104);
#pragma pack(pop)

// Host-facing mode ids. Names, not plane.hpp internals, are the contract.
enum class BridgeMode : std::uint8_t {
    kManual = 0,
    kFbwa = 1,
    kFbwb = 2,
    kCruise = 3,
    kAuto = 4,
    kRtl = 5,
    kLoiter = 6,
    kTakeoff = 7,
    // VTOL models only ("vtol": 1 in the model JSON). QHOVER: the leftover
    // Quad-X mixer stabilizes attitude while the bridge closes an ArduPlane-
    // style climb-rate law on the throttle stick (mid = hold). QLAND:
    // constant-rate descent to ground contact. Any fixed-wing mode on a VTOL
    // model flies the full Plane stack with lift-motor assist blended out as
    // airspeed builds (forward transition).
    kQhover = 8,
    kQland = 9,
};

class Bridge {
public:
    Status init(const InitPayload& p) {
        // Rebuild the whole vehicle+plant in place: INIT is also re-INIT
        // (the host respawning/re-seating an aircraft reuses the process).
        plane_.~Plane();
        new (&plane_) fwcpp::vehicle::Plane();
        sim_.~SimPlane();
        new (&sim_) fwcpp::sim::SimPlane();
        qsim_.~SimQuadPlane();
        new (&qsim_) fwcpp::sim::SimQuadPlane("quadplane");
        qp_.~QuadPlane();
        new (&qp_) fwcpp::quadplane::QuadPlane(1);
        vtol_ = false;
        vtol_mode_ = BridgeMode::kManual;
        vtol_hover_trim_ = 0.0f;
        vtol_transition_done_ = false;
        vtol_climb_gain_ = 0.08f;
        vtol_trim_gain_ = 0.05f;
        vtol_climb_max_ = 2.5f;
        vtol_land_rate_ = 1.0f;
        vtol_assist_full_ = 8.0f;
        vtol_assist_zero_ = 16.0f;
        terrain_hae_m_ = 0.0f;
        terrain_valid_ = false;

        // Airframe model, BEFORE any dynamics run. A given-but-broken model
        // is a hard refusal: silently flying the skywalker defaults under a
        // different name is the exact failure this status exists to prevent.
        // The "vtol" flag is peeked first — it selects which plant every
        // step below (model application, start fix, warm start) targets.
        if (p.model_json[0] != '\0') {
            fwcpp::sim::JsonValue obj;
            std::string err;
            if (!fwcpp::sim::load_json_file(p.model_json, obj, err)) {
                initialized_ = false;
                return Status::kBadModel;
            }
            float vtol_flag = 0.0f;
            fwcpp::sim::json_get_float(obj, "vtol", vtol_flag);
            vtol_ = vtol_flag > 0.5f;

            if (vtol_) {
                // SINGLE-AUTHORITY GUARD: on a VTOL session SimQuadPlane
                // re-reads mass from the FRAME every tick and applies the
                // frame's own moment_of_inertia, so these bridge keys would
                // be silently dead. Refuse rather than let a model author
                // believe they took effect.
                float dummy = 0.0f;
                fwcpp::math::Vector3f vdummy;
                if (fwcpp::sim::json_get_float(obj, "mass", dummy) ||
                    fwcpp::sim::json_get_vector3(obj, "moment_inertia", vdummy)) {
                    std::printf("model rejected: on a vtol model, mass/moment_inertia belong in the "
                                "FRAME file (vtol_frame_params), not the model json\n");
                    std::fflush(stdout);
                    initialized_ = false;
                    return Status::kBadModel;
                }

                // Frame selection: layout string (motor arrangement) plus an
                // optional physical-parameters file, resolved relative to the
                // model json itself. Composed into Frame::init's own
                // "<layout>:<file>.json" convention.
                std::string layout = "quadplane";
                std::string frame_params;
                json_get_string(obj, "vtol_frame_layout", layout);
                std::string frame_str = layout;
                if (json_get_string(obj, "vtol_frame_params", frame_params) && !frame_params.empty()) {
                    const bool absolute = frame_params.find(':') != std::string::npos ||
                                          frame_params.front() == '/' || frame_params.front() == '\\';
                    const std::string full =
                        absolute ? frame_params : path_dirname(p.model_json) + frame_params;
                    // Verify the frame file loads NOW — Frame::init's own
                    // load_frame_params failure is silent (defaults kept),
                    // which is exactly the silent-skywalker trap this status
                    // exists to prevent.
                    fwcpp::sim::JsonValue fobj;
                    if (!fwcpp::sim::load_json_file(full.c_str(), fobj, err)) {
                        std::printf("model rejected: vtol_frame_params '%s' unreadable (%s)\n",
                                    full.c_str(), err.c_str());
                        std::fflush(stdout);
                        initialized_ = false;
                        return Status::kBadModel;
                    }
                    frame_str += ":" + full;
                }
                qsim_.~SimQuadPlane();
                new (&qsim_) fwcpp::sim::SimQuadPlane(frame_str.c_str());
                std::printf("vtol frame: '%s' -> %s, %d lift motors, hover_command=%.3f, mass=%.1f kg\n",
                            frame_str.c_str(), qsim_.frame().name,
                            static_cast<int>(qsim_.frame().num_motors),
                            qsim_.frame().hover_command(), qsim_.mass);
                std::fflush(stdout);
            }

            if (!apply_model(p.model_json)) {
                initialized_ = false;
                return Status::kBadModel;
            }

            if (vtol_ && !(vtol_assist_zero_ > vtol_assist_full_ &&
                           vtol_assist_zero_ <= plane_.aparm.airspeed_cruise &&
                           vtol_climb_max_ > 0.0f && vtol_land_rate_ > 0.0f)) {
                std::printf("model rejected: vtol law keys inconsistent (assist %g..%g vs cruise %g, "
                            "climb_max %g, land_rate %g)\n",
                            vtol_assist_full_, vtol_assist_zero_, plane_.aparm.airspeed_cruise,
                            vtol_climb_max_, vtol_land_rate_);
                std::fflush(stdout);
                initialized_ = false;
                return Status::kBadModel;
            }
        }

        fwcpp::sim::SimPlane& plant = active_sim();

        fwcpp::Location start;
        start.lat = p.lat_1e7;
        start.lng = p.lng_1e7;
        start.alt = static_cast<std::int32_t>(p.alt_m * 100.0f);
        plant.set_start_location(start, p.yaw_deg);

        // Upstream Plane sets GROUND_BEHAVIOR_FWD_ONLY (SIM_Plane.cpp:50);
        // the port's SimPlane defaults kNone for its own tests. The bridge
        // restores upstream's choice — upright, forward-only ground contact
        // is what makes a takeoff roll clean instead of a physics wrestle.
        // The QuadPlane plant keeps its own ctor choice (kNoMovement — a
        // VTOL sits still on its pad instead of creeping forward).
        if (!vtol_) {
            sim_.ground_behavior = fwcpp::sim::GroundBehavior::kFwdOnly;
        }

        // Plane-side navigation frame: Location() offset by NED metres from
        // the start point (update_current_loc's own convention). Home IS the
        // start point, i.e. the frame origin at altitude 0.
        plane_.set_home(fwcpp::Location(0, 0, 0, fwcpp::Location::AltFrame::ABSOLUTE));

        if (p.in_air != 0) {
            plant.position = {p.pos_ned_m[0], p.pos_ned_m[1], p.pos_ned_m[2]};
            plant.velocity_ef = {p.vel_ned_mps[0], p.vel_ned_mps[1], p.vel_ned_mps[2]};
            plant.dcm.from_euler(p.rpy_rad[0], p.rpy_rad[1], p.rpy_rad[2]);
            plant.update_position();
            // Approximate initial airspeed from inertial velocity (no wind
            // at t=0); the plant recomputes it properly on the first step.
            plant.airspeed = std::sqrt(plant.velocity_ef.x * plant.velocity_ef.x +
                                       plant.velocity_ef.y * plant.velocity_ef.y +
                                       plant.velocity_ef.z * plant.velocity_ef.z);
        } else {
            // Ground start: the proven sitl_run boot sequence, including the
            // boot-time airspeed zero-offset calibration (only valid while
            // stationary — deliberately skipped for the in-air warm start,
            // where the sensor's default zero offset is already exact).
            plane_.airspeed_sensor.start_calibration(0);
        }

        if (vtol_) {
            // VCP-011 boot sequence: QuadPlane subsystems up + QHover armed
            // state machine entered. The FW side starts FBWA so the surfaces
            // stabilize once airspeed exists; the lift side stays at zero
            // collective until the host arms and enters qhover.
            (void)qp_.setup();
            qp_.mode_enter();
            (void)fwcpp::q_modes::qhover_enter();
        }

        set_neutral_sticks();
        plane_.control_mode = &plane_.mode_manual;
        mode_ = BridgeMode::kManual;
        now_ms_ = 0;
        initialized_ = true;
        return Status::kOk;
    }

    [[nodiscard]] fwcpp::sim::SimPlane& active_sim() {
        return vtol_ ? static_cast<fwcpp::sim::SimPlane&>(qsim_) : sim_;
    }
    [[nodiscard]] const fwcpp::sim::SimPlane& active_sim() const {
        return vtol_ ? static_cast<const fwcpp::sim::SimPlane&>(qsim_) : sim_;
    }

    // One model file configures BOTH halves of an airframe (see InitPayload's
    // model_json doc): SimPlane::load_coeffs() takes the aero-coefficient
    // subset (upstream's own plane-JSON schema), then the bridge applies the
    // mass/thrust and flight-code envelope keys itself.
    bool apply_model(const char* path) {
        fwcpp::sim::SimPlane& plant = active_sim();
        if (!plant.load_coeffs(path)) {
            return false;
        }
        fwcpp::sim::JsonValue obj;
        std::string err;
        if (!fwcpp::sim::load_json_file(path, obj, err)) {
            return false;
        }
        fwcpp::sim::json_get_float(obj, "mass", plant.mass);
        fwcpp::sim::json_get_float(obj, "hover_throttle", plant.hover_throttle);
        fwcpp::sim::json_get_vector3(obj, "moment_inertia", plant.moment_inertia);
        fwcpp::sim::json_get_float(obj, "ground_friction", plant.ground_friction);
        fwcpp::sim::json_get_float(obj, "airspeed_min", plane_.aparm.airspeed_min);
        fwcpp::sim::json_get_float(obj, "airspeed_cruise", plane_.aparm.airspeed_cruise);
        fwcpp::sim::json_get_float(obj, "airspeed_max", plane_.aparm.airspeed_max);
        fwcpp::sim::json_get_float(obj, "scaling_speed", plane_.aparm.scaling_speed);
        // TECS holds its OWN by-value copy of the airframe params, taken at
        // construction — mirror the envelope into it or every tas demand is
        // clamped to the foamie's [9, 22] m/s forever (measured live).
        fwcpp::tecs::Tecs::FixedWingParams& ta = plane_.tecs.mutable_aparm();
        ta.airspeed_min = plane_.aparm.airspeed_min;
        ta.airspeed_cruise = plane_.aparm.airspeed_cruise;
        ta.airspeed_max = plane_.aparm.airspeed_max;
        ta.airspeed_stall = plane_.aparm.airspeed_stall;
        // TECS energy limits (TECS_CLMB_MAX / SINK_MIN / SINK_MAX). The
        // defaults are foamie-scale (5/2/5 m/s); a jet whose real climb blows
        // past CLMB_MAX hands TECS a permanent energy surplus at the
        // takeoff→AUTO transition (observed: throttle cut to zero, descent
        // into the runway). State the airframe's honest limits instead.
        fwcpp::tecs::Tecs::Gains& tg = plane_.tecs.mutable_gains();
        fwcpp::sim::json_get_float(obj, "tecs_climb_max", tg.max_climb_rate);
        fwcpp::sim::json_get_float(obj, "tecs_sink_min", tg.min_sink_rate);
        fwcpp::sim::json_get_float(obj, "tecs_sink_max", tg.max_sink_rate);
        fwcpp::sim::json_get_float(obj, "tecs_time_const", tg.time_const);
        fwcpp::sim::json_get_float(obj, "tecs_thr_damp", tg.thr_damp);
        fwcpp::sim::json_get_float(obj, "tecs_ptch_damp", tg.ptch_damp);
        fwcpp::sim::json_get_float(obj, "tecs_integ_gain", tg.integ_gain);
        // VTOL law tuning (vtol models only; harmless no-ops otherwise).
        fwcpp::sim::json_get_float(obj, "vtol_climb_gain", vtol_climb_gain_);
        fwcpp::sim::json_get_float(obj, "vtol_trim_gain", vtol_trim_gain_);
        fwcpp::sim::json_get_float(obj, "vtol_climb_max", vtol_climb_max_);
        fwcpp::sim::json_get_float(obj, "vtol_land_rate_mps", vtol_land_rate_);
        fwcpp::sim::json_get_float(obj, "vtol_assist_full_mps", vtol_assist_full_);
        fwcpp::sim::json_get_float(obj, "vtol_assist_zero_mps", vtol_assist_zero_);
        return true;
    }

    // Host-sampled terrain under the aircraft (STEP tail, see
    // kStepTerrainTailLen). Absolute HAE metres — the same datum as
    // home.alt — so it writes straight into the base Aircraft ground plane.
    void set_terrain(float hae_m, bool valid) {
        terrain_hae_m_ = hae_m;
        terrain_valid_ = valid;
    }

    Status step(const StepPayload& p) {
        if (!initialized_) {
            return Status::kNotInitialized;
        }
        if (terrain_valid_) {
            // Slide the plant's ground plane to the real surface. hagl(),
            // on_ground(), and every ground clamp in Aircraft::update_dynamics
            // key off ground_level, so this one write makes ground contact —
            // including qland touchdown — terrain-true. Both plants get it:
            // only one flies, but a VTOL mode handover must agree on the
            // ground. When the host stops sampling (no resident tiles), the
            // LAST surface is kept — a stale-but-nearby ground beats snapping
            // back to the start-fix flat earth mid-flight.
            sim_.ground_level = terrain_hae_m_;
            qsim_.ground_level = terrain_hae_m_;
        }
        std::uint16_t n = p.n_steps;
        if (n == 0) {
            return Status::kOk; // state query — reply carries current truth
        }
        if (n > kMaxStepsPerRequest) {
            n = kMaxStepsPerRequest;
        }
        for (std::uint8_t ch = 0; ch < 4; ++ch) {
            if (p.rc_pwm[ch] != 0) {
                rc_pwm_[ch] = p.rc_pwm[ch];
            }
        }
        for (std::uint16_t i = 0; i < n; ++i) {
            now_ms_ += 20;
            if (vtol_) {
                // The QuadPlane loop runs at 400 Hz — eight 2.5 ms sub-steps
                // inside every 20 ms host tick, sub-tick timestamps spread
                // evenly so the flight code's ms clock stays monotonic.
                //
                // While VERTICAL, hold TECS in reset — FBWA never drives it,
                // so its demand state is stale the moment a fixed-wing mode
                // takes over (same mechanism as the fixed-wing takeoff-stage
                // hold above; measured here as a dive out of hover into
                // CRUISE that flew the aircraft into the ground).
                if (vtol_mode_ == BridgeMode::kQhover || vtol_mode_ == BridgeMode::kQland) {
                    plane_.tecs.reset();
                }
                apply_vtol_sticks();
                // Guard window runs to CRUISE speed, not just assist-zero:
                // the measured dive began the exact tick assist (and an
                // assist-window guard) expired — TECS then rode its -25 deg
                // pitch floor chasing its walking speed demand from 18 all
                // the way into the ground, though the wing flies level there.
                //
                // ONE-WAY LATCH: without it the guard re-engages every time
                // cruise-speed noise dips below the threshold — throttle
                // floor slams on, speed surges, floor releases, dip... a
                // limit cycle the operator sees as wobbly forward flight.
                // Once cruise is first reached, the transition is DONE until
                // the aircraft re-enters a vertical mode.
                if (vtol_mode_ == BridgeMode::kQhover || vtol_mode_ == BridgeMode::kQland) {
                    vtol_transition_done_ = false;
                } else if (qsim_.airspeed >= plane_.aparm.airspeed_cruise) {
                    vtol_transition_done_ = true;
                }
                const bool transitioning = vtol_mode_ != BridgeMode::kQhover &&
                                           vtol_mode_ != BridgeMode::kQland &&
                                           !vtol_transition_done_;
                for (int k = 0; k < 8; ++k) {
                    const std::uint32_t sub_now = now_ms_ - 20 + static_cast<std::uint32_t>(((k + 1) * 20) / 8);
                    // ArduPlane-style assisted-transition guard: while the
                    // lift motors are carrying the aircraft through the
                    // sub-stall corridor, TECS must not trade altitude for
                    // airspeed — its underspeed logic otherwise pitches down
                    // at up to sink_max and dives a 95 m hover into the
                    // ground (measured twice). External limits reset every
                    // TECS iteration, so re-assert per sub-step.
                    if (transitioning) {
                        plane_.tecs.set_pitch_min(-2.0f);
                        // Break the underspeed deadlock: TECS's underspeed
                        // state pins the speed demand to airspeed_min and
                        // flies speed with pitch, so the aircraft equilibrates
                        // just BELOW the underspeed-exit hysteresis and stays
                        // there forever (measured: 100 s pinned at min+1).
                        // A high throttle floor pushes airspeed through the
                        // exit threshold; past cruise the guard drops and
                        // TECS manages normally.
                        plane_.tecs.set_throttle_min(0.8f);
                    }
                    last_vtol_lift_ = vtol_collective();
                    qharness_.step(sub_now, 0.0025f, last_vtol_lift_, plane_.armed);
                }
                continue;
            }
            apply_sticks();
            // Upstream's _initialise_states() continuously RE-SEEDS the TECS
            // height-demand filters during the TAKEOFF flight stage, so AUTO
            // inherits demand state anchored at the aircraft's actual exit
            // height with fresh anti-windup scalers. This port dropped that
            // branch (see tecs.hpp's post_to_hgt_offset_ note); without it the
            // filters evolve garbage under takeoff_calc_* override (measured:
            // hgt_dem frozen at 111 m vs a 487 m exit → commanded dive, and
            // max_climb_scaler_ collapsed to ~0 → the next leg's demand ramps
            // at ~0 m/s). Emulate upstream by holding TECS in reset while the
            // takeoff item is active — its outputs are overridden then anyway.
            if (mode_ == BridgeMode::kAuto) {
                const fwcpp::vehicle::MissionItem* cur = plane_.mission.current();
                if (cur != nullptr && cur->command == fwcpp::vehicle::MissionCommand::Takeoff) {
                    plane_.tecs.reset();
                }
            }
            harness_.step(now_ms_, kDt);
            if (debug_ && (now_ms_ % 5000u) == 0u) {
                const fwcpp::tecs::Tecs::DebugState d = plane_.tecs.debug_state();
                std::printf("TECSDBG t=%.1f h=%.1f hdem_in=%.1f hdem_ltd=%.1f hdem=%.1f hrate=%.2f "
                            "cscale=%.4f sscale=%.4f tasdem=%.1f pitchdem=%.1f thrdem=%.2f clip=%d\n",
                            now_ms_ / 1000.0f, -sim_.position.z, d.hgt_dem_in, d.hgt_dem_rate_ltd,
                            d.hgt_dem, d.hgt_rate_dem, d.max_climb_scaler, d.max_sink_scaler,
                            d.tas_dem_adj, d.pitch_dem_unc * 57.2958f, d.throttle_dem, d.thr_clip);
                std::fflush(stdout);
            }
        }
        return Status::kOk;
    }

    void set_debug(bool enabled) { debug_ = enabled; }

    Status set_mode(const ModePayload& p) {
        if (!initialized_) {
            return Status::kNotInitialized;
        }
        const BridgeMode requested = static_cast<BridgeMode>(p.mode);
        if (requested == BridgeMode::kQhover || requested == BridgeMode::kQland) {
            if (!vtol_) {
                return Status::kBadMode; // fixed-wing model has no lift motors
            }
            // Vertical flight: the leftover mixer stabilizes attitude and the
            // bridge's collective law owns height; the FW side flies FBWA so
            // the surfaces help once airspeed exists.
            (void)plane_.set_mode(plane_.mode_fbwa);
            vtol_mode_ = requested;
            mode_ = requested;
            return Status::kOk;
        }
        fwcpp::vehicle::Mode* target = nullptr;
        switch (requested) {
        case BridgeMode::kManual: target = &plane_.mode_manual; break;
        case BridgeMode::kFbwa:   target = &plane_.mode_fbwa; break;
        case BridgeMode::kFbwb:   target = &plane_.mode_fbwb; break;
        case BridgeMode::kCruise: target = &plane_.mode_cruise; break;
        case BridgeMode::kAuto:   target = &plane_.mode_auto; break;
        case BridgeMode::kRtl:    target = &plane_.mode_rtl; break;
        case BridgeMode::kLoiter: target = &plane_.mode_loiter; break;
        case BridgeMode::kTakeoff: target = &plane_.mode_takeoff; break;
        default:
            return Status::kBadMode;
        }
        vtol_mode_ = BridgeMode::kManual; // leaving vertical flight (transition assist takes over)
        // FBWB/CRUISE/LOITER: their enter()s never seed target_altitude_cm —
        // "A CALLER MUST CALL set_target_altitude_current" (their own class
        // banners). This bridge is that caller, seeding from sim truth.
        if (requested == BridgeMode::kFbwb || requested == BridgeMode::kCruise ||
            requested == BridgeMode::kLoiter) {
            // active_sim(), NOT sim_ — on a VTOL model the fixed-wing plant sits
        // frozen at 0 m, and seeding from it handed CRUISE a 0 m altitude
        // target out of a 69 m hover (TECS then descended at exactly
        // sink_max into the ground — measured).
        plane_.set_target_altitude_current(static_cast<std::int32_t>(-active_sim().position.z * 100.0f));
        }
        if (!plane_.set_mode(*target)) {
            return Status::kModeRefused;
        }
        mode_ = requested;
        return Status::kOk;
    }

    Status set_mission(const MissionItemWire* items, std::uint16_t count) {
        if (!initialized_) {
            return Status::kNotInitialized;
        }
        if (count == 0 || count > fwcpp::vehicle::kMaxMissionItems) {
            return Status::kBadMission;
        }
        std::array<fwcpp::vehicle::MissionItem, fwcpp::vehicle::kMaxMissionItems> converted{};
        for (std::uint16_t i = 0; i < count; ++i) {
            const MissionItemWire& w = items[i];
            fwcpp::vehicle::MissionItem& m = converted[i];
            switch (w.cmd) {
            case 0: m.command = fwcpp::vehicle::MissionCommand::Waypoint; break;
            case 1: m.command = fwcpp::vehicle::MissionCommand::Takeoff; break;
            case 2: m.command = fwcpp::vehicle::MissionCommand::Land; break;
            default:
                return Status::kBadMission;
            }
            m.loc = fwcpp::Location();
            m.loc.offset(w.north_m, w.east_m);
            m.loc.set_alt_m(w.up_m, fwcpp::Location::AltFrame::ABSOLUTE);
            m.acceptance_radius_m = w.acceptance_radius_m;
            m.takeoff_pitch_deg = w.takeoff_pitch_deg;
        }
        if (!plane_.mission.load(std::span<const fwcpp::vehicle::MissionItem>(converted.data(), count))) {
            return Status::kBadMission;
        }
        return Status::kOk;
    }

    Status arm(const ArmPayload& p) {
        if (!initialized_) {
            return Status::kNotInitialized;
        }
        if (p.armed != 0) {
            // Same primitives sitl_run's main() uses instead of plane.arm()
            // (whose rc_received gate would fail before sticks flow) — see
            // that file's own banner.
            plane_.armed = true;
            plane_.hal.rc_output.force_safety_off();
        } else {
            plane_.armed = false;
        }
        return Status::kOk;
    }

    // The LIVE mode, derived from the flight code's own control_mode pointer —
    // not the last host-commanded label. The two diverge when the vehicle
    // transitions internally (ModeAUTO's mission-complete -> RTL), which made
    // the old label lie to the host exactly when it mattered.
    [[nodiscard]] BridgeMode current_mode() const {
        if (vtol_ && (vtol_mode_ == BridgeMode::kQhover || vtol_mode_ == BridgeMode::kQland)) {
            return vtol_mode_;
        }
        const fwcpp::vehicle::Mode* m = plane_.control_mode;
        if (m == &plane_.mode_fbwa) { return BridgeMode::kFbwa; }
        if (m == &plane_.mode_fbwb) { return BridgeMode::kFbwb; }
        if (m == &plane_.mode_cruise) { return BridgeMode::kCruise; }
        if (m == &plane_.mode_auto) { return BridgeMode::kAuto; }
        if (m == &plane_.mode_rtl) { return BridgeMode::kRtl; }
        if (m == &plane_.mode_loiter) { return BridgeMode::kLoiter; }
        if (m == &plane_.mode_takeoff) { return BridgeMode::kTakeoff; }
        return BridgeMode::kManual;
    }

    // ---- GCS-facing entry points (MAVLink endpoint in main) ----

    [[nodiscard]] bool is_initialized() const { return initialized_; }
    [[nodiscard]] bool is_armed() const { return plane_.armed; }
    [[nodiscard]] std::uint32_t sim_time_ms() const { return now_ms_; }
    [[nodiscard]] const fwcpp::sim::SimPlane& sim() const { return active_sim(); }
    [[nodiscard]] const fwcpp::vehicle::Plane& plane() const { return plane_; }

    void gcs_arm(bool armed) {
        ArmPayload p;
        p.armed = armed ? 1 : 0;
        (void)arm(p);
    }

    // ArduPlane custom-mode number -> bridge mode. Returns false for a mode
    // this port doesn't have (STABILIZE, CIRCLE, GUIDED, Q*...).
    bool gcs_set_mode_custom(std::uint32_t custom) {
        ModePayload p;
        switch (custom) {
        case 0: p.mode = static_cast<std::uint8_t>(BridgeMode::kManual); break;
        case 5: p.mode = static_cast<std::uint8_t>(BridgeMode::kFbwa); break;
        case 6: p.mode = static_cast<std::uint8_t>(BridgeMode::kFbwb); break;
        case 7: p.mode = static_cast<std::uint8_t>(BridgeMode::kCruise); break;
        case 10: p.mode = static_cast<std::uint8_t>(BridgeMode::kAuto); break;
        case 11: p.mode = static_cast<std::uint8_t>(BridgeMode::kRtl); break;
        case 12: p.mode = static_cast<std::uint8_t>(BridgeMode::kLoiter); break;
        case 13: p.mode = static_cast<std::uint8_t>(BridgeMode::kTakeoff); break;
        case 18: p.mode = static_cast<std::uint8_t>(BridgeMode::kQhover); break;
        case 20: p.mode = static_cast<std::uint8_t>(BridgeMode::kQland); break;
        default:
            return false;
        }
        return set_mode(p) == Status::kOk;
    }

    // Bridge mode -> ArduPlane custom-mode number (heartbeat).
    [[nodiscard]] static std::uint32_t arduplane_custom_mode(BridgeMode m) {
        switch (m) {
        case BridgeMode::kManual: return 0;
        case BridgeMode::kFbwa: return 5;
        case BridgeMode::kFbwb: return 6;
        case BridgeMode::kCruise: return 7;
        case BridgeMode::kAuto: return 10;
        case BridgeMode::kRtl: return 11;
        case BridgeMode::kLoiter: return 12;
        case BridgeMode::kTakeoff: return 13;
        case BridgeMode::kQhover: return 18;
        case BridgeMode::kQland: return 20;
        }
        return 0;
    }

    // One geodetic mission row from a MAVLink MISSION_ITEM_INT upload.
    struct GcsGeoItem {
        std::uint16_t seq = 0;
        std::uint16_t command = 0;
        std::uint8_t frame = 0;
        float p1 = 0.0f;
        float p2 = 0.0f;
        std::int32_t lat_1e7 = 0;
        std::int32_t lng_1e7 = 0;
        float z = 0.0f;
    };

    // Load an uploaded geodetic mission: same NED-from-start conversion the
    // WOPR wire path uses, but via the sim's real home Location so a GCS and
    // the WOPR host agree on where every waypoint is. Rows the port can't fly
    // (DO_*, CONDITION_*, loiters) are skipped; seq 0 NAV_WAYPOINT = home row.
    // Returns the number of flyable items loaded, or -1 on failure.
    int gcs_mission_load(const GcsGeoItem* rows, std::size_t n) {
        if (!initialized_) {
            return -1;
        }
        const fwcpp::Location home = active_sim().get_home();
        const float home_alt_m = static_cast<float>(home.alt) * 0.01f;
        std::array<fwcpp::vehicle::MissionItem, fwcpp::vehicle::kMaxMissionItems> converted{};
        std::size_t count = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const GcsGeoItem& r = rows[i];
            if (r.seq == 0 && r.command == 16) {
                continue; // home row
            }
            fwcpp::vehicle::MissionItem m;
            switch (r.command) {
            case 16: m.command = fwcpp::vehicle::MissionCommand::Waypoint;
                     m.acceptance_radius_m = std::fmax(0.0f, r.p2);
                     break;
            case 22: m.command = fwcpp::vehicle::MissionCommand::Takeoff;
                     m.takeoff_pitch_deg = r.p1;
                     break;
            case 21: m.command = fwcpp::vehicle::MissionCommand::Land; break;
            default:
                continue; // unsupported row — skipped, not fatal
            }
            fwcpp::Location wp;
            wp.lat = r.lat_1e7;
            wp.lng = r.lng_1e7;
            const fwcpp::math::Vector2f ne = home.get_distance_NE(wp);
            m.loc = fwcpp::Location();
            m.loc.offset(ne.x, ne.y);
            // Frames: 0/5 absolute (rebased on the start alt), 3/6 relative.
            const bool relative = (r.frame == 3) || (r.frame == 6);
            const float up_m = relative ? r.z : (r.z - home_alt_m);
            m.loc.set_alt_m(up_m, fwcpp::Location::AltFrame::ABSOLUTE);
            if (count >= fwcpp::vehicle::kMaxMissionItems) {
                return -1;
            }
            converted[count++] = m;
        }
        if (count == 0) {
            return -1;
        }
        if (!plane_.mission.load(std::span<const fwcpp::vehicle::MissionItem>(converted.data(), count))) {
            return -1;
        }
        observe_mission_base();
        return static_cast<int>(count);
    }

    void fill_state(StateReply& r) const {
        const fwcpp::sim::SimPlane& plant = active_sim();
        r.initialized = initialized_ ? 1 : 0;
        r.mode = static_cast<std::uint8_t>(current_mode());
        r.on_ground = plant.on_ground() ? 1 : 0;
        r.pos_ned_m[0] = plant.position.x;
        r.pos_ned_m[1] = plant.position.y;
        r.pos_ned_m[2] = plant.position.z;
        r.vel_ned_mps[0] = plant.velocity_ef.x;
        r.vel_ned_mps[1] = plant.velocity_ef.y;
        r.vel_ned_mps[2] = plant.velocity_ef.z;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        plant.dcm.to_euler(&roll, &pitch, &yaw);
        r.rpy_rad[0] = roll;
        r.rpy_rad[1] = pitch;
        r.rpy_rad[2] = yaw;
        r.gyro_rps[0] = plant.gyro.x;
        r.gyro_rps[1] = plant.gyro.y;
        r.gyro_rps[2] = plant.gyro.z;
        r.airspeed_mps = plant.airspeed;
        r.hagl_m = plant.hagl();
        r.servo_norm[0] =
            plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) / fwcpp::vehicle::kServoMax;
        r.servo_norm[1] =
            plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator) / fwcpp::vehicle::kServoMax;
        r.servo_norm[2] =
            plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder) / fwcpp::vehicle::kServoMax;
        r.servo_norm[3] = plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) / 100.0f;
        r.vtol_lift = vtol_ ? last_vtol_lift_ : 0.0f;
        r.vtol_hover_cmd = vtol_ ? qsim_.frame().hover_command() : 0.0f;
        r.mission_count = static_cast<std::uint16_t>(plane_.mission.size());
        r.mission_index = mission_progress_index();
        r.sim_time_ms = now_ms_;
        r.armed = plane_.armed ? 1 : 0;
    }

private:
    void set_neutral_sticks() {
        rc_pwm_[0] = 1500; // roll
        rc_pwm_[1] = 1500; // pitch
        rc_pwm_[2] = 1100; // throttle low (auto-throttle modes ignore it)
        rc_pwm_[3] = 1500; // rudder
        apply_sticks();
    }

    void apply_sticks() {
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRoll, rc_pwm_[0]);
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelPitch, rc_pwm_[1]);
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelThrottle, rc_pwm_[2]);
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRudder, rc_pwm_[3]);
        plane_.rc_channels.read_input(plane_.hal.rc_input);
    }

    // VTOL stick routing: while vertical (qhover/qland) the host's throttle
    // stick is the CLIMB command (consumed by vtol_collective), not forward
    // thrust — the plane's own throttle channel is pinned to idle so the FW
    // servo output can't fight the hover. In fixed-wing modes the sticks
    // pass through untouched and the lift motors blend out with airspeed.
    void apply_vtol_sticks() {
        const bool vertical = vtol_mode_ == BridgeMode::kQhover || vtol_mode_ == BridgeMode::kQland;
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRoll, rc_pwm_[0]);
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelPitch, rc_pwm_[1]);
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelThrottle,
                                        vertical ? static_cast<std::uint16_t>(1000) : rc_pwm_[2]);
        plane_.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRudder, rc_pwm_[3]);
        plane_.rc_channels.read_input(plane_.hal.rc_input);
    }

    // Lift-motor collective for this sub-step. QHOVER is ArduPlane's own
    // semantic: throttle stick mid = hold, deflection = climb-rate demand
    // (±2.5 m/s), closed with a proportional law on measured climb rate
    // around the frame's hover command. QLAND is the same law with a fixed
    // -vtol_land_rate_ demand (model "vtol_land_rate_mps") until ground
    // contact. Fixed-wing modes get transition
    // assist: full hover support below ~6 m/s airspeed blending to zero by
    // 14 m/s, after which the (complete) Plane stack owns the aircraft.
    [[nodiscard]] float vtol_collective() {
        const float hover = qsim_.frame().hover_command();
        const float climb_rate = -qsim_.velocity_ef.z;
        constexpr float kSubDt = 0.0025f;
        if (vtol_mode_ == BridgeMode::kQhover || vtol_mode_ == BridgeMode::kQland) {
            float climb_dem;
            if (vtol_mode_ == BridgeMode::kQhover) {
                const float stick = fwcpp::math::constrain_value(
                    (static_cast<float>(rc_pwm_[2]) - 1500.0f) / 400.0f, -1.0f, 1.0f);
                climb_dem = stick * vtol_climb_max_;
            } else {
                if (qsim_.on_ground()) {
                    vtol_hover_trim_ = 0.0f;
                    return 0.0f;
                }
                climb_dem = -vtol_land_rate_;
            }
            // Integral trim kills the steady-state sink a pure-P law leaves
            // when the frame's nominal hover command under-trims (measured
            // -0.5 m/s at mid-stick on the demo frame).
            //
            // The trim spans the FULL collective range (bench-measured
            // 2026-08-31): the old +/-0.25 clamp saturated in sustained rate
            // tracking on both sides — qland converged at -1.7 of a -2.0
            // demand then decayed to -1.4 as rotor inflow added thrust in
            // descent, and full-stick climb pinned at +5.8 of +6.0. Windup is
            // handled the classic way instead: don't integrate further in the
            // direction of an already-saturated output.
            const float err = climb_dem - climb_rate;
            const float p_term = vtol_climb_gain_ * err;
            const float unsat = hover + vtol_hover_trim_ + p_term;
            const bool wound_low = unsat <= 0.0f && err < 0.0f;
            const bool wound_high = unsat >= 1.0f && err > 0.0f;
            if (!wound_low && !wound_high) {
                vtol_hover_trim_ = fwcpp::math::constrain_value(
                    vtol_hover_trim_ + vtol_trim_gain_ * err * kSubDt, -hover, 1.0f - hover);
            }
            return fwcpp::math::constrain_value(hover + vtol_hover_trim_ + p_term, 0.0f, 1.0f);
        }
        // Forward transition assist: full (trimmed) hover support up to
        // vtol_assist_full_mps, blending to zero by vtol_assist_zero_mps —
        // which model validation pins at/below the wing's cruise so wingborne
        // flight is genuinely wingborne. (History on the demo frame: cutting
        // at 14 descended it into the ground mid-acceleration; reaching to 20
        // left it permanently half-assisted at the foamie's 12 m/s cruise.)
        return (hover + vtol_hover_trim_) *
               fwcpp::math::constrain_value(
                   (vtol_assist_zero_ - qsim_.airspeed) / (vtol_assist_zero_ - vtol_assist_full_),
                   0.0f, 1.0f);
    }

    // Mission::current() returns a pointer into its private fixed array but
    // exposes no index; recover progress via pointer arithmetic against the
    // first item (stable across the mission's lifetime — fixed-size array,
    // no reallocation, ADR-0012).
    [[nodiscard]] std::uint16_t mission_progress_index() const {
        const fwcpp::vehicle::MissionItem* cur = plane_.mission.current();
        if (cur == nullptr || plane_.mission.empty()) {
            return 0;
        }
        // reset()+current() would perturb state; instead exploit that
        // current() points into the same array for index 0..size-1. We can
        // recover the base by noting current() at index 0 after load() —
        // cached below on first use.
        if (mission_base_ == nullptr || plane_.mission.size() != mission_base_size_) {
            return 0; // no base cached (observe_mission_base not called yet)
        }
        return static_cast<std::uint16_t>(cur - mission_base_);
    }

public:
    // Called by set_mission's caller path right after load() so index
    // reporting works; kept separate from set_mission for clarity.
    void observe_mission_base() {
        mission_base_ = plane_.mission.current();
        mission_base_size_ = plane_.mission.size();
    }

private:
    fwcpp::vehicle::Plane plane_;
    fwcpp::sim::SimPlane sim_;
    fwcpp::hal_sitl::SitlHarness harness_{plane_, sim_};
    // VTOL variant ("vtol": 1 model flag): QuadPlane subsystems + the
    // original-source SIM_QuadPlane plant, ticked at 400 Hz by the VCP-011
    // harness. Only ONE of (harness_, qharness_) runs per session.
    fwcpp::sim::SimQuadPlane qsim_{"quadplane"};
    fwcpp::quadplane::QuadPlane qp_{1};
    fwcpp::hal_sitl::SitlQuadPlaneHarness qharness_{plane_, qp_, qsim_};
    bool vtol_ = false;
    BridgeMode vtol_mode_ = BridgeMode::kManual; // kQhover/kQland when vertical
    float vtol_hover_trim_ = 0.0f; // integral trim around the frame's hover command
    float last_vtol_lift_ = 0.0f;  // collective applied on the most recent sub-step
    bool vtol_transition_done_ = false; // one-way latch: cruise reached since last vertical mode
    // Host-fed real terrain under the aircraft (absolute HAE m). false until
    // the first STEP tail arrives; the flat start-fix ground applies until then.
    float terrain_hae_m_ = 0.0f;
    bool terrain_valid_ = false;
    // Per-model VTOL law tuning (apply_model keys; defaults = demo-frame values).
    float vtol_climb_gain_ = 0.08f;
    float vtol_trim_gain_ = 0.05f;
    float vtol_climb_max_ = 2.5f;      // m/s at full stick in qhover
    float vtol_land_rate_ = 1.0f;      // m/s qland descent
    float vtol_assist_full_ = 8.0f;    // full lift assist below this airspeed
    float vtol_assist_zero_ = 16.0f;   // assist reaches zero here (<= cruise, validated)
    BridgeMode mode_ = BridgeMode::kManual;
    std::uint32_t now_ms_ = 0;
    std::uint16_t rc_pwm_[4] = {1500, 1500, 1100, 1500};
    bool initialized_ = false;
    bool debug_ = false;
    const fwcpp::vehicle::MissionItem* mission_base_ = nullptr;
    std::size_t mission_base_size_ = 0;
};

// ===================== MAVLink GCS endpoint (--mavlink <port>) ==============
//
// A second, optional UDP socket a real ground station (QGroundControl /
// Mission Planner / pymavlink) connects to. Outbound: HEARTBEAT (1 Hz wall),
// ATTITUDE + GLOBAL_POSITION_INT + VFR_HUD (~10 Hz wall while the host steps
// the sim). Inbound: COMMAND_LONG (arm/disarm 400, DO_SET_MODE 176, RTL 20,
// TAKEOFF 22), PARAM_REQUEST_LIST/SET (minimal), and the full mission-upload
// handshake (MISSION_COUNT -> MISSION_REQUEST_INT xN -> MISSION_ACK) plus
// download of the last uploaded set. Wall-clock time paces ONLY this GCS
// side — the sim itself stays host-stepped and deterministic. No GUIDED mode
// exists in this port, so a GCS "fly to here" cannot work; upload a mission
// and enter AUTO instead.

namespace mavgcs {

constexpr std::uint8_t kCompId = 1; // MAV_COMP_ID_AUTOPILOT1
constexpr std::uint16_t kMaxUploadRows = 64;

struct GcsLink {
    sock_type sock{};
    bool enabled = false;
    // MAVLink SYSTEM id (--sysid): the multi-vehicle discriminator. One
    // aircraft = one sysid; a GCS with several links tells them apart by
    // this, never by compid (which distinguishes components WITHIN one
    // aircraft — this bridge is always compid 1, AUTOPILOT1).
    std::uint8_t sysid = 1;
    bool has_peer = false;
    sockaddr_in peer{};
    std::uint8_t seq = 0;
    std::chrono::steady_clock::time_point last_hb{};
    std::chrono::steady_clock::time_point last_stream{};
    // Upload session state + the last accepted geodetic mission (served back
    // verbatim when the GCS re-downloads after an upload).
    bool up_active = false;
    std::uint16_t up_count = 0;
    std::uint16_t up_next = 0;
    std::array<Bridge::GcsGeoItem, kMaxUploadRows> rows{};
    std::array<Bridge::GcsGeoItem, kMaxUploadRows> cached_rows{};
    std::uint16_t cached_count = 0;
};

inline void send_frame(GcsLink& g, std::uint32_t msgid, const std::uint8_t* payload, std::size_t len) {
    if (!g.enabled || !g.has_peer) {
        return;
    }
    fwcpp::gcs::Frame f;
    if (!fwcpp::gcs::make_frame(g.seq++, g.sysid, kCompId, msgid,
                                std::span<const std::uint8_t>(payload, len), f)) {
        return;
    }
    std::uint8_t buf[300];
    const std::size_t n = fwcpp::gcs::encode_v2(f, buf);
    if (n > 0) {
        ::sendto(g.sock, reinterpret_cast<const char*>(buf), static_cast<int>(n), 0,
                 reinterpret_cast<const sockaddr*>(&g.peer), sizeof(g.peer));
    }
}

inline void send_heartbeat(GcsLink& g, const Bridge& b) {
    std::uint8_t p[9] = {};
    fwcpp::gcs::write_u32_le(p, Bridge::arduplane_custom_mode(b.current_mode()));
    p[4] = 1; // MAV_TYPE_FIXED_WING
    p[5] = 3; // MAV_AUTOPILOT_ARDUPILOTMEGA
    p[6] = static_cast<std::uint8_t>(0x01 /*CUSTOM_MODE_ENABLED*/ | (b.is_armed() ? 0x80 : 0x00));
    p[7] = 4; // MAV_STATE_ACTIVE
    p[8] = 3; // mavlink_version
    send_frame(g, fwcpp::gcs::kMsgIdHeartbeat, p, sizeof(p));
}

inline void send_attitude(GcsLink& g, const Bridge& b) {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    b.sim().dcm.to_euler(&roll, &pitch, &yaw);
    std::uint8_t p[28] = {};
    fwcpp::gcs::write_u32_le(p, b.sim_time_ms());
    fwcpp::gcs::write_f32_le(p + 4, roll);
    fwcpp::gcs::write_f32_le(p + 8, pitch);
    fwcpp::gcs::write_f32_le(p + 12, yaw);
    fwcpp::gcs::write_f32_le(p + 16, b.sim().gyro.x);
    fwcpp::gcs::write_f32_le(p + 20, b.sim().gyro.y);
    fwcpp::gcs::write_f32_le(p + 24, b.sim().gyro.z);
    send_frame(g, fwcpp::gcs::kMsgIdAttitude, p, sizeof(p));
}

inline void send_global_position_int(GcsLink& g, const Bridge& b) {
    const fwcpp::Location& loc = b.sim().get_location();
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    b.sim().dcm.to_euler(&roll, &pitch, &yaw);
    float hdg_deg = fwcpp::math::degrees(yaw);
    if (hdg_deg < 0.0f) {
        hdg_deg += 360.0f;
    }
    std::uint8_t p[28] = {};
    fwcpp::gcs::write_u32_le(p, b.sim_time_ms());
    fwcpp::gcs::write_i32_le(p + 4, loc.lat);
    fwcpp::gcs::write_i32_le(p + 8, loc.lng);
    fwcpp::gcs::write_i32_le(p + 12, loc.alt * 10);                                   // cm -> mm AMSL-ish
    fwcpp::gcs::write_i32_le(p + 16, static_cast<std::int32_t>(-b.sim().position.z * 1000.0f)); // mm above start
    fwcpp::gcs::write_u16_le(p + 20, static_cast<std::uint16_t>(
        static_cast<std::int16_t>(b.sim().velocity_ef.x * 100.0f)));
    fwcpp::gcs::write_u16_le(p + 22, static_cast<std::uint16_t>(
        static_cast<std::int16_t>(b.sim().velocity_ef.y * 100.0f)));
    fwcpp::gcs::write_u16_le(p + 24, static_cast<std::uint16_t>(
        static_cast<std::int16_t>(b.sim().velocity_ef.z * 100.0f)));
    fwcpp::gcs::write_u16_le(p + 26, static_cast<std::uint16_t>(hdg_deg * 100.0f));
    send_frame(g, fwcpp::gcs::kMsgIdGlobalPositionInt, p, sizeof(p));
}

inline void send_vfr_hud(GcsLink& g, const Bridge& b) {
    const auto& s = b.sim();
    const float groundspeed = std::sqrt(s.velocity_ef.x * s.velocity_ef.x + s.velocity_ef.y * s.velocity_ef.y);
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    s.dcm.to_euler(&roll, &pitch, &yaw);
    float hdg_deg = fwcpp::math::degrees(yaw);
    if (hdg_deg < 0.0f) {
        hdg_deg += 360.0f;
    }
    const float throttle_pct =
        b.plane().srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle);
    std::uint8_t p[20] = {};
    fwcpp::gcs::write_f32_le(p, s.airspeed);
    fwcpp::gcs::write_f32_le(p + 4, groundspeed);
    fwcpp::gcs::write_f32_le(p + 8, static_cast<float>(s.get_location().alt) * 0.01f);
    fwcpp::gcs::write_f32_le(p + 12, -s.velocity_ef.z);
    fwcpp::gcs::write_u16_le(p + 16, static_cast<std::uint16_t>(static_cast<std::int16_t>(hdg_deg)));
    fwcpp::gcs::write_u16_le(p + 18, static_cast<std::uint16_t>(throttle_pct));
    send_frame(g, fwcpp::gcs::kMsgIdVfrHud, p, sizeof(p));
}

inline void send_param_value(GcsLink& g, const char* name, float value,
                             std::uint16_t count, std::uint16_t index) {
    std::uint8_t p[25] = {};
    fwcpp::gcs::write_f32_le(p, value);
    fwcpp::gcs::write_u16_le(p + 4, count);
    fwcpp::gcs::write_u16_le(p + 6, index);
    std::strncpy(reinterpret_cast<char*>(p + 8), name, 16);
    p[24] = 9; // MAV_PARAM_TYPE_REAL32
    send_frame(g, fwcpp::gcs::kMsgIdParamValue, p, sizeof(p));
}

inline void send_mission_request_int(GcsLink& g, std::uint16_t seq) {
    std::uint8_t p[5] = {};
    fwcpp::gcs::write_u16_le(p, seq);
    p[2] = 255; // to the GCS
    p[3] = 0;
    p[4] = 0; // MAV_MISSION_TYPE_MISSION
    send_frame(g, fwcpp::gcs::kMsgIdMissionRequestInt, p, sizeof(p));
}

inline void send_mission_ack(GcsLink& g, std::uint8_t type) {
    std::uint8_t p[4] = {};
    p[0] = 255;
    p[1] = 0;
    p[2] = type; // MAV_MISSION_ACCEPTED = 0
    p[3] = 0;
    send_frame(g, fwcpp::gcs::kMsgIdMissionAck, p, sizeof(p));
}

inline void send_mission_count(GcsLink& g, std::uint16_t count) {
    std::uint8_t p[5] = {};
    fwcpp::gcs::write_u16_le(p, count);
    p[2] = 255;
    p[3] = 0;
    p[4] = 0;
    send_frame(g, fwcpp::gcs::kMsgIdMissionCount, p, sizeof(p));
}

inline void send_mission_item_int(GcsLink& g, const Bridge::GcsGeoItem& r) {
    std::uint8_t p[38] = {};
    fwcpp::gcs::write_f32_le(p, r.p1);
    fwcpp::gcs::write_f32_le(p + 4, r.p2);
    fwcpp::gcs::write_i32_le(p + 16, r.lat_1e7);
    fwcpp::gcs::write_i32_le(p + 20, r.lng_1e7);
    fwcpp::gcs::write_f32_le(p + 24, r.z);
    fwcpp::gcs::write_u16_le(p + 28, r.seq);
    fwcpp::gcs::write_u16_le(p + 30, r.command);
    p[32] = 255;
    p[33] = 0;
    p[34] = r.frame;
    p[35] = (r.seq == 0) ? 1 : 0; // current
    p[36] = 1;                    // autocontinue
    p[37] = 0;                    // mission_type
    send_frame(g, fwcpp::gcs::kMsgIdMissionItemInt, p, sizeof(p));
}

inline void send_command_ack(GcsLink& g, std::uint16_t command, std::uint8_t result) {
    std::uint8_t p[3] = {};
    fwcpp::gcs::write_u16_le(p, command);
    p[2] = result; // MAV_RESULT_ACCEPTED=0 / UNSUPPORTED=3 / FAILED=4
    send_frame(g, fwcpp::gcs::kMsgIdCommandAck, p, sizeof(p));
}

inline void handle_frame(GcsLink& g, Bridge& bridge, const fwcpp::gcs::Frame& f) {
    using namespace fwcpp::gcs;
    // Frame.payload is zero-initialized before the copy, so MAVLink v2
    // trailing-zero payload truncation is transparently undone: reads past
    // payload_len see zeros, exactly as the spec intends.
    const std::uint8_t* p = f.payload.data();

    switch (f.msgid) {
    case kMsgIdHeartbeat:
        break; // peer already learned from the datagram source

    case kMsgIdParamRequestList:
        // Minimal parameter surface: enough for a GCS to complete its sync.
        send_param_value(g, "SYSID_THISMAV", static_cast<float>(g.sysid), 1, 0);
        break;

    case kMsgIdParamSet: {
        // Accept-and-echo: no AP_Param storage is wired in this port, so a
        // set is acknowledged (echoed back) but not persisted anywhere.
        char name[17] = {};
        std::memcpy(name, p + 6, 16);
        send_param_value(g, name, read_f32_le(p), 1, 0);
        break;
    }

    case kMsgIdCommandLong: {
        const std::uint8_t target_sys = p[30];
        if (target_sys != 0 && target_sys != g.sysid) {
            break; // addressed to a different aircraft on a shared link
        }
        const std::uint16_t command = read_u16_le(p + 28);
        const float p1 = read_f32_le(p);
        const float p2 = read_f32_le(p + 4);
        switch (command) {
        case 400: // MAV_CMD_COMPONENT_ARM_DISARM
            bridge.gcs_arm(p1 > 0.5f);
            send_command_ack(g, command, 0);
            break;
        case 176: // MAV_CMD_DO_SET_MODE (param2 = custom mode)
            send_command_ack(g, command,
                bridge.gcs_set_mode_custom(static_cast<std::uint32_t>(p2)) ? 0 : 4);
            break;
        case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
            send_command_ack(g, command, bridge.gcs_set_mode_custom(11) ? 0 : 4);
            break;
        case 22: // MAV_CMD_NAV_TAKEOFF -> TAKEOFF mode
            send_command_ack(g, command, bridge.gcs_set_mode_custom(13) ? 0 : 4);
            break;
        default:
            send_command_ack(g, command, 3); // MAV_RESULT_UNSUPPORTED
            break;
        }
        break;
    }

    case kMsgIdMissionCount: {
        if (p[2] != 0 && p[2] != g.sysid) {
            break; // target_system mismatch on a shared link
        }
        const std::uint16_t count = read_u16_le(p);
        if (count == 0 || count > kMaxUploadRows) {
            send_mission_ack(g, count == 0 ? 0 : 1 /*MAV_MISSION_ERROR*/);
            g.up_active = false;
            break;
        }
        g.up_active = true;
        g.up_count = count;
        g.up_next = 0;
        send_mission_request_int(g, 0);
        break;
    }

    case kMsgIdMissionItemInt: {
        if (!g.up_active) {
            break;
        }
        const std::uint16_t seq = read_u16_le(p + 28);
        if (seq != g.up_next) {
            send_mission_request_int(g, g.up_next); // re-request in order
            break;
        }
        Bridge::GcsGeoItem& r = g.rows[seq];
        r.seq = seq;
        r.command = read_u16_le(p + 30);
        r.frame = p[34];
        r.p1 = read_f32_le(p);
        r.p2 = read_f32_le(p + 4);
        r.lat_1e7 = read_i32_le(p + 16);
        r.lng_1e7 = read_i32_le(p + 20);
        r.z = read_f32_le(p + 24);
        ++g.up_next;
        if (g.up_next < g.up_count) {
            send_mission_request_int(g, g.up_next);
            break;
        }
        // Complete — convert + load, cache for download, ack.
        g.up_active = false;
        const int loaded = bridge.gcs_mission_load(g.rows.data(), g.up_count);
        if (loaded > 0) {
            g.cached_rows = g.rows;
            g.cached_count = g.up_count;
            send_mission_ack(g, 0); // MAV_MISSION_ACCEPTED
            std::printf("mavlink: mission upload accepted (%u rows, %d flyable)\n",
                        static_cast<unsigned>(g.up_count), loaded);
        } else {
            send_mission_ack(g, 4); // MAV_MISSION_UNSUPPORTED
        }
        std::fflush(stdout);
        break;
    }

    case kMsgIdMissionRequestList:
        send_mission_count(g, g.cached_count);
        break;

    case kMsgIdMissionRequestInt: {
        const std::uint16_t seq = read_u16_le(p);
        if (seq < g.cached_count) {
            send_mission_item_int(g, g.cached_rows[seq]);
        }
        break;
    }

    default:
        break;
    }
}

inline void handle_datagram(GcsLink& g, Bridge& bridge, const std::uint8_t* data, std::size_t len,
                            const sockaddr_in& from) {
    g.peer = from;
    g.has_peer = true;
    std::size_t off = 0;
    while (off < len) {
        const auto res = fwcpp::gcs::decode_v2(
            std::span<const std::uint8_t>(data + off, len - off));
        if (!res.has_value()) {
            if (res.error() == fwcpp::gcs::DecodeError::kBadMagic) {
                ++off; // resync
                continue;
            }
            break; // truncated / bad crc — drop the rest of the datagram
        }
        const fwcpp::gcs::Frame& f = res.value();
        handle_frame(g, bridge, f);
        off += fwcpp::gcs::kHeaderLenV2 + f.payload_len + fwcpp::gcs::kCrcLen;
    }
}

} // namespace mavgcs

} // namespace

int main(int argc, char** argv) {
    std::uint16_t port = kDefaultPort;
    std::uint16_t mavlink_port = 0;   // 0 = GCS endpoint disabled
    std::uint8_t mavlink_sysid = 1;   // MAVLink system id (per-aircraft discriminator)
    bool debug = false;
    if (argc > 1) {
        const int parsed = std::atoi(argv[1]);
        if (parsed <= 0 || parsed > 65535) {
            std::fprintf(stderr, "usage: %s [udp_port] [--debug] [--mavlink <port>] [--sysid <1-255>]\n", argv[0]);
            return 1;
        }
        port = static_cast<std::uint16_t>(parsed);
    }
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--debug") == 0) {
            debug = true;
        } else if (std::strcmp(argv[i], "--mavlink") == 0 && i + 1 < argc) {
            const int parsed = std::atoi(argv[++i]);
            if (parsed > 0 && parsed <= 65535) {
                mavlink_port = static_cast<std::uint16_t>(parsed);
            }
        } else if (std::strcmp(argv[i], "--sysid") == 0 && i + 1 < argc) {
            const int parsed = std::atoi(argv[++i]);
            if (parsed > 0 && parsed <= 255) {
                mavlink_sysid = static_cast<std::uint8_t>(parsed);
            }
        }
    }

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::fprintf(stderr, "WSAStartup failed\n");
        return 1;
    }
#endif

    const auto sock = ::socket(AF_INET, SOCK_DGRAM, 0);
#ifdef _WIN32
    if (sock == INVALID_SOCKET) {
#else
    if (sock < 0) {
#endif
        std::fprintf(stderr, "socket() failed\n");
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // loopback only, deliberately
    if (::bind(sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::fprintf(stderr, "bind(127.0.0.1:%u) failed\n", static_cast<unsigned>(port));
        return 1;
    }

    // Optional MAVLink GCS socket (loopback, like the host socket).
    mavgcs::GcsLink gcs{};
    gcs.sysid = mavlink_sysid;
    if (mavlink_port != 0) {
        gcs.sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in gaddr{};
        gaddr.sin_family = AF_INET;
        gaddr.sin_port = htons(mavlink_port);
        gaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(gcs.sock, reinterpret_cast<const sockaddr*>(&gaddr), sizeof(gaddr)) != 0) {
            std::fprintf(stderr, "bind mavlink 127.0.0.1:%u failed — GCS endpoint disabled\n",
                         static_cast<unsigned>(mavlink_port));
        } else {
            gcs.enabled = true;
        }
    }

    std::printf("wopr_bridge: Plane+SimPlane+SitlHarness listening on 127.0.0.1:%u (lockstep, %.0fHz ticks)%s%u\n",
                static_cast<unsigned>(port), 1.0f / kDt,
                gcs.enabled ? "; MAVLink GCS on 127.0.0.1:" : "; MAVLink GCS disabled, port ",
                static_cast<unsigned>(mavlink_port));
    std::fflush(stdout);

    Bridge bridge;
    bridge.set_debug(debug);
    std::uint8_t buf[2048];

    auto now_wall = std::chrono::steady_clock::now();
    gcs.last_hb = now_wall - std::chrono::seconds(2);
    gcs.last_stream = now_wall;

    for (;;) {
        // Wait on the host socket (+ the GCS socket when enabled) with a
        // 100 ms cap so wall-clock GCS pacing keeps running while the host
        // is idle. Sim time itself remains host-stepped and deterministic.
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);
        if (gcs.enabled) {
            FD_SET(gcs.sock, &readfds);
        }
#ifdef _WIN32
        const int nfds = 0; // ignored on Windows
#else
        const int nfds = static_cast<int>(std::max<sock_type>(sock, gcs.enabled ? gcs.sock : sock)) + 1;
#endif
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        const int ready = ::select(nfds, &readfds, nullptr, nullptr, &tv);

        now_wall = std::chrono::steady_clock::now();
        if (gcs.enabled && bridge.is_initialized() &&
            now_wall - gcs.last_hb >= std::chrono::seconds(1)) {
            gcs.last_hb = now_wall;
            mavgcs::send_heartbeat(gcs, bridge);
        }

        if (ready <= 0) {
            continue;
        }

        // GCS traffic first — it never advances sim time.
        if (gcs.enabled && FD_ISSET(gcs.sock, &readfds)) {
            sockaddr_in gfrom{};
            socklen_type gfrom_len = sizeof(gfrom);
            const int grecv = static_cast<int>(::recvfrom(gcs.sock, reinterpret_cast<char*>(buf), sizeof(buf), 0,
                                                          reinterpret_cast<sockaddr*>(&gfrom), &gfrom_len));
            if (grecv > 0) {
                mavgcs::handle_datagram(gcs, bridge, buf, static_cast<std::size_t>(grecv), gfrom);
            }
        }

        if (!FD_ISSET(sock, &readfds)) {
            continue;
        }

        sockaddr_in from{};
        socklen_type from_len = sizeof(from);
        const int received = static_cast<int>(::recvfrom(sock, reinterpret_cast<char*>(buf), sizeof(buf), 0,
                                                         reinterpret_cast<sockaddr*>(&from), &from_len));
        if (received <= 0) {
            continue;
        }

        StateReply reply{};
        reply.magic = kMagic;
        reply.type = 0x81;
        reply.status = static_cast<std::uint8_t>(Status::kBadPacket);

        if (received >= static_cast<int>(sizeof(RequestHeader))) {
            RequestHeader header{};
            std::memcpy(&header, buf, sizeof(header));
            reply.seq = header.seq;
            const std::uint8_t* payload = buf + sizeof(RequestHeader);
            const int payload_len = received - static_cast<int>(sizeof(RequestHeader));

            if (header.magic == kMagic) {
                Status status = Status::kBadPacket;
                switch (static_cast<MsgType>(header.type)) {
                case MsgType::kInit: {
                    // Accept the pre-model 56-byte INIT too (missing tail =
                    // zeroed model path = built-in defaults).
                    constexpr int kInitCoreBytes = 56;
                    if (payload_len >= kInitCoreBytes) {
                        InitPayload p{};
                        std::memcpy(&p, payload,
                                    std::min<int>(payload_len, static_cast<int>(sizeof(p))));
                        p.model_json[sizeof(p.model_json) - 1] = '\0';
                        status = bridge.init(p);
                    }
                    break;
                }
                case MsgType::kStep:
                    if (payload_len >= static_cast<int>(sizeof(StepPayload))) {
                        StepPayload p{};
                        std::memcpy(&p, payload, sizeof(p));
                        if (payload_len >= static_cast<int>(sizeof(StepPayload)) + kStepTerrainTailLen) {
                            float terrain_hae = 0.0f;
                            std::uint8_t terrain_valid = 0;
                            std::memcpy(&terrain_hae, payload + sizeof(StepPayload), sizeof(terrain_hae));
                            std::memcpy(&terrain_valid, payload + sizeof(StepPayload) + sizeof(terrain_hae), 1);
                            bridge.set_terrain(terrain_hae, terrain_valid != 0);
                        }
                        status = bridge.step(p);
                    }
                    break;
                case MsgType::kMode:
                    if (payload_len >= static_cast<int>(sizeof(ModePayload))) {
                        ModePayload p{};
                        std::memcpy(&p, payload, sizeof(p));
                        status = bridge.set_mode(p);
                    }
                    break;
                case MsgType::kMission:
                    if (payload_len >= static_cast<int>(sizeof(MissionPayloadHeader))) {
                        MissionPayloadHeader mh{};
                        std::memcpy(&mh, payload, sizeof(mh));
                        const int needed =
                            static_cast<int>(sizeof(MissionPayloadHeader)) + mh.count * static_cast<int>(sizeof(MissionItemWire));
                        if (payload_len >= needed) {
                            const auto* items =
                                reinterpret_cast<const MissionItemWire*>(payload + sizeof(MissionPayloadHeader));
                            status = bridge.set_mission(items, mh.count);
                            if (status == Status::kOk) {
                                bridge.observe_mission_base();
                            }
                        }
                    }
                    break;
                case MsgType::kArm:
                    if (payload_len >= static_cast<int>(sizeof(ArmPayload))) {
                        ArmPayload p{};
                        std::memcpy(&p, payload, sizeof(p));
                        status = bridge.arm(p);
                    }
                    break;
                default:
                    break;
                }
                reply.status = static_cast<std::uint8_t>(status);
            }
        }

        bridge.fill_state(reply);
        ::sendto(sock, reinterpret_cast<const char*>(&reply), sizeof(reply), 0,
                 reinterpret_cast<const sockaddr*>(&from), from_len);

        // GCS telemetry stream, throttled by wall clock (~10 Hz while the
        // host is stepping the sim; silent while the sim is paused).
        if (gcs.enabled && bridge.is_initialized() &&
            now_wall - gcs.last_stream >= std::chrono::milliseconds(100)) {
            gcs.last_stream = now_wall;
            mavgcs::send_attitude(gcs, bridge);
            mavgcs::send_global_position_int(gcs, bridge);
            mavgcs::send_vfr_hud(gcs, bridge);
        }
    }
}
