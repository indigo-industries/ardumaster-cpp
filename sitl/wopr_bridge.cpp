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

#include <fwcpp/hal_sitl/sitl_harness.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

namespace {

constexpr std::uint32_t kMagic = 0x57534231; // 'WSB1'
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
    std::uint16_t mission_index;
    std::uint16_t mission_count;
    std::uint32_t sim_time_ms;
    std::uint8_t armed;
    std::uint8_t initialized;
    std::uint8_t pad[2];
};
static_assert(sizeof(StateReply) == 96);
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

        // Airframe model, BEFORE any dynamics run. A given-but-broken model
        // is a hard refusal: silently flying the skywalker defaults under a
        // different name is the exact failure this status exists to prevent.
        if (p.model_json[0] != '\0' && !apply_model(p.model_json)) {
            initialized_ = false;
            return Status::kBadModel;
        }

        fwcpp::Location start;
        start.lat = p.lat_1e7;
        start.lng = p.lng_1e7;
        start.alt = static_cast<std::int32_t>(p.alt_m * 100.0f);
        sim_.set_start_location(start, p.yaw_deg);

        // Upstream Plane sets GROUND_BEHAVIOR_FWD_ONLY (SIM_Plane.cpp:50);
        // the port's SimPlane defaults kNone for its own tests. The bridge
        // restores upstream's choice — upright, forward-only ground contact
        // is what makes a takeoff roll clean instead of a physics wrestle.
        sim_.ground_behavior = fwcpp::sim::GroundBehavior::kFwdOnly;

        // Plane-side navigation frame: Location() offset by NED metres from
        // the start point (update_current_loc's own convention). Home IS the
        // start point, i.e. the frame origin at altitude 0.
        plane_.set_home(fwcpp::Location(0, 0, 0, fwcpp::Location::AltFrame::ABSOLUTE));

        if (p.in_air != 0) {
            sim_.position = {p.pos_ned_m[0], p.pos_ned_m[1], p.pos_ned_m[2]};
            sim_.velocity_ef = {p.vel_ned_mps[0], p.vel_ned_mps[1], p.vel_ned_mps[2]};
            sim_.dcm.from_euler(p.rpy_rad[0], p.rpy_rad[1], p.rpy_rad[2]);
            sim_.update_position();
            // Approximate initial airspeed from inertial velocity (no wind
            // at t=0); the plant recomputes it properly on the first step.
            sim_.airspeed = std::sqrt(sim_.velocity_ef.x * sim_.velocity_ef.x +
                                      sim_.velocity_ef.y * sim_.velocity_ef.y +
                                      sim_.velocity_ef.z * sim_.velocity_ef.z);
        } else {
            // Ground start: the proven sitl_run boot sequence, including the
            // boot-time airspeed zero-offset calibration (only valid while
            // stationary — deliberately skipped for the in-air warm start,
            // where the sensor's default zero offset is already exact).
            plane_.airspeed_sensor.start_calibration(0);
        }

        set_neutral_sticks();
        plane_.control_mode = &plane_.mode_manual;
        mode_ = BridgeMode::kManual;
        now_ms_ = 0;
        initialized_ = true;
        return Status::kOk;
    }

    // One model file configures BOTH halves of an airframe (see InitPayload's
    // model_json doc): SimPlane::load_coeffs() takes the aero-coefficient
    // subset (upstream's own plane-JSON schema), then the bridge applies the
    // mass/thrust and flight-code envelope keys itself.
    bool apply_model(const char* path) {
        if (!sim_.load_coeffs(path)) {
            return false;
        }
        fwcpp::sim::JsonValue obj;
        std::string err;
        if (!fwcpp::sim::load_json_file(path, obj, err)) {
            return false;
        }
        fwcpp::sim::json_get_float(obj, "mass", sim_.mass);
        fwcpp::sim::json_get_float(obj, "hover_throttle", sim_.hover_throttle);
        fwcpp::sim::json_get_vector3(obj, "moment_inertia", sim_.moment_inertia);
        fwcpp::sim::json_get_float(obj, "ground_friction", sim_.ground_friction);
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
        return true;
    }

    Status step(const StepPayload& p) {
        if (!initialized_) {
            return Status::kNotInitialized;
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
        fwcpp::vehicle::Mode* target = nullptr;
        switch (static_cast<BridgeMode>(p.mode)) {
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
        const BridgeMode requested = static_cast<BridgeMode>(p.mode);
        // FBWB/CRUISE/LOITER: their enter()s never seed target_altitude_cm —
        // "A CALLER MUST CALL set_target_altitude_current" (their own class
        // banners). This bridge is that caller, seeding from sim truth.
        if (requested == BridgeMode::kFbwb || requested == BridgeMode::kCruise ||
            requested == BridgeMode::kLoiter) {
            plane_.set_target_altitude_current(static_cast<std::int32_t>(-sim_.position.z * 100.0f));
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

    void fill_state(StateReply& r) const {
        r.initialized = initialized_ ? 1 : 0;
        r.mode = static_cast<std::uint8_t>(mode_);
        r.on_ground = sim_.on_ground() ? 1 : 0;
        r.pos_ned_m[0] = sim_.position.x;
        r.pos_ned_m[1] = sim_.position.y;
        r.pos_ned_m[2] = sim_.position.z;
        r.vel_ned_mps[0] = sim_.velocity_ef.x;
        r.vel_ned_mps[1] = sim_.velocity_ef.y;
        r.vel_ned_mps[2] = sim_.velocity_ef.z;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        sim_.dcm.to_euler(&roll, &pitch, &yaw);
        r.rpy_rad[0] = roll;
        r.rpy_rad[1] = pitch;
        r.rpy_rad[2] = yaw;
        r.gyro_rps[0] = sim_.gyro.x;
        r.gyro_rps[1] = sim_.gyro.y;
        r.gyro_rps[2] = sim_.gyro.z;
        r.airspeed_mps = sim_.airspeed;
        r.hagl_m = sim_.hagl();
        r.servo_norm[0] =
            plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) / fwcpp::vehicle::kServoMax;
        r.servo_norm[1] =
            plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator) / fwcpp::vehicle::kServoMax;
        r.servo_norm[2] =
            plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder) / fwcpp::vehicle::kServoMax;
        r.servo_norm[3] = plane_.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) / 100.0f;
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
    BridgeMode mode_ = BridgeMode::kManual;
    std::uint32_t now_ms_ = 0;
    std::uint16_t rc_pwm_[4] = {1500, 1500, 1100, 1500};
    bool initialized_ = false;
    bool debug_ = false;
    const fwcpp::vehicle::MissionItem* mission_base_ = nullptr;
    std::size_t mission_base_size_ = 0;
};

} // namespace

int main(int argc, char** argv) {
    std::uint16_t port = kDefaultPort;
    bool debug = false;
    if (argc > 1) {
        const int parsed = std::atoi(argv[1]);
        if (parsed <= 0 || parsed > 65535) {
            std::fprintf(stderr, "usage: %s [udp_port] [--debug]\n", argv[0]);
            return 1;
        }
        port = static_cast<std::uint16_t>(parsed);
    }
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--debug") == 0) {
            debug = true;
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

    std::printf("wopr_bridge: Plane+SimPlane+SitlHarness listening on 127.0.0.1:%u (lockstep, %.0fHz ticks)\n",
                static_cast<unsigned>(port), 1.0f / kDt);
    std::fflush(stdout);

    Bridge bridge;
    bridge.set_debug(debug);
    std::uint8_t buf[2048];

    for (;;) {
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
    }
}
