#pragma once

// CCP-068: Copter's own Mission/MissionItem, direct analogue of
// fwcpp::vehicle::Mission / MissionItem (ap-vehicle/plane.hpp) for the
// fixed-wing port. Same design, same constraints (ADR-0012: header-only,
// no dynamic allocation, no exceptions), same "this port's own bound on
// mission length, not upstream's" reasoning for kMaxMissionItems.
//
// ONE DELIBERATE DIFFERENCE FROM PLANE'S MissionItem: waypoints here are
// NED METRES FROM THE SITL START FIX (north_m/east_m/down_m), not a
// Location (lat/lng/alt). This matches the frame every existing Copter SITL
// primitive already operates in — SimMulticopter::position, WpNav's
// set_wp_destination_ned_m, AC_PosControl NE/D — none of which ever see a
// geodetic coordinate; home_lat/home_lng on LeftoverCopter exist ONLY to
// synthesize a plausible GPS fix, not to navigate by. Converting to/from
// Location here would be a lossy round-trip through a frame nothing else in
// this port's Copter navigation stack actually uses. Down is standard NED
// (negative = up), matching leftover_set_wp_ned / SimMulticopter::position.z
// and every existing PosControl D call site's own sign convention.
//
// leftover_mission_advance (copter_sitl_run_leftover.hpp, CCP-065) is left
// completely unmodified — it is still exactly the fixed
// arm/takeoff/outbound-1-mile/RTL/land demo its own tests assert against.
// This Mission type is consumed by a NEW, separate driver
// (copter_auto_mission.hpp, CCP-068) that walks an arbitrary
// caller-supplied item list using the SAME underlying PosControl/WPNav
// primitives leftover_mission_advance already proved out.

#include <array>
#include <cstddef>
#include <span>

namespace fwcpp::copter {

// upstream vocabulary this slice's ModeAuto-equivalent dispatch acts on —
// mirrors fwcpp::vehicle::MissionCommand's own three-value scope exactly
// (MAV_CMD_NAV_WAYPOINT / _TAKEOFF / _LAND), plus RTL because Copter's own
// leftover_mission_advance already implements a real RTL leg (fly to NED
// 0,0 at the current hold altitude) that has no Plane-side equivalent to
// mirror — DO/jump/loiter/spline/VTOL commands remain entirely absent from
// this vocabulary, same exclusion Plane's own MissionItem documents.
enum class MissionCommand : std::uint8_t {
    Waypoint,  // upstream: MAV_CMD_NAV_WAYPOINT — fly to (north_m, east_m, down_m) via WPNav
    Takeoff,   // upstream: MAV_CMD_NAV_TAKEOFF — climb to -down_m AGL at the current NE position
    Rtl,       // upstream: MAV_CMD_NAV_RETURN_TO_LAUNCH — fly to NED (0,0) at the current hold altitude
    Land,      // upstream: MAV_CMD_NAV_LAND — descend to touchdown at the current NE position
};

// Defaults to Waypoint, matching Plane's MissionItem default-construction
// contract (a value-initialized item is a plain NAV_WAYPOINT).
struct MissionItem {
    MissionCommand command = MissionCommand::Waypoint;
    // NED metres from the SITL start fix. Meaningful for Waypoint (full
    // 3-tuple) and Takeoff (down_m only — north_m/east_m are ignored; a
    // takeoff climbs straight up from wherever the vehicle already is,
    // matching leftover_mission_advance's own kTakeoff case). Rtl and Land
    // read neither field — Rtl's destination is always NED (0,0) at the
    // ALTITUDE the takeoff item commanded (tracked by the mission runner,
    // not stored per-item, exactly like leftover_mission_advance already
    // does for its own hardcoded RTL leg); Land descends from wherever the
    // vehicle already is.
    float north_m = 0.0f;
    float east_m = 0.0f;
    float down_m = 0.0f;
    // 0 means "use WpNav's own default arrival radius" — same
    // zero-means-default convention as Plane's acceptance_radius_m.
    // Meaningful only for Waypoint.
    float acceptance_radius_m = 0.0f;
};

// Fixed-size (ADR-0012: no dynamic allocation), in-memory, ordered list of
// MissionItems, flown sequentially via current()/peek_next()/advance() —
// byte-for-byte the same shape and semantics as fwcpp::vehicle::Mission
// (ap-vehicle/plane.hpp), so a caller already familiar with the Plane side
// needs to learn nothing new here.
inline constexpr std::size_t kMaxMissionItems = 32;

class Mission {
public:
    // Sets the mission list, replacing any previous one, and resets to the
    // first item. Returns false (leaving any existing mission untouched) if
    // items.size() exceeds kMaxMissionItems.
    bool load(std::span<const MissionItem> items) {
        if (items.size() > kMaxMissionItems) {
            return false;
        }
        count_ = items.size();
        for (std::size_t i = 0; i < count_; ++i) {
            items_[i] = items[i];
        }
        current_index_ = 0;
        return true;
    }

    [[nodiscard]] std::size_t size() const { return count_; }
    [[nodiscard]] bool empty() const { return count_ == 0; }

    // The item currently being flown toward. nullptr if no mission loaded.
    [[nodiscard]] const MissionItem* current() const {
        return current_index_ < count_ ? &items_[current_index_] : nullptr;
    }

    // The item after the current one. nullptr if the current item is the
    // last one (or no mission is loaded).
    [[nodiscard]] const MissionItem* peek_next() const {
        return (current_index_ + 1 < count_) ? &items_[current_index_ + 1] : nullptr;
    }

    // True once current() is the LAST item in the mission (or the mission
    // is empty).
    [[nodiscard]] bool at_last() const { return current_index_ + 1 >= count_; }

    // Moves to the next item. No-op, returns false, if already at_last()
    // (or empty) — the runner holds at the current, final item.
    bool advance() {
        if (at_last()) {
            return false;
        }
        ++current_index_;
        return true;
    }

    void reset() { current_index_ = 0; }

private:
    std::array<MissionItem, kMaxMissionItems> items_{};
    std::size_t count_ = 0;
    std::size_t current_index_ = 0;
};

}  // namespace fwcpp::copter
