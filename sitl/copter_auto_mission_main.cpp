// CCP-068: standalone test for the general-purpose Copter mission runner
// (copter_auto_mission.hpp) — flies a genuinely aggressive 7-item mission
// (arm+takeoff, four waypoints with sharp >=90 deg heading reversals and
// large altitude swings, RTL, land), something the old fixed 2-point
// CCP-065 demo (copter_main.cpp) cannot express at all. Validates real
// per-leg arrival (not just final touchdown) and true attitude (bank/pitch)
// during the turns, the same way copter_main.cpp validates true_rpy_deg.
// copter_main.cpp / copter_sitl_run_leftover.hpp are untouched by this file.
//
// USAGE: copter_auto_mission_run [--help] [duration_seconds]

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/mission.hpp>
#include <fwcpp/hal_sitl/copter_auto_mission.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

namespace {

const char* mission_command_name(const fwcpp::copter::MissionItem* item) {
    if (item == nullptr) {
        return "none";
    }
    switch (item->command) {
    case fwcpp::copter::MissionCommand::Waypoint:
        return "WAYPOINT";
    case fwcpp::copter::MissionCommand::Takeoff:
        return "TAKEOFF";
    case fwcpp::copter::MissionCommand::Rtl:
        return "RTL";
    case fwcpp::copter::MissionCommand::Land:
        return "LAND";
    }
    return "?";
}

void print_usage(const char* argv0) {
    std::cout << "usage: " << argv0 << " [--help] [duration_seconds > 0]\n"
              << "  CCP-068: general-purpose Copter Mission runner stress test.\n"
              << "  Mission: takeoff 15m, WP1 (200N,0E,15m), WP2 (200N,200E,35m sharp 90deg+climb),\n"
              << "           WP3 (-100N,200E,35m sharp >90deg reversal), WP4 (-100N,-150E,15m turn+descent),\n"
              << "           RTL (15m), land.\n"
              << "  Default duration: 300 simulated seconds @ 400Hz (until LANDED).\n";
}

}  // namespace

int main(int argc, char** argv) {
    constexpr float kDt = 0.0025f;
    constexpr int kTicksPerSecond = static_cast<int>(1.0f / kDt);

    int duration_s = 300;
    if (argc > 1) {
        const std::string_view arg1{argv[1]};
        if (arg1 == "--help" || arg1 == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        duration_s = std::atoi(argv[1]);
        if (duration_s <= 0) {
            print_usage(argv[0]);
            return 1;
        }
    }
    const int num_ticks = duration_s * kTicksPerSecond;

    fwcpp::copter::LeftoverCopter copter{};
    fwcpp::sim::SimMulticopter sim{"x"};
    fwcpp::hal_sitl::SitlCopterHarness harness(copter, sim);
    fwcpp::hal_sitl::copter_auto::AutoMissionState state{};
    fwcpp::copter::Mission mission{};

    using fwcpp::copter::MissionCommand;
    using fwcpp::copter::MissionItem;
    // Sharp turns: WP1->WP2 is a 90 deg turn (North -> East) with a 20m
    // climb over the leg; WP2->WP3 is a >90 deg reversal (East -> West,
    // heading back South of the outbound track) at constant 35m; WP3->WP4
    // is another sharp turn (West -> South-ish) with a 20m descent. This is
    // the kind of geometry the OLD fixed CCP-065 demo (one straight leg out,
    // straight RTL back) cannot exercise at all.
    const std::array<MissionItem, 7> items{{
        MissionItem{.command = MissionCommand::Takeoff, .down_m = -15.0f},
        MissionItem{.command = MissionCommand::Waypoint, .north_m = 200.0f, .east_m = 0.0f, .down_m = -15.0f},
        MissionItem{.command = MissionCommand::Waypoint, .north_m = 200.0f, .east_m = 200.0f, .down_m = -35.0f},
        MissionItem{.command = MissionCommand::Waypoint, .north_m = -100.0f, .east_m = 200.0f, .down_m = -35.0f},
        MissionItem{.command = MissionCommand::Waypoint, .north_m = -100.0f, .east_m = -150.0f, .down_m = -15.0f},
        MissionItem{.command = MissionCommand::Rtl},
        MissionItem{.command = MissionCommand::Land},
    }};
    if (!mission.load(items)) {
        std::cerr << "mission.load failed\n";
        return 1;
    }

    std::cout << "CCP-068 SITL: aggressive Mission stress test, " << mission.size() << " items, " << duration_s
              << " simulated seconds (" << num_ticks << " ticks @ 400Hz)\n";
    std::cout << std::fixed << std::setprecision(3);

    // Per-item arrival ground truth (waypoints only; Takeoff/Rtl/Land have
    // no fixed (n,e) target of their own to check against).
    struct LegTarget {
        std::size_t item_index;
        float n_m;
        float e_m;
        float d_m;
    };
    std::array<LegTarget, 3> waypoint_targets{{
        {1, 200.0f, 0.0f, -15.0f},
        {2, 200.0f, 200.0f, -35.0f},
        {3, -100.0f, 200.0f, -35.0f},
    }};
    std::array<bool, 3> leg_arrived{};
    std::array<float, 3> leg_miss_m{};

    auto true_rpy_deg = [&](float& roll, float& pitch, float& yaw) {
        float r = 0.0f, p = 0.0f, y = 0.0f;
        sim.dcm.to_euler(&r, &p, &y);
        roll = fwcpp::math::degrees(r);
        pitch = fwcpp::math::degrees(p);
        yaw = fwcpp::math::degrees(y);
    };

    auto print_telemetry = [&](float t_s, const char* why) {
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        true_rpy_deg(roll, pitch, yaw);
        std::cout << "t=" << std::setw(8) << t_s << "s"
                  << "  phase=" << fwcpp::hal_sitl::copter_auto::auto_phase_name(state.phase) << "  " << why
                  << "  item=" << mission_command_name(mission.current())
                  << "  N=" << sim.position.x << "  E=" << sim.position.y << "  D=" << sim.position.z
                  << "  alt=" << (-sim.position.z) << "m"
                  << "  armed=" << (copter.motors_armed ? 1 : 0)
                  << "  land_complete=" << (copter.land_complete ? 1 : 0)
                  << "  horiz_spd=" << std::hypot(sim.velocity_ef.x, sim.velocity_ef.y) << "m/s"
                  << "  vert_spd=" << (-sim.velocity_ef.z) << "m/s"
                  << "  true_rpy_deg=(" << roll << ", " << pitch << ", " << yaw << ")"
                  << "  ticks=" << harness.tick_count() << '\n';
    };

    print_telemetry(0.0f, "START");
    if (!fwcpp::hal_sitl::copter_auto::auto_mission_begin(state, mission)) {
        std::cerr << "auto_mission_begin failed — first item must be Takeoff\n";
        return 1;
    }

    float max_alt_m = 0.0f;
    float min_alt_after_first_climb_m = 1.0e9f;
    float max_horiz_spd = 0.0f;
    float max_climb_ms = 0.0f;
    float max_descent_ms = 0.0f;
    float max_abs_roll_deg = 0.0f;
    float max_abs_pitch_deg = 0.0f;
    int landed_tick = -1;
    auto last_phase = state.phase;
    const fwcpp::copter::MissionItem* last_item = mission.current();
    bool has_climbed_first_leg = false;

    for (int i = 0; i < num_ticks; ++i) {
        // Detect item transitions BEFORE stepping so we record arrival
        // ground-truth (position at the tick the runner advanced off a
        // waypoint) rather than the position one tick into the NEXT leg.
        const fwcpp::copter::MissionItem* cur_before = mission.current();

        fwcpp::hal_sitl::copter_auto::auto_mission_sitl_step(harness, state, mission, kDt);

        const float alt = -sim.position.z;
        if (alt > max_alt_m) {
            max_alt_m = alt;
        }
        const float hspd = std::hypot(sim.velocity_ef.x, sim.velocity_ef.y);
        if (hspd > max_horiz_spd) {
            max_horiz_spd = hspd;
        }
        const float vspd = -sim.velocity_ef.z;
        if (vspd > max_climb_ms) {
            max_climb_ms = vspd;
        }
        if (-vspd > max_descent_ms) {
            max_descent_ms = -vspd;
        }
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        true_rpy_deg(roll, pitch, yaw);
        max_abs_roll_deg = std::max(max_abs_roll_deg, std::abs(roll));
        max_abs_pitch_deg = std::max(max_abs_pitch_deg, std::abs(pitch));
        if (alt > 30.0f) {
            has_climbed_first_leg = true;
        }
        // Gate to the AUTO phase specifically — otherwise the final LAND
        // descent to 0m trivially satisfies "descended after climbing",
        // masking whether the WP3->WP4 mid-mission descent to 15m actually
        // happened before RTL.
        if (has_climbed_first_leg && state.phase == fwcpp::hal_sitl::copter_auto::AutoPhase::kAuto &&
            alt < min_alt_after_first_climb_m) {
            min_alt_after_first_climb_m = alt;
        }

        const fwcpp::copter::MissionItem* cur_after = mission.current();
        if (cur_after != cur_before) {
            // The item we just finished flying is cur_before; find it in
            // waypoint_targets by pointer identity via its address arithmetic
            // isn't available (Mission doesn't expose an index for a raw
            // pointer), so match by comparing recorded NED targets instead.
            for (std::size_t k = 0; k < waypoint_targets.size(); ++k) {
                if (leg_arrived[k]) {
                    continue;
                }
                const auto& t = waypoint_targets[k];
                if (cur_before != nullptr && std::abs(cur_before->north_m - t.n_m) < 0.01f &&
                    std::abs(cur_before->east_m - t.e_m) < 0.01f && std::abs(cur_before->down_m - t.d_m) < 0.01f) {
                    leg_arrived[k] = true;
                    leg_miss_m[k] = std::hypot(sim.position.x - t.n_m, sim.position.y - t.e_m);
                    std::cout << "  -- LEG ARRIVED wp" << (k + 1) << " target=(" << t.n_m << "," << t.e_m << ","
                              << t.d_m << ")  actual=(" << sim.position.x << "," << sim.position.y << ","
                              << sim.position.z << ")  miss=" << leg_miss_m[k] << "m\n";
                    break;
                }
            }
        }
        last_item = cur_after;

        const float t_s = static_cast<float>(i + 1) * kDt;
        if (state.phase != last_phase) {
            print_telemetry(t_s, fwcpp::hal_sitl::copter_auto::auto_phase_name(state.phase));
            last_phase = state.phase;
        }
        if (landed_tick < 0 && state.phase == fwcpp::hal_sitl::copter_auto::AutoPhase::kLanded) {
            landed_tick = i;
        }
        if ((i + 1) % (kTicksPerSecond * 5) == 0) {
            print_telemetry(t_s, "5s");
        }
        if (landed_tick >= 0 && (i - landed_tick) >= kTicksPerSecond) {
            break;
        }
    }
    (void)last_item;

    print_telemetry(static_cast<float>(harness.tick_count()) * kDt, "DONE");
    const float miss_origin = std::hypot(sim.position.x, sim.position.y);
    const bool all_legs_arrived = leg_arrived[0] && leg_arrived[1] && leg_arrived[2];
    const bool legs_accurate = all_legs_arrived && leg_miss_m[0] < 8.0f && leg_miss_m[1] < 8.0f && leg_miss_m[2] < 8.0f;
    const bool turned_hard = max_abs_roll_deg > 10.0f;  // sharp turns must actually bank, not just yaw
    const bool ok = state.phase == fwcpp::hal_sitl::copter_auto::AutoPhase::kLanded && max_alt_m >= 33.0f &&
                    min_alt_after_first_climb_m < 20.0f &&  // proves the WP3->WP4 descent to 15m really happened
                    legs_accurate && turned_hard && copter.land_complete && sim.on_ground() && miss_origin < 10.0f;

    std::cout << "\n=== TELEMETRY SUMMARY ===\n";
    std::cout << "legs arrived: wp1=" << leg_arrived[0] << " (miss=" << leg_miss_m[0] << "m)  wp2=" << leg_arrived[1]
              << " (miss=" << leg_miss_m[1] << "m)  wp3=" << leg_arrived[2] << " (miss=" << leg_miss_m[2] << "m)\n";
    std::cout << "max_alt=" << max_alt_m << "m  min_alt_after_climb=" << min_alt_after_first_climb_m
              << "m  max_horiz_spd=" << max_horiz_spd << "m/s  max_climb=" << max_climb_ms
              << "m/s  max_descent=" << max_descent_ms << "m/s\n";
    std::cout << "max_abs_roll=" << max_abs_roll_deg << "deg  max_abs_pitch=" << max_abs_pitch_deg << "deg\n";
    std::cout << "touchdown pos_ned=(" << sim.position.x << ", " << sim.position.y << ", " << sim.position.z
              << ")  miss_from_origin=" << miss_origin << "m\n";
    std::cout << "Done: ticks=" << harness.tick_count() << " phase="
              << fwcpp::hal_sitl::copter_auto::auto_phase_name(state.phase) << (ok ? " SUCCESS\n" : " FAIL\n");
    return ok ? 0 : 1;
}
