// CCP-068: standalone test for the general-purpose Copter mission runner
// (copter_auto_mission.hpp) — flies a genuine 4-item, variable-altitude
// mission (arm+takeoff, two waypoints at DIFFERENT altitudes, RTL, land),
// something the old fixed 2-point CCP-065 demo (copter_main.cpp) cannot
// express at all. copter_main.cpp / copter_sitl_run_leftover.hpp are
// untouched by this file.
//
// USAGE: copter_auto_mission_run [--help] [duration_seconds]

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
#include <fwcpp/sim/sim_multicopter.hpp>

namespace {

void print_usage(const char* argv0) {
    std::cout << "usage: " << argv0 << " [--help] [duration_seconds > 0]\n"
              << "  CCP-068: general-purpose Copter Mission runner test.\n"
              << "  Mission: takeoff 10m AGL, WP1 (300N, 0E, 10m alt), WP2 (300N, 200E, 25m alt),\n"
              << "           RTL (returns at 25m, the last leg's altitude), land.\n"
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
    const MissionItem items[] = {
        MissionItem{.command = MissionCommand::Takeoff, .down_m = -10.0f},
        MissionItem{.command = MissionCommand::Waypoint, .north_m = 300.0f, .east_m = 0.0f, .down_m = -10.0f},
        MissionItem{.command = MissionCommand::Waypoint, .north_m = 300.0f, .east_m = 200.0f, .down_m = -25.0f},
        MissionItem{.command = MissionCommand::Rtl},
        MissionItem{.command = MissionCommand::Land},
    };
    if (!mission.load(items)) {
        std::cerr << "mission.load failed\n";
        return 1;
    }

    std::cout << "CCP-068 SITL: general Mission runner, " << mission.size() << " items, " << duration_s
              << " simulated seconds (" << num_ticks << " ticks @ 400Hz)\n";
    std::cout << std::fixed << std::setprecision(3);

    auto print_telemetry = [&](float t_s, const char* why) {
        std::cout << "t=" << std::setw(8) << t_s << "s"
                  << "  phase=" << fwcpp::hal_sitl::copter_auto::auto_phase_name(state.phase) << "  " << why
                  << "  N=" << sim.position.x << "  E=" << sim.position.y << "  D=" << sim.position.z
                  << "  alt=" << (-sim.position.z) << "m"
                  << "  armed=" << (copter.motors_armed ? 1 : 0)
                  << "  land_complete=" << (copter.land_complete ? 1 : 0)
                  << "  horiz_spd=" << std::hypot(sim.velocity_ef.x, sim.velocity_ef.y) << "m/s"
                  << "  ticks=" << harness.tick_count() << '\n';
    };

    print_telemetry(0.0f, "START");
    if (!fwcpp::hal_sitl::copter_auto::auto_mission_begin(state, mission)) {
        std::cerr << "auto_mission_begin failed — first item must be Takeoff\n";
        return 1;
    }

    float max_alt_m = 0.0f;
    int landed_tick = -1;
    auto last_phase = state.phase;
    bool visited_wp2_altitude_ok = false;

    for (int i = 0; i < num_ticks; ++i) {
        fwcpp::hal_sitl::copter_auto::auto_mission_sitl_step(harness, state, mission, kDt);
        const float alt = -sim.position.z;
        if (alt > max_alt_m) {
            max_alt_m = alt;
        }
        // WP2 climbs to 25m — confirm the runner actually flies a DIFFERENT
        // altitude leg to leg (something the fixed CCP-065 demo never does).
        if (sim.position.x > 250.0f && sim.position.y > 150.0f && alt > 23.0f) {
            visited_wp2_altitude_ok = true;
        }
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

    print_telemetry(static_cast<float>(harness.tick_count()) * kDt, "DONE");
    const float miss = std::hypot(sim.position.x, sim.position.y);
    const bool ok = state.phase == fwcpp::hal_sitl::copter_auto::AutoPhase::kLanded && max_alt_m >= 22.0f &&
                    visited_wp2_altitude_ok && copter.land_complete && sim.on_ground() && miss < 10.0f;
    std::cout << "touchdown pos_ned=(" << sim.position.x << ", " << sim.position.y << ", " << sim.position.z
              << ")  miss_from_origin=" << miss << "m  max_alt=" << max_alt_m
              << "m  visited_wp2_altitude=" << (visited_wp2_altitude_ok ? 1 : 0) << "\n";
    std::cout << "Done: ticks=" << harness.tick_count() << " phase="
              << fwcpp::hal_sitl::copter_auto::auto_phase_name(state.phase) << (ok ? " SUCCESS\n" : " FAIL\n");
    return ok ? 0 : 1;
}
