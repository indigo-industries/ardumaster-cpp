// CCP-065: standalone copter_sitl_run — arm, takeoff ~10 m AGL, fly 1 mile
// North, RTL to origin NED 0,0, land on the original spot. Vehicle tick is
// leftover_copter_loop. Altitude is AC_PosControl D; horizontal is AC_PosControl
// NE + WPNav (not leftover altitude-only collective). Motors still through
// MotorsMatrix/AP_Motors via SitlCopterHarness differential PWM.
//
// USAGE: copter_sitl_run [--help] [duration_seconds]
// Default duration is 900 simulated seconds @ 400Hz (enough for 1 mile out
// and back at WPNav 10 m/s plus takeoff/land). Exits 0 if LANDED after
// climbing, 1 otherwise. Stops 1 s after LANDED.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/hal_sitl/copter_sitl_run_leftover.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

namespace {

void print_usage(const char* argv0) {
    std::cout << "usage: " << argv0 << " [--help] [duration_seconds > 0]\n"
              << "  CCP-065: LeftoverCopter + AC_PosControl NE/D + WPNav RTL + SimMulticopter\n"
              << "  Mission: takeoff 10m AGL, fly 1 mile North (NED +X, heading 0 deg),\n"
              << "           RTL to origin NED 0,0, land original spot.\n"
              << "  Default duration: 900 simulated seconds @ 400Hz (until LANDED).\n";
}

}  // namespace

int main(int argc, char** argv) {
    constexpr float kDt = 0.0025f;
    constexpr int kTicksPerSecond = static_cast<int>(1.0f / kDt);

    int duration_s = 900;
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
    fwcpp::hal_sitl::copter_sitl_run::LeftoverMission mission{};

    std::cout << "CCP-065 SITL: PosControl NE/D + WPNav 1 mile North then RTL/land, "
              << duration_s << " simulated seconds (" << num_ticks << " ticks @ 400Hz)\n";
    std::cout << "mission: takeoff " << mission.takeoff_alt_m << "m AGL, outbound N=" << mission.outbound_n_m
              << "m E=" << mission.outbound_e_m << "m heading="
              << fwcpp::hal_sitl::copter_sitl_run::kOutboundHeadingDeg << " deg (North), WPNav "
              << fwcpp::hal_sitl::copter_sitl_run::kWpSpeedMs << " m/s, RTL origin, land  frame="
              << sim.frame().name << " motors=" << static_cast<int>(sim.num_motors()) << "\n";
    std::cout << std::fixed << std::setprecision(3);

    auto dist_origin = [&]() {
        return std::hypot(sim.position.x, sim.position.y);
    };

    auto print_telemetry = [&](float t_s, const char* why) {
        float true_roll_rad = 0.0f;
        float true_pitch_rad = 0.0f;
        float true_yaw_rad = 0.0f;
        sim.dcm.to_euler(&true_roll_rad, &true_pitch_rad, &true_yaw_rad);

        std::cout << "t=" << std::setw(8) << t_s << "s"
                  << "  phase=" << fwcpp::hal_sitl::copter_sitl_run::mission_phase_name(mission.phase)
                  << "  " << why
                  << "  N=" << sim.position.x << "  E=" << sim.position.y << "  D=" << sim.position.z
                  << "  dist_origin=" << dist_origin() << "m"
                  << "  alt=" << (-sim.position.z) << "m"
                  << "  armed=" << (copter.motors_armed ? 1 : 0)
                  << "  land_complete=" << (copter.land_complete ? 1 : 0)
                  << "  cmd=" << mission.command
                  << "  pos_ned=(" << sim.position.x << ", " << sim.position.y << ", " << sim.position.z << ")"
                  << "  horiz_spd=" << std::hypot(sim.velocity_ef.x, sim.velocity_ef.y) << "m/s"
                  << "  true_rpy_deg=(" << fwcpp::math::degrees(true_roll_rad) << ", "
                  << fwcpp::math::degrees(true_pitch_rad) << ", " << fwcpp::math::degrees(true_yaw_rad) << ")"
                  << "  ticks=" << harness.tick_count() << '\n';
    };

    print_telemetry(0.0f, "START");

    leftover_copter_sitl_step(harness, mission, kDt);
    fwcpp::hal_sitl::copter_sitl_run::leftover_mission_begin_takeoff(mission);
    print_telemetry(static_cast<float>(harness.tick_count()) * kDt, "TAKEOFF");

    float max_alt_m = 0.0f;
    float max_n_m = 0.0f;
    float max_horiz_spd = 0.0f;
    int landed_tick = -1;
    float t_takeoff = static_cast<float>(harness.tick_count()) * kDt;
    float t_outbound = 0.0f, t_rtl = 0.0f, t_land = 0.0f, t_landed = 0.0f;
    auto last_phase = mission.phase;

    for (int i = 1; i < num_ticks; ++i) {
        leftover_copter_sitl_step(harness, mission, kDt);
        const float alt = -sim.position.z;
        if (alt > max_alt_m) {
            max_alt_m = alt;
        }
        if (sim.position.x > max_n_m) {
            max_n_m = sim.position.x;
        }
        const float hspd = std::hypot(sim.velocity_ef.x, sim.velocity_ef.y);
        if (hspd > max_horiz_spd) {
            max_horiz_spd = hspd;
        }
        const float t_s = static_cast<float>(i + 1) * kDt;
        if (mission.phase != last_phase) {
            const char* tag = fwcpp::hal_sitl::copter_sitl_run::mission_phase_name(mission.phase);
            print_telemetry(t_s, tag);
            if (mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kOutbound) {
                t_outbound = t_s;
            } else if (mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kRtl) {
                t_rtl = t_s;
            } else if (mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kLand) {
                t_land = t_s;
            } else if (mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kLanded) {
                t_landed = t_s;
            } else if (mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kTakeoff) {
                t_takeoff = t_s;
            }
            last_phase = mission.phase;
        }
        if (landed_tick < 0 && mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kLanded) {
            landed_tick = i;
        }
        if ((i + 1) % kTicksPerSecond == 0) {
            print_telemetry(t_s, "1Hz");
        }
        if (landed_tick >= 0 && (i - landed_tick) >= kTicksPerSecond) {
            break;
        }
    }

    print_telemetry(static_cast<float>(harness.tick_count()) * kDt, "DONE");
    const float miss = std::hypot(sim.position.x, sim.position.y);
    const bool ok = mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kLanded &&
                    max_alt_m >= (mission.takeoff_alt_m * 0.9f) && copter.land_complete && sim.on_ground();
    std::cout << "land_start pos_ned=(" << mission.land_start_n_m << ", " << mission.land_start_e_m << ", "
              << mission.land_start_d_m << ")\n";
    std::cout << "touchdown pos_ned=(" << sim.position.x << ", " << sim.position.y << ", " << sim.position.z << ")\n";
    std::cout << "miss hypot(dN,dE)=" << miss << "m  max_n=" << max_n_m << "m  max_alt=" << max_alt_m
              << "m  max_horiz_spd=" << max_horiz_spd << "m/s\n";
    std::cout << "times_s takeoff=" << t_takeoff << " outbound=" << t_outbound << " rtl=" << t_rtl
              << " land=" << t_land << " landed=" << t_landed << "\n";
    std::cout << "Done: ticks=" << harness.tick_count() << " max_alt=" << max_alt_m
              << "m phase=" << fwcpp::hal_sitl::copter_sitl_run::mission_phase_name(mission.phase)
              << (ok ? " SUCCESS\n" : " FAIL\n");
    return ok ? 0 : 1;
}
