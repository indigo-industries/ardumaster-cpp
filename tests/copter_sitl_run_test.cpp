// CCP-044/045/065: copter_sitl_run leftover mission on real Frame/Motor plant.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/hal_sitl/copter_sitl_run_leftover.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

using fwcpp::copter::LeftoverCopter;
using fwcpp::hal_sitl::SitlCopterHarness;
using fwcpp::hal_sitl::copter_sitl_run::LeftoverMission;
using fwcpp::hal_sitl::copter_sitl_run::MissionPhase;
using fwcpp::hal_sitl::copter_sitl_run::PortStatus;
using fwcpp::hal_sitl::copter_sitl_run::completeness_has;
using fwcpp::hal_sitl::copter_sitl_run::completeness_size;
using fwcpp::hal_sitl::copter_sitl_run::leftover_copter_sitl_step;
using fwcpp::hal_sitl::copter_sitl_run::leftover_mission_begin_takeoff;
using fwcpp::hal_sitl::copter_sitl_run::on_main_count;
using fwcpp::hal_sitl::copter_sitl_run::out_of_scope_count;
using fwcpp::hal_sitl::copter_sitl_run::remaining_count;
using fwcpp::hal_sitl::copter_sitl_run::this_slice_count;
using fwcpp::sim::SimMulticopter;

TEST_CASE("zero command stays on ground", "[copter][sitl][ccp-045]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    SitlCopterHarness harness(copter, sim);
    LeftoverMission mission{};
    leftover_copter_sitl_step(harness, mission, 0.0025f);
    REQUIRE(sim.on_ground());
    REQUIRE((-sim.position.z) == Catch::Approx(0.0f).margin(0.01f));
}

TEST_CASE("climb command via leftover PWM leaves the ground", "[copter][sitl][ccp-045]") {
    LeftoverCopter copter{};
    copter.motors_armed = true;
    SimMulticopter sim{};
    SitlCopterHarness harness(copter, sim);
    LeftoverMission mission{};
    leftover_mission_begin_takeoff(mission);
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 1200; ++i) {
        leftover_copter_sitl_step(harness, mission, kDt);
    }
    REQUIRE((-sim.position.z) > 2.0f);
    REQUIRE_FALSE(sim.on_ground());
}

TEST_CASE("hoverThrOut holds altitude on the real plant", "[copter][sitl][ccp-045]") {
    SimMulticopter sim{};
    sim.position.z = -10.0f;
    sim.velocity_ef = {};
    LeftoverCopter copter{};
    copter.motors_armed = true;
    SitlCopterHarness harness(copter, sim);
    const float hover = sim.hover_command();
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 400; ++i) {
        fwcpp::hal_sitl::copter_sitl_run::leftover_apply_collective(copter, sim, hover);
        harness.step(kDt);
    }
    REQUIRE((-sim.position.z) == Catch::Approx(10.0f).margin(1.5f));
}

TEST_CASE("leftover copter sitl mission takeoff outbound rtl land",
          "[copter][sitl][ccp-044][ccp-045][ccp-065]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    SitlCopterHarness harness(copter, sim);
    LeftoverMission mission{};
    mission.outbound_n_m = 12.0f;
    leftover_mission_begin_takeoff(mission);

    constexpr float kDt = 0.0025f;
    float max_alt_m = 0.0f;
    float max_n_m = 0.0f;
    bool saw_outbound = false;
    bool saw_rtl = false;
    const int max_ticks = 30 * 400;
    for (int i = 0; i < max_ticks; ++i) {
        leftover_copter_sitl_step(harness, mission, kDt);
        const float alt = -sim.position.z;
        if (alt > max_alt_m) {
            max_alt_m = alt;
        }
        if (sim.position.x > max_n_m) {
            max_n_m = sim.position.x;
        }
        if (mission.phase == MissionPhase::kOutbound) {
            saw_outbound = true;
        }
        if (mission.phase == MissionPhase::kRtl) {
            saw_rtl = true;
        }
        if (mission.phase == MissionPhase::kLanded) {
            break;
        }
    }

    REQUIRE(saw_outbound);
    REQUIRE(saw_rtl);
    REQUIRE(max_alt_m >= 9.0f);
    REQUIRE(max_n_m >= 8.0f);
    REQUIRE(mission.phase == MissionPhase::kLanded);
    REQUIRE(copter.land_complete);
    REQUIRE_FALSE(copter.motors_armed);
    REQUIRE(sim.on_ground());
    REQUIRE(std::hypot(sim.position.x, sim.position.y) < 5.0f);
    REQUIRE(harness.tick_count() > 0);
    REQUIRE(copter.gyro_injected);
    REQUIRE(copter.baro_injected);
}



TEST_CASE("main-style first disarmed step then takeoff climbs", "[copter][sitl][ccp-065]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    SitlCopterHarness harness(copter, sim);
    LeftoverMission mission{};
    leftover_copter_sitl_step(harness, mission, 0.0025f);
    leftover_mission_begin_takeoff(mission);
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 1200; ++i) {
        leftover_copter_sitl_step(harness, mission, kDt);
    }
    REQUIRE((-sim.position.z) > 2.0f);
}

TEST_CASE("leftover_hold_command uses AC_PosControl altitude not vz damper",
          "[copter][sitl][ccp-064]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    sim.velocity_ef = {};
    sim.position.z = -9.0f;  // 9 m AGL, target 10 m
    const float low = fwcpp::hal_sitl::copter_sitl_run::leftover_hold_command(copter, sim, 10.0f);
    LeftoverCopter copter_high{};
    SimMulticopter sim_high{};
    sim_high.velocity_ef = {};
    sim_high.position.z = -11.0f;  // 11 m AGL
    const float high = fwcpp::hal_sitl::copter_sitl_run::leftover_hold_command(copter_high, sim_high, 10.0f);
    // Same vz=0: a vz damper would emit hover both times. PosControl climbs
    // when below target and reduces throttle when above.
    REQUIRE(low > high);
    REQUIRE(low > sim.hover_command());
    REQUIRE(high < sim_high.hover_command());
}

TEST_CASE("copter_sitl_run leftover catalog remaining_count",
          "[copter][sitl][ccp-044][leftover][ccp-065]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 10);
    REQUIRE(on_main_count() == 4);
    REQUIRE(out_of_scope_count() == 4);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover_mission_advance", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_hold_command", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_poscontrol_throttle", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_poscontrol_ne", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_wpnav_rtl", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_copter_loop", PortStatus::kOnMain));
    REQUIRE(completeness_has("leftover_apply_collective", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_copter_sitl_step", PortStatus::kThisSlice));
    REQUIRE(completeness_has("copter_sitl_run takeoff/outbound/rtl/land", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SIM_Multicopter Frame/Motor mixing", PortStatus::kOnMain));
}
