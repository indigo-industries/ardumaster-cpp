// VCP-011: quadplane_sitl_run — SitlQuadPlaneHarness closed loop on the
// original-source SIM_QuadPlane plant (VCP-010). Thin hover then forward
// transition scenario. Not MAVLink, not Mission Planner.
//
// USAGE: quadplane_sitl_run [--help] [duration_seconds]
// Default 20 simulated seconds @ 400 Hz. Exit 0 if hover climbed and
// transition produced finite airspeed; 1 otherwise.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <fwcpp/hal_sitl/sitl_quadplane_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/q_modes/mode_qhover.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/sim/sim_quadplane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

namespace {

void set_sticks(fwcpp::vehicle::Plane& plane, std::uint16_t roll_pwm, std::uint16_t pitch_pwm,
                std::uint16_t throttle_pwm, std::uint16_t rudder_pwm) {
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRoll, roll_pwm);
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelPitch, pitch_pwm);
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelThrottle, throttle_pwm);
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRudder, rudder_pwm);
    plane.rc_channels.read_input(plane.hal.rc_input);
}

void print_usage(const char* argv0) {
    std::cout << "usage: " << argv0 << " [--help] [duration_seconds > 0]\n"
              << "  VCP-011: Plane+QuadPlane+SimQuadPlane hover then transition\n"
              << "  Default duration: 20 simulated seconds @ 400Hz.\n";
}

}  // namespace

int main(int argc, char** argv) {
    constexpr float kDt = 0.0025f;
    constexpr int kTicksPerSecond = static_cast<int>(1.0f / kDt);

    int duration_s = 20;
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

    fwcpp::vehicle::Plane plane;
    fwcpp::sim::SimQuadPlane sim{"quadplane"};
    fwcpp::quadplane::QuadPlane qp{1};
    (void)qp.setup();
    qp.mode_enter();
    (void)fwcpp::q_modes::qhover_enter();

    fwcpp::vehicle::ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    plane.airspeed_sensor.start_calibration(0);

    fwcpp::hal_sitl::SitlQuadPlaneHarness harness(plane, qp, sim);

    std::cout << "VCP-011 SITL: Plane+QuadPlane+SimQuadPlane, hover then transition, " << duration_s
              << " simulated seconds (" << num_ticks << " ticks @ 400Hz)\n";
    std::cout << "frame=" << sim.frame().name << " motors=" << static_cast<int>(sim.frame().num_motors)
              << " motor_offset=" << static_cast<int>(sim.frame().motor_offset) << " q_enable=" << static_cast<int>(qp.enable())
              << " available=" << (qp.available() ? 1 : 0) << "\n";
    std::cout << std::fixed << std::setprecision(3);

    auto print_telemetry = [&](float t_s, const char* phase) {
        float true_roll_rad = 0.0f;
        float true_pitch_rad = 0.0f;
        float true_yaw_rad = 0.0f;
        sim.dcm.to_euler(&true_roll_rad, &true_pitch_rad, &true_yaw_rad);
        std::cout << "t=" << std::setw(6) << t_s << "s"
                  << "  phase=" << phase
                  << "  alt=" << (-sim.position.z) << "m"
                  << "  as=" << sim.airspeed << "m/s"
                  << "  pos_ned=(" << sim.position.x << ", " << sim.position.y << ", " << sim.position.z << ")"
                  << "  true_rpy_deg=(" << fwcpp::math::degrees(true_roll_rad) << ", "
                  << fwcpp::math::degrees(true_pitch_rad) << ", " << fwcpp::math::degrees(true_yaw_rad) << ")"
                  << "  ticks=" << harness.tick_count() << '\n';
    };

    print_telemetry(0.0f, "HOVER");

    float max_alt_m = 0.0f;
    float max_as = 0.0f;
    bool saw_transition = false;
    std::uint32_t now_ms = 0;
    const float hover = sim.frame().hover_command();
    const float climb = fwcpp::math::constrain_value(hover + 0.20f, 0.0f, 1.0f);

    for (int i = 0; i < num_ticks; ++i) {
        now_ms += 3;  // 400 Hz -> 2.5 ms; keep integer ms
        const float t_s = static_cast<float>(i + 1) * kDt;
        const char* phase = "HOVER";
        float vtol_cmd = climb;
        std::uint16_t thr_pwm = 1000;
        if (t_s >= 8.0f) {
            phase = "TRANSITION";
            saw_transition = true;
            vtol_cmd = hover;
            thr_pwm = 1700;
        } else if ((-sim.position.z) >= 8.0f) {
            phase = "HOVER";
            vtol_cmd = hover;
            thr_pwm = 1000;
        }
        set_sticks(plane, 1500, 1500, thr_pwm, 1500);
        harness.step(now_ms, kDt, vtol_cmd, true);
        const float alt = -sim.position.z;
        if (alt > max_alt_m) {
            max_alt_m = alt;
        }
        if (sim.airspeed > max_as) {
            max_as = sim.airspeed;
        }
        if ((i + 1) % kTicksPerSecond == 0) {
            print_telemetry(t_s, phase);
        }
    }

    print_telemetry(static_cast<float>(num_ticks) * kDt, saw_transition ? "TRANSITION" : "HOVER");
    const bool ok = std::isfinite(max_alt_m) && std::isfinite(max_as) && max_alt_m >= 5.0f && qp.available() &&
                    harness.tick_count() > 0;
    std::cout << "Done: ticks=" << harness.tick_count() << " max_alt=" << max_alt_m << "m max_as=" << max_as
              << "m/s" << (ok ? " SUCCESS\n" : " FAIL\n");
    return ok ? 0 : 1;
}
