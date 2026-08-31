// VCP-011: SitlQuadPlaneHarness sensors + Plane/QuadPlane tick + SIM_QuadPlane.
#include <cmath>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/hal_sitl/sitl_quadplane_harness.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/sim/sim_quadplane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

using fwcpp::hal_sitl::SitlQuadPlaneHarness;
using fwcpp::hal_sitl::sitl_quadplane::PortStatus;
using fwcpp::hal_sitl::sitl_quadplane::completeness_has;
using fwcpp::hal_sitl::sitl_quadplane::completeness_size;
using fwcpp::hal_sitl::sitl_quadplane::on_main_count;
using fwcpp::hal_sitl::sitl_quadplane::out_of_scope_count;
using fwcpp::hal_sitl::sitl_quadplane::remaining_count;
using fwcpp::hal_sitl::sitl_quadplane::this_slice_count;
using fwcpp::quadplane::QuadPlane;
using fwcpp::sim::SimQuadPlane;
using fwcpp::vehicle::ModeFBWA;
using fwcpp::vehicle::Plane;

namespace {
// VCP-013: same fixture-path convention CPP-094/CCP-067 established
// (FWCPP_SIM_FRAME_FIXTURES_DIR, an absolute, CMAKE_CURRENT_SOURCE_DIR-
// derived compile definition - see tests/CMakeLists.txt) reused here rather
// than duplicated, per the ticket's explicit "reuse the existing fixture"
// instruction.
std::string callisto_fixture_path() {
    return std::string(FWCPP_SIM_FRAME_FIXTURES_DIR) + "/Callisto.json";
}
}  // namespace

TEST_CASE("SitlQuadPlaneHarness step ticks Plane and QuadPlane into SimQuadPlane",
          "[quadplane][sitl][vcp-011]") {
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane"};
    SitlQuadPlaneHarness harness(plane, qp, sim);
    REQUIRE(harness.tick_count() == 0);
    const float hover = sim.frame().hover_command();
    harness.step(20, 0.0025f, hover, true);
    REQUIRE(harness.tick_count() == 1);
    REQUIRE(qp.available());
}

TEST_CASE("SitlQuadPlaneHarness climb command leaves the ground", "[quadplane][sitl][vcp-011]") {
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane"};
    SitlQuadPlaneHarness harness(plane, qp, sim);
    const float climb = sim.frame().hover_command() + 0.20f;
    constexpr float kDt = 0.0025f;
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 1600; ++i) {
        now_ms += 3;
        harness.step(now_ms, kDt, climb, true);
    }
    REQUIRE((-sim.position.z) > 2.0f);
    REQUIRE(std::isfinite(sim.airspeed));
}

TEST_CASE("SitlQuadPlaneHarness leftover catalog remaining_count", "[quadplane][sitl][vcp-011][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 7);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("SitlQuadPlaneHarness scaffold", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Plane::tick", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QuadPlane::update", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SIM_QuadPlane plant", PortStatus::kOnMain));
}

TEST_CASE("SimQuadPlane copter_tailsitter frame string sets tailsitter flag and ground behavior",
          "[quadplane][sitl][vcp-012]") {
    // Real upstream libraries/SITL/SIM_QuadPlane.cpp lines 80-84: the
    // "-copter_tailsitter" frame-string suffix selects the "+" motor layout,
    // sets `copter_tailsitter = true`, and sets
    // `ground_behavior = GROUND_BEHAVIOR_TAILSITTER`. Ported at
    // sim_quadplane.hpp lines 58-61 (copter_tailsitter_ / ground_behavior).
    // This is a pure frame-string-parser assertion, independent of flight
    // dynamics: VCP-011's own harness never exercised this suffix before.
    SimQuadPlane sim{"quadplane-copter_tailsitter"};
    REQUIRE(sim.copter_tailsitter());
    REQUIRE(sim.ground_behavior == fwcpp::sim::GroundBehavior::kTailsitter);

    // The bare "quadplane" frame (already covered by the existing tests in
    // this file) must NOT set either, so the assertions above are genuinely
    // exercising the "-copter_tailsitter" suffix branch and not some
    // always-true default.
    SimQuadPlane plain{"quadplane"};
    REQUIRE_FALSE(plain.copter_tailsitter());
    REQUIRE(plain.ground_behavior == fwcpp::sim::GroundBehavior::kNoMovement);
}

TEST_CASE("SitlQuadPlaneHarness scripted flight through copter_tailsitter frame stays numerically sane",
          "[quadplane][sitl][vcp-012]") {
    // Exercises the tailsitter rotation applied at sim_quadplane.hpp line 120
    // (`if (copter_tailsitter_) { ... }`, immediately after
    // frame_.calculate_forces(...) at line 118 - matching real upstream's own
    // structure at SIM_QuadPlane.cpp lines 133-135) for the first time via a
    // scripted flight. This does NOT need to reach a successful hover/climb -
    // that is VCP-007's own already-done Q-mode control-logic scope. Only
    // numerical sanity (finite airspeed/position/attitude, no NaN, no crash)
    // is asserted here.
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane-copter_tailsitter"};
    REQUIRE(sim.copter_tailsitter());
    SitlQuadPlaneHarness harness(plane, qp, sim);
    const float hover = sim.frame().hover_command();
    constexpr float kDt = 0.0025f;
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 500; ++i) {
        now_ms += 3;
        harness.step(now_ms, kDt, hover, true);

        REQUIRE(std::isfinite(sim.airspeed));
        REQUIRE(std::isfinite(sim.position.x));
        REQUIRE(std::isfinite(sim.position.y));
        REQUIRE(std::isfinite(sim.position.z));

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        sim.dcm.to_euler(&roll, &pitch, &yaw);
        REQUIRE(std::isfinite(roll));
        REQUIRE(std::isfinite(pitch));
        REQUIRE(std::isfinite(yaw));
    }
    REQUIRE(harness.tick_count() == 500);
}

TEST_CASE("SimQuadPlane -tilttrivec frame selects a genuinely different motor layout",
          "[quadplane][sitl][vcp-012]") {
    // Real upstream libraries/SITL/SIM_QuadPlane.cpp lines 54-56: the
    // "-tilttrivec" suffix selects the distinct "tilttrivec" frame_type (a
    // 3-motor tilt-tri layout, real SIM_Frame.cpp frame table entry
    // {"tilttrivec", 3, tilttri_vectored_motors}) instead of the default
    // 4-motor "x" quad layout used by the bare "quadplane" frame string. The
    // rotation block at sim_quadplane.hpp line 120 checks ONLY the
    // tailsitter flag, so tilt-rotor variants carry no rotation - their
    // distinguishing behavior lives entirely in the frame's own motor
    // geometry, which is what this test checks directly (real motor count
    // and layout, not a silent fallthrough to the default frame).
    SimQuadPlane tilttrivec{"quadplane-tilttrivec"};
    SimQuadPlane plain{"quadplane"};

    REQUIRE_FALSE(tilttrivec.copter_tailsitter());
    REQUIRE(tilttrivec.frame().num_motors == 3);
    REQUIRE(plain.frame().num_motors == 4);
    REQUIRE(tilttrivec.frame().num_motors != plain.frame().num_motors);
}

TEST_CASE("SitlQuadPlaneHarness scripted flight through -tilttrivec frame stays numerically sane",
          "[quadplane][sitl][vcp-012]") {
    // Scripted-flight numerical-sanity coverage for the tilt-rotor branch,
    // matching the tailsitter scripted-flight test above. No hover/climb bar
    // is required (VCP-007 scope); only finite, non-NaN telemetry throughout.
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane-tilttrivec"};
    REQUIRE(sim.frame().num_motors == 3);
    SitlQuadPlaneHarness harness(plane, qp, sim);
    const float hover = sim.frame().hover_command();
    constexpr float kDt = 0.0025f;
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 500; ++i) {
        now_ms += 3;
        harness.step(now_ms, kDt, hover, true);

        REQUIRE(std::isfinite(sim.airspeed));
        REQUIRE(std::isfinite(sim.position.x));
        REQUIRE(std::isfinite(sim.position.y));
        REQUIRE(std::isfinite(sim.position.z));

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        sim.dcm.to_euler(&roll, &pitch, &yaw);
        REQUIRE(std::isfinite(roll));
        REQUIRE(std::isfinite(pitch));
        REQUIRE(std::isfinite(yaw));
    }
    REQUIRE(harness.tick_count() == 500);
}

TEST_CASE("SimQuadPlane composes Frame::init's real JSON-suffix mechanism end to end with the real Callisto.json fixture",
          "[quadplane][sitl][vcp-013]") {
    // VCP-013: proves the composition VCP-011/CCP-067 never tested together
    // - SimQuadPlane's own frame-type detection (sim_quadplane.hpp lines
    // 21-62, strstr() suffix matching against the whole frame_str) does not
    // corrupt a ":<path>.json" suffix before it reaches
    // frame_.init(frame_str) (sim_quadplane.hpp line 78), which is the same
    // already-tested (CCP-067) Frame::init()/load_frame_params()
    // ":file.json" mechanism (real upstream SIM_Frame.cpp lines 458,
    // 580-588).
    //
    // frame_str = "quadplane-octa-quad:<abs path>/Callisto.json": real
    // upstream's own naming convention for this exact vehicle
    // (Tools/autotest/arducopter.py real line 9422:
    // model="octa-quad:@ROMFS/models/Callisto.json"), with the "quadplane"
    // prefix QuadPlane::QuadPlane()'s own frame_type parser expects added
    // in front. Checked by hand against every strstr() suffix branch in
    // sim_quadplane.hpp (octa-quad-cor, octa-quad-cw-cor, octa/octaquad,
    // hexax, hexa, plus, y6, tri, tilttrivec, tilthvec, tilttri, firefly,
    // tilt, cl84, copter_tailsitter): none of those substrings appear
    // anywhere in this frame_str or in the fixture's own absolute path
    // (".../tests/fixtures/Callisto.json"), so "-octa-quad" is the first
    // and only branch that fires.
    //
    // Finding: this composition ALREADY worked correctly, no fix was
    // needed. frame_.init() (sim_frame.hpp line 522) is unmodified,
    // already-correct, already-tested (CCP-067) code; SimQuadPlane's
    // constructor only ever reads frame_str through read-only strstr()
    // substring checks before passing the same, untouched pointer straight
    // through to frame_.init(frame_str) at line 78 - no truncation, no
    // copy, no reformatting of the ":file.json" suffix anywhere along the
    // way.
    const std::string frame_str = std::string("quadplane-octa-quad:") + callisto_fixture_path();
    SimQuadPlane sim{frame_str.c_str()};

    // Structural motor count: real upstream flies Callisto as an
    // "octa-quad" (8 motor) frame (arducopter.py line 9422 above). This
    // comes from the "-octa-quad" frame_str suffix selecting the
    // "octa-quad" FrameTemplate (sim_frame.hpp line 416, 8 motors), not
    // from the JSON file itself.
    REQUIRE(sim.frame().num_motors == 8);

    // The real Callisto.json fixture's own field values, loaded through
    // this composed constructor path instead of load_frame_params() called
    // directly - same 4 headline fields CCP-067's own round-trip test
    // checks (sim_frame_test.cpp lines 56-59).
    const auto& model = sim.frame().get_model();
    REQUIRE(model.mass == Catch::Approx(32.5f));
    REQUIRE(model.num_motors == Catch::Approx(8.0f));
    REQUIRE(model.disc_area == Catch::Approx(1.82f));
    REQUIRE(model.hoverThrOut == Catch::Approx(0.36f));

    // Real upstream's "increase mass for plane components" step
    // (SIM_QuadPlane.cpp real line 107: mass = frame->get_mass() * 1.5,
    // unconditional, applied AFTER frame->init() so any -heavy/-jet effect
    // on frame mass is discarded regardless - see this commit's own message
    // for why that forwarding is deliberately not added here) is
    // implemented in this port via Frame::set_mass_scale(1.5f)
    // (sim_quadplane.hpp line 75, called BEFORE frame_.init()) baked into
    // Frame::init()'s own mass_ calculation (sim_frame.hpp line 531:
    // mass_ = model.mass * mass_scale) rather than as a separate
    // multiply-after-init step. A structural difference in WHEN the scale
    // is applied, but it produces the identical final value - proven here
    // end to end for the first time.
    REQUIRE(sim.frame().get_mass() == Catch::Approx(32.5f * 1.5f));
    REQUIRE(sim.mass == Catch::Approx(32.5f * 1.5f));
}
