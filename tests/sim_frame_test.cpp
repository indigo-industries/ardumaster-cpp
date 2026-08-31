// CCP-067: Frame::load_frame_params() round-trip fidelity against real
// upstream multicopter frame-config content.
//
// Real gap (verified directly against Copter-4.7.0): Frame::load_frame_params
// (libraries/SITL/SIM_Frame.cpp real line 458), triggered from Frame::init's
// own ":file.json" suffix check (real lines 580-588), is a real, working port
// in sim_frame.hpp (see Frame::load_frame_params() below, real line ~695) -
// but had ZERO test coverage of any kind before this ticket (confirmed with
// `grep -rn load_frame_params tests/*.cpp` finding nothing beyond this file).
//
// Fixtures: tests/fixtures/Callisto.json and tests/fixtures/freestyle.json
// are byte-for-byte copies of upstream's OWN Tools/autotest/models/
// Callisto.json and Tools/autotest/models/freestyle.json (Plane-4.7.0 -
// SITL's multicopter frame-config fixtures live under Plane's autotest tree
// even though they describe copters). Callisto is a real 8-motor coaxial-
// octocopter research drone (mass=32.5, num_motors=8, disc_area=1.82,
// hoverThrOut=0.36); freestyle is a real small 5-inch FPV racing quad
// (mass=0.8, num_motors=4, disc_area=0.204, hoverThrOut=0.125) - two
// genuinely different real multicopter configurations, confirmed to use the
// exact field names Frame::load_frame_params() already parses (verified
// field-for-field against real upstream's own json_search vars[] table,
// SIM_Frame.cpp real lines 489-530). skywalker_2013.json/xplane_*.json are
// NOT used here - wrong vehicle type/format for this function (skywalker is
// a Plane aerodynamic-coefficient file for SimPlane::load_coeffs(), see
// sim_plane_test.cpp's own CPP-094 fixture note; xplane_*.json are X-Plane
// external-FDM DREF configs) - matching this ticket's explicit exclusion.
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/sim/sim_frame.hpp>

using fwcpp::sim::Frame;
using fwcpp::sim::FrameModel;

namespace {
std::string fixture_path(const char* name) {
    return std::string(FWCPP_SIM_FRAME_FIXTURES_DIR) + "/" + name;
}
}  // namespace

TEST_CASE("load_frame_params() loading the real upstream Callisto.json fixture reproduces its own real field values",
          "[sim_frame][load_frame_params]") {
    // Tools/autotest/models/Callisto.json real values (read directly from
    // the pinned upstream file): mass=32.5, diagonal_size=1.325, refSpd=25.0,
    // refAngle=30.0, refVoltage=46.9, refCurrent=65.36, refAlt=26,
    // refTempC=25, refBatRes=0.024, maxVoltage=50.4, battCapacityAh=44,
    // propExpo=0.5, refRotRate=120, hoverThrOut=0.36, pwmMin=1000,
    // pwmMax=1940, spin_min=0.2, spin_max=0.975, slew_max=75,
    // disc_area=1.82, mdrag_coef=0.10, num_motors=8.
    Frame frame;
    REQUIRE(frame.load_frame_params(fixture_path("Callisto.json").c_str()));

    const FrameModel& m = frame.get_model();
    REQUIRE(m.mass == Catch::Approx(32.5f));
    REQUIRE(m.num_motors == Catch::Approx(8.0f));
    REQUIRE(m.disc_area == Catch::Approx(1.82f));
    REQUIRE(m.hoverThrOut == Catch::Approx(0.36f));

    // A wider field-for-field check against the same real file, proving the
    // whole json_search table is wired correctly, not just these 4 fields.
    REQUIRE(m.diagonal_size == Catch::Approx(1.325f));
    REQUIRE(m.refSpd == Catch::Approx(25.0f));
    REQUIRE(m.refAngle == Catch::Approx(30.0f));
    REQUIRE(m.refVoltage == Catch::Approx(46.9f));
    REQUIRE(m.refCurrent == Catch::Approx(65.36f));
    REQUIRE(m.refAlt == Catch::Approx(26.0f));
    REQUIRE(m.refTempC == Catch::Approx(25.0f));
    REQUIRE(m.refBatRes == Catch::Approx(0.024f));
    REQUIRE(m.maxVoltage == Catch::Approx(50.4f));
    REQUIRE(m.battCapacityAh == Catch::Approx(44.0f));
    REQUIRE(m.propExpo == Catch::Approx(0.5f));
    REQUIRE(m.refRotRate == Catch::Approx(120.0f));
    REQUIRE(m.pwmMin == Catch::Approx(1000.0f));
    REQUIRE(m.pwmMax == Catch::Approx(1940.0f));
    REQUIRE(m.spin_min == Catch::Approx(0.2f));
    REQUIRE(m.spin_max == Catch::Approx(0.975f));
    REQUIRE(m.slew_max == Catch::Approx(75.0f));
    REQUIRE(m.mdrag_coef == Catch::Approx(0.10f));
}

TEST_CASE("load_frame_params() loading the real upstream freestyle.json fixture reproduces its own real field values",
          "[sim_frame][load_frame_params]") {
    // Tools/autotest/models/freestyle.json real values (read directly from
    // the pinned upstream file): mass=0.8, diagonal_size=0.25, refSpd=20.0,
    // refAngle=45.0, refVoltage=23.2, refCurrent=5, refAlt=607, refTempC=25,
    // refBatRes=0.0226, maxVoltage=25.2, battCapacityAh=0, propExpo=0.7,
    // refRotRate=700, hoverThrOut=0.125, pwmMin=1000, pwmMax=2000,
    // spin_min=0.01, spin_max=0.95, slew_max=0, disc_area=0.204,
    // mdrag_coef=0.10, num_motors=4.
    Frame frame;
    REQUIRE(frame.load_frame_params(fixture_path("freestyle.json").c_str()));

    const FrameModel& m = frame.get_model();
    REQUIRE(m.mass == Catch::Approx(0.8f));
    REQUIRE(m.num_motors == Catch::Approx(4.0f));
    REQUIRE(m.disc_area == Catch::Approx(0.204f));
    REQUIRE(m.hoverThrOut == Catch::Approx(0.125f));

    // Wider field-for-field check, same discipline as the Callisto case
    // above.
    REQUIRE(m.diagonal_size == Catch::Approx(0.25f));
    REQUIRE(m.refSpd == Catch::Approx(20.0f));
    REQUIRE(m.refAngle == Catch::Approx(45.0f));
    REQUIRE(m.refVoltage == Catch::Approx(23.2f));
    REQUIRE(m.refCurrent == Catch::Approx(5.0f));
    REQUIRE(m.refAlt == Catch::Approx(607.0f));
    REQUIRE(m.refTempC == Catch::Approx(25.0f));
    REQUIRE(m.refBatRes == Catch::Approx(0.0226f));
    REQUIRE(m.maxVoltage == Catch::Approx(25.2f));
    REQUIRE(m.battCapacityAh == Catch::Approx(0.0f));
    REQUIRE(m.propExpo == Catch::Approx(0.7f));
    REQUIRE(m.refRotRate == Catch::Approx(700.0f));
    REQUIRE(m.pwmMin == Catch::Approx(1000.0f));
    REQUIRE(m.pwmMax == Catch::Approx(2000.0f));
    REQUIRE(m.spin_min == Catch::Approx(0.01f));
    REQUIRE(m.spin_max == Catch::Approx(0.95f));
    REQUIRE(m.slew_max == Catch::Approx(0.0f));
    REQUIRE(m.mdrag_coef == Catch::Approx(0.10f));
}

TEST_CASE("Callisto and freestyle load to genuinely different configurations, and both differ from the default frame",
          "[sim_frame][load_frame_params]") {
    // Guards against load_frame_params() silently falling through to the
    // default FrameModel{} on either real file - the same "prove it's not
    // silently falling through to the default" discipline VT-012/VCP-012
    // already established for tailsitter/tiltrotor work.
    const FrameModel default_model{};  // mass=3.0f, num_motors=4.0f,
                                        // disc_area=0.385f, hoverThrOut=0.39f
                                        // (sim_frame.hpp's own hardcoded
                                        // defaults, confirmed by direct read).

    Frame callisto_frame;
    REQUIRE(callisto_frame.load_frame_params(fixture_path("Callisto.json").c_str()));
    const FrameModel& callisto = callisto_frame.get_model();

    Frame freestyle_frame;
    REQUIRE(freestyle_frame.load_frame_params(fixture_path("freestyle.json").c_str()));
    const FrameModel& freestyle = freestyle_frame.get_model();

    // Callisto vs freestyle: genuinely different real vehicles.
    REQUIRE(callisto.mass != Catch::Approx(freestyle.mass));
    REQUIRE(callisto.num_motors != Catch::Approx(freestyle.num_motors));
    REQUIRE(callisto.disc_area != Catch::Approx(freestyle.disc_area));
    REQUIRE(callisto.hoverThrOut != Catch::Approx(freestyle.hoverThrOut));

    // Callisto vs this port's own default frame configuration.
    REQUIRE(callisto.mass != Catch::Approx(default_model.mass));
    REQUIRE(callisto.num_motors != Catch::Approx(default_model.num_motors));
    REQUIRE(callisto.disc_area != Catch::Approx(default_model.disc_area));
    REQUIRE(callisto.hoverThrOut != Catch::Approx(default_model.hoverThrOut));

    // freestyle vs the default frame configuration. Note freestyle's own
    // num_motors (4) coincidentally equals the default's num_motors (4) -
    // mass/disc_area/hoverThrOut are the fields that genuinely distinguish
    // it, so num_motors is deliberately not asserted not-equal here.
    REQUIRE(freestyle.mass != Catch::Approx(default_model.mass));
    REQUIRE(freestyle.disc_area != Catch::Approx(default_model.disc_area));
    REQUIRE(freestyle.hoverThrOut != Catch::Approx(default_model.hoverThrOut));
}

TEST_CASE("load_frame_params() returns false and leaves the model untouched for a nonexistent path",
          "[sim_frame][load_frame_params]") {
    Frame frame;
    const FrameModel before = frame.get_model();
    REQUIRE_FALSE(frame.load_frame_params("/does/not/exist/CCP-067-nonexistent.json"));
    const FrameModel& after = frame.get_model();
    REQUIRE(after.mass == Catch::Approx(before.mass));
    REQUIRE(after.num_motors == Catch::Approx(before.num_motors));
    REQUIRE(after.disc_area == Catch::Approx(before.disc_area));
    REQUIRE(after.hoverThrOut == Catch::Approx(before.hoverThrOut));
}
