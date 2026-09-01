#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/sim/sim_adsb.hpp>
#include <fwcpp/sim/sim_ais.hpp>
#include <fwcpp/sim/sim_precland.hpp>
#include <fwcpp/sim/sim_vicon.hpp>

using namespace fwcpp::sim;
using fwcpp::Location;
using Catch::Matchers::WithinAbs;

TEST_CASE("Sagetech packers match MXS scaled encodings") {
    std::uint8_t buf[3]{};
    pack_scaled_geocoord(buf, 0.0f);
    REQUIRE(buf[0] == 0);
    REQUIRE(buf[1] == 0);
    REQUIRE(buf[2] == 0);
    pack_int32_into_uint8_ts(0x01020304, buf);
    REQUIRE(buf[0] == 0x02);
    REQUIRE(buf[1] == 0x03);
    REQUIRE(buf[2] == 0x04);
    REQUIRE(scaled_groundspeed(0.0f) == 0x01);
    std::uint8_t as[2]{};
    pack_scaled_airspeed(as, 1.0f);
    const std::int16_t scaled = static_cast<std::int16_t>(1.94384449f * 1.0f * 8);
    REQUIRE(as[0] == static_cast<std::uint8_t>(scaled >> 8));
    REQUIRE(as[1] == static_cast<std::uint8_t>(scaled >> 0));
}

TEST_CASE("ADSB vehicles spawn and translate") {
    std::srand(1);
    Adsb adsb;
    adsb.parms.plane_count = 3;
    adsb.parms.radius_m = 500;
    adsb.parms.altitude_m = 100;
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location ac = origin;
    adsb.update_simulated_vehicles(origin, ac, 1);
    REQUIRE(adsb.num_vehicles == 3);
    bool any = false;
    for (int k = 0; k < 40 && !any; k++) {
        adsb.update_simulated_vehicles(origin, ac, static_cast<std::uint32_t>(1000 * (k + 2)));
        for (std::uint8_t i = 0; i < adsb.num_vehicles; i++) {
            if (adsb.vehicles[i].initialised) {
                any = true;
                REQUIRE(adsb.vehicles[i].position.z < 0);
            }
        }
    }
    REQUIRE(any);
    AdsbSagetechMxs mxs;
    for (std::uint8_t i = 0; i < adsb.num_vehicles; i++) {
        if (adsb.vehicles[i].initialised) {
            mxs.send_vehicle_message_state_vector(adsb.vehicles[i]);
        }
    }
    auto bytes = mxs.drain_to_autopilot();
    REQUIRE(bytes.size() > 4);
    REQUIRE(bytes[0] == 0xAA);
    REQUIRE(bytes[1] == 0x91);
}

TEST_CASE("AIS position report is AIVDM with checksum") {
    AIS ais;
    AisVesselInfo info{};
    info.MMSI = 123456789;
    info.flags = AIS_FLAGS_VALID_VELOCITY | AIS_FLAGS_VALID_TURN_RATE;
    info.lat = 515000000;
    info.lon = -39000000;
    info.velocity = 100;
    info.heading = 9000;
    ais.send_position_report(info);
    const std::string s = ais.last_nmea();
    REQUIRE(s.find("!AIVDM,") == 0);
    REQUIRE(s.find("*") != std::string::npos);
    auto drained = ais.drain_to_autopilot();
    REQUIRE(drained.size() == s.size());
}

TEST_CASE("AIS replay emits file lines at 1 Hz") {
    // This fixture is committed in THIS repo, so it must always resolve.
    const std::string ais_path =
        std::string(FWCPP_SOURCE_DIR) + "/modules/ap-sim/include/fwcpp/sim/SIM_AIS_data.txt";
    AIS_Replay replay(ais_path.c_str());
    REQUIRE(replay.has_file());
    replay.update(0);
    replay.update(1000);
    auto bytes = replay.drain_to_autopilot();
    REQUIRE(!bytes.empty());
    REQUIRE(bytes[0] == static_cast<std::uint8_t>(char(0x21)));  // !
}

TEST_CASE("Precland cylinder is healthy inside beam") {
    SIM_Precland pld;
    pld._enable = 1;
    pld._device_lat = -35.0f;
    pld._device_lon = 149.0f;
    pld._device_height = 0;
    pld._alt_limit = 15;
    pld._dist_limit = 10;
    pld._rate = 100;
    pld._type = SIM_Precland::PRECLAND_TYPE_CYLINDER;
    pld._orient = static_cast<std::int8_t>(fwcpp::math::Rotation::PITCH_90);
    Location beacon(static_cast<std::int32_t>(-35.0f * 1.0e7f), static_cast<std::int32_t>(149.0f * 1.0e7f), 500,
                    Location::AltFrame::ABOVE_ORIGIN);
    pld.update(beacon, 20);
    REQUIRE(pld.healthy());
    Location far = beacon;
    far.offset(50, 0);
    pld.update(far, 40);
    REQUIRE_FALSE(pld.healthy());
}

TEST_CASE("Vicon observation applies glitch and fail") {
    std::srand(2);
    Vicon v;
    v.parms.glitch = fwcpp::math::Vector3f(1, 0, 0);
    v.parms.rate_hz = 50;
    fwcpp::math::Quaternion att;
    att.from_euler(0.f, 0.f, 0.f);
    v.update(Location{}, fwcpp::math::Vector3d(0, 0, 0), fwcpp::math::Vector3f(0, 0, 0), att, 20000);
    REQUIRE(v.last_obs.valid);
    REQUIRE_THAT(static_cast<float>(v.last_obs.pos.x), WithinAbs(1.0f, 1e-3f));
    v.parms.fail = 1;
    v.update(Location{}, fwcpp::math::Vector3d(0, 0, 0), fwcpp::math::Vector3f(0, 0, 0), att, 40000);
    REQUIRE_FALSE(v.last_obs.valid);
}
