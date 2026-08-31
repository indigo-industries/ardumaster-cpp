#include <cstdint>
#include <cstring>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include <fwcpp/sim/sim_rf.hpp>
#include <fwcpp/sim/sim_pack.hpp>

using namespace fwcpp::sim;

TEST_CASE("serial RF factory constructs SITL_State_common names") {
    const char* names[] = {
        "ainsteinlrd1", "ainsteinlrd1_v19", "benewake_tf02", "benewake_tf03", "benewake_tfmini",
        "blping", "dts6012m", "gyus42v2", "jre", "lanbao", "leddarone", "lightwareserial-binary",
        "lightwareserial", "maxsonarseriallv", "nmea", "nooploop_tofsense", "rds02uf",
        "teraranger_serial", "USD1_v0", "USD1_v1", "wasp"};
    for (const char* name : names) {
        auto rf = create_serial_rangefinder(name);
        REQUIRE(rf != nullptr);
        std::uint8_t buf[64]{};
        const auto n = rf->packet_for_alt(1.25f, buf, sizeof(buf));
        REQUIRE(n > 0);
    }
    REQUIRE(create_serial_rangefinder("not-a-sensor") == nullptr);
}

TEST_CASE("Benewake TF02 packet is 0x59 0x59 plus checksum") {
    RF_Benewake_TF02 rf;
    std::uint8_t buf[16]{};
    REQUIRE(rf.packet_for_alt(1.00f, buf, sizeof(buf)) == 9);
    REQUIRE(buf[0] == 0x59);
    REQUIRE(buf[1] == 0x59);
    REQUIRE(buf[2] == 100);
    REQUIRE(buf[3] == 0);
    std::uint8_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum = static_cast<std::uint8_t>(sum + buf[i]);
    }
    REQUIRE(buf[8] == sum);
}

TEST_CASE("RF NMEA DPT has checksum trailer") {
    RF_NMEA rf;
    std::uint8_t buf[64]{};
    const auto n = rf.packet_for_alt(1.81f, buf, sizeof(buf));
    const std::string s(reinterpret_cast<char*>(buf), n);
    REQUIRE(s.find("$SMDPT,") == 0);
    REQUIRE(s.find("*") != std::string::npos);
    REQUIRE(s.find("\r\n") != std::string::npos);
}

TEST_CASE("LightWare binary uses 7-bit high-bit framing") {
    RF_LightWareSerialBinary rf;
    std::uint8_t buf[4]{};
    REQUIRE(rf.packet_for_alt(1.27f, buf, sizeof(buf)) == 2);
    REQUIRE((buf[0] & 0x80) != 0);
    REQUIRE((buf[1] & 0x80) == 0);
}

TEST_CASE("Maxsonar original encoding is inches") {
    RF_MaxsonarSerialLV rf;
    std::uint8_t buf[16]{};
    const auto n = rf.packet_for_alt(0.0254f, buf, sizeof(buf));
    REQUIRE(n > 0);
    REQUIRE(buf[n - 1] == 0x0d);
    REQUIRE(buf[0] == 0x31);  // 1 inch
}

TEST_CASE("RF_MAVLink DISTANCE_SENSOR is MAVLink2 msgid 132") {
    auto rf = create_serial_rangefinder("rf_mavlink");
    REQUIRE(rf != nullptr);
    std::uint8_t buf[64]{};
    const auto n = rf->packet_for_alt(1.25f, buf, sizeof(buf));
    REQUIRE(n >= 12);
    REQUIRE(buf[0] == 0xFD);
    const std::uint32_t msgid = static_cast<std::uint32_t>(buf[7] | (buf[8] << 8) | (buf[9] << 16));
    REQUIRE(msgid == 132);
    REQUIRE(buf[1] == 39);
}

TEST_CASE("LightWare GRF stream gate and DISTANCE_DATA_CM encode") {
    REQUIRE(create_serial_rangefinder("lightware_grf") != nullptr);
    RF_LightWareGRF grf;
    std::uint8_t buf[32]{};
    REQUIRE(grf.packet_for_alt(1.25f, buf, sizeof(buf)) == 0);  // stream not enabled
    // STREAM=5 config
    std::uint8_t cfg[16]{};
    cfg[0] = 0xAA;
    std::uint16_t flags = 0x1;
    flags = static_cast<std::uint16_t>(flags | ((1 + 4) << 6));
    cfg[1] = static_cast<std::uint8_t>(flags);
    cfg[2] = static_cast<std::uint8_t>(flags >> 8);
    cfg[3] = 30;  // STREAM
    std::uint32_t stream = 5;
    std::memcpy(&cfg[4], &stream, 4);
    std::uint16_t crc = 0;
    for (int i = 0; i < 8; i++) {
        crc = crc_xmodem_update(crc, cfg[i]);
    }
    cfg[8] = static_cast<std::uint8_t>(crc);
    cfg[9] = static_cast<std::uint8_t>(crc >> 8);
    grf.write_to_device(reinterpret_cast<const char*>(cfg), 10);
    const auto n = grf.packet_for_alt(1.25f, buf, sizeof(buf));
    REQUIRE(n >= 12);
    REQUIRE(buf[0] == 0xAA);
    REQUIRE(buf[3] == 44);  // DISTANCE_DATA_CM
}
