#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/motors/motors_heli.hpp>
#include <fwcpp/sim/sim_blimp.hpp>
#include <fwcpp/sim/sim_calibration.hpp>
#include <fwcpp/sim/sim_engine_periph.hpp>
#include <fwcpp/sim/sim_helicopter.hpp>
#include <fwcpp/sim/sim_i2c.hpp>
#include <fwcpp/sim/sim_i2c_devices.hpp>
#include <fwcpp/sim/sim_i2c_leds.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_longtail.hpp>
#include <fwcpp/sim/sim_ship.hpp>
#include <fwcpp/sim/sim_spi.hpp>
#include <fwcpp/sim/sim_stratoblimp.hpp>

using namespace fwcpp::sim;
using fwcpp::Location;
using Location = fwcpp::Location;
using Catch::Matchers::WithinAbs;

TEST_CASE("Blimp fin thrust produces finite accel") {
    Blimp b;
    SitlInput in{};
    in.servos[0] = 1700;
    in.servos[1] = 1300;
    in.servos[2] = 1600;
    in.servos[3] = 1400;
    for (int i = 0; i < 50; i++) {
        b.update(in, 0.01f);
    }
    REQUIRE(std::isfinite(b.position.x));
    REQUIRE(std::isfinite(b.gyro.z));
}

TEST_CASE("Calibration stop mode damps gyro") {
    Calibration c;
    c.gyro = fwcpp::math::Vector3f(1, 0, 0);
    SitlInput in{};
    in.servos[4] = 1000;
    for (int i = 0; i < 800; i++) {
        c.update(in, 0.01f);
    }
    REQUIRE(std::fabs(c.gyro.x) < 0.5f);
}

TEST_CASE("Calibration angular velocity slews gyro toward axis") {
    Calibration c;
    SitlInput in{};
    in.servos[4] = 2000;
    in.servos[5] = 2000;
    in.servos[6] = 1500;
    in.servos[7] = 1500;
    for (int i = 0; i < 50; i++) {
        c.update(in, 0.01f);
    }
    REQUIRE(std::fabs(c.gyro.x) > 0.01f);
}

TEST_CASE("Ship circular path advances heading") {
    ShipSim sim;
    sim.enable = 1;
    fwcpp::Location home;
    home.lat = -353632621;
    home.lng = 1491652374;
    sim.update(1.0f, 1000000, home);
    REQUIRE(sim.initialised);
    REQUIRE(sim.ship.heading_deg > 0);
    fwcpp::Location loc;
    REQUIRE(sim.get_location(loc));
}

TEST_CASE("StratoBlimp release produces lift") {
    StratoBlimp s;
    SitlInput in{};
    in.servos[4] = 2000;
    in.servos[2] = 1500;
    in.servos[3] = 1500;
    for (int i = 0; i < 20; i++) {
        s.update(in, 0.05f);
    }
    REQUIRE(s.released);
    REQUIRE(std::isfinite(s.accel_body.z));
}

TEST_CASE("MS5611 reset then PROM read") {
    MS5611 ms;
    ms.state = MS5XXX::State::UNINITIALISED;
    std::uint8_t cmd = 0x1E;
    I2cMsg w{2, 0x77, 0, &cmd, 1};
    I2cRdwr wr{&w, 1};
    REQUIRE(ms.rdwr(wr) == 0);
    REQUIRE(ms.state == MS5XXX::State::RESET_START);
    ms.set_now_us(3000);
    Aircraft ac;
    ms.update(ac);
    ms.set_now_us(6000);
    ms.update(ac);
    REQUIRE(ms.prom_loaded);
    std::uint8_t c1 = 0xa2;
    std::uint8_t out[2]{};
    I2cMsg msgs[2]{{2, 0x77, 0, &c1, 1}, {2, 0x77, I2C_M_RD, out, 2}};
    I2cRdwr rd{msgs, 2};
    ms.state = MS5XXX::State::RUNNING;
    REQUIRE(ms.rdwr(rd) == 0);
    const std::uint16_t c = static_cast<std::uint16_t>((out[0] << 8) | out[1]);
    REQUIRE(c == 40127);
}

TEST_CASE("INA3221 channel 2 tracks battery voltage") {
    INA3221 ina;
    Aircraft ac;
    ac.battery_voltage = 12.6f;
    ac.battery_current = 5.0f;
    ac.time_now_us = 200000;
    ina.update(ac);
    REQUIRE(ina.registers.byname.Channel_2_Bus_Voltage != 0);
}

TEST_CASE("SMBus generic cell voltages follow pack") {
    SIM_BattMonitor_SMBus_Generic bat;
    bat.init();
    Aircraft ac;
    ac.battery_voltage = 12.6f;
    ac.time_now_us = 200000;
    bat.update(ac);
    REQUIRE(bat.get_reg_value(SMBusBattDevReg::VOLTAGE) == static_cast<std::uint16_t>(12600));
}

TEST_CASE("Maxell manufacturer block is Hitachi maxell") {
    Maxell m;
    m.init();
    REQUIRE(m.values[SMBusBattDevReg::MANUFACTURE_NAME] == "Hitachi maxell");
}

TEST_CASE("TeraRanger I2C command then CRC reading") {
    TeraRangerI2C tr;
    tr.rangefinder_range = 2.5f;
    tr.set_now_us(1);
    std::uint8_t cmd = 0;
    I2cMsg w{0, 0x31, 0, &cmd, 1};
    I2cRdwr wr{&w, 1};
    REQUIRE(tr.rdwr(wr) == 0);
    tr.set_now_us(1000);
    std::uint8_t buf[3]{};
    I2cMsg r{0, 0x31, I2C_M_RD, buf, 3};
    I2cRdwr rd{&r, 1};
    REQUIRE(tr.rdwr(rd) == 0);
    const std::uint16_t mm = static_cast<std::uint16_t>((buf[0] << 8) | buf[1]);
    REQUIRE(mm == 2500);
}

TEST_CASE("Airspeed DLVR packed pressure is finite") {
    Airspeed_DLVR d;
    d.pressure = 100;
    d.temperature = 25;
    std::uint8_t buf[4]{};
    I2cMsg r{2, 0x28, I2C_M_RD, buf, 4};
    I2cRdwr rd{&r, 1};
    REQUIRE(d.rdwr(rd) == 0);
}

TEST_CASE("TFS20L distance registers") {
    TFS20L t;
    t.rangefinder_range = 1.5f;
    t.strength = 1000;
    std::uint8_t reg = 0;
    I2cMsg w{0, 0x10, 0, &reg, 1};
    I2cRdwr wr{&w, 1};
    REQUIRE(t.rdwr(wr) == 0);
    std::uint8_t buf[2]{};
    I2cMsg msgs[2]{{0, 0x10, 0, &reg, 1}, {0, 0x10, I2C_M_RD, buf, 2}};
    I2cRdwr rd{msgs, 2};
    REQUIRE(t.rdwr(rd) == 0);
    const std::uint16_t cm = static_cast<std::uint16_t>(buf[0] | (buf[1] << 8));
    REQUIRE(cm == 150);
}

TEST_CASE("ICEngine starter path produces idle thrust") {
    ICEngine e;
    e.starter_servo = 3;
    e.ignition_servo = 4;
    SitlInput in{};
    in.servos[2] = 1100;
    in.servos[3] = 1800;
    in.servos[4] = 1800;
    const float p = e.update(in, 1000);
    REQUIRE(p >= 0);
}

TEST_CASE("Gripper EPM field strengthens on grip demand") {
    Gripper_EPM g;
    g.gripper_emp_servo_pin = 1;
    SitlInput in{};
    in.servos[0] = 2000;
    g.last_update_us = 0;
    g.update(in, 1000000);
    REQUIRE(g.field_strength > 0);
}

TEST_CASE("Generator engine RPM slews toward desired") {
    SIM_GeneratorEngine ge;
    ge.desired_rpm = 5000;
    ge.last_rpm_update_ms = 0;
    ge.last_heat_update_ms = 0;
    ge.update(500);
    REQUIRE(ge.current_rpm > 0);
}

TEST_CASE("TeraRangerTower emits TH header") {
    PS_TeraRangerTower tw;
    fwcpp::Location loc;
    tw.update(loc, 1);
    tw.update(loc, 300);
    auto bytes = tw.drain_to_autopilot();
    REQUIRE(bytes.size() >= 2);
    REQUIRE(bytes[0] == 'T');
    REQUIRE(bytes[1] == 'H');
}

TEST_CASE("CRSF cycles original VTX frames") {
    CRSF crsf;
    crsf.update(400);
    auto bytes = crsf.drain_to_autopilot();
    REQUIRE_FALSE(bytes.empty());
    REQUIRE(bytes[0] == 0xC8);
}

TEST_CASE("RAMTRON write-enable then write/read") {
    RAMTRON_FM25V02 ram;
    std::uint8_t wren[1]{0x06};
    SpiIocTransfer t0{wren, nullptr, 1};
    REQUIRE(ram.rdwr(1, &t0) == 0);
    std::uint8_t wr[3]{0x02, 0x00, 0x10};
    SpiIocTransfer t1{wr, nullptr, 3};
    REQUIRE(ram.rdwr(1, &t1) == 0);
    std::uint8_t payload[1]{0xAB};
    SpiIocTransfer t1b{payload, nullptr, 1};
    REQUIRE(ram.rdwr(1, &t1b) == 0);
    std::uint8_t cmd[3]{0x03, 0x00, 0x10};
    SpiIocTransfer t2{cmd, nullptr, 3};
    REQUIRE(ram.rdwr(1, &t2) == 0);
    std::uint8_t rx[1]{};
    SpiIocTransfer t3{nullptr, rx, 1};
    REQUIRE(ram.rdwr(1, &t3) == 0);
    REQUIRE(rx[0] == 0xAB);
}

TEST_CASE("Heli H3_120 swash mixing is not raw PWM identity") {
    fwcpp::motors::MotorsHeliSwash sw;
    sw.configure();
    sw.calculate(0.5f, -0.2f, 0.6f);
    fwcpp::sim::SitlInput in{};
    sw.write_servos(in.servos, 1600);
    REQUIRE(in.servos[0] != in.servos[1]);
    REQUIRE(in.servos[7] == 1600);
    SimHelicopter heli("heli");
    for (int i = 0; i < 30; i++) {
        heli.update(in, 0.01f);
    }
    REQUIRE(std::isfinite(heli.gyro.x));
}

TEST_CASE("Populate default I2C bus matches SIM_I2C.cpp addresses") {
    I2C bus;
    TeraRangerI2C teraranger;
    LightWareI2C_Legacy16Bit lw16;
    MaxSonarI2CXL maxsonar;
    MCP9600 mcp;
    ICM40609 icm;
    SHT3x sht;
    AS5600 as5600;
    MS5525 ms5525;
    INA3221 ina;
    TSYS01 tsys01;
    Rotoye rotoye;
    Maxell maxell;
    SIM_BattMonitor_SMBus_Generic smbus;
    Airspeed_DLVR dlvr;
    Benewake_TFMiniPlus tfmini;
    TSYS03 tsys03;
    MS5611 ms5611;
    QMC5883L qmc;
    TFS20L tfs20l;
    LightWareGRF_I2C grf;
    ToshibaLED toshiba;
    LP5562 lp;
    LM2755 lm;
    IS31FL3195 is31;
    populate_default_i2c_bus(bus, teraranger, lw16, maxsonar, mcp, icm, sht, as5600, ms5525, ina, tsys01, rotoye, maxell,
                             smbus, dlvr, tfmini, tsys03, ms5611, qmc, tfs20l, grf);
    populate_led_i2c_devices(bus, toshiba, lp, lm, is31);
    REQUIRE(bus.devices.size() == 24);
}

TEST_CASE("JEDEC MX25L3206E RDID") {
    JEDEC_MX25L3206E j;
    std::uint8_t cmd[1]{0x9f};
    SpiIocTransfer t0{cmd, nullptr, 1};
    REQUIRE(j.rdwr(1, &t0) == 0);
    std::uint8_t id[3]{};
    SpiIocTransfer t1{nullptr, id, 3};
    REQUIRE(j.rdwr(1, &t1) == 0);
    REQUIRE(id[0] == 0xC2);
    REQUIRE(id[2] == 0x16);
}

TEST_CASE("VectorNav VN300 emits 0xFA IMU header and swabbed CRC16") {
    Aircraft ac;
    ac.time_now_us = 40000;
    VectorNav vn(VectorNav::VNModel::VN300);
    vn.update(ac);
    const auto bytes = vn.drain_to_autopilot();
    REQUIRE(bytes.size() >= 10);
    REQUIRE(bytes[0] == 0xFA);
    REQUIRE(bytes[1] == 0x01);
    REQUIRE(bytes[2] == 0x21);
    REQUIRE(bytes[3] == 0x07);
    // CRC covers header+payload, then swabbed into last two bytes of IMU packet.
    const std::size_t imu_len = 1 + 3 + sizeof(VN_IMU_packet_sim) + 2;
    REQUIRE(bytes.size() >= imu_len);
    std::uint16_t crc = crc16_ccitt(bytes.data() + 1, static_cast<std::uint32_t>(3 + sizeof(VN_IMU_packet_sim)), 0);
    const std::uint16_t crc2 = static_cast<std::uint16_t>((crc << 8) | (crc >> 8));
    REQUIRE(bytes[imu_len - 2] == static_cast<std::uint8_t>(crc2));
    REQUIRE(bytes[imu_len - 1] == static_cast<std::uint8_t>(crc2 >> 8));
}

TEST_CASE("MicroStrain5 IMU packet is 0x75 0x65 0x80 plus Fletcher checksum") {
    Aircraft ac;
    ac.time_now_us = 50000;
    MicroStrain5 ms;
    ms.update(ac);
    const auto bytes = ms.drain_to_autopilot();
    REQUIRE(bytes.size() >= 6);
    REQUIRE(bytes[0] == 0x75);
    REQUIRE(bytes[1] == 0x65);
    REQUIRE(bytes[2] == 0x80);
    const std::uint8_t plen = bytes[3];
    REQUIRE(bytes.size() >= static_cast<std::size_t>(4 + plen + 2));
    std::uint8_t b1 = 0, b2 = 0;
    for (int i = 0; i < 4; i++) {
        b1 = static_cast<std::uint8_t>(b1 + bytes[i]);
        b2 = static_cast<std::uint8_t>(b2 + b1);
    }
    for (int i = 0; i < plen; i++) {
        b1 = static_cast<std::uint8_t>(b1 + bytes[4 + i]);
        b2 = static_cast<std::uint8_t>(b2 + b1);
    }
    REQUIRE(bytes[4 + plen] == b1);
    REQUIRE(bytes[5 + plen] == b2);
}

TEST_CASE("MicroStrain7 GNSS uses descriptors 0x91 and 0x92") {
    Aircraft ac;
    ac.time_now_us = 600000;
    MicroStrain7 ms;
    ms.update(ac);
    const auto bytes = ms.drain_to_autopilot();
    bool saw91 = false, saw92 = false;
    for (std::size_t i = 0; i + 3 < bytes.size(); i++) {
        if (bytes[i] == 0x75 && bytes[i + 1] == 0x65 && bytes[i + 2] == 0x91) saw91 = true;
        if (bytes[i] == 0x75 && bytes[i + 1] == 0x65 && bytes[i + 2] == 0x92) saw92 = true;
    }
    REQUIRE(saw91);
    REQUIRE(saw92);
}

TEST_CASE("InertialLabs packet is magic 0x55AA Table4 with CRC from byte 2") {
    Aircraft ac;
    ac.time_now_us = 10000;
    InertialLabs il;
    il.update(ac);
    const auto bytes = il.drain_to_autopilot();
    REQUIRE(bytes.size() == sizeof(ILabsPacket));
    REQUIRE(bytes[0] == 0xAA);
    REQUIRE(bytes[1] == 0x55);
    const std::uint16_t crc = crc_sum_of_bytes_16(bytes.data() + 2, static_cast<std::uint16_t>(bytes.size() - 4));
    REQUIRE(bytes[bytes.size() - 2] == static_cast<std::uint8_t>(crc));
    REQUIRE(bytes[bytes.size() - 1] == static_cast<std::uint8_t>(crc >> 8));
}

TEST_CASE("SensAItion interleaved packet 0 is 0xFA plus XOR CRC") {
    Aircraft ac;
    ac.time_now_us = 2000000;
    SensAItion s(true);
    s._tick = 0;
    s.update(ac);
    const auto bytes = s.drain_to_autopilot();
    REQUIRE_FALSE(bytes.empty());
    REQUIRE(bytes[0] == 0xFA);
    REQUIRE(bytes[1] == 0x00);
    std::uint8_t crc = 0x00;
    for (std::size_t i = 1; i + 1 < bytes.size() && i < 38; i++) {
        crc ^= bytes[i];
    }
    // first packet: FA | id | 36 payload | crc
    REQUIRE(bytes.size() >= 39);
    std::uint8_t x = bytes[1];
    for (int i = 0; i < 36; i++) {
        x ^= bytes[2 + i];
    }
    REQUIRE(bytes[38] == x);
}

TEST_CASE("LD06 uses crc8 poly 0x4D not ccitt") {
    PS_LD06 ld;
    Location loc{};
    ld.last_scan_output_time_ms = 1;
    ld.update(loc, 20);
    const auto bytes = ld.drain_to_autopilot();
    REQUIRE(bytes.size() == 47);
    REQUIRE(bytes[0] == 0x54);
    REQUIRE(bytes[46] == crc8_generic_poly(bytes.data(), 46, 0x4D));
}

TEST_CASE("RPLidar SCAN command yields A5 5A descriptor then 5-byte scan") {
    PS_RPLidarA2 lidar;
    Location loc{};
    const std::uint8_t cmd[2]{0xA5, 0x20};
    lidar.write_to_device(reinterpret_cast<const char*>(cmd), 2);
    lidar.update(loc, 20);
    const auto bytes = lidar.drain_to_autopilot();
    REQUIRE(bytes.size() >= 7);
    REQUIRE(bytes[0] == 0xA5);
    REQUIRE(bytes[1] == 0x5A);
    REQUIRE(bytes[6] == 0x81);
}

TEST_CASE("SF45B DISTANCE_DATA_CM is 0xAA plus xmodem CRC") {
    PS_LightWare_SF45B sf;
    Location loc{};
    sf.last_scan_output_time_ms = 1;
    sf.update(loc, 20);
    const auto bytes = sf.drain_to_autopilot();
    REQUIRE(bytes.size() >= 10);
    REQUIRE(bytes[0] == 0xAA);
    std::uint16_t crc = 0;
    for (int i = 0; i < 8; i++) {
        crc = crc_xmodem_update(crc, bytes[i]);
    }
    REQUIRE(bytes[8] == static_cast<std::uint8_t>(crc));
    REQUIRE(bytes[9] == static_cast<std::uint8_t>(crc >> 8));
}

TEST_CASE("Siyi PackedMessage is 55 66 with crc16_ccitt") {
    Siyi s;
    SitlInput in{};
    in.servos[5] = 1600;
    in.servos[6] = 1400;
    const std::uint8_t req[] = {0x55, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00};
    // fill CRC for ACQUIRE_GIMBAL_ATTITUDE (id 0x0D, plen 0)
    const std::uint16_t crc = crc16_ccitt(req, 8, 0);
    std::uint8_t framed[10];
    std::memcpy(framed, req, 8);
    framed[8] = static_cast<std::uint8_t>(crc);
    framed[9] = static_cast<std::uint8_t>(crc >> 8);
    s.write_to_device(reinterpret_cast<const char*>(framed), 10);
    s.update(in);
    const auto bytes = s.drain_to_autopilot();
    REQUIRE(bytes.size() >= 10);
    REQUIRE(bytes[0] == 0x55);
    REQUIRE(bytes[1] == 0x66);
    REQUIRE(bytes[7] == 0x0D);
}

TEST_CASE("Topotek attitude frame starts with #tp and hex CRC") {
    Topotek t;
    t.update();
    const auto bytes = t.drain_to_autopilot();
    REQUIRE(bytes.size() >= 8);
    REQUIRE(bytes[0] == '#');
    REQUIRE((bytes[1] == 't' || bytes[1] == 'T'));
    REQUIRE((bytes[2] == 'p' || bytes[2] == 'P'));
}

TEST_CASE("Viewpro T1_F1_B1_D1 starts 55 AA DC") {
    Viewpro v;
    SitlInput in{};
    v.update(in, 200);
    const auto bytes = v.drain_to_autopilot();
    REQUIRE(bytes.size() >= 8);
    REQUIRE(bytes[0] == 0x55);
    REQUIRE(bytes[1] == 0xAA);
    REQUIRE(bytes[2] == 0xDC);
}

TEST_CASE("AVT_CM62 identity and pitch limit") {
    AVT_CM62 avt;
    REQUIRE(std::string(avt.vendor) == "AVTA");
    REQUIRE(std::string(avt.model) == "SIM_AVTA");
    REQUIRE(avt.firmware_version == ((3U << 16) | (2U << 8) | 1U));
    SitlInput in{};
    in.servos[5] = 2000;
    avt.update(in);
    REQUIRE(avt.pitch <= 45.0f);
}

TEST_CASE("RichenPower 70-byte packet magics AA55 / 55AA") {
    RichenPower rp;
    SitlInput in{};
    in.servos[7] = 1900;
    rp.update(in, 2000);
    const auto bytes = rp.drain_to_autopilot();
    REQUIRE(bytes.size() == 70);
    REQUIRE(bytes[0] == 0xAA);
    REQUIRE(bytes[1] == 0x55);
    REQUIRE(bytes[68] == 0x55);
    REQUIRE(bytes[69] == 0xAA);
}

TEST_CASE("IntelligentEnergy24 V1 uses angle-bracket CSV") {
    IntelligentEnergy24 ie;
    ie.enabled = 1;
    ie.update(static_cast<std::uint32_t>(600));
    const auto bytes = ie.drain_to_autopilot();
    REQUIRE_FALSE(bytes.empty());
    REQUIRE(bytes[0] == '<');
    REQUIRE(bytes.back() == '\n');
}

TEST_CASE("Loweheiser emits isolated MAVLink2 HEARTBEAT and GOV_EFI") {
    Loweheiser lh;
    lh.update(4800, 12, 300);
    const auto bytes = lh.drain_to_autopilot();
    REQUIRE(bytes.size() >= 12);
    REQUIRE(bytes[0] == 0xFD);
    bool saw_efi = false;
    for (std::size_t i = 0; i + 10 < bytes.size(); i++) {
        if (bytes[i] == 0xFD) {
            const std::uint32_t msgid = static_cast<std::uint32_t>(bytes[i + 7] | (bytes[i + 8] << 8) | (bytes[i + 9] << 16));
            if (msgid == 10151) {
                saw_efi = true;
            }
        }
    }
    REQUIRE(saw_efi);
}

TEST_CASE("FETtecOneWireESC REQ_TYPE from bootloader/running uses crc8_dvb") {
    FETtecOneWireESC esc;
    esc.escs[0].state = FETtecOneWireESC::ESC::State::RUNNING;
    std::uint8_t req[7]{0x01, 0x01, 0, 0, 7, 5, 0};  // REQ_TYPE=5
    req[6] = crc8_dvb_s2_update(0, req, 6);
    esc.write_to_device(reinterpret_cast<const char*>(req), 7);
    SitlInput in{};
    in.servos[0] = 1600;
    esc.update(in);
    const auto bytes = esc.drain_to_autopilot();
    REQUIRE_FALSE(bytes.empty());
    REQUIRE(bytes[0] == 0x03);
}

TEST_CASE("Heli Single collective scaler is not raw PWM identity") {
    fwcpp::motors::MotorsHeliSingle s;
    s.configure();
    s.move_actuators(0.5f, -0.2f, 0.6f, 0.1f);
    std::uint16_t servos[16]{};
    s.write_servos(servos, 0.01f);
    REQUIRE(servos[0] != servos[1]);
    REQUIRE(servos[3] != 1500);
}

TEST_CASE("Heli Dual tandem mixing differs swash1 vs swash2") {
    fwcpp::motors::MotorsHeliDual d;
    d.configure();
    d.move_actuators(0.4f, 0.0f, 0.5f, 0.3f);
    std::uint16_t servos[16]{};
    d.write_servos(servos);
    REQUIRE(servos[0] != servos[3]);
}

TEST_CASE("Heli Quad four-motor mix is not identical PWM") {
    fwcpp::motors::MotorsHeliQuad q;
    q.move_actuators(0.3f, -0.2f, 0.55f, 0.1f);
    std::uint16_t servos[16]{};
    q.write_servos(servos);
    REQUIRE(servos[0] != servos[1]);
    REQUIRE(servos[2] != servos[3]);
}

TEST_CASE("ToshibaLED PWM registers scale x17 when enabled") {
    ToshibaLED led;
    I2cMsg wr[1]{};
    std::uint8_t buf[2]{ToshibaLED::ENABLE, 1};
    wr[0].addr = 0x55;
    wr[0].flags = 0;
    wr[0].len = 2;
    wr[0].buf = buf;
    I2cRdwr d{wr, 1};
    REQUIRE(led.rdwr(d) == 0);
    std::uint8_t pwm[2]{ToshibaLED::PWM0, 5};
    wr[0].buf = pwm;
    REQUIRE(led.rdwr(d) == 0);
    Aircraft ac;
    led.update(ac);
    REQUIRE(led.last_rgb[0] == 85);
}

TEST_CASE("IS31FL3195 colour update 0xC5 latches OUT registers") {
    IS31FL3195 led;
    led.set_product_id(0x54);
    I2cMsg wr[1]{};
    std::uint8_t out1[2]{IS31FL3195::OUT1, 0xAA};
    wr[0].flags = 0;
    wr[0].len = 2;
    wr[0].buf = out1;
    I2cRdwr d{wr, 1};
    REQUIRE(led.rdwr(d) == 0);
    Aircraft ac;
    led.update(ac);
    REQUIRE(led.rgb[0] == 0);  // not latched yet
    std::uint8_t poke[2]{IS31FL3195::COLOUR_UPDATE, 0xC5};
    wr[0].buf = poke;
    REQUIRE(led.rdwr(d) == 0);
    led.update(ac);
    REQUIRE(led.rgb[0] == 0xAA);
}

TEST_CASE("Frsky_D parses START_STOP_D plus stuffed 0x5E payload") {
    Frsky_D d;
    // 0x5E | BARO_ALT_BP=0x10 | stuffed 0x5E -> 0x5D 0x3E | 0x02
    const std::uint8_t pkt[] = {0x5E, 0x10, 0x5D, 0x3E, 0x02};
    d.write_to_device(reinterpret_cast<const char*>(pkt), sizeof(pkt));
    d.update();
    REQUIRE(d.received.size() == 1);
    REQUIRE(d.received[0].id == 0x10);
    REQUIRE(d.received[0].data == static_cast<std::uint16_t>(0x5E | (0x02 << 8)));
    REQUIRE(std::string(Frsky::dataid_string(Frsky::DataID::BARO_ALT_BP)) == "BARO_ALT_BP");
    REQUIRE(std::string(Frsky::dataid_string(Frsky::DataID::VFAS)) == "VFAS");
}

TEST_CASE("Frsky_D two-message sequence with 0x5D stuffing") {
    Frsky_D d;
    // VFAS 0x39 data 0x5D 0x00 stuffed as 0x5D 0x3D, then CURRENT 0x28 data 0x0064
    const std::uint8_t pkt[] = {0x5E, 0x39, 0x5D, 0x3D, 0x00, 0x5E, 0x28, 0x64, 0x00};
    d.write_to_device(reinterpret_cast<const char*>(pkt), sizeof(pkt));
    d.update();
    REQUIRE(d.received.size() == 2);
    REQUIRE(d.received[0].id == 0x39);
    REQUIRE(d.received[0].data == 0x005D);
    REQUIRE(d.received[1].id == 0x28);
    REQUIRE(d.received[1].data == 0x0064);
}

TEST_CASE("CRSF cycles original VTX then telem then battery frames") {
    CRSF crsf;
    crsf.update(400);
    auto a = crsf.drain_to_autopilot();
    REQUIRE(a.size() == 10);
    REQUIRE(a[0] == 0xC8);
    REQUIRE(a[1] == 0x8);
    REQUIRE(a[2] == 0xF);
    REQUIRE(a[9] == 0x5F);
    crsf.update(800);
    auto b = crsf.drain_to_autopilot();
    REQUIRE(b.size() == 9);
    REQUIRE(b[0] == 0xC8);
    REQUIRE(b[2] == 0x10);
    REQUIRE(b[8] == 0x1B);
    crsf.update(1200);
    auto c = crsf.drain_to_autopilot();
    REQUIRE(c.size() == 11);
    REQUIRE(c[0] == 0xC8);
    REQUIRE(c[2] == 0x8);
    REQUIRE(c[10] == 0x95);
}

TEST_CASE("ELRS emits MAVLink2 RADIO_STATUS msgid 109 crc extra 185") {
    ELRS elrs(2);
    REQUIRE(elrs.device_baud() == 460800);
    REQUIRE(elrs.target_port == 5763);
    elrs.update(20);
    const auto bytes = elrs.drain_to_autopilot();
    REQUIRE(bytes.size() >= 21);
    REQUIRE(bytes[0] == 0xFD);
    REQUIRE(bytes[1] == 9);
    const std::uint32_t msgid = static_cast<std::uint32_t>(bytes[7] | (bytes[8] << 8) | (bytes[9] << 16));
    REQUIRE(msgid == 109);
    REQUIRE(bytes[5] == 255);
    REQUIRE(bytes[6] == 0);
    REQUIRE(bytes[10] == 0);
    REQUIRE(bytes[11] == 0);
    REQUIRE(bytes[12] == 0);
    REQUIRE(bytes[13] == 0);
    REQUIRE(bytes[14] == 255);
    REQUIRE(bytes[15] == 255);
    REQUIRE(bytes[16] == 100);
    const std::uint16_t crc = mavmin::crc_calculate(bytes.data() + 1, static_cast<std::uint32_t>(9 + 9),
                                                    mavmin::kCrcRadioStatus);
    REQUIRE(bytes[19] == static_cast<std::uint8_t>(crc));
    REQUIRE(bytes[20] == static_cast<std::uint8_t>(crc >> 8));
}

TEST_CASE("ELRS forwards a complete GCS HEARTBEAT to the autopilot") {
    ELRS elrs(2);
    mavmin::Status st{};
    auto hb = mavmin::encode_heartbeat(1, 1, st);
    elrs.inject_from_gcs(hb.data(), hb.size());
    elrs.update(0);
    elrs.update(1000);
    const auto bytes = elrs.drain_to_autopilot();
    bool saw_hb = false;
    for (std::size_t i = 0; i + 10 < bytes.size(); i++) {
        if (bytes[i] == 0xFD) {
            const std::uint32_t msgid =
                static_cast<std::uint32_t>(bytes[i + 7] | (bytes[i + 8] << 8) | (bytes[i + 9] << 16));
            if (msgid == 0) {
                saw_hb = true;
            }
        }
    }
    REQUIRE(saw_hb);
}

TEST_CASE("Volz SET_EXTENDED_POSITION uses crc_crc16_ibm not ccitt") {
    Volz v;
    v._enabled = true;
    Volz::Command cmd{};
    cmd.command_id = Volz::CommandId::SET_EXTENDED_POSITION;
    cmd.actuator_id = 1;
    cmd.arg1 = 0x0C;
    cmd.arg2 = 0x00;
    cmd.update_checksum();
    const std::uint16_t ibm = crc_crc16_ibm(0xffff, reinterpret_cast<const std::uint8_t*>(&cmd), 4);
    REQUIRE(cmd.crc_host() == ibm);
    const std::uint16_t ccitt = crc16_ccitt(reinterpret_cast<const std::uint8_t*>(&cmd), 4, 0);
    REQUIRE(ibm != ccitt);
    v.write_to_device(reinterpret_cast<const char*>(&cmd), sizeof(cmd));
    Aircraft ac;
    v.update(ac);
    const auto bytes = v.drain_to_autopilot();
    REQUIRE(bytes.size() == 6);
    REQUIRE(bytes[0] == 0x2C);
    REQUIRE(bytes[1] == 1);
    Volz::Command resp{};
    std::memcpy(&resp, bytes.data(), 6);
    REQUIRE(resp.calculate_checksum() == resp.crc_host());
    REQUIRE(v.servos[0].desired_position > 0.5f);
}

TEST_CASE("Volz READ_VOLTAGE CURRENT TEMPERATURE multi-command sequence") {
    Volz v;
    v._enabled = true;
    auto send = [&](Volz::CommandId id) {
        Volz::Command cmd{};
        cmd.command_id = id;
        cmd.actuator_id = 2;
        cmd.arg1 = 0;
        cmd.arg2 = 0;
        cmd.update_checksum();
        v.write_to_device(reinterpret_cast<const char*>(&cmd), sizeof(cmd));
    };
    send(Volz::CommandId::READ_VOLTAGE);
    send(Volz::CommandId::READ_CURRENT);
    send(Volz::CommandId::READ_TEMPERATURE);
    Aircraft ac;
    v.update(ac);
    const auto bytes = v.drain_to_autopilot();
    REQUIRE(bytes.size() == 18);
    REQUIRE(bytes[0] == 0x31);
    REQUIRE(bytes[6] == 0x30);
    REQUIRE(bytes[12] == 0x10);
    REQUIRE(bytes[1] == 2);
    REQUIRE(bytes[2] == 5);
    REQUIRE(bytes[3] == 6);
    REQUIRE(bytes[8] == 5);
    REQUIRE(bytes[14] == 75);
    REQUIRE(bytes[15] == 75);
}

TEST_CASE("Volz update_sitl_input_pwm respects mask and fail-hold") {
    Volz v;
    v._enabled = true;
    v.servos[0].desired_position = 1.0f;
    v.servos[1].desired_position = -1.0f;
    v.servos[3].desired_position = 0.5f;
    SitlInput in{};
    in.servos[0] = 1500;
    in.servos[1] = 1500;
    in.servos[2] = 1500;
    in.servos[3] = 1500;
    v.update_sitl_input_pwm(in);
    REQUIRE(in.servos[0] == 2000);
    REQUIRE(in.servos[1] == 1000);
    REQUIRE(in.servos[2] == 1500);
    REQUIRE(in.servos[3] == 1750);
    v._failed_mask = 1U;
    v.servos[0].desired_position = 0.0f;
    v.update_sitl_input_pwm(in);
    REQUIRE(in.servos[0] == 2000);
}
