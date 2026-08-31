#pragma once

// Port of libraries/SITL/SIM_RF_* serial rangefinders constructed by
// AP_HAL_SITL/SITL_State_common.cpp serial_rangefinder_definitions[].
// RF_MAVLink encodes MAVLink2 DISTANCE_SENSOR without GCS. LightWare GRF
// stream config (STREAM=5) is original-source.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_serial_rangefinder.hpp>
#include <fwcpp/sim/sim_rf_mavlink_grf.hpp>

namespace fwcpp::sim {

inline std::uint8_t highbyte(std::uint16_t x) { return static_cast<std::uint8_t>(x >> 8); }
inline std::uint8_t lowbyte(std::uint16_t x) { return static_cast<std::uint8_t>(x & 0xff); }

class RF_Benewake : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        if (buflen < 9) {
            return 0;
        }
        const std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        buffer[0] = 0x59;
        buffer[1] = 0x59;
        buffer[3] = static_cast<std::uint8_t>(alt_cm >> 8);
        buffer[2] = static_cast<std::uint8_t>(alt_cm & 0xff);
        buffer[4] = byte4();
        buffer[5] = byte5();
        buffer[6] = byte6();
        buffer[7] = byte7();
        buffer[8] = 0;
        for (std::uint8_t i = 0; i < 8; i++) {
            buffer[8] = static_cast<std::uint8_t>(buffer[8] + buffer[i]);
        }
        return 9;
    }

private:
    virtual std::uint8_t byte4() const = 0;
    virtual std::uint8_t byte5() const = 0;
    virtual std::uint8_t byte6() const = 0;
    virtual std::uint8_t byte7() const = 0;
};

class RF_Benewake_TF02 : public RF_Benewake {
    std::uint8_t byte4() const override { return 1; }
    std::uint8_t byte5() const override { return 1; }
    std::uint8_t byte6() const override { return 7; }
    std::uint8_t byte7() const override { return 0x06; }
};

class RF_Benewake_TF03 : public RF_Benewake {
    std::uint8_t byte4() const override { return 0; }
    std::uint8_t byte5() const override { return 0; }
    std::uint8_t byte6() const override { return 0; }
    std::uint8_t byte7() const override { return 0; }
};

class RF_Benewake_TFmini : public RF_Benewake {
    std::uint8_t byte4() const override { return 1; }
    std::uint8_t byte5() const override { return 1; }
    std::uint8_t byte6() const override { return 0x07; }
    std::uint8_t byte7() const override { return 0; }
};

class RF_Ainstein_LR_D1 : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        const std::uint8_t PACKET_LEN = 32;
        if (buflen < PACKET_LEN) {
            return 0;
        }
        std::uint8_t malfunction_alert = 0;
        std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        std::uint8_t snr = (alt_cm == 0xFFFF) ? 0 : 100;
        if (alt_m > 525) {
            malfunction_alert |= 1U << 0;
        }
        if (alt_m > 500) {
            snr = 0;
        }
        if (alt_m * 100 > 65535) {
            malfunction_alert |= 1U << 7;
        }
        buffer[0] = 0xEB;
        buffer[1] = 0x90;
        buffer[2] = 0;
        buffer[3] = 28;
        buffer[4] = malfunction_alert;
        buffer[5] = 1;
        buffer[6] = highbyte(alt_cm);
        buffer[7] = lowbyte(alt_cm);
        buffer[8] = snr;
        buffer[9] = 0;
        buffer[10] = 0;
        std::memset(&buffer[11], 0xff, 20);
        buffer[31] = crc_sum_of_bytes(&buffer[3], PACKET_LEN - 4);
        return PACKET_LEN;
    }
};

class RF_Ainstein_LR_D1_v19 : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        const std::uint8_t PACKET_LEN = 32;
        if (buflen < PACKET_LEN) {
            return 0;
        }
        std::uint16_t malfunction_alert = 0;
        std::uint8_t snr = 100;
        std::uint8_t altitude_valid = 1;
        if (alt_m > 525) {
            malfunction_alert |= 1U << 0;
        }
        if (alt_m > 500) {
            snr = 0;
            altitude_valid = 0;
        }
        if (alt_m * 100 > 65535) {
            malfunction_alert |= 1U << 10;
        }
        const std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        buffer[0] = 0xEB;
        buffer[1] = 0x90;
        buffer[2] = 0;
        buffer[3] = 28;
        buffer[4] = highbyte(malfunction_alert);
        buffer[5] = lowbyte(malfunction_alert);
        buffer[6] = highbyte(alt_cm);
        buffer[7] = lowbyte(alt_cm);
        buffer[8] = snr;
        buffer[9] = 0;
        buffer[10] = 0;
        std::memset(&buffer[11], 0xff, 20);
        buffer[11] = altitude_valid;
        buffer[31] = crc_sum_of_bytes(&buffer[3], PACKET_LEN - 4);
        return PACKET_LEN;
    }
};

class RF_BLping : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        const std::uint32_t alt_mm = static_cast<std::uint32_t>(alt_m * 1000);
        const std::uint8_t payload[4] = {
            static_cast<std::uint8_t>(alt_mm & 0xff),
            static_cast<std::uint8_t>((alt_mm >> 8) & 0xff),
            static_cast<std::uint8_t>((alt_mm >> 16) & 0xff),
            static_cast<std::uint8_t>((alt_mm >> 24) & 0xff),
        };
        const std::uint16_t message_id = 1211;
        std::uint16_t offs = 0;
        buffer[offs++] = 0x42;
        buffer[offs++] = 0x52;
        buffer[offs++] = 4;
        buffer[offs++] = 0;
        buffer[offs++] = static_cast<std::uint8_t>(message_id & 0xff);
        buffer[offs++] = static_cast<std::uint8_t>(message_id >> 8);
        buffer[offs++] = 1;
        buffer[offs++] = 0;
        std::memcpy(&buffer[offs], payload, 4);
        offs += 4;
        std::uint16_t crc = 0;
        for (std::uint8_t i = 0; i < offs; i++) {
            crc = static_cast<std::uint16_t>(crc + buffer[i]);
        }
        buffer[offs++] = static_cast<std::uint8_t>(crc & 0xff);
        buffer[offs++] = static_cast<std::uint8_t>(crc >> 8);
        return offs;
    }
};

class RF_DTS6012M : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        const std::uint8_t PACKET_LEN = 23;
        if (buflen < PACKET_LEN) {
            return 0;
        }
        const std::uint16_t dist_mm = (alt_m * 1000 > 20000) ? 0xFFFF : static_cast<std::uint16_t>(alt_m * 1000);
        buffer[0] = 0xA5;
        buffer[1] = 0x03;
        buffer[2] = 0x20;
        buffer[3] = 0x01;
        buffer[4] = 0x00;
        buffer[5] = 0x00;
        buffer[6] = 0x0E;
        buffer[7] = 0xFF;
        buffer[8] = 0xFF;
        buffer[9] = 0x00;
        buffer[10] = 0x00;
        buffer[11] = 0x00;
        buffer[12] = 0x00;
        buffer[13] = static_cast<std::uint8_t>(dist_mm & 0xFF);
        buffer[14] = static_cast<std::uint8_t>(dist_mm >> 8);
        buffer[15] = 0x00;
        buffer[16] = 0x00;
        buffer[17] = 0x10;
        buffer[18] = 0x27;
        buffer[19] = 0x00;
        buffer[20] = 0x00;
        const std::uint16_t crc = calc_crc_modbus(buffer, PACKET_LEN - 2);
        buffer[21] = static_cast<std::uint8_t>(crc >> 8);
        buffer[22] = static_cast<std::uint8_t>(crc & 0xFF);
        return PACKET_LEN;
    }
};

class RF_GYUS42v2 : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        const std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        if (buflen < 7) {
            return 0;
        }
        buffer[0] = 0x5A;
        buffer[1] = 0;
        buffer[2] = 0;
        buffer[3] = 0;
        buffer[4] = static_cast<std::uint8_t>(alt_cm >> 8);
        buffer[5] = static_cast<std::uint8_t>(alt_cm & 0xFF);
        buffer[6] = 0;
        for (std::uint8_t i = 0; i < 6; i++) {
            buffer[6] = static_cast<std::uint8_t>(buffer[6] + buffer[i]);
        }
        return 7;
    }
};

class RF_JRE : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        std::uint16_t status = 0;
        if (alt_m > 500) {
            status |= 0x2;
        }
        const std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        buffer[0] = 0x52;  // R
        buffer[1] = 0x41;  // A
        buffer[2] = 0x01;
        buffer[3] = frame_count_++;
        buffer[4] = static_cast<std::uint8_t>(alt_cm >> 8);
        buffer[5] = static_cast<std::uint8_t>(alt_cm & 0xff);
        buffer[6] = 0;
        buffer[7] = 0;
        buffer[8] = 0;
        buffer[9] = 0;
        buffer[10] = 0;
        buffer[11] = 0;
        buffer[12] = static_cast<std::uint8_t>(status >> 8);
        buffer[13] = static_cast<std::uint8_t>(status & 0xff);
        const std::uint16_t crc = crc16_ccitt_r(&buffer[0], 14, 0xffff, 0xffff);
        buffer[14] = static_cast<std::uint8_t>(crc & 0xff);
        buffer[15] = static_cast<std::uint8_t>(crc >> 8);
        return 16;
    }

private:
    std::uint8_t frame_count_{0};
};

class RF_Lanbao : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        const std::uint16_t alt_mm = static_cast<std::uint16_t>(alt_m * 1000);
        buffer[0] = 0xA5;
        buffer[1] = 0x5A;
        buffer[2] = static_cast<std::uint8_t>(alt_mm >> 8);
        buffer[3] = static_cast<std::uint8_t>(alt_mm & 0xff);
        const std::uint16_t crc = calc_crc_modbus(buffer, 4);
        buffer[4] = static_cast<std::uint8_t>(crc & 0xff);
        buffer[5] = static_cast<std::uint8_t>(crc >> 8);
        return 6;
    }
};

class RF_LeddarOne : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        const std::uint8_t response_size = 25;
        const std::uint16_t internal_temperature = 1245;
        const std::uint16_t num_detections = 1;
        const std::uint16_t first_distance = static_cast<std::uint16_t>(alt_m * 1000);
        const std::uint16_t amp = 37;
        const std::uint32_t now = now_ms_for_packet_;
        buffer[0] = 0x01;
        buffer[1] = 0x04;
        buffer[2] = response_size;
        buffer[3] = static_cast<std::uint8_t>((now >> 16) & 0xff);
        buffer[4] = static_cast<std::uint8_t>((now >> 24) & 0xff);
        buffer[5] = static_cast<std::uint8_t>((now >> 0) & 0xff);
        buffer[6] = static_cast<std::uint8_t>((now >> 8) & 0xff);
        buffer[7] = static_cast<std::uint8_t>(internal_temperature & 0xff);
        buffer[8] = static_cast<std::uint8_t>(internal_temperature >> 8);
        buffer[9] = static_cast<std::uint8_t>(num_detections >> 8);
        buffer[10] = static_cast<std::uint8_t>(num_detections & 0xff);
        buffer[11] = static_cast<std::uint8_t>(first_distance >> 8);
        buffer[12] = static_cast<std::uint8_t>(first_distance & 0xff);
        buffer[13] = static_cast<std::uint8_t>(amp >> 8);
        buffer[14] = static_cast<std::uint8_t>(amp & 0xff);
        buffer[15] = buffer[11];
        buffer[16] = buffer[12];
        buffer[17] = buffer[13];
        buffer[18] = buffer[14];
        buffer[19] = buffer[11];
        buffer[20] = buffer[12];
        buffer[21] = buffer[13];
        buffer[22] = buffer[14];
        const std::uint16_t crc = calc_crc_modbus(buffer, 23);
        buffer[23] = static_cast<std::uint8_t>(crc & 0xff);
        buffer[24] = static_cast<std::uint8_t>(crc >> 8);
        return response_size;
    }
    void set_now_ms(std::uint32_t now) { now_ms_for_packet_ = now; }

private:
    std::uint32_t now_ms_for_packet_{0};
};

class RF_LightWareSerial : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        std::uint16_t alt_cm;
        if (alt_m > 100) {
            alt_cm = 13000;
        } else {
            alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        }
        const int n = std::snprintf(reinterpret_cast<char*>(buffer), buflen, "%0.2f\r", alt_cm * 0.01f);
        return n > 0 ? static_cast<std::uint32_t>(n) : 0;
    }
};

class RF_LightWareSerialBinary : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        const std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        buffer[0] = static_cast<std::uint8_t>(((alt_cm >> 7) & 0x7f) | (1 << 7));
        buffer[1] = static_cast<std::uint8_t>(alt_cm & 0x7f);
        return 2;
    }
};

class RF_MaxsonarSerialLV : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        const float inches = alt_m * 100 / 2.54f;
        const int n = std::snprintf(reinterpret_cast<char*>(buffer), buflen, "%u\r", static_cast<unsigned>(inches));
        return n > 0 ? static_cast<std::uint32_t>(n) : 0;
    }
};

class RF_NMEA : public SerialRangeFinder {
public:
    bool has_temperature() const override { return true; }
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        int ret = std::snprintf(reinterpret_cast<char*>(buffer), buflen, "$SMDPT,%f,%f", alt_m, 0.01f);
        std::uint8_t checksum = 0;
        for (int i = 1; i < ret; i++) {
            checksum ^= buffer[i];
        }
        ret += std::snprintf(reinterpret_cast<char*>(&buffer[ret]), buflen - ret, "*%02X\r\n", checksum);
        return static_cast<std::uint32_t>(ret);
    }
    std::uint32_t packet_for_temperature(float temperature, std::uint8_t* buffer, std::uint8_t buflen) override {
        int ret = std::snprintf(reinterpret_cast<char*>(buffer), buflen, "$SMMTW,%f %f", temperature, 0.01);
        std::uint8_t checksum = 0;
        for (int i = 1; i < ret; i++) {
            checksum ^= buffer[i];
        }
        ret += std::snprintf(reinterpret_cast<char*>(&buffer[ret]), buflen - ret, "*%02X\r\n", checksum);
        return static_cast<std::uint32_t>(ret);
    }
};

class RF_Nooploop : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        if (alt_cm == 0) {
            alt_cm = 1;
        }
        const std::int32_t alt_scaled = 2560 * alt_cm;
        buffer[0] = 0x57;
        buffer[1] = 0x00;
        buffer[2] = 0xff;
        buffer[3] = 0x00;
        buffer[4] = 0x9e;
        buffer[5] = 0x8f;
        buffer[6] = 0x00;
        buffer[7] = 0x00;
        buffer[8] = static_cast<std::uint8_t>(alt_scaled >> 8 & 0xff);
        buffer[9] = static_cast<std::uint8_t>(alt_scaled >> 16 & 0xff);
        buffer[10] = static_cast<std::uint8_t>(alt_scaled >> 24 & 0xff);
        buffer[11] = 0x00;
        buffer[12] = 0x03;
        buffer[13] = 0x00;
        buffer[14] = 0x06;
        buffer[15] = 0;
        for (std::uint8_t i = 0; i < 15; i++) {
            buffer[15] = static_cast<std::uint8_t>(buffer[15] + buffer[i]);
        }
        return 16;
    }
};

class RF_RDS02UF : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        const std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        const std::uint16_t fc_code = 0x3ff;
        const std::uint16_t data_fc = 0x70c;
        std::uint8_t resp[21] {};
        resp[0] = 0x55;
        resp[1] = 0x55;
        resp[4] = static_cast<std::uint8_t>(fc_code & 0xff);
        resp[5] = static_cast<std::uint8_t>(fc_code >> 8);
        resp[6] = 10;
        resp[8] = static_cast<std::uint8_t>(data_fc & 0xff);
        resp[9] = static_cast<std::uint8_t>(data_fc >> 8);
        resp[13] = static_cast<std::uint8_t>(alt_cm & 0xff);
        resp[14] = static_cast<std::uint8_t>(alt_cm >> 8);
        resp[18] = crc8_rds02uf(&resp[2], 16);
        resp[19] = 0xAA;
        resp[20] = 0xAA;
        if (buflen < 21) {
            return 0;
        }
        std::memcpy(buffer, resp, 21);
        return 21;
    }
};

class RF_TeraRanger_Serial : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        const std::uint16_t alt_mm = static_cast<std::uint16_t>(alt_m * 1000);
        buffer[0] = 0x54;
        buffer[1] = static_cast<std::uint8_t>(alt_mm >> 8);
        buffer[2] = static_cast<std::uint8_t>(alt_mm & 0xff);
        buffer[3] = (alt_m > 30) ? 0xC4 : 0xC0;
        buffer[4] = crc_crc8(buffer, 4);
        return 5;
    }
};

class RF_USD1_v0 : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        const std::uint16_t reading = static_cast<std::uint16_t>(alt_m * 100 / 2.5f);
        buffer[0] = 0x48;
        buffer[1] = static_cast<std::uint8_t>(reading & 0x7f);
        buffer[2] = static_cast<std::uint8_t>((reading >> 7) & 0xff);
        buffer[3] = 0x48;
        buffer[4] = buffer[1];
        buffer[5] = buffer[2];
        return 6;
    }
};

class RF_USD1_v1 : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t) override {
        const std::uint16_t alt_cm = static_cast<std::uint16_t>(alt_m * 100);
        buffer[0] = 0xFE;
        buffer[1] = 0;
        buffer[2] = static_cast<std::uint8_t>(alt_cm & 0xff);
        buffer[3] = static_cast<std::uint8_t>(alt_cm >> 8);
        buffer[4] = 0;
        buffer[5] = static_cast<std::uint8_t>(buffer[1] + buffer[2] + buffer[3] + buffer[4]);
        return 6;
    }
};

class RF_Wasp : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        if (alt_m < 0.01f) {
            alt_m = 0.01f;
        }
        const int n = std::snprintf(reinterpret_cast<char*>(buffer), buflen, "%f\n", alt_m);
        return n > 0 ? static_cast<std::uint32_t>(n) : 0;
    }
};

inline std::unique_ptr<SerialRangeFinder> create_serial_rangefinder(const char* name) {
    const std::string n(name);
    if (n == "ainsteinlrd1") return std::make_unique<RF_Ainstein_LR_D1>();
    if (n == "ainsteinlrd1_v19") return std::make_unique<RF_Ainstein_LR_D1_v19>();
    if (n == "benewake_tf02") return std::make_unique<RF_Benewake_TF02>();
    if (n == "benewake_tf03") return std::make_unique<RF_Benewake_TF03>();
    if (n == "benewake_tfmini") return std::make_unique<RF_Benewake_TFmini>();
    if (n == "blping") return std::make_unique<RF_BLping>();
    if (n == "dts6012m") return std::make_unique<RF_DTS6012M>();
    if (n == "gyus42v2") return std::make_unique<RF_GYUS42v2>();
    if (n == "jre") return std::make_unique<RF_JRE>();
    if (n == "lanbao") return std::make_unique<RF_Lanbao>();
    if (n == "leddarone") return std::make_unique<RF_LeddarOne>();
    if (n == "lightwareserial-binary") return std::make_unique<RF_LightWareSerialBinary>();
    if (n == "lightwareserial") return std::make_unique<RF_LightWareSerial>();
    if (n == "maxsonarseriallv") return std::make_unique<RF_MaxsonarSerialLV>();
    if (n == "nmea") return std::make_unique<RF_NMEA>();
    if (n == "nooploop_tofsense") return std::make_unique<RF_Nooploop>();
    if (n == "rds02uf") return std::make_unique<RF_RDS02UF>();
    if (n == "teraranger_serial") return std::make_unique<RF_TeraRanger_Serial>();
    if (n == "USD1_v0") return std::make_unique<RF_USD1_v0>();
    if (n == "USD1_v1") return std::make_unique<RF_USD1_v1>();
    if (n == "wasp") return std::make_unique<RF_Wasp>();
    if (n == "rf_mavlink") return std::make_unique<RF_MAVLink>();
    if (n == "lightware_grf") return std::make_unique<RF_LightWareGRF>();
    return nullptr;
}

}  // namespace fwcpp::sim
