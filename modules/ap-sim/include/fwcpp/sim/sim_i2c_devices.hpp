#pragma once

// Remaining SIM_I2C.cpp devices: MS5XXX/MS5611/MS5525, INA3221, SMBus
// Generic/Maxell/Rotoye, TeraRangerI2C, LightWareI2C_Legacy16Bit,
// LightWareGRF_I2C, Airspeed_DLVR, TFMiniPlus I2C, TFS20L, SHT3x/TSYS01/
// TSYS03/MCP9600, AS5600, ICM40609 (via InvensenseV3). LED drivers
// LED drivers (Toshiba/LP5562/LM2755/IS31FL3195) are on the original SIM_I2C bus.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_i2c.hpp>
#include <fwcpp/sim/sim_i2c_leds.hpp>

namespace fwcpp::sim {

inline float kelvin_to_c(float k) { return k - 273.15f; }
inline float c_to_kelvin(float c) { return c + 273.15f; }

inline float aircraft_alt_m(const Aircraft& a) { return a.location.alt * 0.01f; }

inline float sim_board_temp_c(const Aircraft& a) {
    float p, t_k;
    get_pressure_temperature_for_alt_amsl(aircraft_alt_m(a), p, t_k);
    return kelvin_to_c(t_k);
}

inline std::uint8_t crc8_generic(const std::uint8_t* data, std::uint8_t len, std::uint8_t poly, std::uint8_t crc) {
    while (len--) {
        crc ^= *data++;
        for (std::uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = static_cast<std::uint8_t>((crc << 1) ^ poly);
            } else {
                crc = static_cast<std::uint8_t>(crc << 1);
            }
        }
    }
    return crc;
}

class I2CRegisters_ConfigurableLength : public I2CRegisters {
public:
    void add_register(const char* name, std::uint8_t reg, std::uint8_t len, RegMode mode) {
        I2CRegisters::add_register(name, reg, mode);
        reg_data_len[reg] = len;
    }
    void set_register(std::uint8_t reg, std::uint16_t value) {
        if (reg_data_len[reg] != 2) {
            return;
        }
        reg_data[reg] = static_cast<std::uint32_t>((value >> 8) | ((value & 0xFF) << 8));
    }
    void set_register(std::uint8_t reg, std::uint8_t value) {
        if (reg_data_len[reg] != 1) {
            return;
        }
        reg_data[reg] = static_cast<std::uint32_t>(value) << 24;
    }
    void get_reg_value(std::uint8_t reg, std::uint8_t& value) const {
        if (reg_data_len[reg] == 1) {
            value = static_cast<std::uint8_t>(reg_data[reg] >> 24);
        } else {
            value = static_cast<std::uint8_t>(reg_data[reg] & 0xFF);
        }
    }
    int rdwr(I2cRdwr& data) {
        if (data.nmsgs == 2) {
            if (data.msgs[0].flags != 0 || data.msgs[1].flags != I2C_M_RD) {
                return -1;
            }
            const std::uint8_t reg_addr = data.msgs[0].buf[0];
            if (!readable_registers.get(reg_addr)) {
                return -1;
            }
            const std::uint32_t register_value = reg_data[reg_addr];
            if (data.msgs[1].len == 1) {
                data.msgs[1].buf[0] = static_cast<std::uint8_t>(register_value >> 24);
            } else if (data.msgs[1].len == 2) {
                const std::uint16_t v = static_cast<std::uint16_t>(((register_value & 0xFF) << 8) | ((register_value >> 8) & 0xFF));
                data.msgs[1].buf[0] = static_cast<std::uint8_t>(v >> 8);
                data.msgs[1].buf[1] = static_cast<std::uint8_t>(v & 0xFF);
            }
            data.msgs[1].len = reg_data_len[reg_addr];
            return 0;
        }
        if (data.nmsgs == 1) {
            if (data.msgs[0].flags != 0) {
                return -1;
            }
            const std::uint8_t reg_addr = data.msgs[0].buf[0];
            if (!writable_registers.get(reg_addr)) {
                return -1;
            }
            const std::uint8_t data_msg_len = static_cast<std::uint8_t>(data.msgs[0].len - 1);
            if (data_msg_len != reg_data_len[reg_addr]) {
                return -1;
            }
            std::memcpy(reinterpret_cast<std::uint8_t*>(&reg_data[reg_addr]), &data.msgs[0].buf[1], data_msg_len);
            return 0;
        }
        return -1;
    }

protected:
    std::uint32_t reg_data[256]{};
    std::uint8_t reg_data_len[256]{};
};

// ---- MS5XXX / MS5611 / MS5525 ----
class MS5XXX : public I2CDevice {
public:
    enum class Command : std::uint8_t {
        RESET = 0x1E,
        READ_CONVERSION = 0x00,
        READ_C0 = 0xa0,
        READ_C1 = 0xa2,
        READ_C2 = 0xa4,
        READ_C3 = 0xa6,
        READ_C4 = 0xa8,
        READ_C5 = 0xaa,
        READ_C6 = 0xac,
        READ_CRC = 0xae,
        CONVERT_D2_OSR_1024 = 0x54,
        CONVERT_D1_OSR_1024 = 0x44,
    };
    enum class State : std::uint8_t {
        COLD = 5,
        COLD_WAIT = 6,
        UNINITIALISED = 7,
        RUNNING = 17,
        RESET_START = 27,
        RESET_WAIT = 28,
        CONVERSION_D1_START = 37,
        CONVERSION_D1_WAIT = 38,
        CONVERSION_D2_START = 47,
        CONVERSION_D2_WAIT = 48,
    };
    State state = State::COLD;
    std::uint32_t command_start_us = 0;
    std::uint8_t convert_out[3]{};
    bool prom_loaded = false;
    std::uint16_t loaded_prom[8]{};
    std::uint16_t conversion_time_osr_1024_us = 2280;
    std::uint32_t now_us_ = 0;
    float P_Pa = 101325;
    float Temp_C = 25;

    virtual void load_prom(std::uint16_t* p, std::uint8_t /*len*/) const = 0;
    virtual void convert(float P, float T, std::uint32_t& D1, std::uint32_t& D2) = 0;
    virtual void convert_forward(std::int32_t D1, std::int32_t D2, float& P, float& T) = 0;
    void set_now_us(std::uint32_t t) { now_us_ = t; }

    void reset() {
        prom_loaded = true;
        load_prom(loaded_prom, sizeof(loaded_prom));
    }

    void convert_D1() {
        float P = P_Pa < 0.1f ? 0.1f : P_Pa;
        std::uint32_t D1 = 0, D2 = 0;
        convert(P, Temp_C, D1, D2);
        convert_out[2] = static_cast<std::uint8_t>(D1 & 0xff);
        convert_out[1] = static_cast<std::uint8_t>((D1 >> 8) & 0xff);
        convert_out[0] = static_cast<std::uint8_t>((D1 >> 16) & 0xff);
    }
    void convert_D2() {
        float P = P_Pa < 0.1f ? 0.1f : P_Pa;
        std::uint32_t D1 = 0, D2 = 0;
        convert(P, Temp_C, D1, D2);
        convert_out[2] = static_cast<std::uint8_t>(D2 & 0xff);
        convert_out[1] = static_cast<std::uint8_t>((D2 >> 8) & 0xff);
        convert_out[0] = static_cast<std::uint8_t>((D2 >> 16) & 0xff);
    }

    void update(const Aircraft& aircraft) override {
        get_pressure_temperature_for_alt_amsl(aircraft_alt_m(aircraft), P_Pa, Temp_C);
        Temp_C = kelvin_to_c(Temp_C);
        const std::uint32_t now_us = now_us_ != 0 ? now_us_ : static_cast<std::uint32_t>(aircraft.time_us());
        switch (state) {
        case State::COLD:
            command_start_us = now_us;
            prom_loaded = false;
            state = State::COLD_WAIT;
            break;
        case State::COLD_WAIT:
            if (now_us - command_start_us >= 1) {
                state = State::UNINITIALISED;
            }
            break;
        case State::UNINITIALISED:
            break;
        case State::RESET_START:
            command_start_us = now_us;
            state = State::RESET_WAIT;
            break;
        case State::RESET_WAIT:
            if (now_us - command_start_us > 2000) {
                reset();
                state = State::RUNNING;
            }
            break;
        case State::CONVERSION_D1_START:
            command_start_us = now_us;
            convert_out[0] = convert_out[1] = convert_out[2] = 0;
            state = State::CONVERSION_D1_WAIT;
            break;
        case State::CONVERSION_D1_WAIT:
            if (now_us - command_start_us > conversion_time_osr_1024_us) {
                convert_D1();
                state = State::RUNNING;
            }
            break;
        case State::CONVERSION_D2_START:
            command_start_us = now_us;
            convert_out[0] = convert_out[1] = convert_out[2] = 0;
            state = State::CONVERSION_D2_WAIT;
            break;
        case State::CONVERSION_D2_WAIT:
            if (now_us - command_start_us > conversion_time_osr_1024_us) {
                convert_D2();
                state = State::RUNNING;
            }
            break;
        case State::RUNNING:
            break;
        }
    }

    int rdwr(I2cRdwr& data) override {
        I2cMsg& msg = data.msgs[0];
        if (msg.flags == I2C_M_RD) {
            return -1;
        }
        if (msg.len != 1) {
            return -1;
        }
        const auto cmd = static_cast<Command>(msg.buf[0]);
        if (state != State::RUNNING) {
            if (!(state == State::UNINITIALISED && cmd == Command::RESET)) {
                return -1;
            }
        }
        switch (cmd) {
        case Command::RESET:
            state = State::RESET_START;
            break;
        case Command::READ_C0:
        case Command::READ_C1:
        case Command::READ_C2:
        case Command::READ_C3:
        case Command::READ_C4:
        case Command::READ_C5:
        case Command::READ_C6:
        case Command::READ_CRC: {
            if (data.nmsgs < 2 || data.msgs[1].len != 2) {
                return -1;
            }
            const std::uint8_t addr = static_cast<std::uint8_t>((static_cast<unsigned>(cmd) - static_cast<unsigned>(Command::READ_C0)) / 2);
            const std::uint16_t val = loaded_prom[addr];
            data.msgs[1].buf[0] = static_cast<std::uint8_t>(val >> 8);
            data.msgs[1].buf[1] = static_cast<std::uint8_t>(val & 0xff);
            break;
        }
        case Command::CONVERT_D1_OSR_1024:
            state = State::CONVERSION_D1_START;
            break;
        case Command::CONVERT_D2_OSR_1024:
            state = State::CONVERSION_D2_START;
            break;
        case Command::READ_CONVERSION:
            if (data.nmsgs < 2 || data.msgs[1].len == 0) {
                return -1;
            }
            if (data.msgs[1].len != 3) {
                return -1;
            }
            data.msgs[1].buf[0] = convert_out[0];
            data.msgs[1].buf[1] = convert_out[1];
            data.msgs[1].buf[2] = convert_out[2];
            break;
        default:
            return -1;
        }
        return 0;
    }
};

class MS5611 : public MS5XXX {
public:
    const std::uint16_t prom[8]{0xFFFF, 40127, 36924, 23317, 23282, 33464, 28312, 0x0008};
    const std::uint8_t Qx_coeff[6]{15, 16, 8, 7, 8, 23};
    void load_prom(std::uint16_t* p, std::uint8_t /*len*/) const override {
        std::memcpy(p, prom, sizeof(prom));
    }
    void convert_forward(std::int32_t D1, std::int32_t D2, float& P_Pa, float& Temp_C) override {
        const float _D1 = static_cast<float>(D1);
        const float _D2 = static_cast<float>(D2);
        float dT = _D2 - (static_cast<std::uint32_t>(prom[5]) << 8);
        float TEMP = (dT * prom[6]) / 8388608;
        float OFF = prom[2] * 65536.0f + (prom[4] * dT) / 128;
        float SENS = prom[1] * 32768.0f + (prom[3] * dT) / 256;
        TEMP += 2000;
        if (TEMP < 2000) {
            float T2 = (dT * dT) / 0x80000000;
            float Aux = (TEMP - 2000.0f) * (TEMP - 2000.0f);
            float OFF2 = 2.5f * Aux;
            float SENS2 = 1.25f * Aux;
            if (TEMP < -1500) {
                OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
                SENS2 += (TEMP + 1500) * (TEMP + 1500) * 11.0f * 0.5f;
            }
            TEMP = TEMP - T2;
            OFF = OFF - OFF2;
            SENS = SENS - SENS2;
        }
        P_Pa = (_D1 * SENS / 2097152 - OFF) / 32768;
        Temp_C = TEMP * 0.01f;
    }
    void convert(float P_Pa, float Temp_C, std::uint32_t& D1, std::uint32_t& D2) override {
        const std::uint8_t Q1 = Qx_coeff[0], Q2 = Qx_coeff[1], Q3 = Qx_coeff[2], Q4 = Qx_coeff[3], Q5 = Qx_coeff[4],
                           Q6 = Qx_coeff[5];
        const float TEMP = Temp_C * 100;
        if (TEMP < 2000) {
            D2 = static_cast<std::uint32_t>(
                128 * (2 * static_cast<std::int64_t>(prom[5]) -
                       std::sqrt(static_cast<float>(static_cast<std::int64_t>(prom[6]) * static_cast<std::int64_t>(prom[6]) -
                                                   131072 * (TEMP - 2000))) +
                       static_cast<std::int64_t>(prom[6])));
            const float dT = static_cast<float>(D2) - (static_cast<std::int64_t>(prom[5]) << Q5);
            float TEMP_forward = 2000 + (dT * static_cast<std::int64_t>(prom[6])) / (1L << Q6);
            float OFF = static_cast<std::int64_t>(prom[2]) * (1L << Q2) + (static_cast<std::int64_t>(prom[4]) * dT) / (1L << Q4);
            float SENS = static_cast<std::int64_t>(prom[1]) * (1L << Q1) + (static_cast<std::int64_t>(prom[3]) * dT) / (1L << Q3);
            const float Aux = (TEMP_forward - 2000) * (TEMP_forward - 2000);
            float OFF2 = 2.5f * Aux;
            float SENS2 = 1.25f * Aux;
            if (TEMP < -1500) {
                OFF2 += 7 * (TEMP_forward + 1500) * (TEMP_forward + 1500);
                SENS2 += (TEMP_forward + 1500) * (TEMP_forward + 1500) * 11.0f * 0.5f;
            }
            OFF = OFF - OFF2;
            SENS = SENS - SENS2;
            D1 = static_cast<std::uint32_t>(((P_Pa * static_cast<float>(1L << 15) + OFF) * static_cast<float>(1L << 21)) / SENS);
        } else {
            const float dT = (TEMP - 2000) * (1L << Q6) / static_cast<std::int64_t>(prom[6]);
            const float OFF = static_cast<std::int64_t>(prom[2]) * (1L << Q2) + (static_cast<std::int64_t>(prom[4]) * dT) / (1L << Q4);
            const float SENS = static_cast<std::int64_t>(prom[1]) * (1L << Q1) + (static_cast<std::int64_t>(prom[3]) * dT) / (1L << Q3);
            D1 = static_cast<std::uint32_t>(((P_Pa * static_cast<float>(1L << 15) + OFF) * static_cast<float>(1L << 21)) / SENS);
            D2 = static_cast<std::uint32_t>(dT + (static_cast<std::int64_t>(prom[5]) << Q5));
        }
    }
};

class MS5525 : public MS5XXX {
public:
    const std::uint16_t prom[8]{0xFFFF, 36402, 39473, 40393, 29523, 29854, 21917, 0x000c};
    const std::uint8_t Qx_coeff[6]{15, 17, 7, 5, 7, 21};
    float airspeed_raw_pressure = 0;
    void load_prom(std::uint16_t* p, std::uint8_t /*len*/) const override { std::memcpy(p, prom, sizeof(prom)); }
    void convert_forward(std::int32_t D1, std::int32_t D2, float& P_Pa, float& Temp_C) override {
        const std::uint8_t Q1 = Qx_coeff[0], Q2 = Qx_coeff[1], Q3 = Qx_coeff[2], Q4 = Qx_coeff[3], Q5 = Qx_coeff[4],
                           Q6 = Qx_coeff[5];
        float dT = static_cast<float>(D2) - static_cast<std::int64_t>(prom[5]) * (1L << Q5);
        float TEMP = 2000 + (dT * static_cast<std::int64_t>(prom[6])) / (1L << Q6);
        float OFF = static_cast<std::int64_t>(prom[2]) * (1L << Q2) + (static_cast<std::int64_t>(prom[4]) * dT) / (1L << Q4);
        float SENS = static_cast<std::int64_t>(prom[1]) * (1L << Q1) + (static_cast<std::int64_t>(prom[3]) * dT) / (1L << Q3);
        float P = (static_cast<float>(D1) * SENS / (1L << 21) - OFF) / (1L << 15);
        const float PSI_to_Pa = 6894.757f;
        P_Pa = PSI_to_Pa * 1.0e-4f * P;
        Temp_C = TEMP * 0.01f;
    }
    void convert(float P_Pa, float Temp_C, std::uint32_t& D1, std::uint32_t& D2) override {
        const std::uint8_t Q1 = Qx_coeff[0], Q2 = Qx_coeff[1], Q3 = Qx_coeff[2], Q4 = Qx_coeff[3], Q5 = Qx_coeff[4],
                           Q6 = Qx_coeff[5];
        const float TEMP = Temp_C * 100.0f;
        const float dT = ((TEMP - 2000.0f) * (1L << Q6)) / static_cast<std::int64_t>(prom[6]);
        const float PSI_to_Pa = 6894.757f;
        const float P = P_Pa / (PSI_to_Pa * 1.0e-4f);
        const float OFF = static_cast<std::int64_t>(prom[2]) * (1L << Q2) + (static_cast<std::int64_t>(prom[4]) * dT) / (1L << Q4);
        const float SENS = static_cast<std::int64_t>(prom[1]) * (1L << Q1) + (static_cast<std::int64_t>(prom[3]) * dT) / (1L << Q3);
        D1 = static_cast<std::uint32_t>(((P * (1L << 15) + OFF) * (1L << 21)) / SENS);
        D2 = static_cast<std::uint32_t>(dT + static_cast<std::int64_t>(prom[5]) * (1L << Q5));
    }
    void update(const Aircraft& aircraft) override {
        Temp_C = sim_board_temp_c(aircraft);
        P_Pa = airspeed_raw_pressure;
        MS5XXX::update(aircraft);
        P_Pa = airspeed_raw_pressure;
        Temp_C = sim_board_temp_c(aircraft);
    }
};


// ---- INA3221 ----
class INA3221 : public I2CDevice {
public:
    static constexpr std::uint16_t MANUFACTURER_ID = 0b0101010001001001;
    static constexpr std::uint16_t DIE_ID = 0b0011001000100000;
    union {
        std::uint16_t word[256];
        struct {
            std::uint16_t configuration;
            std::uint16_t Channel_1_Shunt_Voltage;
            std::uint16_t Channel_1_Bus_Voltage;
            std::uint16_t Channel_2_Shunt_Voltage;
            std::uint16_t Channel_2_Bus_Voltage;
            std::uint16_t Channel_3_Shunt_Voltage;
            std::uint16_t Channel_3_Bus_Voltage;
        } byname;
    } registers{};
    fwcpp::Bitmask<256> writable_registers;

    INA3221() {
        writable_registers.set(0);
        writable_registers.set(7);
        writable_registers.set(8);
        writable_registers.set(9);
        writable_registers.set(10);
        writable_registers.set(11);
        writable_registers.set(12);
        writable_registers.set(14);
        writable_registers.set(15);
        writable_registers.set(16);
        writable_registers.set(254);
        writable_registers.set(255);
        reset();
    }
    void reset() {
        registers.byname.configuration = 0x7127;
        registers.word[254] = MANUFACTURER_ID;
        registers.word[255] = DIE_ID;
    }
    static std::uint16_t convert_voltage(float voltage_v) {
        float volt_counts = (voltage_v / 8e-3f) * 8;
        volt_counts = math::constrain_value(volt_counts, -32768.0f, 32767.0f);
        return static_cast<std::uint16_t>(static_cast<std::int16_t>(volt_counts));
    }
    static std::uint16_t convert_current(float current_a) {
        const float shunt_voltage = current_a * 0.001f;
        float shunt_counts = (shunt_voltage / 40e-6f) * 8;
        shunt_counts = math::constrain_value(shunt_counts, -32768.0f, 32767.0f);
        return static_cast<std::uint16_t>(static_cast<std::int16_t>(shunt_counts));
    }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 2) {
            if (data.msgs[0].flags != 0 || data.msgs[1].flags != I2C_M_RD) {
                return -1;
            }
            const std::uint8_t reg_addr = data.msgs[0].buf[0];
            const std::uint16_t register_value = registers.word[reg_addr];
            data.msgs[1].buf[0] = static_cast<std::uint8_t>(register_value >> 8);
            data.msgs[1].buf[1] = static_cast<std::uint8_t>(register_value & 0xFF);
            data.msgs[1].len = 2;
            return 0;
        }
        if (data.nmsgs == 1) {
            if (data.msgs[0].flags != 0) {
                return -1;
            }
            const std::uint8_t reg_addr = data.msgs[0].buf[0];
            if (!writable_registers.get(reg_addr)) {
                return -1;
            }
            registers.word[reg_addr] =
                static_cast<std::uint16_t>(data.msgs[0].buf[2] << 8 | data.msgs[0].buf[1]);
            return 0;
        }
        return -1;
    }
    void update(const Aircraft& aircraft) override {
        const std::uint16_t mode = registers.byname.configuration & 0x7;
        if (mode == 0b000 || mode == 0b100) {
            return;
        }
        const bool update_shunt = mode & 0b001;
        const bool update_bus = mode & 0b010;
        if (update_bus) {
            registers.byname.Channel_1_Bus_Voltage = convert_voltage(25);
            registers.byname.Channel_2_Bus_Voltage = convert_voltage(aircraft.get_battery_voltage());
            registers.byname.Channel_3_Bus_Voltage = convert_voltage(3.14159f);
        }
        if (update_shunt) {
            registers.byname.Channel_1_Shunt_Voltage = convert_current(160);
            registers.byname.Channel_2_Shunt_Voltage = convert_current(aircraft.get_battery_current());
            registers.byname.Channel_3_Shunt_Voltage = convert_current(2.71828f);
        }
    }
};

// ---- SMBus battery monitors ----
struct SMBusBattDevReg {
    static constexpr std::uint8_t TEMP = 0x08;
    static constexpr std::uint8_t VOLTAGE = 0x09;
    static constexpr std::uint8_t CURRENT = 0x0A;
    static constexpr std::uint8_t REMAINING_CAPACITY = 0x0F;
    static constexpr std::uint8_t FULL_CHARGE_CAPACITY = 0x10;
    static constexpr std::uint8_t CYCLE_COUNT = 0x17;
    static constexpr std::uint8_t DESIGN_CAPACITY = 0x18;
    static constexpr std::uint8_t DESIGN_VOLTAGE = 0x19;
    static constexpr std::uint8_t SPECIFICATION_INFO = 0x1A;
    static constexpr std::uint8_t MANUFACTURE_DATE = 0x1B;
    static constexpr std::uint8_t SERIAL = 0x1C;
    static constexpr std::uint8_t MANUFACTURE_NAME = 0x20;
    static constexpr std::uint8_t DEVICE_NAME = 0x21;
    static constexpr std::uint8_t DEVICE_CHEMISTRY = 0x22;
    static constexpr std::uint8_t MANUFACTURE_DATA = 0x23;
};
struct SMBusBattGenericDevReg : SMBusBattDevReg {
    static constexpr std::uint8_t CELL1 = 0x3f;
    static constexpr std::uint8_t CELL2 = 0x3e;
    static constexpr std::uint8_t CELL3 = 0x3d;
    static constexpr std::uint8_t CELL4 = 0x3c;
    static constexpr std::uint8_t CELL5 = 0x3b;
    static constexpr std::uint8_t CELL6 = 0x3a;
    static constexpr std::uint8_t CELL7 = 0x39;
    static constexpr std::uint8_t CELL8 = 0x38;
    static constexpr std::uint8_t CELL9 = 0x37;
    static constexpr std::uint8_t CELL10 = 0x36;
    static constexpr std::uint8_t CELL11 = 0x35;
    static constexpr std::uint8_t CELL12 = 0x34;
    static constexpr std::uint8_t CELL13 = 0x33;
    static constexpr std::uint8_t CELL14 = 0x32;
};

class SMBusDevice : public I2CDevice, public I2CRegisters_16Bit {
public:
    const char* blockname[256]{};
    std::string values[256];
    void add_block(const char* name, std::uint8_t reg, RegMode mode) {
        blockname[reg] = name;
        if (mode == RegMode::RDONLY || mode == RegMode::RDWR) {
            readable_registers.set(reg);
        }
    }
    void set_block(std::uint8_t block, const char* value) { values[block] = value ? value : ""; }
    int rdwr(I2cRdwr& data) override {
        const std::uint8_t addr = data.msgs[0].buf[0];
        if (blockname[addr] == nullptr) {
            return I2CRegisters_16Bit::rdwr(data);
        }
        if (data.nmsgs == 2) {
            if (data.msgs[0].flags != 0 || data.msgs[1].flags != I2C_M_RD) {
                return -1;
            }
            const auto& v = values[addr];
            data.msgs[1].buf[0] = static_cast<std::uint8_t>(v.size());
            const std::uint8_t to_copy = static_cast<std::uint8_t>(
                std::min<std::size_t>(data.msgs[1].len > 0 ? data.msgs[1].len - 1 : 0, v.size()));
            std::memcpy(&data.msgs[1].buf[1], v.data(), to_copy);
            data.msgs[1].len = static_cast<std::uint16_t>(to_copy + 1);
            return 0;
        }
        return -1;
    }
};

class SIM_BattMonitor_SMBus : public SMBusDevice {
public:
    std::uint32_t last_update_ms = 0;
    SIM_BattMonitor_SMBus() {
        add_register("Temperature", SMBusBattDevReg::TEMP, RegMode::RDONLY);
        add_register("Voltage", SMBusBattDevReg::VOLTAGE, RegMode::RDONLY);
        add_register("Current", SMBusBattDevReg::CURRENT, RegMode::RDONLY);
        add_register("Remaining Capacity", SMBusBattDevReg::REMAINING_CAPACITY, RegMode::RDONLY);
        add_register("Full Charge Capacity", SMBusBattDevReg::FULL_CHARGE_CAPACITY, RegMode::RDONLY);
        add_register("Cycle_Count", SMBusBattDevReg::CYCLE_COUNT, RegMode::RDONLY);
        add_register("Design Charge Capacity", SMBusBattDevReg::DESIGN_CAPACITY, RegMode::RDONLY);
        add_register("Design Maximum Voltage", SMBusBattDevReg::DESIGN_VOLTAGE, RegMode::RDONLY);
        add_register("Specification Info", SMBusBattDevReg::SPECIFICATION_INFO, RegMode::RDONLY);
        add_register("Manufacture Date", SMBusBattDevReg::MANUFACTURE_DATE, RegMode::RDONLY);
        add_register("Serial", SMBusBattDevReg::SERIAL, RegMode::RDONLY);
        add_block("Manufacture Name", SMBusBattDevReg::MANUFACTURE_NAME, RegMode::RDONLY);
        add_block("Device Name", SMBusBattDevReg::DEVICE_NAME, RegMode::RDONLY);
        add_block("Device Chemistry", SMBusBattDevReg::DEVICE_CHEMISTRY, RegMode::RDONLY);
        add_register("Manufacture Data", SMBusBattDevReg::MANUFACTURE_DATA, RegMode::RDONLY);
        set_register(SMBusBattDevReg::TEMP, static_cast<std::uint16_t>(c_to_kelvin(15) * 10));
        set_register(SMBusBattDevReg::DESIGN_VOLTAGE, static_cast<std::uint16_t>(50400U));
        set_register(SMBusBattDevReg::REMAINING_CAPACITY, static_cast<std::uint16_t>(42042U));
        set_register(SMBusBattDevReg::FULL_CHARGE_CAPACITY, static_cast<std::uint16_t>(45000U));
        set_register(SMBusBattDevReg::DESIGN_CAPACITY, static_cast<std::uint16_t>(52500U));
        set_register(SMBusBattDevReg::CYCLE_COUNT, static_cast<std::uint16_t>(42U));
        set_register(SMBusBattDevReg::SPECIFICATION_INFO, static_cast<std::uint16_t>(0x0001));
        set_register(SMBusBattDevReg::SERIAL, static_cast<std::uint16_t>(12345));
        set_block(SMBusBattDevReg::MANUFACTURE_NAME, "ArduPilot");
        set_block(SMBusBattDevReg::DEVICE_NAME, "SITLBatMon_V0.99");
        set_block(SMBusBattDevReg::DEVICE_CHEMISTRY, "LION");
        set_register(SMBusBattDevReg::MANUFACTURE_DATE, static_cast<std::uint16_t>(((2021 - 1980) << 9) + (4 << 5) + 24));
    }
    void update(const Aircraft& aircraft) override {
        const std::uint32_t now = static_cast<std::uint32_t>(aircraft.time_us() / 1000);
        if (now - last_update_ms > 100) {
            const float millivolts = aircraft.get_battery_voltage() * 1000.0f;
            set_register(SMBusBattDevReg::VOLTAGE, static_cast<std::uint16_t>(millivolts));
            const float current = math::constrain_value(aircraft.get_battery_current() * -1000.0f, -32768.0f, 32767.0f);
            set_register(SMBusBattDevReg::CURRENT, static_cast<std::uint16_t>(static_cast<std::int16_t>(current)));
            last_update_ms = now;
        }
    }
};

class SIM_BattMonitor_SMBus_Generic : public SIM_BattMonitor_SMBus {
public:
    static constexpr std::uint8_t kCells[] = {
        SMBusBattGenericDevReg::CELL1,  SMBusBattGenericDevReg::CELL2,  SMBusBattGenericDevReg::CELL3,
        SMBusBattGenericDevReg::CELL4,  SMBusBattGenericDevReg::CELL5,  SMBusBattGenericDevReg::CELL6,
        SMBusBattGenericDevReg::CELL7,  SMBusBattGenericDevReg::CELL8,  SMBusBattGenericDevReg::CELL9,
        SMBusBattGenericDevReg::CELL10, SMBusBattGenericDevReg::CELL11, SMBusBattGenericDevReg::CELL12,
        SMBusBattGenericDevReg::CELL13, SMBusBattGenericDevReg::CELL14};
    SIM_BattMonitor_SMBus_Generic() { set_block(SMBusBattDevReg::MANUFACTURE_NAME, "sitl_smbus_generic"); }
    virtual std::uint8_t cellcount() const { return 12; }
    virtual std::uint8_t connected_cells() const { return 3; }
    void init() override {
        for (std::uint8_t i = 0; i < cellcount(); i++) {
            add_register("Cell", kCells[i], RegMode::RDONLY);
        }
    }
    void update(const Aircraft& aircraft) override {
        SIM_BattMonitor_SMBus::update(aircraft);
        const float millivolts = aircraft.get_battery_voltage() * 1000.0f;
        const std::uint8_t n = connected_cells();
        const float volts_per_cell = n ? millivolts / float(n) : 0;
        const std::uint16_t value_even = millivolts > 0 ? static_cast<std::uint16_t>(volts_per_cell - 100.0f) : 0xFFFF;
        const std::uint16_t value_odd = millivolts > 0 ? static_cast<std::uint16_t>(volts_per_cell + 100.0f) : 0xFFFF;
        for (std::uint8_t i = 0; i < n; i++) {
            set_register(kCells[i], (i % 2 == 0) ? value_odd : value_even);
        }
    }
};

class Maxell : public SIM_BattMonitor_SMBus_Generic {
public:
    Maxell() {
        set_block(SMBusBattDevReg::MANUFACTURE_NAME, "Hitachi maxell");
        set_block(SMBusBattDevReg::DEVICE_NAME, "SITL_maxell");
        set_register(SMBusBattDevReg::SERIAL, static_cast<std::uint16_t>(37));
    }
};

class Rotoye : public SIM_BattMonitor_SMBus_Generic {
public:
    static constexpr std::uint8_t TEMP_EXT = 0x07;
    std::uint32_t last_temperature_update_ms = 0;
    Rotoye() {
        add_register("External Temperature", TEMP_EXT, RegMode::RDONLY);
        set_register(SMBusBattDevReg::SERIAL, static_cast<std::uint16_t>(39));
        set_block(SMBusBattDevReg::MANUFACTURE_NAME, "Rotoye");
        set_block(SMBusBattDevReg::DEVICE_NAME, "SITL_BatMon v4.03");
        set_register(SMBusBattDevReg::SERIAL, static_cast<std::uint16_t>(278));
    }
    void update(const Aircraft& aircraft) override {
        SIM_BattMonitor_SMBus_Generic::update(aircraft);
        const std::uint32_t now = static_cast<std::uint32_t>(aircraft.time_us() / 1000);
        if (now - last_temperature_update_ms > 1000) {
            last_temperature_update_ms = now;
            const std::uint16_t outside = get_reg_value(SMBusBattDevReg::TEMP);
            set_register(TEMP_EXT, static_cast<std::uint16_t>(outside + 100));
        }
    }
};

// ---- rangefinders / airspeed / IMU / temps ----
class TeraRangerI2C : public I2CDevice, public I2CRegisters_8Bit {
public:
    std::uint32_t reading_start_us = 0;
    float rangefinder_range = 0;
    std::uint32_t now_us_ = 0;
    void set_now_us(std::uint32_t t) { now_us_ = t; }
    void update(const Aircraft& aircraft) override { rangefinder_range = aircraft.hagl(); }
    int rdwr(I2cRdwr& data) override {
        if (data.msgs[0].flags == I2C_M_RD) {
            if (reading_start_us == 0) {
                return -1;
            }
            const std::uint32_t now_us = now_us_ ? now_us_ : 1;
            if (now_us - reading_start_us < 500) {
                return -1;
            }
            const std::uint16_t reading = static_cast<std::uint16_t>(std::min(rangefinder_range * 1000.0f, 65535.0f));
            data.msgs[0].buf[0] = static_cast<std::uint8_t>(reading >> 8);
            data.msgs[0].buf[1] = static_cast<std::uint8_t>(reading & 0xff);
            data.msgs[0].buf[2] = crc_crc8(data.msgs[0].buf, 2);
            reading_start_us = 0;
            return 0;
        }
        if (data.msgs[0].flags == 0 && data.msgs[0].buf[0] == 0) {
            reading_start_us = now_us_ ? now_us_ : 1;
            return 0;
        }
        return I2CRegisters_8Bit::rdwr(data);
    }
};

class LightWareI2C_Legacy16Bit : public I2CDevice, public I2CRegisters_16Bit {
public:
    void init() override {
        add_register("RANGE", 0, RegMode::RDONLY);
        set_register(0, static_cast<std::uint16_t>(0));
        add_register("LOST_SIGNAL_TIMEOUT_READ", 1, RegMode::RDONLY);
        set_register(1, static_cast<std::uint16_t>(0));
        add_register("LOST_SIGNAL_TIMEOUT_WRITE", 2, RegMode::WRONLY);
        set_register(2, static_cast<std::uint16_t>(0));
    }
    void update(const Aircraft& aircraft) override {
        set_register(0, static_cast<std::uint16_t>(aircraft.hagl() * 100.0f));
    }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 1 && data.msgs[0].len > 6) {
            return -1;
        }
        if (data.nmsgs >= 1 && data.msgs[0].buf && data.msgs[0].buf[0] == 0x3f) {
            return -1;
        }
        return I2CRegisters_16Bit::rdwr(data);
    }
};

class LightWareGRF_I2C : public I2CDevice {
public:
    std::uint32_t distance_output_mask = (1U << 0) | (1U << 2);
    std::uint8_t update_rate_hz = 5;
    float range_m = 0;
    std::uint8_t last_register = 0;
    void update(const Aircraft& aircraft) override { range_m = aircraft.hagl(); }
    std::uint8_t apply_write(const std::uint8_t* buf, std::uint16_t len) {
        if (len == 0) {
            return last_register;
        }
        last_register = buf[0];
        if (len == 1) {
            return last_register;
        }
        if (last_register == 74 && len >= 5) {
            const std::uint32_t rate = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);
            update_rate_hz = rate > 0 ? static_cast<std::uint8_t>(rate) : 1;
        }
        if (last_register == 27 && len >= 5) {
            distance_output_mask = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);
        }
        return last_register;
    }
    std::uint16_t read_register(std::uint8_t reg, std::uint8_t* buf, std::uint16_t buf_max) {
        std::memset(buf, 0, buf_max);
        if (reg == 0) {
            const char name[] = "GRF250";
            std::memcpy(buf, name, std::min<std::size_t>(sizeof(name), buf_max));
            return buf_max;
        }
        if (reg == 44) {
            float dist_m = range_m;
            std::uint32_t strength = 100;
            if (dist_m < 0.2f) {
                dist_m = 0.2f;
                strength = 0;
            }
            if (dist_m > 500.0f) {
                dist_m = 500.0f;
                strength = 0;
            }
            const std::uint32_t raw = static_cast<std::uint32_t>(dist_m * 100.0f) / 10U;
            buf[0] = static_cast<std::uint8_t>(raw);
            buf[1] = static_cast<std::uint8_t>(raw >> 8);
            buf[2] = static_cast<std::uint8_t>(raw >> 16);
            buf[3] = static_cast<std::uint8_t>(raw >> 24);
            buf[4] = static_cast<std::uint8_t>(strength);
            buf[5] = static_cast<std::uint8_t>(strength >> 8);
            buf[6] = static_cast<std::uint8_t>(strength >> 16);
            buf[7] = static_cast<std::uint8_t>(strength >> 24);
            return buf_max;
        }
        return buf_max;
    }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 2 && data.msgs[0].flags == 0 && data.msgs[1].flags == I2C_M_RD) {
            const std::uint8_t reg = apply_write(data.msgs[0].buf, data.msgs[0].len);
            read_register(reg, data.msgs[1].buf, data.msgs[1].len);
            return 0;
        }
        if (data.nmsgs == 1 && data.msgs[0].flags == 0) {
            apply_write(data.msgs[0].buf, data.msgs[0].len);
            return 0;
        }
        if (data.nmsgs == 1 && data.msgs[0].flags == I2C_M_RD) {
            read_register(last_register, data.msgs[0].buf, data.msgs[0].len);
            return 0;
        }
        return -1;
    }
};

class Airspeed_DLVR : public I2CDevice {
public:
    float pressure = 0;
    float temperature = 25;
    std::uint32_t last_update_ms = 0;
    std::uint32_t last_sent_ms = 0;
    void update(const Aircraft& aircraft) override {
        const std::uint32_t now_ms = static_cast<std::uint32_t>(aircraft.time_us() / 1000);
        if (now_ms - last_update_ms < 50) {
            return;
        }
        last_update_ms = now_ms;
        pressure = 0.5f * kSslAirDensity * aircraft.airspeed_pitot * aircraft.airspeed_pitot;
        temperature = sim_board_temp_c(aircraft);
    }
    int rdwr(I2cRdwr& data) override {
        I2cMsg& msg = data.msgs[0];
        if (msg.flags != I2C_M_RD || msg.len != 4) {
            return -1;
        }
        std::uint8_t status = 0;
        if (last_sent_ms == last_update_ms) {
            status |= 0b10;
        }
        last_sent_ms = last_update_ms;
        constexpr float INCH_OF_H2O_TO_PASCAL = 249.0889f;
        const float press_h2o = pressure * (1.0f / INCH_OF_H2O_TO_PASCAL);
        const std::uint32_t pressure_raw =
            static_cast<std::uint32_t>(((press_h2o / (1.25f * 2.0f * 5)) * 16384.0f) + 8192.0f);
        const std::uint32_t temp_raw = static_cast<std::uint32_t>((temperature + 50.0f) * (2047.0f / 200.0f));
        const std::uint32_t packed = (status << 30) | ((pressure_raw & 0x3fff) << 16) | ((temp_raw & 0x7ff) << 5);
        msg.buf[0] = static_cast<std::uint8_t>((packed >> 24) & 0xff);
        msg.buf[1] = static_cast<std::uint8_t>((packed >> 16) & 0xff);
        msg.buf[2] = static_cast<std::uint8_t>((packed >> 8) & 0xff);
        msg.buf[3] = static_cast<std::uint8_t>(packed & 0xff);
        return 0;
    }
};

class Benewake_TFMiniPlus : public I2CDevice {
public:
    float range = 0;
    bool data_output_enabled = false;
    bool output_format_is_cm = true;
    void update(const Aircraft& aircraft) override { range = aircraft.hagl(); }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 1 && data.msgs[0].flags == 0) {
            const std::uint8_t cmd = data.msgs[0].buf[0];
            if (cmd == 0x07) {
                data_output_enabled = true;
            }
            if (cmd == 0x05) {
                output_format_is_cm = true;
            }
            return 0;
        }
        if (data.nmsgs >= 1 && data.msgs[0].flags == I2C_M_RD && data_output_enabled) {
            const std::uint16_t dist = static_cast<std::uint16_t>(range * (output_format_is_cm ? 100.0f : 10.0f));
            data.msgs[0].buf[0] = 0x59;
            data.msgs[0].buf[1] = 0x59;
            data.msgs[0].buf[2] = static_cast<std::uint8_t>(dist & 0xff);
            data.msgs[0].buf[3] = static_cast<std::uint8_t>(dist >> 8);
            return 0;
        }
        return -1;
    }
};

class TFS20L : public I2CDevice {
public:
    float rangefinder_range = 0;
    std::uint16_t strength = 0;
    std::uint8_t reg = 0;
    std::uint32_t last_update_ms = 0;
    void update(const Aircraft& aircraft) override {
        const std::uint32_t now_ms = static_cast<std::uint32_t>(aircraft.time_us() / 1000);
        if (now_ms - last_update_ms < 50) {
            return;
        }
        last_update_ms = now_ms;
        rangefinder_range = aircraft.hagl();
        if (rangefinder_range <= 0 || rangefinder_range > 20.0f) {
            rangefinder_range = 0;
            strength = 0;
            return;
        }
        const std::uint16_t range_cm = static_cast<std::uint16_t>(rangefinder_range * 100.0f);
        if (range_cm < 1) {
            rangefinder_range = 0;
            strength = 0;
        } else {
            const std::uint32_t str = 65535 - (range_cm * 20);
            strength = static_cast<std::uint16_t>(str < 100 ? 100 : (str > 65535 ? 65535 : str));
        }
    }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 1) {
            reg = data.msgs[0].buf[0];
            return 0;
        }
        if (data.nmsgs != 2 || !(data.msgs[1].flags & I2C_M_RD)) {
            return -1;
        }
        const std::uint16_t dist_cm = static_cast<std::uint16_t>(rangefinder_range * 100.0f);
        switch (reg) {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03: {
            const std::uint8_t count = static_cast<std::uint8_t>(data.msgs[1].len);
            for (std::uint8_t i = 0; i < count; i++) {
                const std::uint8_t curr = static_cast<std::uint8_t>(reg + i);
                if (curr == 0x00) {
                    data.msgs[1].buf[i] = static_cast<std::uint8_t>(dist_cm & 0xFF);
                } else if (curr == 0x01) {
                    data.msgs[1].buf[i] = static_cast<std::uint8_t>((dist_cm >> 8) & 0xFF);
                } else if (curr == 0x02) {
                    data.msgs[1].buf[i] = static_cast<std::uint8_t>(strength & 0xFF);
                } else if (curr == 0x03) {
                    data.msgs[1].buf[i] = static_cast<std::uint8_t>((strength >> 8) & 0xFF);
                }
            }
            return 0;
        }
        case 0x0C:
            data.msgs[1].buf[0] = 1;
            return 0;
        }
        return -1;
    }
};

class AS5600 : public I2CDevice, public I2CRegisters_8Bit {
public:
    void init() override {
        add_register("ZMCO", 0x00, RegMode::RDONLY);
        set_register(0x00, static_cast<std::uint8_t>(1));
        add_register("RAH", 0x0C, RegMode::RDONLY);
        set_register(0x0C, static_cast<std::uint8_t>(0));
        add_register("RAL", 0x0D, RegMode::RDONLY);
        set_register(0x0D, static_cast<std::uint8_t>(0));
    }
    void update(const Aircraft& aircraft) override {
        float r, p, y;
        aircraft.get_dcm().to_euler(&r, &p, &y);
        const std::uint16_t pitch_180 = static_cast<std::uint16_t>(math::degrees(p) + 90);
        set_register(0x0C, static_cast<std::uint8_t>(pitch_180 >> 8));
        set_register(0x0D, static_cast<std::uint8_t>(pitch_180 & 0xff));
    }
    int rdwr(I2cRdwr& data) override { return I2CRegisters_8Bit::rdwr(data); }
};

class ICM40609 : public I2CDevice, public I2CRegisters_8Bit {
public:
    void init() override {
        add_register("WHOAMI", 0x75, RegMode::RDONLY);
        set_register(0x75, static_cast<std::uint8_t>(0x3b));
        add_register("PWR_MGMT0", 0x4E, RegMode::RDWR);
        add_register("GYRO_DATA_X1", 0x25, RegMode::RDONLY);
        add_register("ACCEL_DATA_X1", 0x1F, RegMode::RDONLY);
    }
    void update(const Aircraft& aircraft) override {
        const float as = kGravityMss / 1024.0f;
        auto pack = [](float v, float scale) -> std::int16_t {
            return static_cast<std::int16_t>(math::constrain_value(v / scale, -32768.0f, 32767.0f));
        };
        const std::int16_t ax = pack(aircraft.accel_body.x, as);
        set_register(0x1F, static_cast<std::uint8_t>(ax >> 8));
        set_register(0x20, static_cast<std::uint8_t>(ax & 0xff));
        (void)aircraft;
    }
    int rdwr(I2cRdwr& data) override { return I2CRegisters_8Bit::rdwr(data); }
};

class SHT3x : public I2CDevice {
public:
    enum class State { UNKNOWN, RESET, CLEAR_STATUS, IDLE, MEASURING, MEASURED };
    State state = State::UNKNOWN;
    std::uint32_t state_start_ms = 0;
    float temperature = 25;
    float humidity = 33.3f;
    std::uint32_t now_ms_ = 0;
    void set_now_ms(std::uint32_t t) { now_ms_ = t; }
    void set_state(State s) {
        state = s;
        state_start_ms = now_ms_;
    }
    void update(const Aircraft& aircraft) override {
        now_ms_ = static_cast<std::uint32_t>(aircraft.time_us() / 1000);
        if (state == State::MEASURING && now_ms_ - state_start_ms > 16) {
            temperature = sim_board_temp_c(aircraft) + 25;
            humidity = 33.3f;
            set_state(State::MEASURED);
        }
        if (state == State::RESET && now_ms_ - state_start_ms > 2) {
            set_state(State::IDLE);
        }
    }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 1 && data.msgs[0].flags == 0) {
            const std::uint16_t cmd = static_cast<std::uint16_t>(data.msgs[0].buf[0] << 8 | data.msgs[0].buf[1]);
            if (cmd == 0x30A2) {
                set_state(State::RESET);
            }
            if (cmd == 0x2400) {
                set_state(State::MEASURING);
            }
            return 0;
        }
        if (data.nmsgs == 1 && data.msgs[0].flags == I2C_M_RD && state == State::MEASURED) {
            const std::uint16_t t_deconverted = static_cast<std::uint16_t>(((temperature + 45) / 175) * 65535);
            const std::uint16_t h_deconverted = static_cast<std::uint16_t>((humidity * 0.01f) * 65535);
            data.msgs[0].buf[0] = static_cast<std::uint8_t>(t_deconverted >> 8);
            data.msgs[0].buf[1] = static_cast<std::uint8_t>(t_deconverted & 0xff);
            data.msgs[0].buf[2] = crc8_generic(&data.msgs[0].buf[0], 2, 0x31, 0xff);
            data.msgs[0].buf[3] = static_cast<std::uint8_t>(h_deconverted >> 8);
            data.msgs[0].buf[4] = static_cast<std::uint8_t>(h_deconverted & 0xff);
            data.msgs[0].buf[5] = crc8_generic(&data.msgs[0].buf[3], 2, 0x31, 0xff);
            return 0;
        }
        return -1;
    }
};

class TSYS01 : public I2CDevice {
public:
    enum class State { RESET = 23, READ_PROM = 24, IDLE = 25, CONVERTING = 26, CONVERTED = 27 };
    State state = State::RESET;
    std::uint32_t adc = 0;
    float last_temperature = -1000;
    std::uint32_t now_ms_ = 0;
    static constexpr std::int32_t _k[6]{0, 40781, 32791, 36016, 24926, 28446};
    void update(const Aircraft& aircraft) override {
        now_ms_ = static_cast<std::uint32_t>(aircraft.time_us() / 1000);
        if (state == State::CONVERTING) {
            last_temperature = sim_board_temp_c(aircraft) + 25;
            adc = 800000;
            state = State::CONVERTED;
        }
    }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 1 && data.msgs[0].flags == 0) {
            const std::uint8_t cmd = data.msgs[0].buf[0];
            if (cmd == 0x1E) {
                state = State::RESET;
            }
            if (cmd == 0x40) {
                state = State::CONVERTING;
            }
            return 0;
        }
        if (data.nmsgs == 2 && (data.msgs[0].buf[0] & 0xA0) == 0xA0) {
            const std::uint8_t idx = static_cast<std::uint8_t>((data.msgs[0].buf[0] - 0xA0) / 2);
            const std::uint16_t k = idx < 6 ? static_cast<std::uint16_t>(_k[idx]) : 0;
            data.msgs[1].buf[0] = static_cast<std::uint8_t>(k >> 8);
            data.msgs[1].buf[1] = static_cast<std::uint8_t>(k & 0xff);
            return 0;
        }
        if (data.nmsgs >= 1 && data.msgs[0].flags == I2C_M_RD && state == State::CONVERTED) {
            data.msgs[0].buf[0] = static_cast<std::uint8_t>((adc >> 16) & 0xff);
            data.msgs[0].buf[1] = static_cast<std::uint8_t>((adc >> 8) & 0xff);
            data.msgs[0].buf[2] = static_cast<std::uint8_t>(adc & 0xff);
            return 0;
        }
        return -1;
    }
};

class TSYS03 : public I2CDevice {
public:
    float temperature = 25;
    bool converted = false;
    void update(const Aircraft& aircraft) override { temperature = sim_board_temp_c(aircraft) + 25; }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 1 && data.msgs[0].flags == 0) {
            if (data.msgs[0].buf[0] == 0x46) {
                converted = true;
            }
            return 0;
        }
        if (converted && data.nmsgs >= 1 && data.msgs[0].flags == I2C_M_RD) {
            const std::uint16_t raw = static_cast<std::uint16_t>(((temperature + 45) / 175.0f) * 65535);
            data.msgs[0].buf[0] = static_cast<std::uint8_t>(raw >> 8);
            data.msgs[0].buf[1] = static_cast<std::uint8_t>(raw & 0xff);
            return 0;
        }
        return -1;
    }
};

class MCP9600 : public I2CDevice, public I2CRegisters_ConfigurableLength {
public:
    std::uint32_t last_temperature_update_ms = 0;
    void init() override {
        add_register("WHOAMI", 0x20, 1, RegMode::RDONLY);
        set_register(0x20, static_cast<std::uint8_t>(0x40));
        add_register("SENSOR_STATUS", 0x04, 1, RegMode::RDWR);
        set_register(0x04, static_cast<std::uint8_t>(0x00));
        add_register("SENSOR_CONFIG", 0x05, 1, RegMode::RDWR);
        set_register(0x05, static_cast<std::uint8_t>(0x00));
        add_register("HOT_JUNC", 0x00, 2, RegMode::RDONLY);
    }
    void update(const Aircraft& aircraft) override {
        const std::uint32_t now_ms = static_cast<std::uint32_t>(aircraft.time_us() / 1000);
        if (now_ms - last_temperature_update_ms < 100) {
            return;
        }
        last_temperature_update_ms = now_ms;
        std::uint8_t config = 0;
        get_reg_value(0x05, config);
        if (config == 0) {
            return;
        }
        const float t = sim_board_temp_c(aircraft);
        const std::uint16_t raw = static_cast<std::uint16_t>(t / 0.0625f);
        set_register(0x00, raw);
        set_register(0x04, static_cast<std::uint8_t>(1 << 6));
    }
    int rdwr(I2cRdwr& data) override { return I2CRegisters_ConfigurableLength::rdwr(data); }
};

inline void populate_default_i2c_bus(I2C& bus, TeraRangerI2C& teraranger, LightWareI2C_Legacy16Bit& lw16,
                                     MaxSonarI2CXL& maxsonar, MCP9600& mcp, ICM40609& icm, SHT3x& sht, AS5600& as5600,
                                     MS5525& ms5525, INA3221& ina, TSYS01& tsys01, Rotoye& rotoye, Maxell& maxell,
                                     SIM_BattMonitor_SMBus_Generic& smbus, Airspeed_DLVR& dlvr,
                                     Benewake_TFMiniPlus& tfmini, TSYS03& tsys03, MS5611& ms5611, QMC5883L& qmc,
                                     TFS20L& tfs20l, LightWareGRF_I2C& grf) {
    bus.devices = {
        {0, 0x31, &teraranger}, {0, 0x66, &lw16},     {0, 0x70, &maxsonar}, {0, 0x60, &mcp},    {1, 0x01, &icm},
        {1, 0x44, &sht},        {1, 0x36, &as5600},   {1, 0x76, &ms5525},   {1, 0x42, &ina},    {1, 0x77, &tsys01},
        {1, 0x0B, &rotoye},     {2, 0x0B, &maxell},   {3, 0x0B, &smbus},    {2, 0x28, &dlvr},   {2, 0x09, &tfmini},
        {2, 0x40, &tsys03},     {2, 0x77, &ms5611},   {2, 0x0D, &qmc},      {0, 0x10, &tfs20l}, {2, 0x66, &grf},
    };
}


inline void populate_led_i2c_devices(I2C& bus, ToshibaLED& toshiba, LP5562& lp, LM2755& lm, IS31FL3195& is31) {
    is31.set_product_id(0x54);
    bus.devices.push_back({1, 0x55, &toshiba});
    bus.devices.push_back({2, 0x30, &lp});
    bus.devices.push_back({2, 0x67, &lm});
    bus.devices.push_back({2, 0x54, &is31});
}

}  // namespace fwcpp::sim
