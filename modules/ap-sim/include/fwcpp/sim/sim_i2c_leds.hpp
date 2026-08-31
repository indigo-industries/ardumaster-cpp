#pragma once

// Original-source ports of SIM_ToshibaLED, SIM_LP5562, SIM_LM2755, SIM_IS31FL3195.
// Register maps match the original SIM_* headers; Copter/Plane may never probe.

#include <cstdint>

#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_i2c.hpp>

namespace fwcpp::sim {

class ToshibaLED : public I2CDevice, public I2CRegisters_8Bit {
public:
    static constexpr std::uint8_t PWM0 = 0x01;
    static constexpr std::uint8_t PWM1 = 0x02;
    static constexpr std::uint8_t PWM2 = 0x03;
    static constexpr std::uint8_t ENABLE = 0x04;
    std::uint8_t last_rgb[3]{};
    ToshibaLED() { init(); }
    void init() override {
        add_register("PWM0", PWM0, I2CRegisters::RegMode::WRONLY);
        add_register("PWM1", PWM1, I2CRegisters::RegMode::WRONLY);
        add_register("PWM2", PWM2, I2CRegisters::RegMode::WRONLY);
        add_register("ENABLE", ENABLE, I2CRegisters::RegMode::WRONLY);
    }
    void update(const Aircraft&) override {
        // original: rgb = PWM * 17 when ENABLE
        if (get_register(ENABLE)) {
            last_rgb[0] = static_cast<std::uint8_t>(get_register(PWM0) * 17);
            last_rgb[1] = static_cast<std::uint8_t>(get_register(PWM1) * 17);
            last_rgb[2] = static_cast<std::uint8_t>(get_register(PWM2) * 17);
        }
    }
    int rdwr(I2cRdwr& data) override { return I2CRegisters_8Bit::rdwr(data); }
};

class LP5562 : public I2CDevice, public I2CRegisters_8Bit {
public:
    static constexpr std::uint8_t ENABLE = 0x00;
    static constexpr std::uint8_t OP_MODE = 0x01;
    static constexpr std::uint8_t B_PWM = 0x02;
    static constexpr std::uint8_t G_PWM = 0x03;
    static constexpr std::uint8_t R_PWM = 0x04;
    static constexpr std::uint8_t B_CURRENT = 0x05;
    static constexpr std::uint8_t G_CURRENT = 0x06;
    static constexpr std::uint8_t R_CURRENT = 0x07;
    static constexpr std::uint8_t CONFIG = 0x08;
    static constexpr std::uint8_t RESET = 0x0D;
    static constexpr std::uint8_t LED_MAP = 0x70;
    std::uint8_t rgb[3]{};
    LP5562() { init(); }
    void init() override {
        add_register("ENABLE", ENABLE, I2CRegisters::RegMode::RDWR);
        add_register("OP_MODE", OP_MODE, I2CRegisters::RegMode::RDWR);
        add_register("B_PWM", B_PWM, I2CRegisters::RegMode::RDWR);
        add_register("G_PWM", G_PWM, I2CRegisters::RegMode::RDWR);
        add_register("R_PWM", R_PWM, I2CRegisters::RegMode::RDWR);
        add_register("B_CURRENT", B_CURRENT, I2CRegisters::RegMode::RDWR);
        add_register("G_CURRENT", G_CURRENT, I2CRegisters::RegMode::RDWR);
        add_register("R_CURRENT", R_CURRENT, I2CRegisters::RegMode::RDWR);
        add_register("CONFIG", CONFIG, I2CRegisters::RegMode::RDWR);
        add_register("RESET", RESET, I2CRegisters::RegMode::WRONLY);
        add_register("LED_MAP", LED_MAP, I2CRegisters::RegMode::RDWR);
        reset_registers();
    }
    void reset_registers() {
        set_register(ENABLE, 0);
        set_register(B_CURRENT, 0xAF);
        set_register(G_CURRENT, 0xAF);
        set_register(R_CURRENT, 0xAF);
    }
    void update(const Aircraft&) override {
        if (get_register(RESET)) {
            reset_registers();
            set_register(RESET, 0);
        }
        rgb[0] = get_register(R_PWM);
        rgb[1] = get_register(G_PWM);
        rgb[2] = get_register(B_PWM);
    }
    int rdwr(I2cRdwr& data) override { return I2CRegisters_8Bit::rdwr(data); }
};

class LM2755 : public I2CDevice, public I2CRegisters_8Bit {
public:
    static constexpr std::uint8_t GENERAL_PURPOSE = 0x10;
    static constexpr std::uint8_t D1_HIGH_LEVEL = 0xA9;
    static constexpr std::uint8_t D2_HIGH_LEVEL = 0xB9;
    static constexpr std::uint8_t D3_HIGH_LEVEL = 0xC9;
    std::uint8_t rgb[3]{};
    LM2755() { init(); }
    void init() override {
        add_register("GENERAL_PURPOSE", GENERAL_PURPOSE, I2CRegisters::RegMode::RDWR);
        add_register("D1_HIGH_LEVEL", D1_HIGH_LEVEL, I2CRegisters::RegMode::RDWR);
        add_register("D2_HIGH_LEVEL", D2_HIGH_LEVEL, I2CRegisters::RegMode::RDWR);
        add_register("D3_HIGH_LEVEL", D3_HIGH_LEVEL, I2CRegisters::RegMode::RDWR);
        set_register(D1_HIGH_LEVEL, 0);
        set_register(D2_HIGH_LEVEL, 0);
        set_register(D3_HIGH_LEVEL, 0);
    }
    void update(const Aircraft&) override {
        rgb[0] = get_register(D1_HIGH_LEVEL);
        rgb[1] = get_register(D2_HIGH_LEVEL);
        rgb[2] = get_register(D3_HIGH_LEVEL);
    }
    int rdwr(I2cRdwr& data) override { return I2CRegisters_8Bit::rdwr(data); }
};

class IS31FL3195 : public I2CDevice, public I2CRegisters_8Bit {
public:
    static constexpr std::uint8_t PRODUCT_ID = 0x00;
    static constexpr std::uint8_t SHUTDOWN_CONTROL = 0x01;
    static constexpr std::uint8_t OUT1 = 0x10;
    static constexpr std::uint8_t OUT2 = 0x21;
    static constexpr std::uint8_t OUT3 = 0x32;
    static constexpr std::uint8_t OUT4 = 0x40;
    static constexpr std::uint8_t COLOUR_UPDATE = 0x50;
    static constexpr std::uint8_t RESET_REGISTER = 0x5f;
    bool colour_update_register_poked = false;
    std::uint8_t rgb[3]{};
    IS31FL3195() { init(); }
    void init() override {
        add_register("PRODUCT_ID", PRODUCT_ID, I2CRegisters::RegMode::RDONLY);
        add_register("SHUTDOWN_CONTROL", SHUTDOWN_CONTROL, I2CRegisters::RegMode::RDWR);
        add_register("OUT1", OUT1, I2CRegisters::RegMode::RDWR);
        add_register("OUT2", OUT2, I2CRegisters::RegMode::RDWR);
        add_register("OUT3", OUT3, I2CRegisters::RegMode::RDWR);
        add_register("OUT4", OUT4, I2CRegisters::RegMode::RDWR);
        add_register("COLOUR_UPDATE", COLOUR_UPDATE, I2CRegisters::RegMode::WRONLY);
        add_register("RESET_REGISTER", RESET_REGISTER, I2CRegisters::RegMode::WRONLY);
        reset_registers();
    }
    void set_product_id(std::uint8_t addr) { set_register(PRODUCT_ID, static_cast<std::uint8_t>(addr << 1)); }
    void reset_registers() {
        set_register(SHUTDOWN_CONTROL, 0);
        set_register(OUT1, 0);
        set_register(OUT2, 0);
        set_register(OUT3, 0);
        colour_update_register_poked = false;
    }
    void rdwr_store_register_value(std::uint8_t reg, std::uint8_t value) override {
        if (reg == COLOUR_UPDATE) {
            colour_update_register_poked = (value == 0xC5);
            return;
        }
        if (reg == RESET_REGISTER) {
            reset_registers();
            return;
        }
        I2CRegisters_8Bit::rdwr_store_register_value(reg, value);
    }
    void update(const Aircraft&) override {
        if (colour_update_register_poked) {
            rgb[0] = get_register(OUT1);
            rgb[1] = get_register(OUT2);
            rgb[2] = get_register(OUT3);
            colour_update_register_poked = false;
        }
    }
    int rdwr(I2cRdwr& data) override { return I2CRegisters_8Bit::rdwr(data); }
};

}  // namespace fwcpp::sim
