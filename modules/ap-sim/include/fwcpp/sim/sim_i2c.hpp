#pragma once

// Port of libraries/SITL/SIM_I2C.h/.cpp, SIM_I2CDevice.h/.cpp (8/16-bit
// registers + I2CCommandResponseDevice), SIM_QMC5883L, SIM_MaxSonarI2CXL.
// ioctl dispatch matches SIM_I2C::ioctl_rdwr. AP_HAL panic becomes false
// return. LED drivers are instantiated via populate_led_i2c_devices (original SIM_I2C bus).

#include <cstdint>
#include <cstring>
#include <vector>

#include <fwcpp/bitmask.hpp>
#include <fwcpp/math/rotations.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>

#ifndef I2C_M_RD
#define I2C_M_RD 1
#endif
#ifndef I2C_RDWR
#define I2C_RDWR 0
#endif

namespace fwcpp::sim {

struct I2cMsg {
    std::uint8_t bus = 0;
    std::uint8_t addr = 0;
    std::uint8_t flags = 0;
    std::uint8_t* buf = nullptr;
    std::uint16_t len = 0;
};

struct I2cRdwr {
    I2cMsg* msgs = nullptr;
    std::uint8_t nmsgs = 0;
};

class I2CDevice {
public:
    virtual ~I2CDevice() = default;
    virtual void init() {}
    virtual void update(const Aircraft& /*aircraft*/) {}
    virtual int rdwr(I2cRdwr& data) = 0;
};

class I2CRegisters {
public:
    enum class RegMode { RDONLY = 11, WRONLY = 22, RDWR = 33 };

public:
    void add_register(const char* name, std::uint8_t reg, RegMode mode) {
        regname[reg] = name;
        if (mode == RegMode::RDONLY || mode == RegMode::RDWR) {
            readable_registers.set(reg);
        }
        if (mode == RegMode::WRONLY || mode == RegMode::RDWR) {
            writable_registers.set(reg);
        }
    }
    const char* regname[256]{};
    fwcpp::Bitmask<256> writable_registers;
    fwcpp::Bitmask<256> readable_registers;
};

class I2CRegisters_16Bit : public I2CRegisters {
public:
    void set_register(std::uint8_t reg, std::uint16_t value) { word[reg] = static_cast<std::uint16_t>((value >> 8) | (value << 8)); }
    void set_register_i16(std::uint8_t reg, std::int16_t value) { set_register(reg, static_cast<std::uint16_t>(value)); }
    std::uint16_t get_reg_value(std::uint8_t reg) {
        const std::uint16_t be = word[reg];
        return static_cast<std::uint16_t>((be >> 8) | (be << 8));
    }
    int rdwr(I2cRdwr& data) {
        if (data.nmsgs == 2) {
            if (data.msgs[0].flags != 0 || data.msgs[1].flags != I2C_M_RD) {
                return -1;
            }
            const std::uint8_t reg_base_addr = data.msgs[0].buf[0];
            std::uint8_t bytes_copied = 0;
            while (bytes_copied < data.msgs[1].len) {
                const std::uint8_t reg_addr = static_cast<std::uint8_t>(reg_base_addr + bytes_copied / 2);
                if (!readable_registers.get(reg_addr)) {
                    return -1;
                }
                const std::uint16_t register_value = word[reg_addr];
                data.msgs[1].buf[bytes_copied++] = static_cast<std::uint8_t>(register_value >> 8);
                if (bytes_copied < data.msgs[1].len) {
                    data.msgs[1].buf[bytes_copied++] = static_cast<std::uint8_t>(register_value & 0xFF);
                }
            }
            data.msgs[1].len = bytes_copied;
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
            const std::uint16_t register_value =
                static_cast<std::uint16_t>(data.msgs[0].buf[2] << 8 | data.msgs[0].buf[1]);
            word[reg_addr] = register_value;
            return 0;
        }
        return -1;
    }

protected:
    std::uint16_t word[256]{};
};

class I2CRegisters_8Bit : public I2CRegisters {
public:
    void set_register(std::uint8_t reg, std::uint8_t value) { byte[reg] = value; }
    std::uint8_t get_register(std::uint8_t num) { return byte[num]; }
    int rdwr(I2cRdwr& data) {
        if (data.nmsgs == 2) {
            if (data.msgs[0].flags != 0 || data.msgs[1].flags != I2C_M_RD) {
                return -1;
            }
            const std::uint8_t reg_base_addr = data.msgs[0].buf[0];
            std::uint8_t bytes_copied = 0;
            while (bytes_copied < data.msgs[1].len) {
                const std::uint8_t reg_addr = static_cast<std::uint8_t>(reg_base_addr + bytes_copied);
                if (!readable_registers.get(reg_addr)) {
                    return -1;
                }
                data.msgs[1].buf[bytes_copied++] = byte[reg_addr];
            }
            data.msgs[1].len = bytes_copied;
            return 0;
        }
        if (data.nmsgs == 1) {
            if (data.msgs[0].flags != 0) {
                return -1;
            }
            const std::uint8_t reg_base_addr = data.msgs[0].buf[0];
            std::uint8_t bytes_copied = 0;
            while (bytes_copied < data.msgs[0].len - 1) {
                const std::uint8_t reg_addr = static_cast<std::uint8_t>(reg_base_addr + bytes_copied);
                if (!writable_registers.get(reg_addr)) {
                    return -1;
                }
                rdwr_store_register_value(reg_addr, data.msgs[0].buf[1 + bytes_copied]);
                bytes_copied++;
            }
            return 0;
        }
        return -1;
    }

protected:
    virtual void rdwr_store_register_value(std::uint8_t reg, std::uint8_t value) { byte[reg] = value; }
    std::uint8_t byte[256]{};
};

class I2CCommandResponseDevice {
public:
    int rdwr(I2cRdwr& data, std::uint32_t now_ms) {
        I2cMsg& msg = data.msgs[0];
        if (msg.flags == I2C_M_RD) {
            if (now_ms - cmd_take_reading_received_ms < command_processing_time_ms()) {
                return -1;
            }
            if (msg.len != 2) {
                return -1;
            }
            const std::uint16_t value = reading();
            msg.buf[0] = static_cast<std::uint8_t>(value >> 8);
            msg.buf[1] = static_cast<std::uint8_t>(value & 0xff);
            return 0;
        }
        const std::uint8_t cmd = msg.buf[0];
        if (cmd != command_take_reading()) {
            return -1;
        }
        cmd_take_reading_received_ms = now_ms;
        return 0;
    }

protected:
    std::uint16_t command_processing_time_ms() const { return 20; }
    virtual std::uint8_t command_take_reading() const = 0;
    virtual std::uint16_t reading() const = 0;
    std::uint32_t cmd_take_reading_received_ms = 0;
};

class QMC5883L : public I2CDevice {
public:
    QMC5883L() {
        writable_registers.set(0);
        writable_registers.set(0x0b);
        writable_registers.set(0x20);
        writable_registers.set(0x21);
        writable_registers.set(0x09);
        writable_registers.set(0x0A);
        reset();
    }
    void reset() {
        std::memset(registers, 0, sizeof(registers));
        registers[0x0C] = 0x01;
        registers[0x06] = 0x0;
        registers[0x0D] = 0xFF;
    }
    int rdwr(I2cRdwr& data) override {
        if (data.nmsgs == 2) {
            if (data.msgs[0].flags != 0 || data.msgs[1].flags != I2C_M_RD) {
                return -1;
            }
            const std::uint8_t reg_addr = data.msgs[0].buf[0];
            for (std::uint8_t i = 0; i < data.msgs[1].len; i++) {
                data.msgs[1].buf[i] = registers[reg_addr + i];
                if (reg_addr == 0x05) {
                    registers[0x06] = static_cast<std::uint8_t>(registers[0x06] & ~0x04);
                }
            }
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
            registers[reg_addr] = data.msgs[0].buf[1];
            return 0;
        }
        return -1;
    }
    void update(const Aircraft& aircraft) override {
        math::Vector3f f = aircraft.mag_bf;
        f.rotate_inverse(math::Rotation::ROLL_180_YAW_270);
        f.x = -f.x;
        f.z = -f.z;
        f.x *= 3;
        f.y *= 3;
        f.z *= 3;
        const std::int16_t kx = static_cast<std::int16_t>(f.x);
        const std::int16_t ky = static_cast<std::int16_t>(f.y);
        const std::int16_t kz = static_cast<std::int16_t>(f.z);
        if (registers[0x09] & 0x01) {
            registers[0x06] = static_cast<std::uint8_t>(registers[0x06] | 0x04);
            registers[0x00] = static_cast<std::uint8_t>(kx & 0xFF);
            registers[0x01] = static_cast<std::uint8_t>(kx >> 8);
            registers[0x02] = static_cast<std::uint8_t>(ky & 0xFF);
            registers[0x03] = static_cast<std::uint8_t>(ky >> 8);
            registers[0x04] = static_cast<std::uint8_t>(kz & 0xFF);
            registers[0x05] = static_cast<std::uint8_t>(kz >> 8);
        }
    }
    std::uint8_t registers[256]{};

private:
    fwcpp::Bitmask<256> writable_registers;
};

class MaxSonarI2CXL : public I2CDevice, public I2CCommandResponseDevice {
public:
    std::uint8_t command_take_reading() const override { return 0x51; }
    std::uint16_t reading() const override { return static_cast<std::uint16_t>(rangefinder_range * 100); }
    void update(const Aircraft& aircraft) override { rangefinder_range = aircraft.hagl(); }
    int rdwr(I2cRdwr& data) override { return I2CCommandResponseDevice::rdwr(data, now_ms_); }
    void set_now_ms(std::uint32_t now_ms) { now_ms_ = now_ms; }
    float rangefinder_range = 0;

private:
    std::uint32_t now_ms_ = 0;
};

struct I2cDeviceAtAddress {
    std::uint8_t bus;
    std::uint8_t addr;
    I2CDevice* device;
};

class I2C {
public:
    std::vector<I2cDeviceAtAddress> devices;

    void update(const Aircraft& aircraft) {
        for (auto& d : devices) {
            if (d.device != nullptr) {
                d.device->update(aircraft);
            }
        }
    }

    int ioctl_rdwr(I2cRdwr* data) {
        if (data == nullptr || data->nmsgs == 0) {
            return -1;
        }
        const std::uint8_t bus = data->msgs[0].bus;
        const std::uint8_t addr = data->msgs[0].addr;
        for (auto& d : devices) {
            if (d.bus == bus && d.addr == addr && d.device != nullptr) {
                return d.device->rdwr(*data);
            }
        }
        return -1;
    }
};

}  // namespace fwcpp::sim
