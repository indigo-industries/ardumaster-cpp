#pragma once

// Port of libraries/SITL/SIM_Volz.{h,cpp}. Original 6-byte Command packets
// with crc_crc16_ibm(0xffff, first 4 bytes) stored big-endian. Commands:
// SET_EXTENDED_POSITION 0xDC -> 0x2C, READ_CURRENT 0xB0 -> 0x30,
// READ_VOLTAGE 0xB1 -> 0x31, READ_TEMPERATURE 0xC0 -> 0x10. Actuator IDs
// are 1-based. Position scale is linear_interpolate between 0x80 and 0x0F80
// for [-1, 1]. Logging (SMVZ) is omitted; AP_Param tables become plain
// members. Aircraft::servo_filter is not in this port — position tracks
// desired_position after SET (the original reads servo_filter.angle()).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class Volz : public SerialDevice {
public:
    enum class CommandId : std::uint8_t {
        SET_EXTENDED_POSITION = 0xDC,
        EXTENDED_POSITION_RESPONSE = 0x2C,
        READ_CURRENT = 0xB0,
        CURRENT_RESPONSE = 0x30,
        READ_VOLTAGE = 0xB1,
        VOLTAGE_RESPONSE = 0x31,
        READ_TEMPERATURE = 0xC0,
        TEMPERATURE_RESPONSE = 0x10,
    };

#pragma pack(push, 1)
    struct Command {
        CommandId command_id;
        std::uint8_t actuator_id;
        std::uint8_t arg1;
        std::uint8_t arg2;
        std::uint16_t crc;  // big-endian on the wire

        std::uint16_t calculate_checksum() const {
            return crc_crc16_ibm(0xffff, reinterpret_cast<const std::uint8_t*>(this), 4);
        }
        void update_checksum() {
            const std::uint16_t c = calculate_checksum();
            crc = static_cast<std::uint16_t>((c >> 8) | (c << 8));  // htobe16 on LE
        }
        std::uint16_t crc_host() const {
            return static_cast<std::uint16_t>((crc >> 8) | (crc << 8));  // be16toh on LE
        }
    };
#pragma pack(pop)
    static_assert(sizeof(Command) == 6, "Volz Command is 6 bytes");

    class Servo {
    public:
        std::uint8_t id = 0;
        float position = 0.0f;
        float desired_position = 0.0f;
        float pcb_temperature = 25.0f;
        float motor_temperature = 25.0f;
        float primary_current = 0.1f;
        float secondary_current = 0.1f;
        float primary_voltage = 1.1f;
        float secondary_voltage = 1.2f;
        bool failed = false;
        std::uint16_t last_good_pwm = 1500;

        std::uint16_t pwm() {
            if (failed) {
                return last_good_pwm;
            }
            last_good_pwm = static_cast<std::uint16_t>(1500 + desired_position * 500);
            return last_good_pwm;
        }
    };

    bool _enabled = false;
    std::uint32_t _output_mask = (1U << 0) | (1U << 1) | (1U << 3);
    std::uint32_t _failed_mask = 0;
    Servo servos[16]{};
    std::uint32_t last_servo_update_us = 0;

    union {
        Command command;
        std::uint8_t buffer[128];
    } u{};
    std::uint8_t buflen = 0;

    Volz() {
        for (std::uint8_t n = 0; n < 16; n++) {
            auto& s = servos[n];
            s.id = static_cast<std::uint8_t>(n + 1);
            s.pcb_temperature = 25;
            s.motor_temperature = 25;
            s.primary_current = 0.1f;
            s.secondary_current = 0.1f;
            s.primary_voltage = 1.1f;
            s.secondary_voltage = 1.2f;
        }
    }

    bool enabled() const { return _enabled; }

    void consume_command() {
        if (buflen < sizeof(Command)) {
            return;
        }
        if (buflen > sizeof(Command)) {
            std::memmove(&u.buffer[0], &u.buffer[sizeof(Command)], buflen - sizeof(Command));
        }
        buflen = static_cast<std::uint8_t>(buflen - sizeof(Command));
    }

    bool shift_command_to_front_of_buffer() {
        while (buflen >= sizeof(Command)) {
            if (u.command.calculate_checksum() == u.command.crc_host()) {
                return true;
            }
            std::memmove(&u.buffer[0], &u.buffer[1], buflen - 1);
            buflen--;
        }
        return false;
    }

    void process_command(const Command& command) {
        if (command.actuator_id == 0 || command.actuator_id >= 17) {
            return;
        }
        Servo& servo = servos[command.actuator_id - 1];
        Command response{};
        response.actuator_id = command.actuator_id;
        switch (command.command_id) {
        case CommandId::SET_EXTENDED_POSITION: {
            const std::uint16_t scaled = static_cast<std::uint16_t>((command.arg1 << 8) | command.arg2);
            servo.desired_position = math::linear_interpolate(-1.0f, 1.0f, static_cast<float>(scaled), 0x80, 0x0F80);
            servo.position = servo.desired_position;
            response.command_id = CommandId::EXTENDED_POSITION_RESPONSE;
            const std::uint16_t scaled_position = static_cast<std::uint16_t>(
                math::linear_interpolate(static_cast<float>(0x80), static_cast<float>(0x0F80), servo.position, -1.0f, 1.0f) +
                0.5f);
            response.arg1 = static_cast<std::uint8_t>(scaled_position >> 8);
            response.arg2 = static_cast<std::uint8_t>(scaled_position);
            break;
        }
        case CommandId::READ_TEMPERATURE:
            response.command_id = CommandId::TEMPERATURE_RESPONSE;
            response.arg1 = static_cast<std::uint8_t>(std::max(servo.motor_temperature, -50.0f) + 50.0f);
            response.arg2 = static_cast<std::uint8_t>(std::max(servo.pcb_temperature, -50.0f) + 50.0f);
            break;
        case CommandId::READ_CURRENT:
            response.command_id = CommandId::CURRENT_RESPONSE;
            response.arg1 = static_cast<std::uint8_t>(std::min(servo.primary_current * 50.0f, 255.0f));
            response.arg2 = static_cast<std::uint8_t>(std::min(servo.secondary_current * 50.0f, 255.0f));
            break;
        case CommandId::READ_VOLTAGE:
            response.command_id = CommandId::VOLTAGE_RESPONSE;
            response.arg1 = static_cast<std::uint8_t>(std::min(servo.primary_voltage * 5.0f, 255.0f));
            response.arg2 = static_cast<std::uint8_t>(std::min(servo.secondary_voltage * 5.0f, 255.0f));
            break;
        case CommandId::CURRENT_RESPONSE:
        case CommandId::VOLTAGE_RESPONSE:
        case CommandId::TEMPERATURE_RESPONSE:
        case CommandId::EXTENDED_POSITION_RESPONSE:
        default:
            return;
        }
        response.update_checksum();
        write_to_autopilot(reinterpret_cast<const char*>(&response), sizeof(response));
    }

    void update_input() {
        const ssize_t n = read_from_autopilot(reinterpret_cast<char*>(&u.buffer[buflen]), sizeof(u.buffer) - buflen);
        if (n > 0) {
            buflen = static_cast<std::uint8_t>(buflen + n);
        }
        while (shift_command_to_front_of_buffer()) {
            process_command(u.command);
            consume_command();
        }
    }

    void update_servos(const Aircraft& aircraft, std::uint32_t now_us) {
        const float ambient_degC = 25.0f;
        const std::uint32_t delta_t_us = now_us - last_servo_update_us;
        if (delta_t_us < 1000) {
            return;
        }
        last_servo_update_us = now_us;
        const float dt = delta_t_us / 1000000.0f;
        for (auto& servo : servos) {
            servo.primary_current = std::fabs(std::sin(servo.position * 3.14159265f / 180.0f)) * aircraft.airspeed_pitot * 10.0f;
            servo.pcb_temperature -= (servo.pcb_temperature - ambient_degC) * 0.1f * dt;
            (void)aircraft;
        }
    }

    void update(const Aircraft& aircraft) {
        if (!_enabled) {
            return;
        }
        update_servos(aircraft, static_cast<std::uint32_t>(aircraft.time_now_us));
        update_input();
    }

    void update_sitl_input_pwm(SitlInput& input) {
        for (auto& servo : servos) {
            const std::uint8_t idx = static_cast<std::uint8_t>(servo.id - 1);
            if (idx >= kSitlServoChannels) {
                continue;
            }
            if ((_output_mask & (1U << idx)) == 0) {
                continue;
            }
            servo.failed = (_failed_mask & (1U << idx)) != 0;
            input.servos[idx] = servo.pwm();
        }
    }
};

}  // namespace fwcpp::sim
