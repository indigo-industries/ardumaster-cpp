#pragma once

// Original-source ports of SIM_FETtecOneWireESC, SIM_Loweheiser,
// SIM_IntelligentEnergy24, SIM_RichenPower.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_engine_periph.hpp>
#include <fwcpp/sim/sim_mavlink_min.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class FETtecOneWireESC : public SerialDevice {
public:
    enum class ConfigMessageType : std::uint8_t {
        OK = 0, BL_PAGE_CORRECT = 1, NOT_OK = 2, BL_START_FW = 3, BL_PAGES_TO_FLASH = 4,
        REQ_TYPE = 5, REQ_SN = 6, REQ_SW_VER = 7, BEEP = 13,
        SET_FAST_COM_LENGTH = 26, SET_TLM_TYPE = 27, SET_LED_TMP_COLOR = 51
    };
    enum class ResponseFrameHeaderID : std::uint8_t { MASTER = 0x01, BOOTLOADER = 0x02, ESC = 0x03 };
    struct ESC {
        enum class State { POWERED_OFF = 17, IN_BOOTLOADER, RUNNING_START, RUNNING };
        State state = State::IN_BOOTLOADER;
        std::uint8_t sn[12]{};
        std::uint8_t id = 0;
        std::uint8_t type = 34;
        std::uint8_t sw_version = 3;
        std::uint8_t sw_subversion = 4;
        std::uint16_t pwm = 0;
        bool telem_request = false;
        float temperature = 25;
        std::uint8_t ofs = 0;
        struct {
            std::uint8_t length = 0, byte_count = 0, min_esc_id = 0, id_count = 255;
        } fast_com;
    };
    ESC escs[16];
    bool _enabled = true;
    std::uint32_t _powered_mask = 0xfff;
    std::uint8_t rx[256]{};
    std::uint8_t rxlen = 0;
    struct {
        std::uint8_t min_esc_id = 1;
        std::uint8_t id_count = 4;
    } fast_com;
    std::uint8_t telem_id = 1;
    float voltage = 16;
    float current = 0;
    float rpm = 0;

    FETtecOneWireESC() {
        for (std::uint8_t n = 0; n < 16; n++) {
            escs[n].ofs = n;
            escs[n].id = static_cast<std::uint8_t>(n + 1);
            std::memset(escs[n].sn, n + 1, 12);
            escs[n].state = ESC::State::IN_BOOTLOADER;
        }
    }

    void send_ok(ESC& esc, ResponseFrameHeaderID hdr) {
        std::uint8_t msg[7]{};
        msg[0] = static_cast<std::uint8_t>(hdr);
        msg[1] = esc.id;
        msg[2] = 0;
        msg[3] = 0;
        msg[4] = 6 + 1;  // frame_len
        msg[5] = static_cast<std::uint8_t>(ConfigMessageType::OK);
        msg[6] = crc8_dvb_s2_update(0, msg, 6);
        write_to_autopilot(reinterpret_cast<const char*>(msg), 7);
    }
    void send_type(ESC& esc) {
        std::uint8_t msg[7]{};
        msg[0] = static_cast<std::uint8_t>(ResponseFrameHeaderID::ESC);
        msg[1] = esc.id;
        msg[4] = 7;
        msg[5] = esc.type;
        msg[6] = crc8_dvb_s2_update(0, msg, 6);
        write_to_autopilot(reinterpret_cast<const char*>(msg), 7);
    }
    void send_sn(ESC& esc) {
        std::uint8_t msg[18]{};
        msg[0] = static_cast<std::uint8_t>(ResponseFrameHeaderID::ESC);
        msg[1] = esc.id;
        msg[4] = 18;
        std::memcpy(&msg[5], esc.sn, 12);
        msg[17] = crc8_dvb_s2_update(0, msg, 17);
        write_to_autopilot(reinterpret_cast<const char*>(msg), 18);
    }
    void send_ver(ESC& esc) {
        std::uint8_t msg[8]{};
        msg[0] = static_cast<std::uint8_t>(ResponseFrameHeaderID::ESC);
        msg[1] = esc.id;
        msg[4] = 8;
        msg[5] = esc.sw_version;
        msg[6] = esc.sw_subversion;
        msg[7] = crc8_dvb_s2_update(0, msg, 7);
        write_to_autopilot(reinterpret_cast<const char*>(msg), 8);
    }
    void send_telem(ESC& esc, const Aircraft* ac) {
        std::uint8_t msg[17]{};
        msg[0] = static_cast<std::uint8_t>(ResponseFrameHeaderID::ESC);
        msg[1] = esc.id;
        msg[4] = 17;
        msg[5] = static_cast<std::int8_t>(esc.temperature);
        const std::uint16_t v = static_cast<std::uint16_t>((ac ? ac->battery_voltage : voltage) * 100);
        const std::uint16_t c = static_cast<std::uint16_t>(current * 100);
        const std::int16_t r = static_cast<std::int16_t>(esc.pwm);
        msg[6] = static_cast<std::uint8_t>(v >> 8);
        msg[7] = static_cast<std::uint8_t>(v);
        msg[8] = static_cast<std::uint8_t>(c >> 8);
        msg[9] = static_cast<std::uint8_t>(c);
        msg[10] = static_cast<std::uint8_t>(r >> 8);
        msg[11] = static_cast<std::uint8_t>(r);
        msg[16] = crc8_dvb_s2_update(0, msg, 16);
        write_to_autopilot(reinterpret_cast<const char*>(msg), 17);
        telem_id = esc.id;
        rpm = static_cast<float>(esc.pwm);
    }

    void handle_config(const std::uint8_t* buf, std::uint8_t flen) {
        if (flen < 6) {
            return;
        }
        const std::uint8_t target = buf[1];
        if (target == 0 || target > 16) {
            return;
        }
        ESC& esc = escs[target - 1];
        const auto req = static_cast<ConfigMessageType>(buf[5]);
        if (esc.state == ESC::State::IN_BOOTLOADER) {
            if (req == ConfigMessageType::OK) {
                send_ok(esc, ResponseFrameHeaderID::BOOTLOADER);
            } else if (req == ConfigMessageType::BL_START_FW) {
                esc.state = ESC::State::RUNNING_START;
            }
            return;
        }
        if (esc.state != ESC::State::RUNNING && esc.state != ESC::State::RUNNING_START) {
            return;
        }
        switch (req) {
        case ConfigMessageType::OK:
            send_ok(esc, ResponseFrameHeaderID::ESC);
            break;
        case ConfigMessageType::REQ_TYPE:
            send_type(esc);
            break;
        case ConfigMessageType::REQ_SN:
            send_sn(esc);
            break;
        case ConfigMessageType::REQ_SW_VER:
            send_ver(esc);
            break;
        case ConfigMessageType::SET_TLM_TYPE:
            esc.telem_request = true;
            send_ok(esc, ResponseFrameHeaderID::ESC);
            break;
        case ConfigMessageType::SET_FAST_COM_LENGTH:
            if (flen >= 10) {
                fast_com.min_esc_id = buf[7];
                fast_com.id_count = buf[8];
            }
            send_ok(esc, ResponseFrameHeaderID::ESC);
            break;
        default:
            send_ok(esc, ResponseFrameHeaderID::ESC);
            break;
        }
    }

    void update(const SitlInput& input) {
        update(nullptr, &input);
    }
    void update(const Aircraft& aircraft) { update(&aircraft, nullptr); }
    void update(const Aircraft* ac, const SitlInput* input) {
        if (!_enabled) {
            return;
        }
        for (auto& esc : escs) {
            if (esc.state == ESC::State::RUNNING_START) {
                esc.state = ESC::State::RUNNING;
                send_ok(esc, ResponseFrameHeaderID::ESC);
            }
            if (esc.state == ESC::State::RUNNING && input) {
                esc.pwm = input->servos[esc.ofs] ? input->servos[esc.ofs] : 1000;
                esc.temperature += esc.pwm / 100000.0f;
                esc.temperature *= 0.95f;
            }
        }
        const ssize_t n = read_from_autopilot(reinterpret_cast<char*>(rx + rxlen), sizeof(rx) - rxlen);
        if (n > 0) {
            rxlen = static_cast<std::uint8_t>(rxlen + n);
        }
        while (rxlen >= 6) {
            std::uint8_t i = 0;
            while (i < rxlen && rx[i] != 0x01 && rx[i] != 0x02 && rx[i] != 0x03) {
                i++;
            }
            if (i) {
                std::memmove(rx, rx + i, rxlen - i);
                rxlen = static_cast<std::uint8_t>(rxlen - i);
            }
            if (rxlen < 6) {
                break;
            }
            const std::uint8_t flen = rx[4];
            if (flen < 6 || rxlen < flen) {
                break;
            }
            handle_config(rx, flen);
            std::memmove(rx, rx + flen, rxlen - flen);
            rxlen = static_cast<std::uint8_t>(rxlen - flen);
        }
        for (auto& esc : escs) {
            if (esc.telem_request && esc.state == ESC::State::RUNNING) {
                send_telem(esc, ac);
                esc.telem_request = false;
            }
        }
    }
};

class Loweheiser : public SerialDevice {
public:
    float rpm = 0;
    float current = 0;
    std::uint8_t sysid = 32;
    std::uint8_t compid = 32;
    mavmin::Status st{};
    mavmin::Decoder dec{};
    std::uint32_t last_heartbeat_ms = 0;
    std::uint32_t last_efi_ms = 0;
    bool generator_on = true;
    float efi_fuel_consumed = 0;

    void update(float rpm_in, float current_in, std::uint32_t now_ms = 0) {
        rpm = rpm_in;
        current = current_in;
        if (now_ms == 0) {
            now_ms = last_heartbeat_ms + 200;
        }
        char tmp[128];
        const ssize_t n = read_from_autopilot(tmp, sizeof(tmp));
        for (ssize_t i = 0; i < n; i++) {
            mavmin::CommandLong cmd{};
            if (dec.feed(static_cast<std::uint8_t>(tmp[i]), &cmd)) {
                if (cmd.command == mavmin::kMavCmdLoweheiserSetState) {
                    generator_on = cmd.param[0] > 0.5f;
                    auto ack = mavmin::encode_command_ack(sysid, compid, st, cmd.command, mavmin::kMavResultAccepted,
                                                          cmd.src_sys, cmd.src_comp);
                    write_to_autopilot(reinterpret_cast<const char*>(ack.data()), ack.size());
                }
            }
        }
        if (now_ms - last_heartbeat_ms >= 100) {
            last_heartbeat_ms = now_ms;
            auto hb = mavmin::encode_heartbeat(sysid, compid, st);
            write_to_autopilot(reinterpret_cast<const char*>(hb.data()), hb.size());
        }
        if (now_ms - last_efi_ms >= 200) {
            last_efi_ms = now_ms;
            mavmin::LoweheiserGovEfi pkt{};
            pkt.volt_batt = 50.0f;
            pkt.curr_batt = current;
            pkt.curr_gen = current;
            pkt.curr_rot = current * 0.1f;
            pkt.fuel_level = 80;
            pkt.throttle = generator_on ? 0.6f : 0;
            pkt.runtime = now_ms / 1000;
            pkt.until_maintenance = 3600;
            pkt.rectifier_temp = 40;
            pkt.generator_temp = 70;
            pkt.efi_batt = 12.4f;
            pkt.efi_rpm = generator_on ? rpm : 0;
            pkt.efi_pw = 2.5f;
            pkt.efi_fuel_flow = generator_on ? 0.4f : 0;
            efi_fuel_consumed += pkt.efi_fuel_flow * 0.2f / 3600.0f;
            pkt.efi_fuel_consumed = efi_fuel_consumed;
            pkt.efi_baro = 1013;
            pkt.efi_mat = 25;
            pkt.efi_clt = 80;
            pkt.efi_tps = pkt.throttle;
            pkt.efi_exhaust_gas_temperature = 400;
            pkt.efi_index = 1;
            pkt.generator_status = generator_on ? 1 : 0;
            pkt.efi_status = generator_on ? 1 : 0;
            auto bytes = mavmin::encode_loweheiser(sysid, compid, st, pkt);
            write_to_autopilot(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        }
    }
};

class IntelligentEnergy24 : public SerialDevice {
public:
    int enabled = 1;  // 1=V1, 2=V2
    float tank_bar = 300;
    float battery_v = 50;
    float bat_capacity_mAh = 3300;
    bool discharge = true;
    std::uint32_t last_data_sent_ms = 0;
    std::uint32_t last_ver_sent_ms = 0;
    std::int32_t set_state = -1;
    std::uint32_t err_code = 0;

    void update(float dt) {
        update(static_cast<std::uint32_t>(last_data_sent_ms + dt * 1000));
    }
    void update(std::uint32_t now) {
        if (!enabled) {
            return;
        }
        if (now - last_data_sent_ms < 500) {
            return;
        }
        float amps = discharge ? -20.0f : 20.0f;
        bat_capacity_mAh += amps * (now - last_data_sent_ms) / 3600.0f;
        const float min_bat_vol = 42.0f;
        const float max_bat_vol = 50.4f;
        const float max_bat_capacity_mAh = 3300;
        const float frac = math::constrain_value(bat_capacity_mAh / max_bat_capacity_mAh, 0.0f, 1.0f);
        tank_bar = 5 + frac * (300 - 5);
        battery_v = frac * (max_bat_vol - min_bat_vol) + min_bat_vol;
        if (battery_v <= min_bat_vol) {
            discharge = false;
        } else if (battery_v >= max_bat_vol) {
            discharge = true;
        }
        const std::int32_t battery_pwr = static_cast<std::int32_t>(battery_v * amps);
        const std::int32_t pwr_out = static_cast<std::int32_t>(battery_pwr * 1.4f);
        const std::uint32_t spm_pwr = static_cast<std::uint32_t>(std::fabs(battery_pwr * 0.3f));
        std::uint32_t state = set_state == -1 ? 2 : static_cast<std::uint32_t>(set_state);
        last_data_sent_ms = now;
        char message[160]{};
        if (enabled == 1) {
            std::snprintf(message, sizeof(message), "<%i,%.1f,%i,%u,%i,%u,%u>\n", static_cast<int>(tank_bar), battery_v,
                          static_cast<int>(pwr_out), static_cast<unsigned>(spm_pwr), static_cast<int>(battery_pwr),
                          static_cast<unsigned>(state), static_cast<unsigned>(err_code));
        } else {
            if (now - last_ver_sent_ms > 5000) {
                const char* ver = "[10011867,2.132,4,IE12160A8040015,7]\n";
                write_to_autopilot(ver, std::strlen(ver));
                last_ver_sent_ms = now;
            }
            const int tank_pct = static_cast<int>(tank_bar / 300.0f * 100.0f);
            std::snprintf(message, sizeof(message), "<%i,%.2f,%.1f,%i,%u,%i,%i,%u,%u,%i,%s,", tank_pct, 0.67f, battery_v,
                          static_cast<int>(pwr_out), static_cast<unsigned>(spm_pwr), 0, static_cast<int>(battery_pwr),
                          static_cast<unsigned>(state), static_cast<unsigned>(err_code), 0, "");
            std::uint8_t checksum = 0;
            for (const char* p = message; *p; ++p) {
                checksum = static_cast<std::uint8_t>(checksum + *p);
            }
            checksum = static_cast<std::uint8_t>(~checksum);
            char end[8];
            std::snprintf(end, sizeof(end), "%u>\n", checksum);
            std::strncat(message, end, sizeof(message) - std::strlen(message) - 1);
        }
        write_to_autopilot(message, std::strlen(message));
    }
};

class RichenPower : public SerialDevice {
public:
    enum class State : std::uint8_t { STOP = 21, IDLE = 22, RUN = 23, STOPPING = 24 };
    State state = State::STOP;
    float rpm = 0;
    std::int8_t _ctrl_pin = 8;
    std::uint32_t last_sent_ms = 0;
    std::uint32_t _runtime_ms = 0;
    std::uint32_t _last_runtime_ms = 0;
    std::uint32_t stop_start_ms = 0;
    float _current_current = 0;
    SIM_GeneratorEngine generatorengine{};
    static constexpr float base_supply_voltage = 50.0f;
    static constexpr float max_current = 50.0f;
    static constexpr std::uint32_t original_seconds_until_maintenance = 20 * 60;

#pragma pack(push, 1)
    struct RichenPacket {
        std::uint8_t magic1;
        std::uint8_t magic2;
        std::uint8_t version_minor;
        std::uint8_t version_major;
        std::uint8_t runtime_minutes;
        std::uint8_t runtime_seconds;
        std::uint16_t runtime_hours;
        std::uint32_t seconds_until_maintenance;
        std::uint16_t errors;
        std::uint16_t rpm;
        std::uint16_t throttle;
        std::uint16_t idle_throttle;
        std::uint16_t output_voltage;
        std::uint16_t output_current;
        std::uint16_t dynamo_current;
        std::uint8_t unknown1;
        std::uint8_t mode;
        std::uint8_t unknown6[38];
        std::uint16_t checksum;
        std::uint8_t footermagic1;
        std::uint8_t footermagic2;
    };
#pragma pack(pop)
    static_assert(sizeof(RichenPacket) == 70, "Richen packet is 70 bytes");
    union RichenUnion {
        std::uint8_t parse_buffer[70];
        std::uint16_t checksum_buffer[35];
        RichenPacket packet;
    } u{};

    RichenPower() {
        u.packet.magic1 = 0xAA;
        u.packet.magic2 = 0x55;
        u.packet.version_major = 0x0A;
        u.packet.version_minor = 0x00;
        u.packet.footermagic1 = 0x55;
        u.packet.footermagic2 = 0xAA;
        generatorengine.max_slew_rpm_per_second = 2000;
        generatorengine.max_current = max_current;
    }

    void update_checksum() {
        u.packet.checksum = 0;
        std::uint16_t sum = 0;
        for (std::uint8_t i = 1; i < 6; i++) {
            const std::uint16_t be = static_cast<std::uint16_t>((u.checksum_buffer[i] >> 8) | (u.checksum_buffer[i] << 8));
            sum = static_cast<std::uint16_t>(sum + be);
        }
        u.packet.checksum = static_cast<std::uint16_t>((sum >> 8) | (sum << 8));
    }

    void update(bool run, float desired_rpm) {
        SitlInput in{};
        in.servos[7] = run ? 1900 : 1100;
        (void)desired_rpm;
        update(in, last_sent_ms + 1000);
    }
    void update(const SitlInput& input, std::uint32_t now_ms = 0) {
        if (now_ms == 0) {
            now_ms = last_sent_ms + 1000;
        }
        static constexpr std::uint16_t PWM_STOP = 1200;
        static constexpr std::uint16_t PWM_IDLE = 1500;
        const std::uint16_t control_pwm = _ctrl_pin >= 1 ? input.servos[_ctrl_pin - 1] : 1500;
        if (state != State::STOPPING) {
            if (control_pwm <= PWM_STOP && state != State::STOP) {
                if (stop_start_ms == 0) {
                    stop_start_ms = now_ms;
                }
            } else {
                stop_start_ms = 0;
            }
        }
        State newstate;
        if (control_pwm <= PWM_STOP) {
            newstate = (stop_start_ms == 0 || now_ms - stop_start_ms > 30000) ? State::STOP : State::STOPPING;
        } else if (control_pwm <= PWM_IDLE) {
            newstate = State::IDLE;
        } else {
            newstate = State::RUN;
        }
        if (newstate != state) {
            state = newstate;
        } else if (state != State::STOP) {
            _runtime_ms += now_ms - _last_runtime_ms;
        }
        _last_runtime_ms = now_ms;
        switch (state) {
        case State::STOP:
            generatorengine.desired_rpm = 0;
            break;
        case State::IDLE:
        case State::STOPPING:
            generatorengine.desired_rpm = 4800;
            break;
        case State::RUN:
            generatorengine.desired_rpm = 13000;
            break;
        }
        generatorengine.current_current = _current_current;
        generatorengine.update(now_ms);
        rpm = generatorengine.current_rpm;
        if (now_ms - last_sent_ms < 1000) {
            return;
        }
        last_sent_ms = now_ms;
        auto be16 = [](std::uint16_t v) { return static_cast<std::uint16_t>((v >> 8) | (v << 8)); };
        auto be32 = [](std::uint32_t v) {
            return (v << 24) | ((v << 8) & 0x00FF0000) | ((v >> 8) & 0x0000FF00) | (v >> 24);
        };
        u.packet.rpm = be16(static_cast<std::uint16_t>(rpm));
        std::uint32_t rem = _runtime_ms / 1000;
        u.packet.runtime_hours = static_cast<std::uint16_t>(rem / 3600);
        rem %= 3600;
        u.packet.runtime_minutes = static_cast<std::uint8_t>(rem / 60);
        u.packet.runtime_seconds = static_cast<std::uint8_t>(rem % 60);
        const std::int32_t until = static_cast<std::int32_t>(original_seconds_until_maintenance - _runtime_ms / 1000);
        std::uint16_t errors = 0;
        if (until <= 0) {
            u.packet.seconds_until_maintenance = be32(0);
            errors |= 1;
        } else {
            u.packet.seconds_until_maintenance = be32(static_cast<std::uint32_t>(until));
        }
        u.packet.errors = be16(errors);
        if (state == State::IDLE || state == State::RUN) {
            u.packet.output_current = be16(static_cast<std::uint16_t>(_current_current * 100));
            u.packet.output_voltage = be16(static_cast<std::uint16_t>(100 * base_supply_voltage));
        } else {
            u.packet.output_current = 0;
            u.packet.output_voltage = 0;
        }
        u.packet.mode = state == State::STOP ? 4 : (state == State::RUN ? 1 : 0);
        update_checksum();
        write_to_autopilot(reinterpret_cast<const char*>(u.parse_buffer), 70);
    }
};

}  // namespace fwcpp::sim
