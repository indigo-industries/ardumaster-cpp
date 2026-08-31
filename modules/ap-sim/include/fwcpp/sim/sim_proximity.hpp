#pragma once

// Original-source ports of SIM_PS_TeraRangerTower, SIM_PS_LD06,
// SIM_PS_RPLidar{A1,A2,S2}, SIM_PS_LightWare_SF45B.

#include <cmath>
#include <cstdint>
#include <cstring>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_pack.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

class SerialProximitySensor : public SerialDevice {
public:
    std::uint32_t last_sent_ms = 0;
    float default_range_m = 10.0f;
    virtual std::uint16_t reading_interval_ms() const { return 200; }
    virtual std::uint32_t packet_for_location(const Location&, std::uint8_t*, std::uint8_t) { return 0; }
    float measure_distance_at_angle_bf(const Location&, float) const { return default_range_m; }
};

class PS_TeraRangerTower : public SerialProximitySensor {
public:
    static constexpr float MAX_RANGE = 60;
    std::uint32_t last_output_time_ms = 0;
    void update(const Location& location, std::uint32_t now) {
        if (last_output_time_ms == 0) {
            last_output_time_ms = now;
            return;
        }
        if (now - last_output_time_ms < 200) {
            return;
        }
        last_output_time_ms = now;
        std::uint8_t send_buffer[20]{};
        send_buffer[0] = 'T';
        send_buffer[1] = 'H';
        for (std::uint8_t i = 0; i < 8; i++) {
            const std::uint16_t bf_angle = static_cast<std::uint16_t>((360 - (i * 45)) % 360);
            float distance = measure_distance_at_angle_bf(location, bf_angle);
            std::uint16_t mm = distance > MAX_RANGE ? 0xffff : static_cast<std::uint16_t>(distance * 1000);
            send_buffer[2 + i * 2] = static_cast<std::uint8_t>(mm >> 8);
            send_buffer[3 + i * 2] = static_cast<std::uint8_t>(mm & 0xff);
        }
        send_buffer[18] = crc_crc8(send_buffer, 18);
        write_to_autopilot(reinterpret_cast<const char*>(send_buffer), 19);
    }
};

class PS_LD06 : public SerialProximitySensor {
public:
    std::uint32_t last_scan_output_time_ms = 0;
    float last_degrees_bf = 0;
    void update(const Location& location, std::uint32_t now) {
        char trash[64]{};
        read_from_autopilot(trash, sizeof(trash));
        if (last_scan_output_time_ms == 0) {
            last_scan_output_time_ms = now;
            return;
        }
        const std::uint32_t sample_count = 12;
        const std::uint32_t samples_per_second = 4500;
        const float samples_per_ms = samples_per_second / 1000.0f;
        const std::uint32_t required_time_delta_ms = static_cast<std::uint32_t>(sample_count / samples_per_ms);
        if (now - last_scan_output_time_ms < required_time_delta_ms) {
            return;
        }
        last_scan_output_time_ms += required_time_delta_ms;
        const float degrees_per_s = 2152;
        const float degrees_per_sample = (degrees_per_s / 1000.0f) / samples_per_ms;
        std::uint8_t pkt[47]{};
        pkt[0] = 0x54;
        pkt[1] = static_cast<std::uint8_t>(sample_count);  // length in low 5 bits
        put_le16(&pkt[2], static_cast<std::uint16_t>(degrees_per_s));
        put_le16(&pkt[4], static_cast<std::uint16_t>(last_degrees_bf * 100));
        for (std::uint32_t i = 0; i < sample_count; i++) {
            last_degrees_bf = std::fmod(last_degrees_bf + degrees_per_sample, 360.0f);
            float d = measure_distance_at_angle_bf(location, last_degrees_bf);
            std::uint8_t conf = 29;
            if (d > 12.0f) {
                d = 0;
                conf = 0;
            }
            put_le16(&pkt[6 + i * 3], static_cast<std::uint16_t>(d * 1000));
            pkt[8 + i * 3] = conf;
        }
        put_le16(&pkt[42], static_cast<std::uint16_t>(last_degrees_bf * 100));
        put_le16(&pkt[44], 0);
        pkt[46] = crc8_generic_poly(pkt, 46, 0x4D);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 47);
    }
};

class PS_RPLidar : public SerialProximitySensor {
public:
    enum class State { IDLE = 17, SCANNING = 18 };
    enum class Command : std::uint8_t {
        STOP = 0x25, SCAN = 0x20, FORCE_SCAN = 0x21, RESET = 0x40,
        GET_DEVICE_INFO = 0x50, GET_HEALTH = 0x52, EXPRESS_SCAN = 0x82
    };
    enum class ScanMode { SCAN = 0, EXPRESS_SCAN_DENSE = 1 };
    State _state = State::IDLE;
    ScanMode _scan_mode = ScanMode::SCAN;
    bool scanning = false;
    float last_degrees_bf = 0;
    float _express_w_i_deg = 0;
    std::uint32_t last_scan_output_time_ms = 0;
    char _buffer[256]{};
    std::uint8_t _buflen = 0;
    virtual std::uint8_t device_info_model() const { return 0x28; }
    virtual std::uint8_t max_range() const { return 16; }

    void send_response_descriptor(std::uint32_t len, std::uint8_t sendmode, std::uint8_t datatype) {
        const std::uint8_t buf[7] = {
            0xA5, 0x5A,
            static_cast<std::uint8_t>(len),
            static_cast<std::uint8_t>(len >> 8),
            static_cast<std::uint8_t>(len >> 16),
            static_cast<std::uint8_t>(((len >> 24) & 0xff) | sendmode),
            datatype};
        write_to_autopilot(reinterpret_cast<const char*>(buf), 7);
    }

    void update_input() {
        const ssize_t n = read_from_autopilot(&_buffer[_buflen], sizeof(_buffer) - _buflen);
        if (n > 0) {
            _buflen = static_cast<std::uint8_t>(_buflen + n);
        }
        while (_buflen >= 2) {
            std::uint8_t i = 0;
            while (i < _buflen && static_cast<std::uint8_t>(_buffer[i]) != 0xA5) {
                i++;
            }
            if (i > 0) {
                std::memmove(_buffer, _buffer + i, _buflen - i);
                _buflen = static_cast<std::uint8_t>(_buflen - i);
            }
            if (_buflen < 2) {
                return;
            }
            const auto cmd = static_cast<Command>(static_cast<std::uint8_t>(_buffer[1]));
            std::memmove(_buffer, _buffer + 2, _buflen - 2);
            _buflen = static_cast<std::uint8_t>(_buflen - 2);
            switch (cmd) {
            case Command::STOP:
                _state = State::IDLE;
                scanning = false;
                _scan_mode = ScanMode::SCAN;
                break;
            case Command::SCAN:
            case Command::FORCE_SCAN:
                send_response_descriptor(0x05, 0x40, 0x81);
                _state = State::SCANNING;
                scanning = true;
                _scan_mode = ScanMode::SCAN;
                last_scan_output_time_ms = 0;
                last_degrees_bf = 0;
                break;
            case Command::EXPRESS_SCAN: {
                if (_buflen >= 1) {
                    const std::uint8_t payload_size = static_cast<std::uint8_t>(_buffer[0]);
                    const std::uint8_t discard = static_cast<std::uint8_t>(1 + payload_size + 1);
                    if (_buflen >= discard) {
                        std::memmove(_buffer, _buffer + discard, _buflen - discard);
                        _buflen = static_cast<std::uint8_t>(_buflen - discard);
                    } else {
                        _buflen = 0;
                    }
                }
                send_response_descriptor(0x54, 0x40, 0x85);
                _state = State::SCANNING;
                scanning = true;
                _scan_mode = ScanMode::EXPRESS_SCAN_DENSE;
                _express_w_i_deg = 0;
                last_scan_output_time_ms = 0;
                break;
            }
            case Command::GET_DEVICE_INFO: {
                send_response_descriptor(0x14, 0x00, 0x04);
                std::uint8_t info[20]{};
                info[0] = device_info_model();
                info[1] = 1;  // firmware minor
                info[2] = 1;  // firmware major
                info[3] = 1;  // hardware
                write_to_autopilot(reinterpret_cast<const char*>(info), 20);
                break;
            }
            case Command::GET_HEALTH: {
                send_response_descriptor(0x03, 0x00, 0x06);
                const std::uint8_t health[3]{0, 0, 0};
                write_to_autopilot(reinterpret_cast<const char*>(health), 3);
                break;
            }
            case Command::RESET: {
                _state = State::IDLE;
                scanning = false;
                const char* fw = "R12345678901234567890123456789012345678901234567890123456789012";
                write_to_autopilot(fw, 63);
                break;
            }
            }
        }
    }

    void update_output_scan(const Location& location, std::uint32_t now) {
        if (last_scan_output_time_ms == 0) {
            last_scan_output_time_ms = now;
            return;
        }
        const std::uint32_t time_delta = now - last_scan_output_time_ms;
        const std::uint32_t sample_count = time_delta;  // 1000 samples/s
        const float degrees_per_sample = 3.6f;
        last_scan_output_time_ms += sample_count;
        for (std::uint32_t i = 0; i < sample_count && i < 32; i++) {
            const float current = std::fmod(last_degrees_bf + degrees_per_sample, 360.0f);
            const bool is_start = current < last_degrees_bf;
            last_degrees_bf = current;
            float distance = measure_distance_at_angle_bf(location, current);
            if (distance > max_range()) {
                distance = 0;
            }
            const std::uint16_t angle_q6 = static_cast<std::uint16_t>(current * 64);
            const std::uint16_t distance_q2 = static_cast<std::uint16_t>(distance * 1000 * 4);
            std::uint8_t pkt[5]{};
            pkt[0] = static_cast<std::uint8_t>((is_start ? 1 : 0) | ((is_start ? 0 : 1) << 1) | (17 << 2));
            pkt[1] = static_cast<std::uint8_t>(1 | ((angle_q6 & 0x7F) << 1));
            pkt[2] = static_cast<std::uint8_t>(angle_q6 >> 7);
            pkt[3] = static_cast<std::uint8_t>(distance_q2);
            pkt[4] = static_cast<std::uint8_t>(distance_q2 >> 8);
            write_to_autopilot(reinterpret_cast<const char*>(pkt), 5);
        }
    }

    void update_output_express(const Location& location, std::uint32_t now) {
        if (last_scan_output_time_ms == 0) {
            last_scan_output_time_ms = now;
            return;
        }
        const std::uint32_t packet_period_ms = 1000 / 80;
        if (now - last_scan_output_time_ms < packet_period_ms) {
            return;
        }
        last_scan_output_time_ms += packet_period_ms;
        std::uint8_t packet[84]{};
        const std::uint16_t start_angle_q6 = static_cast<std::uint16_t>(_express_w_i_deg * 64.0f);
        packet[2] = static_cast<std::uint8_t>(start_angle_q6);
        packet[3] = static_cast<std::uint8_t>(start_angle_q6 >> 8);
        for (std::uint32_t k = 0; k < 40; k++) {
            float angle = std::fmod(_express_w_i_deg + 0.1125f * k, 360.0f);
            float d = measure_distance_at_angle_bf(location, angle);
            if (d > max_range()) {
                d = 0;
            }
            const std::uint16_t mm = static_cast<std::uint16_t>(d * 1000);
            packet[4 + k * 2] = static_cast<std::uint8_t>(mm);
            packet[5 + k * 2] = static_cast<std::uint8_t>(mm >> 8);
        }
        std::uint8_t checksum = 0;
        for (std::uint32_t i = 2; i < 84; i++) {
            checksum ^= packet[i];
        }
        packet[0] = static_cast<std::uint8_t>((0x0A << 4) | (checksum & 0x0F));
        packet[1] = static_cast<std::uint8_t>((0x05 << 4) | ((checksum >> 4) & 0x0F));
        write_to_autopilot(reinterpret_cast<const char*>(packet), 84);
        _express_w_i_deg = std::fmod(_express_w_i_deg + 4.5f, 360.0f);
    }

    void update(const Location& location, std::uint32_t now = 0) {
        update_input();
        if (_state != State::SCANNING) {
            return;
        }
        if (_scan_mode == ScanMode::SCAN) {
            update_output_scan(location, now);
        } else {
            update_output_express(location, now);
        }
    }
};
class PS_RPLidarA1 : public PS_RPLidar {
public:
    std::uint8_t device_info_model() const override { return 0x18; }
    std::uint8_t max_range() const override { return 8; }
};
class PS_RPLidarA2 : public PS_RPLidar {
public:
    std::uint8_t device_info_model() const override { return 0x28; }
    std::uint8_t max_range() const override { return 16; }
};
class PS_RPLidarS2 : public PS_RPLidar {
public:
    std::uint8_t device_info_model() const override { return 0x71; }
    std::uint8_t max_range() const override { return 50; }
};

class PS_LightWare_SF45B : public SerialProximitySensor {
public:
    static constexpr std::uint8_t PREAMBLE = 0xAA;
    enum class MessageID : std::uint8_t { DISTANCE_OUTPUT = 27, STREAM = 30, DISTANCE_DATA_CM = 44, UPDATE_RATE = 66 };
    std::uint32_t stream = 0;
    std::uint32_t desired_fields = 0;
    std::uint8_t update_rate = 1;
    struct {
        bool stream = false;
        bool desired_fields = false;
        bool update_rate = false;
    } send_response;
    float last_degrees_bf = 0;
    float last_dir = 1;
    std::uint32_t last_scan_output_time_ms = 0;
    std::uint8_t rx[256]{};
    std::uint8_t rxlen = 0;
    float yaw_deg = 0;

#pragma pack(push, 1)
    template <typename T>
    struct PackedMessage {
        std::uint8_t preamble = PREAMBLE;
        std::uint16_t flags = 0;
        T msg{};
        std::uint16_t checksum = 0;
        PackedMessage() = default;
        explicit PackedMessage(T m, std::uint16_t f) : flags(static_cast<std::uint16_t>(f | (sizeof(T) << 6))), msg(m) {}
        void update_checksum() {
            checksum = 0;
            const auto* p = reinterpret_cast<const std::uint8_t*>(this);
            for (std::uint8_t i = 0; i < 3 + sizeof(T); i++) {
                checksum = crc_xmodem_update(checksum, p[i]);
            }
        }
    };
    struct MsgStream {
        std::uint8_t msgid = static_cast<std::uint8_t>(MessageID::STREAM);
        std::uint32_t stream = 0;
    };
    struct DistanceOutput {
        std::uint8_t msgid = static_cast<std::uint8_t>(MessageID::DISTANCE_OUTPUT);
        std::uint32_t desired_fields = 0;
    };
    struct UpdateRate {
        std::uint8_t msgid = static_cast<std::uint8_t>(MessageID::UPDATE_RATE);
        std::uint8_t rate = 0;
    };
    struct DistanceDataCM {
        std::uint8_t msgid = static_cast<std::uint8_t>(MessageID::DISTANCE_DATA_CM);
        std::uint16_t distance_cm = 0;
        std::uint16_t angle_cd = 0;
    };
#pragma pack(pop)

    void handle_message(const std::uint8_t* buf, std::uint8_t payload_len) {
        const auto id = static_cast<MessageID>(buf[3]);
        if (id == MessageID::STREAM && payload_len >= 4) {
            std::memcpy(&stream, &buf[4], 4);
            send_response.stream = true;
        } else if (id == MessageID::DISTANCE_OUTPUT && payload_len >= 4) {
            std::memcpy(&desired_fields, &buf[4], 4);
            send_response.desired_fields = true;
        } else if (id == MessageID::UPDATE_RATE && payload_len >= 1) {
            update_rate = buf[4];
            send_response.update_rate = true;
        }
    }

    void update_input() {
        const ssize_t n = read_from_autopilot(reinterpret_cast<char*>(rx + rxlen), sizeof(rx) - rxlen);
        if (n > 0) {
            rxlen = static_cast<std::uint8_t>(rxlen + n);
        }
        while (rxlen >= 6) {
            std::uint8_t i = 0;
            while (i < rxlen && rx[i] != PREAMBLE) {
                i++;
            }
            if (i > 0) {
                std::memmove(rx, rx + i, rxlen - i);
                rxlen = static_cast<std::uint8_t>(rxlen - i);
            }
            if (rxlen < 4) {
                return;
            }
            const std::uint16_t flags = static_cast<std::uint16_t>(rx[1] | (rx[2] << 8));
            const std::uint8_t plen = static_cast<std::uint8_t>((flags >> 6) - 1);
            const std::uint8_t need = static_cast<std::uint8_t>(4 + plen + 2);
            if (rxlen < need) {
                return;
            }
            std::uint16_t crc = 0;
            for (std::uint8_t k = 0; k < static_cast<std::uint8_t>(4 + plen); k++) {
                crc = crc_xmodem_update(crc, rx[k]);
            }
            const std::uint16_t got = static_cast<std::uint16_t>(rx[need - 2] | (rx[need - 1] << 8));
            if (crc == got) {
                handle_message(rx, plen);
            }
            std::memmove(rx, rx + need, rxlen - need);
            rxlen = static_cast<std::uint8_t>(rxlen - need);
        }
    }

    void update_output_responses() {
        auto send_packed = [&](auto packed) {
            packed.update_checksum();
            write_to_autopilot(reinterpret_cast<const char*>(&packed), sizeof(packed));
        };
        if (send_response.stream) {
            send_response.stream = false;
            PackedMessage<MsgStream> p{MsgStream{static_cast<std::uint8_t>(MessageID::STREAM), stream}, 0x1};
            send_packed(p);
        }
        if (send_response.update_rate) {
            send_response.update_rate = false;
            PackedMessage<UpdateRate> p{UpdateRate{static_cast<std::uint8_t>(MessageID::UPDATE_RATE), update_rate}, 0x1};
            send_packed(p);
        }
        if (send_response.desired_fields) {
            send_response.desired_fields = false;
            PackedMessage<DistanceOutput> p{DistanceOutput{static_cast<std::uint8_t>(MessageID::DISTANCE_OUTPUT), desired_fields}, 0x1};
            send_packed(p);
        }
    }

    void update(const Location& location, std::uint32_t now = 0) {
        update_input();
        update_output_responses();
        if (now == 0) {
            now = last_scan_output_time_ms + 8;
        }
        if (last_scan_output_time_ms == 0) {
            last_scan_output_time_ms = now;
            return;
        }
        const std::uint32_t time_delta_ms = now - last_scan_output_time_ms;
        if (time_delta_ms > 1000) {
            last_scan_output_time_ms = now;
            return;
        }
        const float samples_per_ms = 133 / 1000.0f;
        const std::uint32_t sample_count = static_cast<std::uint32_t>(samples_per_ms * time_delta_ms);
        if (sample_count == 0) {
            return;
        }
        const float degrees_per_sample = (390 / 1000.0f) / samples_per_ms;
        last_scan_output_time_ms += static_cast<std::uint32_t>(sample_count / samples_per_ms);
        for (std::uint32_t i = 0; i < sample_count && i < 16; i++) {
            float current = last_degrees_bf + last_dir * degrees_per_sample;
            if (current < -170) {
                current += (-170 - current);
                last_dir = -last_dir;
            }
            if (current > 170) {
                current += (170 - current);
                last_dir = -last_dir;
            }
            last_degrees_bf = current;
            yaw_deg = current;
            float d = measure_distance_at_angle_bf(location, current);
            if (d > 53.0f) {
                d = -1.0f;
            }
            PackedMessage<DistanceDataCM> packed{
                DistanceDataCM{static_cast<std::uint8_t>(MessageID::DISTANCE_DATA_CM),
                               static_cast<std::uint16_t>(d * 100.0f),
                               static_cast<std::uint16_t>(math::wrap_180(current) * 100)},
                0x1};
            packed.update_checksum();
            write_to_autopilot(reinterpret_cast<const char*>(&packed), sizeof(packed));
        }
    }
};

}  // namespace fwcpp::sim
