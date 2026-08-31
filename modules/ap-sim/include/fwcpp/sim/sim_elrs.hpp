#pragma once

// Port of libraries/SITL/SIM_ELRS.{h,cpp}. Original is a MAVLink TCP radio:
// rate-limited AP<->GCS forwarding plus RADIO_STATUS (msgid 109, crc extra 185)
// at 100Hz with txbuf = percent remaining of the AP->GCS buffer. TCP sockets
// (SocketAPM_native on 5761+port) are replaced by injectable ByteBuffers so
// unit tests can drive the same packet path without a listener. Serial
// device baud remains 460800. Buffer sizes match original (UART 64/128,
// MAVLink 2048/2048). Air data rate 500 B/s. sysid 255, compid MAV_COMP_ID_ALL (0).

#include <cstdint>
#include <cstring>
#include <vector>

#include <fwcpp/sim/sim_mavlink_min.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

class ELRS : public SerialDevice {
public:
    explicit ELRS(std::uint8_t port_number = 2)
        : SerialDevice(64, 128),
          mavlinkInputBuffer(2048),
          mavlinkOutputBuffer(2048),
          this_system_id(255),
          this_component_id(0),
          input_data_rate(500.0f),
          output_data_rate(500.0f),
          target_port(static_cast<std::uint16_t>(5761 + port_number)) {}

    std::uint32_t device_baud() const override { return 460800; }

    ByteBuffer mavlinkInputBuffer;
    ByteBuffer mavlinkOutputBuffer;
    ByteBuffer gcs_from_radio{2048};
    ByteBuffer gcs_to_radio{2048};

    std::uint8_t this_system_id;
    std::uint8_t this_component_id;
    float input_data_rate;
    float output_data_rate;
    std::uint16_t target_port;
    bool connected = true;

    std::uint32_t lastSentFlowCtrl = 0;
    mavmin::Status radio_st{};
    mavmin::FrameDecoder decoder{};

    struct RateLimit {
        std::uint32_t last_ms = 0;
        float remaining = 0.0f;
        bool primed = false;
        std::uint32_t max_bytes(float rate_Bps, std::uint32_t now_ms) {
            if (!primed) {
                primed = true;
                last_ms = now_ms;
                remaining = 0.0f;
                return 0;
            }
            const float dt = (now_ms - last_ms) * 0.001f;
            last_ms = now_ms;
            remaining += dt * rate_Bps;
            const auto n = static_cast<std::uint32_t>(remaining);
            remaining -= static_cast<float>(n);
            return n;
        }
    };
    RateLimit input_limit;
    RateLimit output_limit;

    void inject_from_gcs(const std::uint8_t* data, std::size_t n) {
        gcs_to_radio.write(data, n);
    }

    std::vector<std::uint8_t> drain_to_gcs() {
        std::vector<std::uint8_t> out(gcs_from_radio.available());
        if (!out.empty()) {
            gcs_from_radio.read(out.data(), out.size());
        }
        return out;
    }

    void sendQueuedData(std::uint32_t now_ms) {
        if ((now_ms - lastSentFlowCtrl) > 10) {
            lastSentFlowCtrl = now_ms;
            const std::uint8_t percentage_remaining =
                static_cast<std::uint8_t>((mavlinkInputBuffer.space() * 100) / mavlinkInputBuffer.get_size());
            auto bytes = mavmin::encode_radio_status(this_system_id, this_component_id, radio_st, percentage_remaining);
            write_to_autopilot(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        }

        while (true) {
            std::uint8_t c = 0;
            if (!mavlinkOutputBuffer.read_byte(&c)) {
                break;
            }
            mavmin::Frame frame;
            if (decoder.feed(c, &frame)) {
                write_to_autopilot(reinterpret_cast<const char*>(frame.bytes), frame.len);
            }
        }
    }

    void update(std::uint32_t now_ms) {
        if (!connected) {
            return;
        }

        const std::uint32_t input_space = static_cast<std::uint32_t>(mavlinkInputBuffer.space());
        if (input_space > 0) {
            std::vector<std::uint8_t> buf(input_space);
            const ssize_t len = read_from_autopilot(reinterpret_cast<char*>(buf.data()), input_space);
            if (len > 0) {
                mavlinkInputBuffer.write(buf.data(), static_cast<std::size_t>(len));
            }
        }

        const std::uint32_t send_bytes = input_limit.max_bytes(input_data_rate, now_ms);
        if (send_bytes > 0) {
            std::vector<std::uint8_t> buf(send_bytes);
            const std::size_t len = mavlinkInputBuffer.read(buf.data(), send_bytes);
            if (len > 0) {
                gcs_from_radio.write(buf.data(), len);
            }
        }

        const std::uint32_t receive_bytes = output_limit.max_bytes(output_data_rate, now_ms);
        if (receive_bytes > 0) {
            std::vector<std::uint8_t> buf(receive_bytes);
            const std::size_t len = gcs_to_radio.read(buf.data(), receive_bytes);
            if (len > 0) {
                mavlinkOutputBuffer.write(buf.data(), len);
            }
        }

        sendQueuedData(now_ms);
    }
};

}  // namespace fwcpp::sim
