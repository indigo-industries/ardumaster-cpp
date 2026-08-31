#pragma once

// Port of libraries/SITL/SIM_SerialDevice.h/.cpp. Original ByteBuffer
// (AP_HAL RingBuffer) is a std::deque of bytes; baud-mismatch still drops
// the transfer. Corruption (AP_SIM_SERIALDEVICE_CORRUPTION_ENABLED) is
// omitted unless a caller sets enable_corruption.

#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

namespace fwcpp::sim {

class ByteBuffer {
public:
    explicit ByteBuffer(std::size_t size) : cap_(size) {}

    std::size_t write(const std::uint8_t* data, std::size_t n) {
        std::size_t wrote = 0;
        while (wrote < n && buf_.size() < cap_) {
            buf_.push_back(data[wrote++]);
        }
        return wrote;
    }

    std::size_t read(std::uint8_t* data, std::size_t n) {
        std::size_t got = 0;
        while (got < n && !buf_.empty()) {
            data[got++] = buf_.front();
            buf_.pop_front();
        }
        return got;
    }

    [[nodiscard]] std::size_t available() const { return buf_.size(); }
    [[nodiscard]] std::size_t space() const { return cap_ - buf_.size(); }
    [[nodiscard]] std::size_t get_size() const { return cap_; }
    bool read_byte(std::uint8_t* b) {
        if (buf_.empty() || b == nullptr) {
            return false;
        }
        *b = buf_.front();
        buf_.pop_front();
        return true;
    }
    void clear() { buf_.clear(); }

private:
    std::size_t cap_;
    std::deque<std::uint8_t> buf_{};
};

class SerialDevice {
public:
    explicit SerialDevice(std::uint16_t tx_bufsize = 512, std::uint16_t rx_bufsize = 512)
        : to_autopilot_(tx_bufsize), from_autopilot_(rx_bufsize) {}

    virtual ~SerialDevice() = default;

    ssize_t read_from_device(char* buffer, std::size_t size) {
        if (!is_match_baud()) {
            return -1;
        }
        return static_cast<ssize_t>(to_autopilot_.read(reinterpret_cast<std::uint8_t*>(buffer), size));
    }

    ssize_t write_to_device(const char* buffer, std::size_t size) {
        return static_cast<ssize_t>(from_autopilot_.write(reinterpret_cast<const std::uint8_t*>(buffer), size));
    }

    void set_autopilot_baud(std::uint32_t baud) { autopilot_baud_ = baud; }

    ssize_t read_from_autopilot(char* buffer, std::size_t size) {
        return static_cast<ssize_t>(from_autopilot_.read(reinterpret_cast<std::uint8_t*>(buffer), size));
    }

    virtual ssize_t write_to_autopilot(const char* buffer, std::size_t size) {
        if (!is_match_baud()) {
            return -1;
        }
        return static_cast<ssize_t>(to_autopilot_.write(reinterpret_cast<const std::uint8_t*>(buffer), size));
    }

    [[nodiscard]] virtual std::uint32_t device_baud() const { return 0; }

    [[nodiscard]] std::vector<std::uint8_t> drain_to_autopilot() {
        std::vector<std::uint8_t> out(to_autopilot_.available());
        if (!out.empty()) {
            to_autopilot_.read(out.data(), out.size());
        }
        return out;
    }

protected:
    ByteBuffer to_autopilot_;
    ByteBuffer from_autopilot_;

private:
    std::uint32_t autopilot_baud_{0};

    [[nodiscard]] bool is_match_baud() const {
        if (device_baud() != 0 && autopilot_baud_ != 0 && device_baud() != autopilot_baud_) {
            return false;
        }
        return true;
    }
};

}  // namespace fwcpp::sim
