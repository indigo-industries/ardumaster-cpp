#pragma once

#include <cstdint>

namespace fwcpp::vtol_assist {

class AssistHysteresis {
public:
    void reset() {
        start_ms_ = 0;
        last_ms_ = 0;
        active_ = false;
    }

    [[nodiscard]] bool update(bool trigger, std::uint32_t now_ms, std::uint32_t trigger_delay_ms,
                            std::uint32_t clear_delay_ms) {
        bool ret = false;
        if (trigger) {
            last_ms_ = now_ms;
            if (start_ms_ == 0) {
                start_ms_ = now_ms;
            }
            if ((now_ms - start_ms_) > trigger_delay_ms) {
                if (!active_) {
                    ret = true;
                }
                active_ = true;
            }
        } else if (active_) {
            if ((last_ms_ == 0) || ((now_ms - last_ms_) > clear_delay_ms)) {
                reset();
            }
        } else {
            reset();
        }
        return ret;
    }

    [[nodiscard]] bool is_active() const { return active_; }

private:
    std::uint32_t start_ms_{0};
    std::uint32_t last_ms_{0};
    bool active_{false};
};

[[nodiscard]] inline std::uint32_t trigger_delay_ms(float delay_s) {
    return static_cast<std::uint32_t>(delay_s * 1000.0f);
}

[[nodiscard]] inline std::uint32_t clear_delay_ms(float delay_s) {
    return trigger_delay_ms(delay_s) * 2u;
}

}  // namespace fwcpp::vtol_assist
