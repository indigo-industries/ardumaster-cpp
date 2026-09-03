#pragma once

#include <cstdint>

namespace fwcpp::vtol_assist {

inline constexpr float kAssistSpeedDefault = 0.0f;
inline constexpr std::int8_t kAssistAngleDefault = 30;
inline constexpr std::int16_t kAssistAltDefault = 0;
inline constexpr float kAssistDelayDefault = 0.5f;
inline constexpr std::int16_t kAssistOptionsDefault = 0;

inline constexpr std::uint32_t kQAssistForceEnable = 1u << 7;
inline constexpr std::uint32_t kDisableSyntheticAirspeedAssist = 1u << 12;

enum class AssistOption : std::int16_t {
    kFwForceDisabled = 1 << 0,
    kSpinDisabled = 1 << 1,
};

enum class AssistState : std::uint8_t {
    kAssistDisabled = 0,
    kAssistEnabled = 1,
    kForceEnabled = 2,
};

enum class AuxSwitchPos : std::uint8_t { kLow, kMiddle, kHigh };

[[nodiscard]] inline constexpr bool q_assist_force_enable_set(std::uint32_t q_options) {
    return (q_options & kQAssistForceEnable) != 0u;
}

[[nodiscard]] inline constexpr AssistState assist_state_from_aux(AuxSwitchPos pos) {
    switch (pos) {
        case AuxSwitchPos::kLow:
            return AssistState::kAssistDisabled;
        case AuxSwitchPos::kMiddle:
            return AssistState::kAssistEnabled;
        case AuxSwitchPos::kHigh:
            return AssistState::kForceEnabled;
    }
    return AssistState::kAssistEnabled;
}

class VtolAssist {
public:
    [[nodiscard]] static VtolAssist with_defaults() { return VtolAssist{}; }

    [[nodiscard]] float speed() const { return speed_; }
    [[nodiscard]] std::int8_t angle() const { return angle_; }
    [[nodiscard]] std::int16_t alt() const { return alt_; }
    [[nodiscard]] float delay() const { return delay_; }
    [[nodiscard]] std::int16_t options() const { return options_; }
    [[nodiscard]] AssistState state() const { return state_; }

    void set_speed(float speed) { speed_ = speed; }
    void set_angle(std::int8_t angle) { angle_ = angle; }
    void set_alt(std::int16_t alt) { alt_ = alt; }
    void set_delay(float delay) { delay_ = delay; }
    void set_options(std::int16_t options) { options_ = options; }
    void set_state(AssistState state) { state_ = state; }
    void set_state_from_aux(AuxSwitchPos pos) { state_ = assist_state_from_aux(pos); }

    void apply_q_options(std::uint32_t q_options) {
        if (q_assist_force_enable_set(q_options)) {
            state_ = AssistState::kForceEnabled;
        }
    }

    [[nodiscard]] bool option_is_set(AssistOption option) const {
        return (options_ & static_cast<std::int16_t>(option)) != 0;
    }

    [[nodiscard]] bool speed_checks_enabled() const { return speed_ > 0.0f; }

    [[nodiscard]] bool alt_check_enabled() const {
        return speed_checks_enabled() && alt_ > 0;
    }

    [[nodiscard]] bool should_check() const {
        return state_ != AssistState::kAssistDisabled && speed_checks_enabled();
    }

    [[nodiscard]] bool is_enabled() const {
        switch (state_) {
            case AssistState::kAssistDisabled:
                return false;
            case AssistState::kForceEnabled:
                return true;
            case AssistState::kAssistEnabled:
                return speed_checks_enabled();
        }
        return false;
    }

private:
    float speed_{kAssistSpeedDefault};
    std::int8_t angle_{kAssistAngleDefault};
    std::int16_t alt_{kAssistAltDefault};
    float delay_{kAssistDelayDefault};
    std::int16_t options_{kAssistOptionsDefault};
    AssistState state_{AssistState::kAssistEnabled};
};

}  // namespace fwcpp::vtol_assist
