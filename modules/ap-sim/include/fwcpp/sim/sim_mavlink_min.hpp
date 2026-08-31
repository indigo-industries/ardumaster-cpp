#pragma once

// Isolated MAVLink 2 encode/decode used by SIM_RF_MAVLink and SIM_Loweheiser.
// No GCS_MAVLink / GCS stack: CRC extra and payload layouts match the
// original mavgen headers (DISTANCE_SENSOR, HEARTBEAT, COMMAND_LONG,
// COMMAND_ACK, LOWEHEISER_GOV_EFI).

#include <cstdint>
#include <cstring>
#include <vector>

#include <fwcpp/sim/sim_crc.hpp>

namespace fwcpp::sim::mavmin {

inline constexpr std::uint8_t kStxV2 = 0xFD;
inline constexpr std::uint8_t kStxV1 = 0xFE;

inline constexpr std::uint32_t kMsgHeartbeat = 0;
inline constexpr std::uint32_t kMsgCommandAck = 77;
inline constexpr std::uint32_t kMsgCommandLong = 76;
inline constexpr std::uint32_t kMsgDistanceSensor = 132;
inline constexpr std::uint32_t kMsgLoweheiserGovEfi = 10151;
inline constexpr std::uint32_t kMsgRadioStatus = 109;

inline constexpr std::uint8_t kCrcHeartbeat = 50;
inline constexpr std::uint8_t kCrcCommandLong = 152;
inline constexpr std::uint8_t kCrcCommandAck = 143;
inline constexpr std::uint8_t kCrcDistanceSensor = 85;
inline constexpr std::uint8_t kCrcLoweheiserGovEfi = 195;
inline constexpr std::uint8_t kCrcRadioStatus = 185;

inline constexpr std::uint16_t kMavCmdLoweheiserSetState = 10151;
inline constexpr std::uint8_t kMavSensorRotationPitch270 = 24;
inline constexpr std::uint8_t kMavTypeGcs = 6;
inline constexpr std::uint8_t kMavAutopilotInvalid = 8;
inline constexpr std::uint8_t kMavResultAccepted = 0;

struct Status {
    std::uint8_t seq = 0;
};

inline std::uint16_t crc_accumulate(std::uint16_t crc, std::uint8_t data) {
    std::uint8_t tmp = data ^ static_cast<std::uint8_t>(crc & 0xff);
    tmp ^= static_cast<std::uint8_t>(tmp << 4);
    crc = static_cast<std::uint16_t>((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
    return crc;
}

inline std::uint16_t crc_calculate(const std::uint8_t* buf, std::uint32_t len, std::uint8_t extra) {
    std::uint16_t crc = 0xFFFF;
    for (std::uint32_t i = 0; i < len; i++) {
        crc = crc_accumulate(crc, buf[i]);
    }
    crc = crc_accumulate(crc, extra);
    return crc;
}

inline std::vector<std::uint8_t> pack_v2(std::uint8_t sysid, std::uint8_t compid, std::uint32_t msgid,
                                         const std::uint8_t* payload, std::uint8_t payload_len, std::uint8_t crc_extra,
                                         Status& st) {
    std::vector<std::uint8_t> out(12 + payload_len);  // stx+len+incompat+compat+seq+sys+comp+msgid3 + payload + crc2
    out[0] = kStxV2;
    out[1] = payload_len;
    out[2] = 0;  // incompat
    out[3] = 0;  // compat
    out[4] = st.seq++;
    out[5] = sysid;
    out[6] = compid;
    out[7] = static_cast<std::uint8_t>(msgid);
    out[8] = static_cast<std::uint8_t>(msgid >> 8);
    out[9] = static_cast<std::uint8_t>(msgid >> 16);
    if (payload_len && payload) {
        std::memcpy(&out[10], payload, payload_len);
    }
    const std::uint16_t crc = crc_calculate(&out[1], static_cast<std::uint32_t>(9 + payload_len), crc_extra);
    out[10 + payload_len] = static_cast<std::uint8_t>(crc);
    out[11 + payload_len] = static_cast<std::uint8_t>(crc >> 8);
    return out;
}

inline std::vector<std::uint8_t> encode_distance_sensor(std::uint8_t sysid, std::uint8_t compid, Status& st,
                                                        std::uint32_t time_boot_ms, std::uint16_t alt_cm) {
    std::uint8_t p[39]{};
    std::memcpy(&p[0], &time_boot_ms, 4);
    const std::uint16_t min_d = 10, max_d = 1000;
    std::memcpy(&p[4], &min_d, 2);
    std::memcpy(&p[6], &max_d, 2);
    std::memcpy(&p[8], &alt_cm, 2);
    p[10] = 0;   // type
    p[11] = 72;  // id
    p[12] = kMavSensorRotationPitch270;
    p[13] = 255;  // covariance unknown
    // remaining extension fields stay 0
    return pack_v2(sysid, compid, kMsgDistanceSensor, p, 39, kCrcDistanceSensor, st);
}

inline std::vector<std::uint8_t> encode_heartbeat(std::uint8_t sysid, std::uint8_t compid, Status& st) {
    std::uint8_t p[9]{};
    p[4] = kMavTypeGcs;
    p[5] = kMavAutopilotInvalid;
    return pack_v2(sysid, compid, kMsgHeartbeat, p, 9, kCrcHeartbeat, st);
}

inline std::vector<std::uint8_t> encode_command_ack(std::uint8_t sysid, std::uint8_t compid, Status& st,
                                                   std::uint16_t command, std::uint8_t result, std::uint8_t target_sys,
                                                   std::uint8_t target_comp) {
    std::uint8_t p[10]{};
    std::memcpy(&p[0], &command, 2);
    p[2] = result;
    p[3] = 0;
    p[4] = 0;
    p[5] = 0;
    p[6] = 0;
    p[7] = 0;
    p[8] = target_sys;
    p[9] = target_comp;
    return pack_v2(sysid, compid, kMsgCommandAck, p, 10, kCrcCommandAck, st);
}

struct CommandLong {
    float param[7]{};
    std::uint16_t command = 0;
    std::uint8_t target_system = 0;
    std::uint8_t target_component = 0;
    std::uint8_t confirmation = 0;
    std::uint8_t src_sys = 0;
    std::uint8_t src_comp = 0;
};

struct Decoder {
    std::uint8_t buf[280]{};
    std::uint16_t n = 0;
    std::uint8_t payload_len = 0;
    bool in_frame = false;

    bool feed(std::uint8_t b, CommandLong* out_cmd) {
        if (!in_frame) {
            if (b != kStxV2) {
                return false;
            }
            buf[0] = b;
            n = 1;
            in_frame = true;
            return false;
        }
        if (n == 1) {
            payload_len = b;
        }
        if (n < sizeof(buf)) {
            buf[n++] = b;
        } else {
            in_frame = false;
            n = 0;
            return false;
        }
        const std::uint16_t need = static_cast<std::uint16_t>(12 + payload_len);
        if (n < 2 || n < need) {
            return false;
        }
        in_frame = false;
        n = 0;
        const std::uint32_t msgid = static_cast<std::uint32_t>(buf[7] | (buf[8] << 8) | (buf[9] << 16));
        if (msgid != kMsgCommandLong || payload_len < 33 || out_cmd == nullptr) {
            return false;
        }
        const std::uint8_t* p = &buf[10];
        for (int i = 0; i < 7; i++) {
            std::memcpy(&out_cmd->param[i], p + i * 4, 4);
        }
        std::memcpy(&out_cmd->command, p + 28, 2);
        out_cmd->target_system = p[30];
        out_cmd->target_component = p[31];
        out_cmd->confirmation = p[32];
        out_cmd->src_sys = buf[5];
        out_cmd->src_comp = buf[6];
        return true;
    }
};


inline std::vector<std::uint8_t> encode_radio_status(std::uint8_t sysid, std::uint8_t compid, Status& st,
                                                     std::uint8_t txbuf, std::uint8_t rssi = 255, std::uint8_t remrssi = 255,
                                                     std::uint8_t noise = 255, std::uint8_t remnoise = 255,
                                                     std::uint16_t rxerrors = 0, std::uint16_t fixed = 0) {
    std::uint8_t p[9]{};
    std::memcpy(&p[0], &rxerrors, 2);
    std::memcpy(&p[2], &fixed, 2);
    p[4] = rssi;
    p[5] = remrssi;
    p[6] = txbuf;
    p[7] = noise;
    p[8] = remnoise;
    return pack_v2(sysid, compid, kMsgRadioStatus, p, 9, kCrcRadioStatus, st);
}

struct Frame {
    std::uint8_t bytes[280]{};
    std::uint16_t len = 0;
    std::uint32_t msgid = 0;
    std::uint8_t sysid = 0;
    std::uint8_t compid = 0;
};

struct FrameDecoder {
    std::uint8_t buf[280]{};
    std::uint16_t n = 0;
    std::uint8_t payload_len = 0;
    bool in_frame = false;
    bool v2 = true;

    bool feed(std::uint8_t b, Frame* out) {
        if (!in_frame) {
            if (b == kStxV2) {
                buf[0] = b;
                n = 1;
                in_frame = true;
                v2 = true;
                return false;
            }
            if (b == kStxV1) {
                buf[0] = b;
                n = 1;
                in_frame = true;
                v2 = false;
                return false;
            }
            return false;
        }
        if (n == 1) {
            payload_len = b;
        }
        if (n < sizeof(buf)) {
            buf[n++] = b;
        } else {
            in_frame = false;
            n = 0;
            return false;
        }
        const std::uint16_t need = v2 ? static_cast<std::uint16_t>(12 + payload_len)
                                      : static_cast<std::uint16_t>(8 + payload_len);
        if (n < 2 || n < need) {
            return false;
        }
        in_frame = false;
        if (out) {
            out->len = n;
            std::memcpy(out->bytes, buf, n);
            if (v2) {
                out->msgid = static_cast<std::uint32_t>(buf[7] | (buf[8] << 8) | (buf[9] << 16));
                out->sysid = buf[5];
                out->compid = buf[6];
            } else {
                out->msgid = buf[5];
                out->sysid = buf[3];
                out->compid = buf[4];
            }
        }
        n = 0;
        return true;
    }
};

#pragma pack(push, 1)
struct LoweheiserGovEfi {
    float volt_batt;
    float curr_batt;
    float curr_gen;
    float curr_rot;
    float fuel_level;
    float throttle;
    std::uint32_t runtime;
    std::int32_t until_maintenance;
    float rectifier_temp;
    float generator_temp;
    float efi_batt;
    float efi_rpm;
    float efi_pw;
    float efi_fuel_flow;
    float efi_fuel_consumed;
    float efi_baro;
    float efi_mat;
    float efi_clt;
    float efi_tps;
    float efi_exhaust_gas_temperature;
    std::uint8_t efi_index;
    std::uint16_t generator_status;
    std::uint16_t efi_status;
};
#pragma pack(pop)
static_assert(sizeof(LoweheiserGovEfi) == 85, "LOWEHEISER_GOV_EFI payload is 85 bytes");

inline std::vector<std::uint8_t> encode_loweheiser(std::uint8_t sysid, std::uint8_t compid, Status& st,
                                                   const LoweheiserGovEfi& pkt) {
    return pack_v2(sysid, compid, kMsgLoweheiserGovEfi, reinterpret_cast<const std::uint8_t*>(&pkt), 85,
                   kCrcLoweheiserGovEfi, st);
}

}  // namespace fwcpp::sim::mavmin
