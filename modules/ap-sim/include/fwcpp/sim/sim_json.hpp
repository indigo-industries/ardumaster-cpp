#pragma once

// Minimal JSON object parser for SIM_Frame::load_frame_params and
// SIM_Plane::load_coeffs. Original uses AP_JSON (picojson-based; see that
// library's own header for the "nlohmann" note this comment used to carry -
// AP_JSON.cpp is in fact a picojson fork, verified directly).
// Supports objects, arrays, numbers, strings, bools, null. Enough for
// Tools/autotest/models/*.json.
//
// CPP-094: "#"-prefixed line comments are supported (skip(), below),
// matching upstream's OWN real AP_JSON.cpp (lines 81-90): it strips
// everything from a bare '#' to end-of-line as a preprocessing pass before
// handing the text to its parser, specifically so hand-written model files
// like skywalker_2013.json (which use "#" throughout, not "//") load
// correctly. This was a real, disclosed gap until CPP-094: a byte-for-byte
// copy of that real upstream file failed to parse here at all before this
// fix - caught by CPP-094's own round-trip fidelity test in
// sim_plane_test.cpp. Unlike upstream's naive whole-text prescan (which
// clobbers a bare '#' even inside a quoted string value), this parser only
// treats '#' as a comment starter between tokens (skip() is never called
// mid-string) - a real, deliberate divergence, and a strictly safer one for
// any JSON string value that happens to contain '#'; skywalker_2013.json
// itself has no such string, so this doesn't affect this ticket's fixture.

#include <cctype>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <fwcpp/math/vector3.hpp>

namespace fwcpp::sim {

struct JsonValue {
    enum class Type : std::uint8_t { kNull, kNumber, kString, kBool, kArray, kObject };
    Type type{Type::kNull};
    double number{0.0};
    std::string str;
    bool boolean{false};
    std::vector<JsonValue> array;
    std::map<std::string, JsonValue> object;

    [[nodiscard]] bool is_null() const { return type == Type::kNull; }
    [[nodiscard]] bool is_number() const { return type == Type::kNumber; }
    [[nodiscard]] bool is_array() const { return type == Type::kArray; }
    [[nodiscard]] const JsonValue* get(const char* key) const {
        auto it = object.find(key);
        if (it == object.end()) {
            return nullptr;
        }
        return &it->second;
    }
};

class JsonParser {
public:
    explicit JsonParser(const std::string& text) : s_(text) {}

    bool parse(JsonValue& out) {
        skip();
        return parse_value(out) && (skip(), i_ >= s_.size());
    }

    [[nodiscard]] const std::string& error() const { return err_; }

private:
    const std::string& s_;
    std::size_t i_{0};
    std::string err_;

    void skip() {
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
                ++i_;
                continue;
            }
            if (c == '/' && i_ + 1 < s_.size() && s_[i_ + 1] == '/') {
                i_ += 2;
                while (i_ < s_.size() && s_[i_] != '\n') {
                    ++i_;
                }
                continue;
            }
            // CPP-094: '#'-to-end-of-line, matching upstream's real
            // AP_JSON.cpp comment stripping (lines 81-90) - see this file's
            // own banner for why this exists (skywalker_2013.json uses '#',
            // not '//').
            if (c == '#') {
                while (i_ < s_.size() && s_[i_] != '\n') {
                    ++i_;
                }
                continue;
            }
            break;
        }
    }

    bool parse_value(JsonValue& v) {
        skip();
        if (i_ >= s_.size()) {
            err_ = "unexpected eof";
            return false;
        }
        const char c = s_[i_];
        if (c == '{') {
            return parse_object(v);
        }
        if (c == '[') {
            return parse_array(v);
        }
        if (c == '"') {
            v.type = JsonValue::Type::kString;
            return parse_string(v.str);
        }
        if (c == 't' || c == 'f') {
            return parse_bool(v);
        }
        if (c == 'n') {
            return parse_null(v);
        }
        return parse_number(v);
    }

    bool parse_object(JsonValue& v) {
        v.type = JsonValue::Type::kObject;
        ++i_;
        skip();
        if (i_ < s_.size() && s_[i_] == '}') {
            ++i_;
            return true;
        }
        while (i_ < s_.size()) {
            skip();
            std::string key;
            if (!parse_string(key)) {
                return false;
            }
            skip();
            if (i_ >= s_.size() || s_[i_] != ':') {
                err_ = "expected :";
                return false;
            }
            ++i_;
            JsonValue child;
            if (!parse_value(child)) {
                return false;
            }
            v.object.emplace(key, child);
            skip();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == '}') {
                ++i_;
                return true;
            }
            err_ = "expected }";
            return false;
        }
        err_ = "unterminated object";
        return false;
    }

    bool parse_array(JsonValue& v) {
        v.type = JsonValue::Type::kArray;
        ++i_;
        skip();
        if (i_ < s_.size() && s_[i_] == ']') {
            ++i_;
            return true;
        }
        while (i_ < s_.size()) {
            JsonValue child;
            if (!parse_value(child)) {
                return false;
            }
            v.array.push_back(child);
            skip();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == ']') {
                ++i_;
                return true;
            }
            err_ = "expected ]";
            return false;
        }
        err_ = "unterminated array";
        return false;
    }

    bool parse_string(std::string& out) {
        skip();
        if (i_ >= s_.size() || s_[i_] != '"') {
            err_ = "expected string";
            return false;
        }
        ++i_;
        out.clear();
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '"') {
                return true;
            }
            if (c == '\\' && i_ < s_.size()) {
                out.push_back(s_[i_++]);
                continue;
            }
            out.push_back(c);
        }
        err_ = "unterminated string";
        return false;
    }

    bool parse_number(JsonValue& v) {
        skip();
        const std::size_t start = i_;
        if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) {
            ++i_;
        }
        while (i_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[i_])) || s_[i_] == '.' || s_[i_] == 'e' ||
                                  s_[i_] == 'E' || s_[i_] == '+' || s_[i_] == '-')) {
            ++i_;
        }
        if (i_ == start) {
            err_ = "expected number";
            return false;
        }
        v.type = JsonValue::Type::kNumber;
        v.number = std::strtod(s_.c_str() + start, nullptr);
        return true;
    }

    bool parse_bool(JsonValue& v) {
        if (s_.compare(i_, 4, "true") == 0) {
            v.type = JsonValue::Type::kBool;
            v.boolean = true;
            i_ += 4;
            return true;
        }
        if (s_.compare(i_, 5, "false") == 0) {
            v.type = JsonValue::Type::kBool;
            v.boolean = false;
            i_ += 5;
            return true;
        }
        err_ = "expected bool";
        return false;
    }

    bool parse_null(JsonValue& v) {
        if (s_.compare(i_, 4, "null") == 0) {
            v.type = JsonValue::Type::kNull;
            i_ += 4;
            return true;
        }
        err_ = "expected null";
        return false;
    }
};

inline bool load_json_file(const char* path, JsonValue& out, std::string& err) {
    FILE* f = std::fopen(path, "rb");
    if (f == nullptr) {
        err = std::string("cannot open ") + path;
        return false;
    }
    std::fseek(f, 0, SEEK_END);
    const long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::string text(static_cast<std::size_t>(n > 0 ? n : 0), '\0');
    if (n > 0) {
        std::fread(text.data(), 1, static_cast<std::size_t>(n), f);
    }
    std::fclose(f);
    JsonParser p(text);
    if (!p.parse(out)) {
        err = p.error();
        return false;
    }
    return true;
}

inline bool json_get_float(const JsonValue& obj, const char* key, float& dest) {
    const JsonValue* v = obj.get(key);
    if (v == nullptr || !v->is_number()) {
        return false;
    }
    dest = static_cast<float>(v->number);
    return true;
}

inline bool json_get_vector3(const JsonValue& obj, const char* key, math::Vector3f& dest) {
    const JsonValue* v = obj.get(key);
    if (v == nullptr || !v->is_array() || v->array.size() < 3) {
        return false;
    }
    if (!v->array[0].is_number() || !v->array[1].is_number() || !v->array[2].is_number()) {
        return false;
    }
    dest.x = static_cast<float>(v->array[0].number);
    dest.y = static_cast<float>(v->array[1].number);
    dest.z = static_cast<float>(v->array[2].number);
    return true;
}

}  // namespace fwcpp::sim
