#include <catch2/catch_test_macros.hpp>
#include <fwcpp/logger/logger.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>

using fwcpp::logger::MemoryBackend;

TEST_CASE("MemoryBackend WriteBlock records blocks", "[logger][backend]") {
    MemoryBackend<16> log;
    const std::uint8_t head[] = {'D', 'F'};
    const std::uint8_t rest[] = {0x80, 0x01};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(head, sizeof(head))));
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(rest, sizeof(rest))));
    REQUIRE(log.size() == 4);
    REQUIRE(log.num_dropped() == 0);
}

TEST_CASE("MemoryBackend WriteBlock round-trips bytes", "[logger][backend]") {
    MemoryBackend<32> log;
    log.StartWrite(1);
    const std::uint8_t head[] = {'H', 'E', 'A', 'D'};
    const std::uint8_t payload[] = {0x10, 0x20, 0x30};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(head, sizeof(head))));
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(payload, sizeof(payload))));
    log.EndWrite();

    const std::array<std::uint8_t, 7> expected = {
        'H', 'E', 'A', 'D', 0x10, 0x20, 0x30,
    };
    const auto rec = log.recorded();
    REQUIRE(rec.size() == expected.size());
    REQUIRE(std::equal(rec.begin(), rec.end(), expected.begin()));
    REQUIRE(log.page_adr() == 1);
    REQUIRE(log.ended_writes() == 1);
    REQUIRE_FALSE(log.is_writing());
}

TEST_CASE("MemoryBackend drops and counts when the cap is full", "[logger][backend][drop]") {
    MemoryBackend<2> log;
    const std::uint8_t first[] = {1, 2};
    const std::uint8_t extra[] = {3};
    const std::uint8_t too_big[] = {4, 5};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(first, sizeof(first))));
    REQUIRE(log.num_dropped() == 0);
    REQUIRE_FALSE(log.WriteBlock(std::span<const std::uint8_t>(extra, sizeof(extra))));
    REQUIRE(log.num_dropped() == 1);
    REQUIRE_FALSE(log.WriteBlock(std::span<const std::uint8_t>(too_big, sizeof(too_big))));
    REQUIRE(log.num_dropped() == 2);

    const auto rec = log.recorded();
    REQUIRE(rec.size() == 2);
    REQUIRE(rec[0] == 1);
    REQUIRE(rec[1] == 2);
    REQUIRE(log.capacity() == 2);
}

TEST_CASE("StartWrite and EndWrite bookend a page write", "[logger][backend]") {
    MemoryBackend<8> log;
    REQUIRE_FALSE(log.is_writing());
    log.StartWrite(7);
    REQUIRE(log.page_adr() == 7);
    REQUIRE(log.is_writing());
    const std::uint8_t ab[] = {'A', 'B'};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(ab, sizeof(ab))));
    log.EndWrite();
    REQUIRE_FALSE(log.is_writing());
    REQUIRE(log.ended_writes() == 1);
    REQUIRE(log.page_adr() == 7);
    const auto rec = log.recorded();
    REQUIRE(rec.size() == 2);
    REQUIRE(rec[0] == 'A');
    REQUIRE(rec[1] == 'B');
}

TEST_CASE("Fill_Format copies LogStructure into a packed FMT packet", "[logger][fmt]") {
    using fwcpp::logger::Fill_Format;
    using fwcpp::logger::HEAD_BYTE1;
    using fwcpp::logger::HEAD_BYTE2;
    using fwcpp::logger::LOG_FORMAT_MSG;
    using fwcpp::logger::LogStructure;
    using fwcpp::logger::log_Format;

    REQUIRE(offsetof(log_Format, head1) == 0);
    REQUIRE(offsetof(log_Format, head2) == 1);
    REQUIRE(offsetof(log_Format, msgid) == 2);
    REQUIRE(offsetof(log_Format, type) == 3);
    REQUIRE(offsetof(log_Format, length) == 4);
    REQUIRE(offsetof(log_Format, name) == 5);
    REQUIRE(offsetof(log_Format, format) == 9);
    REQUIRE(offsetof(log_Format, labels) == 25);
    REQUIRE(sizeof(log_Format) == 89);

    const LogStructure s{1, 12, "DUMY", "Qf", "TimeUS,Val"};
    log_Format pkt{};
    Fill_Format(s, pkt);

    REQUIRE(pkt.head1 == HEAD_BYTE1);
    REQUIRE(pkt.head2 == HEAD_BYTE2);
    REQUIRE(pkt.msgid == LOG_FORMAT_MSG);
    REQUIRE(pkt.type == 1);
    REQUIRE(pkt.length == 12);
    REQUIRE(std::memcmp(pkt.name, "DUMY", 4) == 0);
    REQUIRE(std::memcmp(pkt.format, "Qf", 3) == 0);
    REQUIRE(std::memcmp(pkt.labels, "TimeUS,Val", 11) == 0);
}

TEST_CASE("Write_Format round-trips a FMT packet through MemoryBackend", "[logger][fmt]") {
    using fwcpp::logger::Fill_Format;
    using fwcpp::logger::LOG_FORMAT_MSG;
    using fwcpp::logger::Write_Format;
    using fwcpp::logger::log_Format;
    using fwcpp::logger::structure_for_name;

    MemoryBackend<128> log;
    const auto* fmt = structure_for_name("FMT");
    REQUIRE(fmt != nullptr);
    REQUIRE(fmt->msg_type == LOG_FORMAT_MSG);
    REQUIRE(Write_Format(log, *fmt));

    log_Format expected{};
    Fill_Format(*fmt, expected);
    const auto rec = log.recorded();
    REQUIRE(rec.size() == sizeof(log_Format));
    REQUIRE(std::memcmp(rec.data(), &expected, sizeof(expected)) == 0);
    REQUIRE(rec[0] == fwcpp::logger::HEAD_BYTE1);
    REQUIRE(rec[1] == fwcpp::logger::HEAD_BYTE2);
    REQUIRE(rec[2] == LOG_FORMAT_MSG);
    REQUIRE(log.num_dropped() == 0);
}

TEST_CASE("FMT registry lookup by name and msg type", "[logger][fmt]") {
    using fwcpp::logger::LOG_DUMMY_MSG;
    using fwcpp::logger::LOG_FORMAT_MSG;
    using fwcpp::logger::log_structure_count;
    using fwcpp::logger::structure_for_msg_type;
    using fwcpp::logger::structure_for_name;

    REQUIRE(log_structure_count() >= 2);

    const auto* fmt = structure_for_name("FMT");
    REQUIRE(fmt != nullptr);
    REQUIRE(fmt->msg_type == LOG_FORMAT_MSG);
    REQUIRE(structure_for_msg_type(LOG_FORMAT_MSG) == fmt);

    const auto* dumy = structure_for_name("DUMY");
    REQUIRE(dumy != nullptr);
    REQUIRE(dumy->msg_type == LOG_DUMMY_MSG);
    REQUIRE(structure_for_msg_type(LOG_DUMMY_MSG) == dumy);

    REQUIRE(structure_for_name("NOPE") == nullptr);
    REQUIRE(structure_for_msg_type(255) == nullptr);
}

TEST_CASE("WriteStreaming pause blocks without incrementing num_dropped",
          "[logger][streaming]") {
    using fwcpp::logger::RateLimiter;

    RateLimiter limiter;
    REQUIRE_FALSE(limiter.should_log_streaming(1, 1000, 10.0f, true));
    REQUIRE(limiter.last_send_ms(1) == 0);

    MemoryBackend<32> log;
    const std::uint8_t block[] = {0xA3, 0x95, 0x01, 0x00};
    REQUIRE_FALSE(log.WriteStreaming(std::span<const std::uint8_t>(block, sizeof(block)),
                                     1, 1000, 10.0f, true));
    REQUIRE(log.size() == 0);
    REQUIRE(log.num_dropped() == 0);
}

TEST_CASE("WriteStreaming too-soon at 10 Hz drops without incrementing num_dropped",
          "[logger][streaming]") {
    MemoryBackend<32> log;
    const std::uint8_t first[] = {0xA3, 0x95, 0x02, 0x11};
    const std::uint8_t second[] = {0xA3, 0x95, 0x02, 0x22};

    REQUIRE(log.WriteStreaming(std::span<const std::uint8_t>(first, sizeof(first)),
                               2, 1000, 10.0f, false));
    REQUIRE(log.size() == sizeof(first));
    REQUIRE(log.num_dropped() == 0);

    // 10 Hz => 1000/rate_hz == 100 ms. delta 99 < 100 is too soon.
    REQUIRE_FALSE(log.WriteStreaming(std::span<const std::uint8_t>(second, sizeof(second)),
                                     2, 1099, 10.0f, false));
    REQUIRE(log.size() == sizeof(first));
    REQUIRE(log.num_dropped() == 0);

    const auto rec = log.recorded();
    REQUIRE(rec.size() == sizeof(first));
    REQUIRE(std::equal(rec.begin(), rec.end(), first));
}

TEST_CASE("WriteStreaming millis16 wrap-around subtraction", "[logger][streaming]") {
    using fwcpp::logger::RateLimiter;

    RateLimiter limiter;
    REQUIRE(limiter.should_log_streaming(3, 65530, 10.0f, false));
    REQUIRE(limiter.last_send_ms(3) == 65530);

    // now=44: uint16 (44 - 65530) == 50, 50 < 100 => too soon.
    REQUIRE_FALSE(limiter.should_log_streaming(3, 44, 10.0f, false));
    REQUIRE(limiter.last_send_ms(3) == 65530);

    // now=94: uint16 (94 - 65530) == 100, 100 < 100 is false => pass.
    REQUIRE(limiter.should_log_streaming(3, 94, 10.0f, false));
    REQUIRE(limiter.last_send_ms(3) == 94);

    MemoryBackend<32> log;
    const std::uint8_t a[] = {0xA3, 0x95, 0x03, 0xAA};
    const std::uint8_t b[] = {0xA3, 0x95, 0x03, 0xBB};
    const std::uint8_t c[] = {0xA3, 0x95, 0x03, 0xCC};
    REQUIRE(log.WriteStreaming(std::span<const std::uint8_t>(a, sizeof(a)),
                               3, 65530, 10.0f, false));
    REQUIRE_FALSE(log.WriteStreaming(std::span<const std::uint8_t>(b, sizeof(b)),
                                     3, 44, 10.0f, false));
    REQUIRE(log.num_dropped() == 0);
    REQUIRE(log.WriteStreaming(std::span<const std::uint8_t>(c, sizeof(c)),
                               3, 94, 10.0f, false));
    REQUIRE(log.size() == sizeof(a) + sizeof(c));
    REQUIRE(log.num_dropped() == 0);
}

TEST_CASE("WriteStreaming passing write lands in MemoryBackend", "[logger][streaming]") {
    MemoryBackend<32> log;
    const std::uint8_t block[] = {0xA3, 0x95, 0x04, 0x42};
    REQUIRE(log.WriteStreaming(std::span<const std::uint8_t>(block, sizeof(block)),
                               4, 1000, 10.0f, false));
    REQUIRE(log.size() == sizeof(block));
    REQUIRE(log.num_dropped() == 0);
    const auto rec = log.recorded();
    REQUIRE(std::equal(rec.begin(), rec.end(), block));
    REQUIRE(log.rate_limiter().last_send_ms(4) == 1000);
}

TEST_CASE("FileBackend tmpfile WriteBlock round-trip", "[logger][file]") {
    using fwcpp::logger::FileBackend;

    std::FILE* f = std::tmpfile();
    REQUIRE(f != nullptr);
    FileBackend log(f);
    REQUIRE(log.logging_started());
    REQUIRE(log.WritesOK());
    REQUIRE(log.CardInserted());
    REQUIRE(log.StartNewLogOK());
    REQUIRE(log.num_short_writes() == 0);

    log.StartWrite(3);
    REQUIRE(log.is_writing());
    REQUIRE(log.page_adr() == 3);
    const std::uint8_t head[] = {0xA3, 0x95};
    const std::uint8_t payload[] = {0x01, 0x42, 0xFF};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(head, sizeof(head))));
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(payload, sizeof(payload))));
    log.EndWrite();
    REQUIRE_FALSE(log.is_writing());
    REQUIRE(log.ended_writes() == 1);
    REQUIRE(log.num_short_writes() == 0);

    REQUIRE(std::fflush(f) == 0);
    std::rewind(f);
    std::uint8_t got[5]{};
    REQUIRE(std::fread(got, 1, sizeof(got), f) == sizeof(got));
    REQUIRE(got[0] == 0xA3);
    REQUIRE(got[1] == 0x95);
    REQUIRE(got[2] == 0x01);
    REQUIRE(got[3] == 0x42);
    REQUIRE(got[4] == 0xFF);
    std::fclose(f);
}

TEST_CASE("FileBackend closed FILE* WritesOK false", "[logger][file]") {
    using fwcpp::logger::FileBackend;

    FileBackend log(static_cast<std::FILE*>(nullptr));
    REQUIRE_FALSE(log.logging_started());
    REQUIRE_FALSE(log.WritesOK());
    REQUIRE_FALSE(log.CardInserted());
    REQUIRE_FALSE(log.StartNewLogOK());

    const std::uint8_t block[] = {0x10, 0x20};
    REQUIRE_FALSE(log.WriteBlock(std::span<const std::uint8_t>(block, sizeof(block))));
    REQUIRE(log.num_short_writes() == 1);
}

TEST_CASE("FileBackend short write returns false and counts locally", "[logger][file]") {
    using fwcpp::logger::FileBackend;

    // Needs a stream whose writes always come up short. /dev/full does exactly
    // that and has no Windows equivalent -- NUL swallows everything and reports
    // success, which would assert the opposite of what this covers. Keep the
    // real assertion on Linux and skip elsewhere rather than weaken it.
#ifndef __linux__
    SKIP("requires /dev/full, which only Linux provides");
#else
    std::FILE* f = std::fopen("/dev/full", "w");
    REQUIRE(f != nullptr);
    FileBackend log(f);
    REQUIRE(log.WritesOK());
    const std::uint8_t block[] = {1, 2, 3, 4};
    REQUIRE_FALSE(log.WriteBlock(std::span<const std::uint8_t>(block, sizeof(block))));
    REQUIRE(log.num_short_writes() == 1);
    REQUIRE(log.logging_started());
    std::fclose(f);
#endif
}

TEST_CASE("FileBackend EraseAll armed is a no-op", "[logger][file][erase]") {
    using fwcpp::logger::FileBackend;

    std::FILE* f = std::tmpfile();
    REQUIRE(f != nullptr);
    FileBackend log(f);
    const std::uint8_t block[] = {1, 2, 3, 4};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(block, sizeof(block))));
    REQUIRE(std::fflush(f) == 0);
    REQUIRE(std::fseek(f, 0, SEEK_END) == 0);
    REQUIRE(std::ftell(f) == 4);

    log.EraseAll(true);
    REQUIRE_FALSE(log.was_logging());
    REQUIRE(log.logging_started());

    REQUIRE(std::fseek(f, 0, SEEK_END) == 0);
    REQUIRE(std::ftell(f) == 4);
    std::rewind(f);
    std::uint8_t got[4]{};
    REQUIRE(std::fread(got, 1, sizeof(got), f) == sizeof(got));
    REQUIRE(std::equal(got, got + sizeof(got), block));
    std::fclose(f);
}

TEST_CASE("FileBackend EraseAll uninitialised is a no-op", "[logger][file][erase]") {
    using fwcpp::logger::FileBackend;

    FileBackend log;
    REQUIRE_FALSE(log.logging_started());
    REQUIRE_FALSE(log.was_logging());
    log.EraseAll(false);
    REQUIRE_FALSE(log.was_logging());
    REQUIRE_FALSE(log.logging_started());
}

TEST_CASE("FileBackend EraseAll truncates and WriteBlock works after",
          "[logger][file][erase]") {
    using fwcpp::logger::FileBackend;

    std::FILE* f = std::tmpfile();
    REQUIRE(f != nullptr);
    FileBackend log(f);
    const std::uint8_t first[] = {0xAA, 0xBB, 0xCC};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(first, sizeof(first))));
    REQUIRE(std::fflush(f) == 0);
    REQUIRE(std::fseek(f, 0, SEEK_END) == 0);
    REQUIRE(std::ftell(f) == static_cast<long>(sizeof(first)));

    log.EraseAll(false);
    REQUIRE(log.was_logging());
    REQUIRE(log.logging_started());
    REQUIRE(std::fseek(f, 0, SEEK_END) == 0);
    REQUIRE(std::ftell(f) == 0);

    const std::uint8_t second[] = {0x11, 0x22};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(second, sizeof(second))));
    REQUIRE(log.num_short_writes() == 0);
    REQUIRE(std::fflush(f) == 0);
    REQUIRE(std::fseek(f, 0, SEEK_END) == 0);
    REQUIRE(std::ftell(f) == static_cast<long>(sizeof(second)));
    std::rewind(f);
    std::uint8_t got[2]{};
    REQUIRE(std::fread(got, 1, sizeof(got), f) == sizeof(got));
    REQUIRE(got[0] == 0x11);
    REQUIRE(got[1] == 0x22);
    std::fclose(f);
}
