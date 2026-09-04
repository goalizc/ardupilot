#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <AC_ShowManager/ShowProtocol.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ShowProtocol, PacketTypeConstants)
{
    EXPECT_EQ(ShowProtocol::GCS_TO_DRONE, 0x5c);
    EXPECT_EQ(ShowProtocol::DRONE_TO_GCS, 0x5d);
    EXPECT_EQ(ShowProtocol::START_CONFIG, 1);
    EXPECT_EQ(ShowProtocol::ACKNOWLEDGMENT, 4);
    EXPECT_EQ(ShowProtocol::STATUS_REQUEST, 6);
    EXPECT_EQ(ShowProtocol::STATUS, 7);
}

TEST(ShowProtocol, ParseType)
{
    uint8_t data[16] = {ShowProtocol::START_CONFIG};
    EXPECT_EQ(ShowProtocol::parse_type(data, sizeof(data)), ShowProtocol::START_CONFIG);
    // empty payload: no type
    EXPECT_EQ(ShowProtocol::parse_type(data, 0), 0);
}

TEST(ShowProtocol, ParseStartConfigValid)
{
    uint8_t data[16] = {ShowProtocol::START_CONFIG};
    // start_tow_sec = 215025 (LE: 215025 = 0x347F1), authorization = 1,
    // countdown_ms = 5000 (LE: 5000 = 0x1388)
    memcpy(&data[1], "\xF1\x47\x03\x00", 4);   // 215025 = 0x347F1
    data[5] = 1;
    memcpy(&data[6], "\x88\x13\x00\x00", 4);   // 5000 = 0x1388
    int32_t tow = 0; uint8_t auth = 0xFF; int32_t cd = 0;
    EXPECT_TRUE(ShowProtocol::parse_start_config(data, sizeof(data), tow, auth, cd));
    EXPECT_EQ(tow, 215025);
    EXPECT_EQ(auth, 1);
    EXPECT_EQ(cd, 5000);
}

TEST(ShowProtocol, ParseStartConfigRejectsBadTypeOrShort)
{
    uint8_t data[16] = {ShowProtocol::STATUS_REQUEST};
    int32_t tow; uint8_t auth; int32_t cd;
    // wrong inner type
    EXPECT_FALSE(ShowProtocol::parse_start_config(data, sizeof(data), tow, auth, cd));
    // too short (type byte + 9 payload bytes needed)
    uint8_t short_data[5] = {ShowProtocol::START_CONFIG, 0, 0, 0, 0};
    EXPECT_FALSE(ShowProtocol::parse_start_config(short_data, sizeof(short_data), tow, auth, cd));
}

TEST(ShowProtocol, BuildAck)
{
    uint8_t buf[16];
    uint8_t n = ShowProtocol::build_ack(buf, 7, 1);
    EXPECT_EQ(n, 3U);   // type + token + result
    EXPECT_EQ(buf[0], ShowProtocol::ACKNOWLEDGMENT);
    EXPECT_EQ(buf[1], 7);
    EXPECT_EQ(buf[2], 1);
}

TEST(ShowProtocol, BuildStatus)
{
    uint8_t buf[32];
    // flags = 0x05 (loaded + authorized), stage = 3 (performing), tow = 215025, duration = 20000
    uint8_t n = ShowProtocol::build_status(buf, 0x05, 3, 215025, 20000);
    EXPECT_EQ(n, 12U);   // type(1) + flags(1) + stage(1) + tow(4) + duration(4)
    EXPECT_EQ(buf[0], ShowProtocol::STATUS);
    EXPECT_EQ(buf[1], 0x05);
    EXPECT_EQ(buf[2], 3);
    int32_t tow; memcpy(&tow, &buf[3], 4);
    EXPECT_EQ(tow, 215025);
    uint32_t dur; memcpy(&dur, &buf[7], 4);
    EXPECT_EQ(dur, 20000U);
}

TEST(ShowProtocol, BuildAndParseClock)
{
    uint8_t buf[ShowProtocol::CLOCK_LEN];
    const uint64_t gps = 1788454421853615ULL;
    const uint64_t internal = 1234567890123ULL;
    const uint8_t n = ShowProtocol::build_clock(buf, 0x03, 3, 0, gps, internal);
    EXPECT_EQ(n, ShowProtocol::CLOCK_LEN);
    EXPECT_EQ(buf[0], ShowProtocol::CLOCK);
    uint8_t flags, stage, sync_mode;
    uint64_t gps_out, internal_out;
    EXPECT_TRUE(ShowProtocol::parse_clock(buf, n, flags, stage, sync_mode,
                                          gps_out, internal_out));
    EXPECT_EQ(flags, 0x03);
    EXPECT_EQ(stage, 3);
    EXPECT_EQ(sync_mode, 0);
    EXPECT_EQ(gps_out, gps);
    EXPECT_EQ(internal_out, internal);
    // reject wrong type or short payload
    uint8_t bad[ShowProtocol::CLOCK_LEN];
    memcpy(bad, buf, sizeof(bad));
    bad[0] = ShowProtocol::STATUS;
    EXPECT_FALSE(ShowProtocol::parse_clock(bad, n, flags, stage, sync_mode,
                                           gps_out, internal_out));
    EXPECT_FALSE(ShowProtocol::parse_clock(buf, ShowProtocol::CLOCK_LEN - 1,
                                           flags, stage, sync_mode,
                                           gps_out, internal_out));
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
