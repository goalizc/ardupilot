#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <AC_ShowManager/ShowFileParser.h>

#include <vector>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// little-endian byte writers
static void put_u8(std::vector<uint8_t> &b, uint8_t v)
{
    b.push_back(v);
}
static void put_u16(std::vector<uint8_t> &b, uint16_t v)
{
    b.push_back(v & 0xff);
    b.push_back((v >> 8) & 0xff);
}
static void put_u32(std::vector<uint8_t> &b, uint32_t v)
{
    b.push_back(v & 0xff);
    b.push_back((v >> 8) & 0xff);
    b.push_back((v >> 16) & 0xff);
    b.push_back((v >> 24) & 0xff);
}
static void put_i16(std::vector<uint8_t> &b, int16_t v)
{
    put_u16(b, (uint16_t)v);
}
static void put_i32(std::vector<uint8_t> &b, int32_t v)
{
    put_u32(b, (uint32_t)v);
}

// build a v2 file shell with placeholders for counts and crc
static std::vector<uint8_t> build_v2(uint16_t drone_id, uint32_t duration_ms,
                                     uint8_t version = ShowFile::FORMAT_VERSION,
                                     bool bad_magic = false)
{
    std::vector<uint8_t> b;
    if (bad_magic) {
        b.insert(b.end(), {'X', 'H', 'O', 'W'});
    } else {
        b.insert(b.end(), {'S', 'H', 'O', 'W'});
    }
    put_u8(b, version);
    put_u8(b, 0);                       // flags
    put_u16(b, drone_id);
    put_u32(b, duration_ms);
    put_u32(b, 0);                      // event_count placeholder
    put_u32(b, 0);                      // keyframe_count placeholder
    put_u32(b, 0);                      // light_count placeholder
    put_u8(b, 0);                       // segment_count
    b.insert(b.end(), 3, 0);            // reserved
    b.insert(b.end(), 4, 0);            // crc placeholder at [24,28)
    return b;
}

static void put_position(std::vector<uint8_t> &b, uint32_t t_ms,
                         int32_t x, int32_t y, int32_t z,
                         int16_t vx, int16_t vy, int16_t vz, int16_t yaw)
{
    put_u8(b, ShowFile::EVENT_POSITION);
    put_u32(b, t_ms);
    put_i32(b, x);
    put_i32(b, y);
    put_i32(b, z);
    put_i16(b, vx);
    put_i16(b, vy);
    put_i16(b, vz);
    put_i16(b, yaw);
}

static void put_light(std::vector<uint8_t> &b, uint32_t t_ms, uint8_t index,
                      uint8_t r, uint8_t g, uint8_t bl)
{
    put_u8(b, ShowFile::EVENT_LIGHT);
    put_u32(b, t_ms);
    put_u8(b, index);
    put_u8(b, r);
    put_u8(b, g);
    put_u8(b, bl);
}

// finalise header counts and crc using the parser's own accumulator
static void finalise(std::vector<uint8_t> &b, uint32_t n_pos, uint32_t n_light)
{
    const uint32_t n_events = n_pos + n_light;
    for (int i = 0; i < 4; i++) {
        b[4 + 8 + i] = (n_events >> (8 * i)) & 0xff;
        b[4 + 12 + i] = (n_pos >> (8 * i)) & 0xff;
        b[4 + 16 + i] = (n_light >> (8 * i)) & 0xff;
    }
    uint32_t crc = ShowFileParser::crc_accumulate(b.data(), b.size(), 0, 0);
    for (int i = 0; i < 4; i++) {
        b[4 + ShowFile::CRC_OFFSET + i] = (crc >> (8 * i)) & 0xff;
    }
}

// a simple 3-position + 2-light file (2000ms duration)
static std::vector<uint8_t> simple_file()
{
    std::vector<uint8_t> b = build_v2(7, 2000);
    put_position(b, 0, 0, 0, -5000, 0, 0, 0, 0);
    put_light(b, 0, 0, 255, 0, 0);
    put_position(b, 1000, 1000, 2000, -5000, 0, 0, 0, 0);
    put_light(b, 1000, 0, 0, 255, 0);
    put_position(b, 2000, 2000, 4000, -5000, 0, 0, 0, 0);
    finalise(b, 3, 2);
    return b;
}

TEST(ShowFileParser, HeaderFields)
{
    std::vector<uint8_t> b = simple_file();
    ShowFileParser p;
    EXPECT_TRUE(p.parse_header(b.data(), b.size()));
    EXPECT_TRUE(p.loaded());
    EXPECT_EQ(p.drone_id(), 7);
    EXPECT_EQ(p.duration_ms(), 2000U);
    EXPECT_EQ(p.keyframe_count(), 3U);
    EXPECT_EQ(p.light_count(), 2U);
    EXPECT_EQ(p.event_count(), 5U);
    EXPECT_EQ(p.failure(), ShowFileParser::Failure::NONE);
}

TEST(ShowFileParser, RejectsBadMagic)
{
    std::vector<uint8_t> b = build_v2(7, 1000, ShowFile::FORMAT_VERSION, true);
    ShowFileParser p;
    EXPECT_FALSE(p.parse_header(b.data(), b.size()));
    EXPECT_EQ(p.failure(), ShowFileParser::Failure::INVALID_MAGIC);
}

TEST(ShowFileParser, RejectsBadVersion)
{
    std::vector<uint8_t> b = build_v2(7, 1000, 99);
    ShowFileParser p;
    EXPECT_FALSE(p.parse_header(b.data(), b.size()));
    EXPECT_EQ(p.failure(), ShowFileParser::Failure::UNSUPPORTED_VERSION);
}

TEST(ShowFileParser, RejectsTruncatedHeader)
{
    std::vector<uint8_t> b = simple_file();
    b.resize(4 + 8);    // header cut short
    ShowFileParser p;
    EXPECT_FALSE(p.parse_header(b.data(), b.size()));
    EXPECT_EQ(p.failure(), ShowFileParser::Failure::TRUNCATED);
}

TEST(ShowFileParser, RejectsBadEventCount)
{
    std::vector<uint8_t> b = build_v2(7, 1000);
    put_position(b, 0, 0, 0, -5000, 0, 0, 0, 0);
    finalise(b, 1, 0);
    b[4 + 8] = 99;      // corrupt event_count to 99
    ShowFileParser p;
    EXPECT_FALSE(p.parse_header(b.data(), b.size()));
    EXPECT_EQ(p.failure(), ShowFileParser::Failure::BAD_EVENT_COUNT);
}

TEST(ShowFileParser, ParsePositionEvent)
{
    std::vector<uint8_t> b = simple_file();
    ShowFileParser p;
    ASSERT_TRUE(p.parse_header(b.data(), b.size()));
    const uint8_t *pos = b.data() + 4 + ShowFile::HEADER_SIZE;
    const uint8_t *end = b.data() + b.size();
    uint8_t type;
    uint32_t t_ms;
    ShowFile::Keyframe kf;
    ShowFile::LightEvent le;
    ASSERT_TRUE(p.parse_event(pos, end, type, t_ms, kf, le));
    EXPECT_EQ(type, ShowFile::EVENT_POSITION);
    EXPECT_EQ(t_ms, 0U);
    EXPECT_EQ(kf.pos_x_mm, 0);
    EXPECT_EQ(kf.pos_z_mm, -5000);
    ASSERT_TRUE(p.parse_event(pos, end, type, t_ms, kf, le));
    EXPECT_EQ(type, ShowFile::EVENT_LIGHT);
    EXPECT_EQ(le.r, 255);
    ASSERT_TRUE(p.parse_event(pos, end, type, t_ms, kf, le));
    EXPECT_EQ(type, ShowFile::EVENT_POSITION);
    EXPECT_EQ(t_ms, 1000U);
    EXPECT_EQ(kf.pos_y_mm, 2000);
    while (p.parse_event(pos, end, type, t_ms, kf, le)) {
    }
    EXPECT_EQ(pos, end);
}

TEST(ShowFileParser, RejectsUnknownEventType)
{
    std::vector<uint8_t> b = build_v2(7, 1000);
    put_position(b, 0, 0, 0, -5000, 0, 0, 0, 0);
    put_u8(b, 99);              // unknown event type
    put_u32(b, 100);
    finalise(b, 1, 0);
    ShowFileParser p;
    ASSERT_TRUE(p.parse_header(b.data(), b.size()));
    const uint8_t *pos = b.data() + 4 + ShowFile::HEADER_SIZE;
    const uint8_t *end = b.data() + b.size();
    uint8_t type;
    uint32_t t_ms;
    ShowFile::Keyframe kf;
    ShowFile::LightEvent le;
    ASSERT_TRUE(p.parse_event(pos, end, type, t_ms, kf, le));
    EXPECT_FALSE(p.parse_event(pos, end, type, t_ms, kf, le));
}

TEST(ShowFileParser, CrcAccumulateSkipsMagicAndField)
{
    std::vector<uint8_t> b = simple_file();
    uint32_t crc = ShowFileParser::crc_accumulate(b.data(), b.size(), 0, 0);
    uint32_t stored = 0;
    for (int i = 0; i < 4; i++) {
        stored |= (uint32_t)b[4 + ShowFile::CRC_OFFSET + i] << (8 * i);
    }
    EXPECT_EQ(crc, stored);
    // chunked accumulation gives the same result as a single pass
    uint32_t c2 = ShowFileParser::crc_accumulate(b.data(), 10, 0, 0);
    c2 = ShowFileParser::crc_accumulate(b.data() + 10, b.size() - 10, 10, c2);
    EXPECT_EQ(c2, stored);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
