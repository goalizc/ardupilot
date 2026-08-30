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

// CRC-32 identical to AP_Math crc_crc32(): init 0, no final xor, poly 0xEDB88320
static uint32_t crc32_ap(const uint8_t *data, uint32_t size, uint32_t crc = 0)
{
    for (uint32_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(crc & 1));
        }
    }
    return crc;
}

// recompute the crc field after mutating a buffer. The crc covers all
// bytes after the magic, excluding the crc32 field itself.
static void fix_crc(std::vector<uint8_t> &b)
{
    uint32_t crc = crc32_ap(b.data() + 4, ShowFile::CRC_OFFSET - 4);
    crc = crc32_ap(b.data() + ShowFile::CRC_OFFSET + 4, b.size() - (ShowFile::CRC_OFFSET + 4), crc);
    b[20] = crc & 0xff;
    b[21] = (crc >> 8) & 0xff;
    b[22] = (crc >> 16) & 0xff;
    b[23] = (crc >> 24) & 0xff;
}

// build a valid v1 show file: 1 segment, keyframe_count keyframes, 2 light events.
// keyframe times span [0, duration_ms] evenly (last keyframe == duration_ms).
static std::vector<uint8_t> build_file(uint16_t keyframe_count = 3,
                                       uint32_t duration_ms = 5000)
{
    std::vector<uint8_t> b;
    b.insert(b.end(), {'S', 'H', 'O', 'W'});
    put_u8(b, ShowFile::FORMAT_VERSION);
    put_u8(b, 0);                 // flags
    put_u16(b, 7);                // drone_id
    put_u32(b, duration_ms);
    put_u16(b, keyframe_count);
    put_u16(b, 2);                // light_count
    put_u8(b, 1);                 // segment_count
    b.insert(b.end(), 3, 0);      // reserved
    put_u32(b, 0);                // crc placeholder

    // one segment covering the whole show
    b.insert(b.end(), {'s', 'e', 'g', '0'});
    put_u32(b, 0);
    put_u32(b, duration_ms);

    // keyframes with monotonically increasing time
    for (uint16_t i = 0; i < keyframe_count; i++) {
        uint32_t t = 0;
        if (keyframe_count > 1) {
            t = (uint32_t)((uint64_t)i * duration_ms / (keyframe_count - 1));
        }
        put_u32(b, t);
        put_i32(b, 0);
        put_i32(b, 100 * i);
        put_i32(b, -5000);
        put_i16(b, 0);
        put_i16(b, 100);
        put_i16(b, 0);
        put_i16(b, 0);
    }

    // light events
    put_u32(b, 0);
    put_u8(b, 0);                 // overall colour
    put_u8(b, 255); put_u8(b, 0); put_u8(b, 0);
    put_u32(b, duration_ms / 2);
    put_u8(b, 1);                 // pixel 1
    put_u8(b, 0); put_u8(b, 255); put_u8(b, 0);

    fix_crc(b);
    return b;
}

// field offsets within the default build (1 segment, 3 keyframes, 2 lights)
static const uint32_t SEG_OFFSET = ShowFile::HEADER_SIZE;            // 24
static const uint32_t KF_OFFSET = SEG_OFFSET + 12;                   // 36
static const uint32_t LIGHT_OFFSET = KF_OFFSET + 3 * 24;             // 108
// within a keyframe
static const uint32_t KF_T = 0;
static const uint32_t KF_POS_X = 4;
static const uint32_t KF_VEL_Y = 18;
static const uint32_t KF_YAW = 22;
static const uint32_t KF_SIZE = 24;

TEST(ShowFileParser, ValidFileParses)
{
    std::vector<uint8_t> data = build_file();
    ShowFileParser parser;
    EXPECT_TRUE(parser.parse(data.data(), data.size()));
    EXPECT_TRUE(parser.loaded());
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::NONE);
    EXPECT_EQ(parser.drone_id(), 7U);
    EXPECT_EQ(parser.duration_ms(), 5000U);
    EXPECT_EQ(parser.keyframe_count(), 3U);
    EXPECT_EQ(parser.light_count(), 2U);
    EXPECT_EQ(parser.segment_count(), 1U);
    // keyframe values
    EXPECT_EQ(parser.keyframes()[0].t_ms, 0U);
    EXPECT_EQ(parser.keyframes()[1].t_ms, 2500U);
    EXPECT_EQ(parser.keyframes()[2].t_ms, 5000U);
    EXPECT_EQ(parser.keyframes()[1].pos_y_mm, 100);
    EXPECT_EQ(parser.keyframes()[2].pos_y_mm, 200);
    EXPECT_EQ(parser.keyframes()[0].pos_z_mm, -5000);
    EXPECT_EQ(parser.keyframes()[1].vel_y_mms, 100);
    // light events
    EXPECT_EQ(parser.lights()[0].index, 0U);
    EXPECT_EQ(parser.lights()[0].r, 255U);
    EXPECT_EQ(parser.lights()[1].index, 1U);
    EXPECT_EQ(parser.lights()[1].g, 255U);
    // segment
    EXPECT_EQ(parser.segments()[0].end_ms, 5000U);
    EXPECT_STREQ(parser.segments()[0].name, "seg0");
}

TEST(ShowFileParser, RejectsBadMagic)
{
    std::vector<uint8_t> data = build_file();
    data[0] = 'X';
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::INVALID_MAGIC);
}

TEST(ShowFileParser, RejectsUnsupportedVersion)
{
    std::vector<uint8_t> data = build_file();
    data[4] = 2;
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::UNSUPPORTED_VERSION);
}

TEST(ShowFileParser, RejectsBadCrc)
{
    std::vector<uint8_t> data = build_file();
    data[KF_OFFSET + 8] ^= 0x01;  // corrupt a keyframe byte without fixing crc
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::BAD_CRC);
}

TEST(ShowFileParser, RejectsTruncatedHeader)
{
    std::vector<uint8_t> data = build_file();
    data.resize(20);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::TRUNCATED);
}

TEST(ShowFileParser, RejectsTruncatedKeyframes)
{
    std::vector<uint8_t> data = build_file();
    data.resize(data.size() - 8);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::TRUNCATED);
}

TEST(ShowFileParser, RejectsExtraData)
{
    std::vector<uint8_t> data = build_file();
    data.push_back(0);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::EXTRA_DATA);
}

TEST(ShowFileParser, RejectsTooManyKeyframes)
{
    std::vector<uint8_t> data = build_file(ShowFile::MAX_KEYFRAMES + 1);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::TOO_MANY_KEYFRAMES);
}

TEST(ShowFileParser, RejectsNonMonotonicTime)
{
    std::vector<uint8_t> data = build_file();
    // third keyframe time (i=2) -> 1000 ms, out of order
    const uint32_t off = KF_OFFSET + 2 * KF_SIZE + KF_T;
    data[off] = 0xE8; data[off + 1] = 0x03; data[off + 2] = 0; data[off + 3] = 0;
    fix_crc(data);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::BAD_KEYFRAME_TIME);
}

TEST(ShowFileParser, RejectsPosOutOfRange)
{
    std::vector<uint8_t> data = build_file();
    // second keyframe pos_x = 10000001 mm (> 10 km)
    const uint32_t off = KF_OFFSET + 1 * KF_SIZE + KF_POS_X;
    data[off] = 0x81; data[off + 1] = 0x96; data[off + 2] = 0x98; data[off + 3] = 0x00;
    fix_crc(data);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::POS_OUT_OF_RANGE);
}

TEST(ShowFileParser, RejectsVelOutOfRange)
{
    std::vector<uint8_t> data = build_file();
    // first keyframe vel_y = 30001 mm/s (> 30 m/s limit, fits in int16)
    const uint32_t off = KF_OFFSET + 0 * KF_SIZE + KF_VEL_Y;
    data[off] = 0x31; data[off + 1] = 0x75;
    fix_crc(data);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::VEL_OUT_OF_RANGE);
}

TEST(ShowFileParser, RejectsYawOutOfRange)
{
    std::vector<uint8_t> data = build_file();
    // first keyframe yaw_cd = 18001 (> 18000)
    const uint32_t off = KF_OFFSET + 0 * KF_SIZE + KF_YAW;
    data[off] = 0x51; data[off + 1] = 0x46;
    fix_crc(data);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::YAW_OUT_OF_RANGE);
}

TEST(ShowFileParser, RejectsLightAfterDuration)
{
    std::vector<uint8_t> data = build_file();
    // second light event t = 5001 ms (> duration 5000)
    const uint32_t off = LIGHT_OFFSET + 8;
    data[off] = 0x89; data[off + 1] = 0x13; data[off + 2] = 0; data[off + 3] = 0;
    fix_crc(data);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::BAD_LIGHT_TIME);
}

TEST(ShowFileParser, RejectsBadSegment)
{
    std::vector<uint8_t> data = build_file();
    // segment start_ms = 6000 > end_ms = 5000
    const uint32_t off = SEG_OFFSET + 4;
    data[off] = 0x70; data[off + 1] = 0x17; data[off + 2] = 0; data[off + 3] = 0;
    fix_crc(data);
    ShowFileParser parser;
    EXPECT_FALSE(parser.parse(data.data(), data.size()));
    EXPECT_EQ(parser.failure(), ShowFileParser::Failure::BAD_SEGMENT);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
