#include "ShowFileParser.h"

#include <AP_Math/crc.h>

bool ShowFileParser::read_u8(const uint8_t *&p, const uint8_t *end, uint8_t &v) const
{
    if (p >= end) {
        return false;
    }
    v = *p++;
    return true;
}

bool ShowFileParser::read_u16(const uint8_t *&p, const uint8_t *end, uint16_t &v) const
{
    if (end - p < 2) {
        return false;
    }
    v = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    p += 2;
    return true;
}

bool ShowFileParser::read_u32(const uint8_t *&p, const uint8_t *end, uint32_t &v) const
{
    if (end - p < 4) {
        return false;
    }
    v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    p += 4;
    return true;
}

bool ShowFileParser::read_i16(const uint8_t *&p, const uint8_t *end, int16_t &v) const
{
    uint16_t tmp;
    if (!read_u16(p, end, tmp)) {
        return false;
    }
    v = (int16_t)tmp;
    return true;
}

bool ShowFileParser::read_i32(const uint8_t *&p, const uint8_t *end, int32_t &v) const
{
    uint32_t tmp;
    if (!read_u32(p, end, tmp)) {
        return false;
    }
    v = (int32_t)tmp;
    return true;
}

// reset all state so a new parse can be attempted
void ShowFileParser::reset()
{
    _loaded = false;
    _failure = Failure::NONE;
    _drone_id = 0;
    _duration_ms = 0;
    _event_count = 0;
    _keyframe_count = 0;
    _light_count = 0;
    _segment_count = 0;
}

// parse_header - magic, 28-byte header and segments; metadata only
bool ShowFileParser::parse_header(const uint8_t *data, uint32_t len)
{
    reset();

    const uint8_t *p = data;
    const uint8_t *end = data + len;

    // magic
    if (len < 4 || p[0] != ShowFile::MAGIC[0] || p[1] != ShowFile::MAGIC[1] ||
        p[2] != ShowFile::MAGIC[2] || p[3] != ShowFile::MAGIC[3]) {
        _failure = Failure::INVALID_MAGIC;
        return false;
    }
    p += 4;
    if (len < 4 + ShowFile::HEADER_SIZE) {
        _failure = Failure::TRUNCATED;
        return false;
    }

    uint8_t version;
    uint8_t flags;
    if (!read_u8(p, end, version) || !read_u8(p, end, flags) ||
        !read_u16(p, end, _drone_id) || !read_u32(p, end, _duration_ms) ||
        !read_u32(p, end, _event_count) || !read_u32(p, end, _keyframe_count) ||
        !read_u32(p, end, _light_count) || !read_u8(p, end, _segment_count)) {
        _failure = Failure::TRUNCATED;
        return false;
    }
    // skip 3 reserved bytes and the stored crc32 (bytes 21..28)
    p += 3;
    if (end - p < 4) {
        _failure = Failure::TRUNCATED;
        return false;
    }
    p += 4;

    if (version != ShowFile::FORMAT_VERSION) {
        _failure = Failure::UNSUPPORTED_VERSION;
        return false;
    }
    if (_event_count != (uint64_t)_keyframe_count + _light_count) {
        _failure = Failure::BAD_EVENT_COUNT;
        return false;
    }

    // segments themselves are display metadata inside the CRC-covered
    // span; they are not interpreted here (only their count is kept).
    _loaded = true;
    return true;
}

// crc_accumulate - crc32 over one block at a known file offset,
// skipping the magic bytes [0,4) and the stored-crc field
// [4+CRC_OFFSET, 4+CRC_OFFSET+4) (absolute file offsets)
uint32_t ShowFileParser::crc_accumulate(const uint8_t *data, uint32_t len,
                                        uint32_t file_offset, uint32_t crc)
{
    for (uint32_t i = 0; i < len; i++) {
        const uint32_t off = file_offset + i;
        if (off < 4 || (off >= 4 + ShowFile::CRC_OFFSET &&
                        off < 4 + ShowFile::CRC_OFFSET + 4)) {
            continue;
        }
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return crc;
}

// parse_event - parse one v2 event frame and advance p past it
bool ShowFileParser::parse_event(const uint8_t *&p, const uint8_t *end,
                                 uint8_t &type, uint32_t &t_ms,
                                 ShowFile::Keyframe &kf, ShowFile::LightEvent &le) const
{
    if (!read_u8(p, end, type)) {
        return false;
    }
    if (!read_u32(p, end, t_ms)) {
        return false;
    }
    switch (type) {
    case ShowFile::EVENT_POSITION: {
        // payload: pos x/y/z (12) + vel x/y/z (6) + yaw (2) = 20 bytes
        if (end - p < 20) {
            return false;
        }
        kf.t_ms = t_ms;
        if (!read_i32(p, end, kf.pos_x_mm) || !read_i32(p, end, kf.pos_y_mm) ||
            !read_i32(p, end, kf.pos_z_mm) ||
            !read_i16(p, end, kf.vel_x_mms) || !read_i16(p, end, kf.vel_y_mms) ||
            !read_i16(p, end, kf.vel_z_mms) || !read_i16(p, end, kf.yaw_cd)) {
            return false;
        }
        return true;
    }
    case ShowFile::EVENT_LIGHT: {
        if (end - p < 4) {
            return false;
        }
        le.t_ms = t_ms;
        if (!read_u8(p, end, le.index) || !read_u8(p, end, le.r) ||
            !read_u8(p, end, le.g) || !read_u8(p, end, le.b)) {
            return false;
        }
        return true;
    }
    default:
        return false;
    }
}
