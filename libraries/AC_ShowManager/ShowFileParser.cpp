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
    _keyframe_count = 0;
    _light_count = 0;
    _segment_count = 0;
}

bool ShowFileParser::parse(const uint8_t *data, uint32_t len)
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

    // header (the crc32 field covers everything from here to the end of the file)
    uint8_t version;
    uint8_t flags;
    uint16_t drone_id;
    uint32_t duration_ms;
    uint16_t keyframe_count;
    uint16_t light_count;
    uint8_t segment_count;
    uint32_t stored_crc;
    if (!read_u8(p, end, version) || !read_u8(p, end, flags) ||
        !read_u16(p, end, drone_id) || !read_u32(p, end, duration_ms) ||
        !read_u16(p, end, keyframe_count) || !read_u16(p, end, light_count) ||
        !read_u8(p, end, segment_count)) {
        _failure = Failure::TRUNCATED;
        return false;
    }
    // skip 3 reserved bytes
    p += 3;
    if (!read_u32(p, end, stored_crc)) {
        _failure = Failure::TRUNCATED;
        return false;
    }

    if (version != ShowFile::FORMAT_VERSION) {
        _failure = Failure::UNSUPPORTED_VERSION;
        return false;
    }

    // exact length check
    const uint32_t expected = ShowFile::HEADER_SIZE +
        (uint32_t)segment_count * 12 + (uint32_t)keyframe_count * 24 + (uint32_t)light_count * 8;
    if (len < expected) {
        _failure = Failure::TRUNCATED;
        return false;
    }
    if (len > expected) {
        _failure = Failure::EXTRA_DATA;
        return false;
    }

    // crc over all bytes after the magic, excluding the crc32 field itself
    // (the field is at [CRC_OFFSET, CRC_OFFSET+4), covered in two runs)
    uint32_t calc_crc = crc_crc32(0, data + 4, ShowFile::CRC_OFFSET - 4);
    calc_crc = crc_crc32(calc_crc, data + ShowFile::CRC_OFFSET + 4, len - (ShowFile::CRC_OFFSET + 4));
    if (calc_crc != stored_crc) {
        _failure = Failure::BAD_CRC;
        return false;
    }

    // count limits
    if (keyframe_count > ShowFile::MAX_KEYFRAMES) {
        _failure = Failure::TOO_MANY_KEYFRAMES;
        return false;
    }
    if (light_count > ShowFile::MAX_LIGHT_EVENTS) {
        _failure = Failure::TOO_MANY_LIGHT_EVENTS;
        return false;
    }
    if (segment_count > ShowFile::MAX_SEGMENTS) {
        _failure = Failure::TOO_MANY_SEGMENTS;
        return false;
    }

    // segments
    for (uint8_t i = 0; i < segment_count; i++) {
        uint8_t ch;
        for (uint8_t j = 0; j < 4; j++) {
            if (!read_u8(p, end, ch)) {
                _failure = Failure::TRUNCATED;
                return false;
            }
            _segments[i].name[j] = (char)ch;
        }
        _segments[i].name[4] = 0;
        uint32_t start_ms, end_ms;
        if (!read_u32(p, end, start_ms) || !read_u32(p, end, end_ms)) {
            _failure = Failure::TRUNCATED;
            return false;
        }
        if (start_ms > end_ms || end_ms > duration_ms) {
            _failure = Failure::BAD_SEGMENT;
            return false;
        }
        _segments[i].start_ms = start_ms;
        _segments[i].end_ms = end_ms;
    }

    // keyframes
    uint32_t last_t_ms = 0;
    for (uint16_t i = 0; i < keyframe_count; i++) {
        ShowFile::Keyframe &kf = _keyframes[i];
        uint32_t t_ms;
        if (!read_u32(p, end, t_ms)) {
            _failure = Failure::TRUNCATED;
            return false;
        }
        if (t_ms < last_t_ms || t_ms > duration_ms) {
            _failure = Failure::BAD_KEYFRAME_TIME;
            return false;
        }
        int32_t px, py, pz;
        int16_t vx, vy, vz, yaw_cd;
        if (!read_i32(p, end, px) || !read_i32(p, end, py) || !read_i32(p, end, pz) ||
            !read_i16(p, end, vx) || !read_i16(p, end, vy) || !read_i16(p, end, vz) ||
            !read_i16(p, end, yaw_cd)) {
            _failure = Failure::TRUNCATED;
            return false;
        }
        if (px > ShowFile::POS_LIMIT_MM || px < -ShowFile::POS_LIMIT_MM ||
            py > ShowFile::POS_LIMIT_MM || py < -ShowFile::POS_LIMIT_MM ||
            pz > ShowFile::POS_LIMIT_MM || pz < -ShowFile::POS_LIMIT_MM) {
            _failure = Failure::POS_OUT_OF_RANGE;
            return false;
        }
        if (vx > ShowFile::VEL_LIMIT_MMS || vx < -ShowFile::VEL_LIMIT_MMS ||
            vy > ShowFile::VEL_LIMIT_MMS || vy < -ShowFile::VEL_LIMIT_MMS ||
            vz > ShowFile::VEL_LIMIT_MMS || vz < -ShowFile::VEL_LIMIT_MMS) {
            _failure = Failure::VEL_OUT_OF_RANGE;
            return false;
        }
        if (yaw_cd > ShowFile::YAW_LIMIT_CD || yaw_cd < -ShowFile::YAW_LIMIT_CD) {
            _failure = Failure::YAW_OUT_OF_RANGE;
            return false;
        }
        kf.t_ms = t_ms;
        kf.pos_x_mm = px;
        kf.pos_y_mm = py;
        kf.pos_z_mm = pz;
        kf.vel_x_mms = vx;
        kf.vel_y_mms = vy;
        kf.vel_z_mms = vz;
        kf.yaw_cd = yaw_cd;
        last_t_ms = t_ms;
    }

    // light events
    last_t_ms = 0;
    for (uint16_t i = 0; i < light_count; i++) {
        ShowFile::LightEvent &le = _lights[i];
        uint32_t t_ms;
        uint8_t index, r, g, b;
        if (!read_u32(p, end, t_ms) || !read_u8(p, end, index) ||
            !read_u8(p, end, r) || !read_u8(p, end, g) || !read_u8(p, end, b)) {
            _failure = Failure::TRUNCATED;
            return false;
        }
        if (t_ms < last_t_ms || t_ms > duration_ms) {
            _failure = Failure::BAD_LIGHT_TIME;
            return false;
        }
        le.t_ms = t_ms;
        le.index = index;
        le.r = r;
        le.g = g;
        le.b = b;
        last_t_ms = t_ms;
    }

    _drone_id = drone_id;
    _duration_ms = duration_ms;
    _keyframe_count = keyframe_count;
    _light_count = light_count;
    _segment_count = segment_count;
    _loaded = true;
    return true;
}
