#pragma once

#include <stdint.h>
#include "ShowFile.h"

/*
  Parser for the drone show choreography file format. Pure logic with no
  HAL dependencies so it can be unit-tested with gtest.
*/
class ShowFileParser {
public:

    // failure reasons reported by the parser
    enum class Failure : uint8_t {
        NONE = 0,
        INVALID_MAGIC,
        UNSUPPORTED_VERSION,
        BAD_CRC,
        TRUNCATED,
        EXTRA_DATA,
        TOO_MANY_KEYFRAMES,
        TOO_MANY_LIGHT_EVENTS,
        TOO_MANY_SEGMENTS,
        BAD_KEYFRAME_TIME,
        POS_OUT_OF_RANGE,
        VEL_OUT_OF_RANGE,
        YAW_OUT_OF_RANGE,
        BAD_LIGHT_TIME,
        BAD_SEGMENT,
    };

    // parse a show file from a memory buffer; returns true on success
    bool parse(const uint8_t *data, uint32_t len);

    // reset all state so a new parse can be attempted
    void reset();

    bool loaded() const { return _loaded; }
    Failure failure() const { return _failure; }

    uint16_t drone_id() const { return _drone_id; }
    uint32_t duration_ms() const { return _duration_ms; }
    uint16_t keyframe_count() const { return _keyframe_count; }
    uint16_t light_count() const { return _light_count; }
    uint8_t segment_count() const { return _segment_count; }

    const ShowFile::Keyframe *keyframes() const { return _keyframes; }
    const ShowFile::LightEvent *lights() const { return _lights; }
    const ShowFile::Segment *segments() const { return _segments; }

private:

    // bounds-checked little-endian readers; return false on overrun
    bool read_u8(const uint8_t *&p, const uint8_t *end, uint8_t &v) const;
    bool read_u16(const uint8_t *&p, const uint8_t *end, uint16_t &v) const;
    bool read_u32(const uint8_t *&p, const uint8_t *end, uint32_t &v) const;
    bool read_i16(const uint8_t *&p, const uint8_t *end, int16_t &v) const;
    bool read_i32(const uint8_t *&p, const uint8_t *end, int32_t &v) const;

    // in-memory show data
    ShowFile::Keyframe _keyframes[ShowFile::MAX_KEYFRAMES];
    ShowFile::LightEvent _lights[ShowFile::MAX_LIGHT_EVENTS];
    ShowFile::Segment _segments[ShowFile::MAX_SEGMENTS];

    uint16_t _drone_id;
    uint32_t _duration_ms;
    uint16_t _keyframe_count;
    uint16_t _light_count;
    uint8_t _segment_count;
    bool _loaded;
    Failure _failure;
};
