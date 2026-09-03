#pragma once

#include <stdint.h>
#include "ShowFile.h"

/*
  Parser for the drone show v2 event-stream file format.  Pure logic
  with no HAL dependencies so it can be unit-tested with gtest.

  The v2 file is streamed from storage (see ShowStreamReader); this
  parser only:
    - parses the fixed header and segments (metadata),
    - walks the event stream one frame at a time (parse_event),
    - accumulates the file CRC over streaming blocks (crc_accumulate).
  It does NOT hold the frames in memory.
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
        UNKNOWN_EVENT,
        BAD_EVENT_COUNT,
    };

    // parse the header (magic + 28B header + segments) from the start
    // of the file.  data must hold at least the header plus segments;
    // length checks and CRC are handled by the streaming verifier.
    // returns true on success.
    bool parse_header(const uint8_t *data, uint32_t len);

    // reset all state so a new parse can be attempted
    void reset();

    bool loaded() const { return _loaded; }
    Failure failure() const { return _failure; }

    uint16_t drone_id() const { return _drone_id; }
    uint32_t duration_ms() const { return _duration_ms; }
    uint32_t event_count() const { return _event_count; }
    uint32_t keyframe_count() const { return _keyframe_count; }
    uint32_t light_count() const { return _light_count; }
    uint8_t segment_count() const { return _segment_count; }

    // crc32 (init 0, no final xor, poly 0xEDB88320) over one block of
    // the file at the given absolute file offset.  The magic bytes
    // [0,4) and the stored-crc field [24,28) are skipped so the caller
    // can feed arbitrary read blocks in order.
    static uint32_t crc_accumulate(const uint8_t *data, uint32_t len,
                                   uint32_t file_offset, uint32_t crc);

    // parse one event frame from the stream; advances p past the whole
    // frame.  On success sets type and t_ms and fills the payload of
    // the matching struct (Keyframe for EVENT_POSITION, LightEvent for
    // EVENT_LIGHT).  returns false on overrun or unknown type.
    bool parse_event(const uint8_t *&p, const uint8_t *end,
                     uint8_t &type, uint32_t &t_ms,
                     ShowFile::Keyframe &kf, ShowFile::LightEvent &le) const;

private:

    // bounds-checked little-endian readers; return false on overrun
    bool read_u8(const uint8_t *&p, const uint8_t *end, uint8_t &v) const;
    bool read_u16(const uint8_t *&p, const uint8_t *end, uint16_t &v) const;
    bool read_u32(const uint8_t *&p, const uint8_t *end, uint32_t &v) const;
    bool read_i16(const uint8_t *&p, const uint8_t *end, int16_t &v) const;
    bool read_i32(const uint8_t *&p, const uint8_t *end, int32_t &v) const;

    uint16_t _drone_id;
    uint32_t _duration_ms;
    uint32_t _event_count;
    uint32_t _keyframe_count;
    uint32_t _light_count;
    uint8_t _segment_count;
    bool _loaded;
    Failure _failure;
};
