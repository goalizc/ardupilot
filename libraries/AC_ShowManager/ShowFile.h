#pragma once

#include <stdint.h>

/*
  Drone show choreography file format v2 - time-ordered event stream.

  File layout (little-endian, fixed-size integers, no padding):
    magic:        "SHOW" (4 bytes)
    header:       28 bytes (version, flags, drone_id, duration_ms,
                   event_count, keyframe_count, light_count,
                   segment_count, reserved, crc32)
    segments:     segment_count x 12 bytes
    event stream: event_count frames, sorted by t_ms (monotonic
                   non-decreasing); at the same t_ms position frames
                   precede light frames

  Each event frame:
    type(1B) t_ms(4B) <payload>
      type 1 = position frame: pos_x/y/z int32 mm, vel_x/y/z int16 mm/s,
               yaw int16 0.01 deg            (25 bytes total)
      type 2 = light frame:   index u8, r/g/b u8  (9 bytes total)
    Unknown types are rejected at parse time; new types may be added
    without a format bump as long as they are self-describing by type.

  The crc32 field holds the CRC of every byte after the 4 magic bytes,
  excluding the crc32 field itself (i.e. bytes [4, 24) concatenated with
  bytes [28, end of file)), computed with AP_Math crc_crc32() (init 0,
  no final XOR, poly 0xEDB88320).
  Coordinates are in the show frame: NED, down positive, millimetres.

  The stream is played sequentially from storage with a sliding window
  (see ShowStreamReader); no frame count limit is imposed beyond the
  uint32 header fields and what fits on the storage medium.  Semantic
  validation (ranges, timing) is the responsibility of the show
  authoring tool; the firmware only verifies the file CRC before flight.
*/

namespace ShowFile {

    static const uint8_t MAGIC[4] = { 'S', 'H', 'O', 'W' };
    static const uint8_t FORMAT_VERSION = 2;

    // event frame types
    static const uint8_t EVENT_POSITION = 1;
    static const uint8_t EVENT_LIGHT = 2;

    // header field offsets (bytes after the magic)
    static const uint8_t HEADER_SIZE = 28;
    static const uint8_t CRC_OFFSET = 24;

    // per-field range limits (enforced by the authoring tool)
    static const int32_t POS_LIMIT_MM = 10000000;   // 10 km
    static const int32_t VEL_LIMIT_MMS = 30000;     // 30 m/s (fits in int16)
    static const int16_t YAW_LIMIT_CD = 18000;      // 180 deg

    // a single trajectory keyframe (position event payload)
    struct Keyframe {
        uint32_t t_ms;          // time since show start
        int32_t pos_x_mm;       // show coordinates, NED (down positive)
        int32_t pos_y_mm;
        int32_t pos_z_mm;
        int16_t vel_x_mms;
        int16_t vel_y_mms;
        int16_t vel_z_mms;
        int16_t yaw_cd;         // 0.01 deg, 0 = no yaw control
    };

    // a single light event (light event payload)
    struct LightEvent {
        uint32_t t_ms;
        uint8_t index;          // 0 = overall colour, 1..255 = per-pixel
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    // a show segment (12 bytes in file) - display metadata only
    struct Segment {
        char name[5];           // 4 chars + NUL
        uint32_t start_ms;
        uint32_t end_ms;
    };
}
