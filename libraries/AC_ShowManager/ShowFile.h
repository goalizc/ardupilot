#pragma once

#include <stdint.h>

/*
  Drone show choreography file format v1.

  File layout (little-endian, fixed-size integers, no padding):
    header:       24 bytes (magic, version, flags, drone_id, duration_ms,
                   keyframe_count, light_count, segment_count, reserved, crc32)
    segments:     segment_count x 12 bytes
    keyframes:    keyframe_count x 24 bytes
    light events: light_count x 8 bytes

  The crc32 field holds the CRC of every byte after the 4 magic bytes,
  excluding the crc32 field itself (i.e. bytes [4, 20) concatenated with
  bytes [24, end of file)), computed with AP_Math crc_crc32() (init 0,
  no final XOR, poly 0xEDB88320).
  Coordinates are in the show frame: NED, down positive, millimetres.
*/
namespace ShowFile {

    static const uint8_t MAGIC[4] = { 'S', 'H', 'O', 'W' };
    static const uint8_t FORMAT_VERSION = 1;

    // header field offsets (bytes)
    static const uint8_t HEADER_SIZE = 24;
    static const uint8_t CRC_OFFSET = 20;

    // in-memory limits; files exceeding these are rejected at load
    static const uint16_t MAX_KEYFRAMES = 2048;
    static const uint16_t MAX_LIGHT_EVENTS = 1024;
    static const uint8_t MAX_SEGMENTS = 16;

    // per-field range limits
    static const int32_t POS_LIMIT_MM = 10000000;   // 10 km
    static const int32_t VEL_LIMIT_MMS = 30000;     // 30 m/s (fits in int16)
    static const int16_t YAW_LIMIT_CD = 18000;      // 180 deg

    // a single trajectory keyframe (24 bytes in file)
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

    // a single light event (8 bytes in file)
    struct LightEvent {
        uint32_t t_ms;
        uint8_t index;          // 0 = overall colour, 1..255 = per-pixel
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    // a show segment (12 bytes in file)
    struct Segment {
        char name[5];           // 4 chars + NUL
        uint32_t start_ms;
        uint32_t end_ms;
    };
}
