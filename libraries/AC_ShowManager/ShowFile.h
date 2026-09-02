#pragma once

#include <stdint.h>

#include <AP_HAL/AP_HAL_Boards.h>

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

// in-memory capacity of one show.  The full choreography is loaded into
// RAM (see the D10 hardware spec: F722/H743-class boards with >=512KB
// RAM), so boards with very little RAM get a reduced capacity so that
// the Copter firmware still links and can run smaller shows.
#ifndef AC_SHOW_MAX_KEYFRAMES
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(HAL_MEMORY_TOTAL_KB) && (HAL_MEMORY_TOTAL_KB < 256)
#define AC_SHOW_MAX_KEYFRAMES 512
#define AC_SHOW_MAX_LIGHT_EVENTS 256
#define AC_SHOW_MAX_SEGMENTS 8
#else
#define AC_SHOW_MAX_KEYFRAMES 2048
#define AC_SHOW_MAX_LIGHT_EVENTS 1024
#define AC_SHOW_MAX_SEGMENTS 16
#endif
#endif

namespace ShowFile {

    static const uint8_t MAGIC[4] = { 'S', 'H', 'O', 'W' };
    static const uint8_t FORMAT_VERSION = 1;

    // header field offsets (bytes)
    static const uint8_t HEADER_SIZE = 24;
    static const uint8_t CRC_OFFSET = 20;

    // in-memory limits; files exceeding these are rejected at load
    static const uint16_t MAX_KEYFRAMES = AC_SHOW_MAX_KEYFRAMES;
    static const uint16_t MAX_LIGHT_EVENTS = AC_SHOW_MAX_LIGHT_EVENTS;
    static const uint8_t MAX_SEGMENTS = AC_SHOW_MAX_SEGMENTS;

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
