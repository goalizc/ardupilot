#pragma once

#include <stdint.h>

/*
  ShowProtocol - codec for the custom drone-show MAVLink packets.

  The show control protocol rides on the standard MAVLink DATA16/32/64/96
  messages (D4-A: no mavlink submodule changes).  DATA.type selects the
  direction: 0x5c GCS-to-drone, 0x5d drone-to-GCS.  data[0] carries the
  inner packet type; the remaining bytes are the payload, laid out as
  little-endian fixed-size integers (same convention as ShowFile).
*/
class ShowProtocol {
public:
    // DATA.type direction markers
    static const uint8_t GCS_TO_DRONE = 0x5c;
    static const uint8_t DRONE_TO_GCS = 0x5d;

    // inner packet types (data[0])
    static const uint8_t START_CONFIG = 1;
    static const uint8_t ACKNOWLEDGMENT = 4;
    static const uint8_t STATUS_REQUEST = 6;
    static const uint8_t STATUS = 7;

    // minimum payload lengths (data[0] included)
    static const uint8_t START_CONFIG_LEN = 10;
    static const uint8_t ACK_LEN = 3;
    static const uint8_t STATUS_LEN = 12;

    // inner packet type from a received payload; 0 if empty
    static uint8_t parse_type(const uint8_t *data, uint8_t len);

    // decode a START_CONFIG payload (data[0] must be START_CONFIG and
    // len >= START_CONFIG_LEN); fills start_tow_sec / authorization /
    // countdown_ms. returns false on type/size mismatch.
    static bool parse_start_config(const uint8_t *data, uint8_t len,
                                   int32_t &start_tow_sec,
                                   uint8_t &authorization,
                                   int32_t &countdown_ms);

    // build an ACKNOWLEDGMENT payload into buf (buf[0]=type); returns the
    // payload length (ACK_LEN)
    static uint8_t build_ack(uint8_t *buf, uint8_t ack_token, uint8_t result);

    // build a STATUS payload into buf; returns the payload length (STATUS_LEN)
    static uint8_t build_status(uint8_t *buf, uint8_t flags, uint8_t stage,
                                int32_t start_tow_sec, uint32_t duration_ms);
};
