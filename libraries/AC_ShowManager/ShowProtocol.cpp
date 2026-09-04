#include "ShowProtocol.h"

#include <string.h>

// out-of-line definitions for odr-used constants (C++11)
const uint8_t ShowProtocol::GCS_TO_DRONE;
const uint8_t ShowProtocol::DRONE_TO_GCS;
const uint8_t ShowProtocol::START_CONFIG;
const uint8_t ShowProtocol::ACKNOWLEDGMENT;
const uint8_t ShowProtocol::STATUS_REQUEST;
const uint8_t ShowProtocol::STATUS;
const uint8_t ShowProtocol::CLOCK;
const uint8_t ShowProtocol::START_CONFIG_LEN;
const uint8_t ShowProtocol::ACK_LEN;
const uint8_t ShowProtocol::STATUS_LEN;
const uint8_t ShowProtocol::CLOCK_LEN;

uint8_t ShowProtocol::parse_type(const uint8_t *data, uint8_t len)
{
    if (data == nullptr || len == 0) {
        return 0;
    }
    return data[0];
}

bool ShowProtocol::parse_start_config(const uint8_t *data, uint8_t len,
                                      int32_t &start_tow_sec,
                                      uint8_t &authorization,
                                      int32_t &countdown_ms)
{
    if (data == nullptr || len < START_CONFIG_LEN || data[0] != START_CONFIG) {
        return false;
    }
    memcpy(&start_tow_sec, &data[1], 4);
    authorization = data[5];
    memcpy(&countdown_ms, &data[6], 4);
    return true;
}

uint8_t ShowProtocol::build_ack(uint8_t *buf, uint8_t ack_token, uint8_t result)
{
    buf[0] = ACKNOWLEDGMENT;
    buf[1] = ack_token;
    buf[2] = result;
    return ACK_LEN;
}

uint8_t ShowProtocol::build_status(uint8_t *buf, uint8_t flags, uint8_t stage,
                                   int32_t start_tow_sec, uint32_t duration_ms)
{
    buf[0] = STATUS;
    buf[1] = flags;
    buf[2] = stage;
    memcpy(&buf[3], &start_tow_sec, 4);
    memcpy(&buf[7], &duration_ms, 4);
    return STATUS_LEN;
}

// build_clock - CLOCK payload for DATA32
uint8_t ShowProtocol::build_clock(uint8_t *buf, uint8_t flags, uint8_t stage,
                                  uint8_t sync_mode, uint64_t gps_epoch_us,
                                  uint64_t internal_us)
{
    buf[0] = CLOCK;
    buf[1] = flags;
    buf[2] = stage;
    buf[3] = sync_mode;
    memcpy(&buf[4], &gps_epoch_us, 8);
    memcpy(&buf[12], &internal_us, 8);
    return CLOCK_LEN;
}

// parse_clock - decode a CLOCK payload
bool ShowProtocol::parse_clock(const uint8_t *data, uint8_t len,
                               uint8_t &flags, uint8_t &stage, uint8_t &sync_mode,
                               uint64_t &gps_epoch_us, uint64_t &internal_us)
{
    if (len < CLOCK_LEN || data[0] != CLOCK) {
        return false;
    }
    flags = data[1];
    stage = data[2];
    sync_mode = data[3];
    memcpy(&gps_epoch_us, &data[4], 8);
    memcpy(&internal_us, &data[12], 8);
    return true;
}
