#include "AC_PrecLand_QrCode.h"

#if AC_PRECLAND_QRCODE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

static constexpr uint32_t kMagicNumber = 0xA8;
static constexpr float kQrCodeWidth = 0.4f;

struct QrCodeRecord {
    float angle1, angle2;
    float scale;
    uint32_t elapsed;
    uint32_t payload;
};

void AC_PrecLand_QrCode::init(void) {
    auto& manager = AP::serialmanager();
    if ((_uart = manager.find_serial(AP_SerialManager::SerialProtocol_QrCode, 0)))
        _uart->begin(manager.find_baudrate(AP_SerialManager::SerialProtocol_QrCode, 0));
    _state.healthy = _uart && _uart->is_initialized();
}

void AC_PrecLand_QrCode::update(void) {
    while (_uart && _uart->available()) {
        int8_t flag = _uart->read();
        if (kMagicNumber != (flag & 0xF8))
            continue;
        QrCodeRecord qcr;
        for (int8_t count = flag & 7, i = 0; i < count; ++i) {
            if (sizeof(qcr) != _uart->read((uint8_t*)&qcr, sizeof(qcr)))
                return;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "uart read: %.2f, %.2f, %.2f, %lu, %lu", 
                qcr.angle1, qcr.angle2, qcr.scale, qcr.elapsed, qcr.payload);
            if (12345 == qcr.payload) {
                float alt;
                AP::ahrs().get_relative_position_D_home(alt);
                if (alt < 0) {
                    float dist = qcr.scale * kQrCodeWidth;
                    _los_meas_body = Vector3f(dist * cosf(qcr.angle2), dist * sinf(qcr.angle2), -alt);
                    _los_meas_time_ms = AP_HAL::millis() - qcr.elapsed;
                    _have_los_meas = true;
                    _distance_to_target = hypotf(dist, -alt);
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand_QrCode: (%.2f, %.2f, %.2f)",
                        _los_meas_body.x, _los_meas_body.y, _los_meas_body.z);
                }
                return;
            }
        }
    }
}

#endif // AC_PRECLAND_QRCODE_ENABLED