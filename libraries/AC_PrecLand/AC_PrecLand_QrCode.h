#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_QRCODE_ENABLED

#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AC_PrecLand_QrCode : public AC_PrecLand_Backend {
public:
    using AC_PrecLand_Backend::AC_PrecLand_Backend;

    // perform any required initialisation of backend
    void init(void) override;
    // retrieve updates from serial
    void update(void) override;

private:
    AP_HAL::UARTDriver* _uart;
};

#endif // AC_PRECLAND_QRCODE_ENABLED