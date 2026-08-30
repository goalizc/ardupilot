#pragma once

#include <AP_Param/AP_Param.h>

/*
  AC_ShowManager - library managing the drone show (choreography flight
  show). Loads the choreography, plays the timeline on a GPS-time base and
  generates position/velocity targets for the guided controller, plus
  synchronised RGB light output.

  P0: skeleton holding the SHOW_ parameter table only. Later phases add
  choreography loading (P1), time sync and the state machine (P2),
  trajectory playback (P3) and lights (P4).
*/
class AC_ShowManager {
public:

    // Constructor
    AC_ShowManager(void);

    CLASS_NO_COPY(AC_ShowManager);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameters (SHOW_*)
    AP_Float _ctrl_rate_hz;         // show player control update rate (Hz)
    AP_Int32 _start_time_gps_sec;   // show start time as GPS time-of-week (s), -1 if unset
    AP_Int32 _start_time_msec;      // extra millisecond offset for the start time
    AP_Int8 _sync_mode;             // timeline time base: 0=GPS time, 1=internal clock
    AP_Int32 _origin_lat;           // show origin latitude (1e-7 deg)
    AP_Int32 _origin_lng;           // show origin longitude (1e-7 deg)
    AP_Int32 _origin_amsl_mm;       // show origin AMSL altitude (mm)
    AP_Float _orientation_deg;      // CW rotation of show X axis from North (deg), -1 if unset
    AP_Float _max_xy_err_m;         // max horizontal trajectory tracking error (m)
    AP_Float _max_z_err_m;          // max vertical trajectory tracking error (m)
};
