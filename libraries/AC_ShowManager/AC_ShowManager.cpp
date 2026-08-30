#include "AC_ShowManager.h"

const AP_Param::GroupInfo AC_ShowManager::var_info[] = {

    // @Param: CTRL_RATE
    // @DisplayName: Show control update rate
    // @Description: Rate in Hz at which the show player evaluates the choreography timeline and sends position/velocity targets to the guided controller during the show.
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("CTRL_RATE", 1, AC_ShowManager, _ctrl_rate_hz, 10.0f),

    // @Param: START_TIME
    // @DisplayName: Show start time (GPS time of week)
    // @Description: Start time of the show as GPS time-of-week in seconds, -1 if unset. Combined with SHOW_START_MSEC for the exact start instant. Set by the GCS before the show.
    // @Range: -1 604799
    // @Increment: 1
    // @Units: s
    // @Volatile: True
    // @User: Standard
    AP_GROUPINFO("START_TIME", 2, AC_ShowManager, _start_time_gps_sec, -1),

    // @Param: START_MSEC
    // @DisplayName: Show start time millisecond offset
    // @Description: Extra millisecond offset added to SHOW_START_TIME to give the exact show start instant.
    // @Range: 0 999
    // @Increment: 1
    // @Units: ms
    // @Volatile: True
    // @User: Standard
    AP_GROUPINFO("START_MSEC", 3, AC_ShowManager, _start_time_msec, 0),

    // @Param: SYNC_MODE
    // @DisplayName: Show time sync mode
    // @Description: Time base used for the show timeline. GPS time is the primary base shared by all aircraft in the formation; the internal clock is a fallback when GPS time is unavailable.
    // @Values: 0:GPS time,1:Internal clock
    // @User: Advanced
    AP_GROUPINFO("SYNC_MODE", 4, AC_ShowManager, _sync_mode, 0),

    // @Param: ORIGIN_LAT
    // @DisplayName: Show origin latitude
    // @Description: Latitude of the origin of the show coordinate system in 1e-7 degrees, zero if unset.
    // @Range: -900000000 900000000
    // @Increment: 1
    // @Units: 1e-7 deg
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LAT", 5, AC_ShowManager, _origin_lat, 0),

    // @Param: ORIGIN_LNG
    // @DisplayName: Show origin longitude
    // @Description: Longitude of the origin of the show coordinate system in 1e-7 degrees, zero if unset.
    // @Range: -1800000000 1800000000
    // @Increment: 1
    // @Units: 1e-7 deg
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LNG", 6, AC_ShowManager, _origin_lng, 0),

    // @Param: ORIGIN_AMSL
    // @DisplayName: Show origin altitude
    // @Description: AMSL altitude of the origin of the show coordinate system in millimetres.
    // @Range: -10000000 10000000
    // @Increment: 1
    // @Units: mm
    // @User: Standard
    AP_GROUPINFO("ORIGIN_AMSL", 7, AC_ShowManager, _origin_amsl_mm, 0),

    // @Param: ORIENTATION
    // @DisplayName: Show orientation
    // @Description: Clockwise rotation of the X axis of the show coordinate system relative to North in degrees, -1 if unset.
    // @Range: -1 360
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ORIENTATION", 8, AC_ShowManager, _orientation_deg, -1),

    // @Param: MAX_XY_ERR
    // @DisplayName: Maximum horizontal position error
    // @Description: Maximum allowed horizontal distance in metres between the vehicle and the choreography trajectory before the show is considered failed by the drift monitor.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MAX_XY_ERR", 9, AC_ShowManager, _max_xy_err_m, 3.0f),

    // @Param: MAX_Z_ERR
    // @DisplayName: Maximum vertical position error
    // @Description: Maximum allowed vertical distance in metres between the vehicle and the choreography trajectory before the show is considered failed by the drift monitor.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MAX_Z_ERR", 10, AC_ShowManager, _max_z_err_m, 3.0f),

    AP_GROUPEND
};

// Constructor
AC_ShowManager::AC_ShowManager(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
