#include "AC_ShowManager.h"

#include <sys/stat.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#ifndef HAL_BOARD_SHOW_DIRECTORY
#  if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#    define HAL_BOARD_SHOW_DIRECTORY "./show"
#  else
#    define HAL_BOARD_SHOW_DIRECTORY "/SHOW"
#  endif
#endif
#define SHOW_FILE (HAL_BOARD_SHOW_DIRECTORY "/show.bin")

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

// update - called at 50Hz from the scheduler
void AC_ShowManager::update()
{
    // attempt the initial load once at startup; later loads are
    // command-triggered via reload()
    if (!_load_attempted) {
        _load_attempted = true;
        load_from_file(false);
    }
}

// reload - reload the show file from storage
bool AC_ShowManager::reload()
{
    if (AP::motors() == nullptr || AP::motors()->armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Show: reload refused while armed");
        return false;
    }
    return load_from_file(true);
}

// clear - clear the loaded show and delete the show file
bool AC_ShowManager::clear()
{
    if (AP::motors() == nullptr || AP::motors()->armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Show: clear refused while armed");
        return false;
    }
    if (AP::FS().unlink(SHOW_FILE) != 0 && errno != ENOENT) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: failed to delete show file");
        return false;
    }
    _parser.reset();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Show cleared");
    return true;
}

// load_from_file - read and parse the show file from storage
bool AC_ShowManager::load_from_file(bool report)
{
    // tell the scheduler to expect slow SD card IO
    EXPECT_DELAY_MS(3000);

    struct stat st;
    if (AP::FS().stat(SHOW_FILE, &st) != 0) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Show file not found");
        }
        return true;
    }
    if (st.st_size <= 0 || st.st_size > 65536) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: file size %ld unsupported", (long)st.st_size);
        return false;
    }

    const int fd = AP::FS().open(SHOW_FILE, O_RDONLY);
    if (fd == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: failed to open show file");
        return false;
    }

    uint8_t *buf = (uint8_t *)malloc(st.st_size);
    if (buf == nullptr) {
        AP::FS().close(fd);
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: out of memory loading show");
        return false;
    }

    bool ok = false;
    const int32_t read_len = AP::FS().read(fd, buf, st.st_size);
    if (read_len == st.st_size) {
        ok = _parser.parse(buf, st.st_size);
        if (ok) {
            // keep the text within the 50 char STATUSTEXT limit
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Show loaded: %u keyframes, %u lights, %u segments",
                          _parser.keyframe_count(), _parser.light_count(),
                          _parser.segment_count());
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show load failed: %s", failure_string(_parser.failure()));
        }
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: read error loading show file");
    }

    free(buf);
    AP::FS().close(fd);
    return ok;
}

// failure_string - map a parser failure to a human readable string
const char *AC_ShowManager::failure_string(ShowFileParser::Failure failure) const
{
    switch (failure) {
    case ShowFileParser::Failure::NONE:
        return "ok";
    case ShowFileParser::Failure::INVALID_MAGIC:
        return "invalid magic";
    case ShowFileParser::Failure::UNSUPPORTED_VERSION:
        return "unsupported version";
    case ShowFileParser::Failure::BAD_CRC:
        return "bad checksum";
    case ShowFileParser::Failure::TRUNCATED:
        return "truncated file";
    case ShowFileParser::Failure::EXTRA_DATA:
        return "extra data";
    case ShowFileParser::Failure::TOO_MANY_KEYFRAMES:
        return "too many keyframes";
    case ShowFileParser::Failure::TOO_MANY_LIGHT_EVENTS:
        return "too many light events";
    case ShowFileParser::Failure::TOO_MANY_SEGMENTS:
        return "too many segments";
    case ShowFileParser::Failure::BAD_KEYFRAME_TIME:
        return "keyframe time out of order";
    case ShowFileParser::Failure::POS_OUT_OF_RANGE:
        return "position out of range";
    case ShowFileParser::Failure::VEL_OUT_OF_RANGE:
        return "velocity out of range";
    case ShowFileParser::Failure::YAW_OUT_OF_RANGE:
        return "yaw out of range";
    case ShowFileParser::Failure::BAD_LIGHT_TIME:
        return "light time out of order";
    case ShowFileParser::Failure::BAD_SEGMENT:
        return "bad segment bounds";
    }
    return "unknown";
}
