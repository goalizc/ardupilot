#include "AC_ShowManager.h"

#include <sys/stat.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "ShowProtocol.h"

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
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LAT", 5, AC_ShowManager, _origin_lat, 0),

    // @Param: ORIGIN_LNG
    // @DisplayName: Show origin longitude
    // @Description: Longitude of the origin of the show coordinate system in 1e-7 degrees, zero if unset.
    // @Range: -1800000000 1800000000
    // @Increment: 1
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

    // @Param: TAKEOFF_ALT
    // @DisplayName: Show takeoff altitude
    // @Description: Target altitude above home that the vehicle climbs to when the show starts.
    // @Range: 1 50
    // @Increment: 0.5
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("TAKEOFF_ALT", 11, AC_ShowManager, _takeoff_alt_m, 10.0f),

    // @Param: TAKEOFF_ERR
    // @DisplayName: Show takeoff position error
    // @Description: Maximum horizontal distance in metres between the vehicle and its launch position that is allowed when starting the takeoff, to prevent takeoff from a wrong position.
    // @Range: 0 20
    // @Increment: 0.5
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("TAKEOFF_ERR", 12, AC_ShowManager, _takeoff_err_m, 3.0f),

    // @Param: POST_ACTION
    // @DisplayName: Show post-show action
    // @Description: Action taken when the show has finished.
    // @Values: 0:Loiter,1:Land,2:RTL,3:RTL or Land
    // @User: Standard
    AP_GROUPINFO("POST_ACTION", 13, AC_ShowManager, _post_action, 2),

    // @Param: VEL_FF_GAIN
    // @DisplayName: Show velocity feedforward gain
    // @Description: Gain applied to the choreography velocity when sending position/velocity targets to the guided controller during the show. 1.0 commands the full choreography velocity, 0 disables velocity feedforward.
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VEL_FF_GAIN", 14, AC_ShowManager, _vel_ff_gain, 1.0f),

    // @Param: AUTH
    // @DisplayName: Show authorization level
    // @Description: Whether the show is authorized to take off. The GCS can override this at runtime via the START_CONFIG command (0=revoked, 1=granted); the parameter is the persistent default.
    // @Values: 0:Revoked,1:Granted
    // @User: Advanced
    AP_GROUPINFO("AUTH", 15, AC_ShowManager, _authorization, 1),

    AP_GROUPEND
};

// Constructor
AC_ShowManager::AC_ShowManager(void)
{
    AP_Param::setup_object_defaults(this, var_info);

    _last_seen_start_time_sec = -1;
    _last_seen_start_time_msec = 0;
    _start_epoch_usec = 0;
    _start_internal_usec = 0;
    _stage = 0;
    _status_requested = false;
}

// compute_start_epoch_ms - convert a GPS time-of-week start time to an
// epoch time in milliseconds, selecting the current GPS week if the start
// is still in the future, otherwise the next week.
uint64_t AC_ShowManager::compute_start_epoch_ms(uint16_t gps_week, uint32_t gps_week_ms, uint32_t start_ms)
{
    const uint16_t start_week = (gps_week_ms < start_ms) ? gps_week : (uint16_t)(gps_week + 1);
    return AP::gps().istate_time_to_epoch_ms(start_week, start_ms);
}

// gps_time_ok - GPS time is usable for time sync
bool AC_ShowManager::gps_time_ok() const
{
    return AP::gps().time_week() > 0;
}

// start_time_valid - SHOW_START_TIME is set and the start reference is valid
bool AC_ShowManager::start_time_valid() const
{
    return _start_epoch_usec > 0;
}

// elapsed_usec - microseconds since the show start (negative before start)
int64_t AC_ShowManager::elapsed_usec() const
{
    if (!start_time_valid()) {
        return INT64_MIN;
    }
    uint64_t now;
    uint64_t reference;
    if (_sync_mode == 1) {
        // internal clock fallback
        now = AP_HAL::micros64();
        reference = _start_internal_usec;
    } else {
        // GPS time (extrapolates through short outages)
        now = AP::gps().time_epoch_usec();
        reference = _start_epoch_usec;
    }
    if (now >= reference) {
        return (int64_t)(now - reference);
    }
    return -(int64_t)(reference - now);
}

// time_until_start_ms - milliseconds until the show starts
uint32_t AC_ShowManager::time_until_start_ms() const
{
    const int64_t elapsed = elapsed_usec();
    if (elapsed >= 0) {
        return 0;
    }
    return (uint32_t)((-elapsed) / 1000);
}

// is_show_started - the show clock reached zero
bool AC_ShowManager::is_show_started() const
{
    return elapsed_usec() >= 0;
}

// is_performance_completed - the show clock passed the choreography duration
bool AC_ShowManager::is_performance_completed() const
{
    if (!is_loaded()) {
        return false;
    }
    return elapsed_usec() >= (int64_t)duration_ms() * 1000;
}

// update_start_reference - recompute the start reference from the
// SHOW_START_* parameters; returns false when it cannot be resolved
bool AC_ShowManager::update_start_reference()
{
    if (_start_time_gps_sec < 0) {
        _start_epoch_usec = 0;
        _start_internal_usec = 0;
        return false;
    }
    if (!gps_time_ok()) {
        _start_epoch_usec = 0;
        _start_internal_usec = 0;
        return false;
    }
    const uint32_t start_ms = (uint32_t)_start_time_gps_sec * 1000U + (uint32_t)_start_time_msec;
    const uint64_t start_epoch_ms = compute_start_epoch_ms(AP::gps().time_week(), AP::gps().time_week_ms(), start_ms);
    const uint64_t gps_now_usec = AP::gps().time_epoch_usec();
    _start_epoch_usec = start_epoch_ms * 1000ULL;
    // anchor the internal clock at the same future instant
    if (_start_epoch_usec > gps_now_usec) {
        _start_internal_usec = AP_HAL::micros64() + (_start_epoch_usec - gps_now_usec);
    } else {
        _start_internal_usec = AP_HAL::micros64();
    }
    return true;
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

    // detect SHOW_START_* parameter changes and recompute the start reference
    if (_start_time_gps_sec != _last_seen_start_time_sec ||
        _start_time_msec != _last_seen_start_time_msec) {
        _last_seen_start_time_sec = _start_time_gps_sec;
        _last_seen_start_time_msec = _start_time_msec;
        if (!update_start_reference()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: start time needs valid GPS time");
        }
    }

    // a status report was requested from the packet-receive context; push
    // MSG_SHOW_STATUS now, from the scheduler main loop, so the DATA16
    // send happens in the GCS update_send context where it is reliable
    if (_status_requested) {
        _status_requested = false;
        gcs().send_message(MSG_SHOW_STATUS);
    }
}

// handle_start_config - apply a START_CONFIG from the GCS
bool AC_ShowManager::handle_start_config(int32_t start_tow_sec, uint8_t authorization)
{
    _authorization.set((int8_t)authorization);
    if (start_tow_sec < 0) {
        // clear the start reference
        _start_time_gps_sec.set(-1);
        _start_time_msec.set(0);
        _start_epoch_usec = 0;
        _start_internal_usec = 0;
        _last_seen_start_time_sec = -1;
        _last_seen_start_time_msec = 0;
        return true;
    }
    // clamp to a valid time-of-week
    if (start_tow_sec >= 604800) {
        return false;
    }
    _start_time_gps_sec.set(start_tow_sec);
    _start_time_msec.set(0);
    // keep the parameter-change detector quiet: the reference was set
    // internally (not via AP_Param), so mark the state as seen
    _last_seen_start_time_sec = _start_time_gps_sec.get();
    _last_seen_start_time_msec = 0;
    // recompute the reference from the internal state
    if (!update_start_reference()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Show: start needs GPS time");
        return false;
    }
    return true;
}

// send_status_data - report the show status as a DATA16 packet
void AC_ShowManager::send_status_data(mavlink_channel_t chan)
{
    uint8_t flags = 0;
    if (is_loaded()) {
        flags |= 0x01;
    }
    if (start_time_valid()) {
        flags |= 0x02;
    }
    if (authorized()) {
        flags |= 0x04;
    }
    // build the STATUS payload (data[0] = STATUS type) and wrap it in a
    // DATA16 packet (STATUS_LEN=12 fits in the 16-byte payload).  The
    // DATA message family is fully supported by mavlink2 ground stations
    // (pymavlink v20); Mission Planner does not render it, so structured
    // consumers parse the DATA16 stream instead.
    uint8_t buf[ShowProtocol::STATUS_LEN];
    const uint8_t n = ShowProtocol::build_status(buf, flags, stage(), start_tow_sec(), duration_ms());
    mavlink_data16_t m;
    memset(&m, 0, sizeof(m));
    m.type = ShowProtocol::DRONE_TO_GCS;
    m.len = n;
    memcpy(m.data, buf, n);
    mavlink_msg_data16_send(chan, m.type, m.len, m.data);
}

// request_status_send - request a status report
void AC_ShowManager::request_status_send()
{
    // set a flag that the 50Hz update() picks up and pushes through the
    // GCS message scheduler.  Sending directly from this context (the
    // packet-receive path) loses the packet, so the actual DATA16 send
    // always happens from the scheduler main loop.
    _status_requested = true;
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

// rotate_show_NE_mm - rotate a show-frame position by the configured
// orientation, returning the North/East offset in metres
void AC_ShowManager::rotate_show_NE_mm(float orientation_deg, int32_t x_mm, int32_t y_mm,
                                       float &north_m, float &east_m)
{
    const float ori_rad = radians(orientation_deg);
    const float x_m = x_mm * 0.001f;
    const float y_m = y_mm * 0.001f;
    // the show frame is NED (x=north, y=east); rotate the vector by the
    // clockwise orientation of the show X axis
    north_m = cosf(ori_rad) * x_m - sinf(ori_rad) * y_m;
    east_m = sinf(ori_rad) * x_m + cosf(ori_rad) * y_m;
}

// show_to_global_location - convert a show keyframe to a global Location
// using the configured show origin and orientation
bool AC_ShowManager::show_to_global_location(const ShowFile::Keyframe &kf, Location &loc) const
{
    if (_origin_lat == 0 && _origin_lng == 0) {
        return false;
    }
    float north_m, east_m;
    rotate_show_NE_mm(orientation_deg_effective(), kf.pos_x_mm, kf.pos_y_mm, north_m, east_m);

    loc.zero();
    loc.lat = _origin_lat;
    loc.lng = _origin_lng;
    const float alt_m = kf.pos_z_mm * 0.001f;   // show altitude is down positive
    if (_origin_amsl_mm > 0) {
        // AMSL mode: show altitude above the configured origin AMSL
        loc.set_alt_m(alt_m + _origin_amsl_mm * 0.001f, Location::AltFrame::ABSOLUTE);
    } else {
        // AGL mode: show altitude above home
        loc.set_alt_m(-alt_m, Location::AltFrame::ABOVE_HOME);
    }
    loc.offset(north_m, east_m);
    return true;
}
