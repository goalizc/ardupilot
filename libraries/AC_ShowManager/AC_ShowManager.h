#pragma once

#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "ShowFile.h"
#include "ShowFileParser.h"

/*
  AC_ShowManager - library managing the drone show (choreography flight
  show). Loads the choreography, plays the timeline on a GPS-time base and
  generates position/velocity targets for the guided controller, plus
  synchronised RGB light output.

  P1: show file loading from storage (SD card) with a parser-validated
  in-memory copy of the choreography. Later phases add time sync and the
  state machine (P2), trajectory playback (P3) and lights (P4).
*/
class AC_ShowManager {
public:

    // Constructor
    AC_ShowManager(void);

    CLASS_NO_COPY(AC_ShowManager);

    static const struct AP_Param::GroupInfo var_info[];

    // called at 50Hz from the scheduler
    void update();

    // (re)load the show file from storage; refused while armed.
    // returns false if the vehicle is armed or the file fails to parse.
    bool reload();
    // clear the loaded show and delete the show file; refused while armed
    bool clear();

    // accessors for the loaded show data
    bool is_loaded() const { return _parser.loaded(); }
    ShowFileParser::Failure failure() const { return _parser.failure(); }
    uint16_t drone_id() const { return _parser.drone_id(); }
    uint32_t duration_ms() const { return _parser.duration_ms(); }
    uint16_t keyframe_count() const { return _parser.keyframe_count(); }
    uint16_t light_count() const { return _parser.light_count(); }
    uint8_t segment_count() const { return _parser.segment_count(); }
    const ShowFile::Keyframe *keyframes() const { return _parser.keyframes(); }
    const ShowFile::LightEvent *lights() const { return _parser.lights(); }
    const ShowFile::Segment *segments() const { return _parser.segments(); }

    // time sync (D6 locked base)
    // convert a GPS time-of-week start time (ms) to an epoch time (ms);
    // the start is interpreted in the current GPS week if it is still in
    // the future, otherwise in the next week. pure function, unit-tested.
    static uint64_t compute_start_epoch_ms(uint16_t gps_week, uint32_t gps_week_ms, uint32_t start_ms);

    // true when GPS time is usable for time sync
    bool gps_time_ok() const;
    // true when SHOW_START_TIME is set and the start reference is valid
    bool start_time_valid() const;
    // microseconds since the show start (negative before the show starts)
    int64_t elapsed_usec() const;
    // milliseconds until the show starts (0 if already started)
    uint32_t time_until_start_ms() const;
    // true once the show clock reached zero
    bool is_show_started() const;
    // true when the show clock passed the choreography duration
    bool is_performance_completed() const;

    // accessors for the show parameters used by the mode
    float takeoff_alt_m() const { return _takeoff_alt_m; }
    float takeoff_err_m() const { return _takeoff_err_m; }
    int8_t post_action() const { return _post_action; }
    float ctrl_rate_hz() const { return _ctrl_rate_hz; }
    float vel_ff_gain() const { return _vel_ff_gain; }
    float max_xy_err_m() const { return _max_xy_err_m; }
    float max_z_err_m() const { return _max_z_err_m; }

    // coordinate system (show frame -> global)
    // rotate a show-frame position (mm) by the configured orientation;
    // returns the North/East offset in metres. pure function, unit-tested.
    static void rotate_show_NE_mm(float orientation_deg, int32_t x_mm, int32_t y_mm,
                                  float &north_m, float &east_m);

    // convert a show keyframe to a global Location (WGS84) using the
    // configured show origin and orientation
    bool show_to_global_location(const ShowFile::Keyframe &kf, Location &loc) const;

    // effective show orientation in degrees (unset -1 becomes 0)
    float orientation_deg_effective() const { return MAX((float)_orientation_deg, 0.0f); }

    // custom protocol (P5)
    // apply a START_CONFIG from the GCS: set/clear the start reference
    // (internal state only, not persisted via AP_Param) and the
    // authorization level. returns true if accepted.
    bool handle_start_config(int32_t start_tow_sec, uint8_t authorization);

    // true when the show is authorized to take off
    bool authorized() const { return _authorization.get() != 0; }

    // current start time as GPS time-of-week (seconds); -1 when unset
    int32_t start_tow_sec() const { return _start_time_gps_sec.get(); }

    // send the show status as a structured DATA16 packet (drone-to-GCS)
    // on the given GCS channel; called from the GCS message scheduler so
    // the mavlink send happens with the channel lock held correctly
    void send_status_data(mavlink_channel_t chan);

    // request a status report; the next 50Hz update() pushes
    // MSG_SHOW_STATUS so the DATA16 send happens from the GCS main send
    // loop (sending directly from a packet-receive context loses packets)
    void request_status_send();

    // current show stage (0=off,1=waiting,2=takeoff,3=performing,4=end);
    // reported by ModeShow via set_stage
    uint8_t stage() const { return _stage; }
    void set_stage(uint8_t stage) { _stage = stage; }

private:

    // read + parse the show file from storage; report controls whether a
    // missing file is reported to the GCS. returns false on parse failure.
    bool load_from_file(bool report);

    // map a parser failure to a human readable string
    const char *failure_string(ShowFileParser::Failure failure) const;

    // recompute the start reference from the SHOW_START_* parameters;
    // returns false when it cannot be resolved (e.g. no GPS time)
    bool update_start_reference();

    // current show stage reported by ModeShow
    uint8_t _stage;

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
    AP_Float _takeoff_alt_m;        // takeoff altitude for the show (m)
    AP_Float _takeoff_err_m;        // max horizontal distance from launch point (m)
    AP_Int8 _post_action;           // action at the end of the show
    AP_Float _vel_ff_gain;          // velocity feedforward gain for the show player

    // P5 protocol state
    AP_Int8 _authorization;         // 0=revoked, 1=granted
    bool _status_requested;         // a status report was requested; the
                                    // 50Hz update() pushes MSG_SHOW_STATUS
                                    // so the DATA16 send always happens from
                                    // the GCS main send loop (not from the
                                    // packet-receive context, where direct
                                    // mavlink sends get lost)

    // loaded show data
    ShowFileParser _parser;
    bool _load_attempted;

    // time sync state
    int32_t _last_seen_start_time_sec;
    int32_t _last_seen_start_time_msec;
    uint64_t _start_epoch_usec;          // show start on the GPS (unix) clock
    uint64_t _start_internal_usec;       // show start on the internal clock
};
