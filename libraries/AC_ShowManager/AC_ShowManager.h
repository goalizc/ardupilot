#pragma once

#include <AP_Param/AP_Param.h>

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

private:

    // read + parse the show file from storage; report controls whether a
    // missing file is reported to the GCS. returns false on parse failure.
    bool load_from_file(bool report);

    // map a parser failure to a human readable string
    const char *failure_string(ShowFileParser::Failure failure) const;

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

    // loaded show data
    ShowFileParser _parser;
    bool _load_attempted;
};
