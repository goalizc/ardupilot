#include "Copter.h"

#if MODE_SHOW_ENABLED

/*
 * Init and run calls for SHOW flight mode
 *
 * P2: state machine - Off/Init/WaitForStart/Takeoff/Performing/end action.
 * The choreography timeline is maintained by AC_ShowManager; while
 * performing, the P2 placeholder hovers (P3 replaces this with trajectory
 * playback).
 */

// init - initialise show controller
bool ModeShow::init(bool ignore_checks)
{
    // begin the state machine
    _set_stage(Stage::INIT);
    _performing_initialised = false;
    _end_mode_initialised = false;
    return true;
}

// exit - exit show mode
void ModeShow::exit()
{
    _set_stage(Stage::OFF);
}

// run - runs the show controller
// should be called at 100hz or more
void ModeShow::run()
{
    switch (_stage) {
    case Stage::INIT:
        _init_run();
        break;
    case Stage::WAIT_FOR_START:
        _wait_for_start_run();
        break;
    case Stage::TAKEOFF:
        _takeoff_run();
        break;
    case Stage::PERFORMING:
        _performing_run();
        break;
    case Stage::LOITER:
        _loiter_run();
        break;
    case Stage::RTL:
        _rtl_run();
        break;
    case Stage::LANDING:
        _landing_run();
        break;
    case Stage::LANDED:
        _landed_run();
        break;
    case Stage::ERROR:
        _error_run();
        break;
    default:
        break;
    }
}

// _set_stage - change the stage and report it to the GCS
void ModeShow::_set_stage(Stage stage)
{
    if (stage == _stage) {
        return;
    }
    _stage = stage;
    if (_stage != Stage::OFF) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Show stage: %s", _stage_name());
    }
}

// _stage_name - human readable name of the current stage
const char *ModeShow::_stage_name() const
{
    switch (_stage) {
    case Stage::OFF:
        return "off";
    case Stage::INIT:
        return "init";
    case Stage::WAIT_FOR_START:
        return "waiting";
    case Stage::TAKEOFF:
        return "takeoff";
    case Stage::PERFORMING:
        return "performing";
    case Stage::LOITER:
        return "loiter";
    case Stage::RTL:
        return "rtl";
    case Stage::LANDING:
        return "landing";
    case Stage::LANDED:
        return "landed";
    case Stage::ERROR:
        return "error";
    }
    return "unknown";
}

// _init_run - decide whether to wait for the start time or hold position
void ModeShow::_init_run()
{
    if (is_disarmed_or_landed()) {
        _wait_for_start_start();
    } else {
        // already airborne; hold position until the operator takes over
        _loiter_start();
    }
}

// _wait_for_start_start - start waiting for the show start time
void ModeShow::_wait_for_start_start()
{
    _set_stage(Stage::WAIT_FOR_START);
    if (!copter.show_manager.start_time_valid()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Show: no valid start time");
        _error_start();
    }
}

// _wait_for_start_run - standby on the ground until the show clock reaches zero
void ModeShow::_wait_for_start_run()
{
    // keep the controllers inert while on the ground
    copter.attitude_control->reset_yaw_target_and_rate();
    copter.attitude_control->reset_rate_controller_I_terms();
    copter.pos_control->NED_standby_reset();
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(0.0f, 0.0f, 0.0f);

    if (!copter.motors->armed()) {
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (!copter.ap.land_complete) {
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    if (!copter.motors->armed() && copter.show_manager.elapsed_usec() > 5000000LL) {
        // the show started but nobody armed the vehicle in time
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Show: not armed in time");
        _landed_start();
        return;
    }

    if (copter.motors->armed() && copter.show_manager.is_show_started()) {
        // do not take off from a position far from the launch point
        if (copter.current_loc.get_distance(copter.ahrs.get_home()) > copter.show_manager.takeoff_err_m()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Show: outside takeoff position");
            _error_start();
            return;
        }
        _takeoff_start();
    }
}

// _takeoff_start - begin the takeoff climb
void ModeShow::_takeoff_start()
{
    _set_stage(Stage::TAKEOFF);

    // initialise yaw hold and the vertical position controller, mirroring
    // the guided takeoff
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    pos_control->D_init_controller();

    // pretend that the throttle was raised by the user so the position
    // controller will take over the throttle
    copter.set_auto_armed(true);

    // interpret the takeoff altitude as altitude above home, mirroring the
    // guided takeoff
    Location target_loc = copter.current_loc;
    target_loc.set_alt_m(copter.show_manager.takeoff_alt_m(), Location::AltFrame::ABOVE_HOME);
    float alt_target_m;
    if (target_loc.get_alt_m(Location::AltFrame::ABOVE_ORIGIN, alt_target_m)) {
        copter.mode_auto.auto_takeoff.start_m(alt_target_m, false);
    }
}

// _takeoff_run - climb to the takeoff altitude
void ModeShow::_takeoff_run()
{
    if (!copter.motors->armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Show: motors disarmed during takeoff");
        _error_start();
        return;
    }

    // run the shared auto takeoff controller (as guided does)
    copter.mode_auto.auto_takeoff.run();

    if (copter.mode_auto.auto_takeoff.complete) {
        _performing_start();
    }
}

// _performing_start - begin the performance (P2: hover placeholder)
void ModeShow::_performing_start()
{
    _set_stage(Stage::PERFORMING);
    _performing_initialised = false;
}

// _performing_run - hold position while the show clock runs
void ModeShow::_performing_run()
{
    if (!_performing_initialised) {
        _performing_initialised = true;
        copter.mode_guided.init(true);
    }
    copter.mode_guided.run();

    if (!copter.motors->armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Show: motors disarmed during show");
        _error_start();
        return;
    }

    if (copter.show_manager.is_performance_completed()) {
        // the show is over; execute the configured post-show action
        switch (copter.show_manager.post_action()) {
        case 0:
            _loiter_start();
            break;
        case 1:
            _landing_start();
            break;
        default:
            _rtl_start();
            break;
        }
    }
}

// _loiter_start - hold position
void ModeShow::_loiter_start()
{
    _set_stage(Stage::LOITER);
    _end_mode_initialised = false;
}

// _loiter_run
void ModeShow::_loiter_run()
{
    if (!_end_mode_initialised) {
        _end_mode_initialised = true;
        copter.mode_loiter.init(true);
    }
    copter.mode_loiter.run();
}

// _rtl_start - return to launch
void ModeShow::_rtl_start()
{
    _set_stage(Stage::RTL);
    _end_mode_initialised = false;
}

// _rtl_run
void ModeShow::_rtl_run()
{
    if (!_end_mode_initialised) {
        _end_mode_initialised = true;
        copter.mode_rtl.init(true);
    }
    copter.mode_rtl.run();
}

// _landing_start - land in place
void ModeShow::_landing_start()
{
    _set_stage(Stage::LANDING);
    _end_mode_initialised = false;
}

// _landing_run
void ModeShow::_landing_run()
{
    if (!_end_mode_initialised) {
        _end_mode_initialised = true;
        copter.mode_land.init(true);
    }
    copter.mode_land.run();
    if (copter.ap.land_complete) {
        _landed_start();
    }
}

// _landed_start - show finished, vehicle on the ground
void ModeShow::_landed_start()
{
    _set_stage(Stage::LANDED);
}

// _landed_run
void ModeShow::_landed_run()
{
    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
}

// _error_start - the show failed; fall back to landing
void ModeShow::_error_start()
{
    _set_stage(Stage::ERROR);
}

// _error_run
void ModeShow::_error_run()
{
    _landing_start();
}

bool ModeShow::allows_arming(AP_Arming::Method method) const
{
    // the show is started from the ground station, so only allow arming
    // from the GCS or scripting
    return AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING;
}

#endif // MODE_SHOW_ENABLED
