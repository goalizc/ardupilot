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

// _performing_start - begin the performance
void ModeShow::_performing_start()
{
    _set_stage(Stage::PERFORMING);
    _performing_initialised = false;
    // the choreography clock starts when the takeoff has completed, so the
    // first keyframe (t=0) matches the position reached by the takeoff
    _performance_t0_usec = copter.show_manager.elapsed_usec();
    // load the choreography track into the player
    _player.set_track(copter.show_manager.keyframes(), copter.show_manager.keyframe_count());
    _last_play_ms = 0;
    _drift_counter = 0;
}

// _performing_run - play the choreography trajectory while the show clock runs
void ModeShow::_performing_run()
{
    if (!_performing_initialised) {
        _performing_initialised = true;
        copter.mode_guided.init(true);
    }

    // evaluate and command the trajectory at the configured rate
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t ctrl_interval_ms = 1000U / MAX(1U, (uint32_t)copter.show_manager.ctrl_rate_hz());
    if (now_ms - _last_play_ms >= ctrl_interval_ms) {
        _last_play_ms = now_ms;
        _send_play_target();
        _check_drift();
    }

    copter.mode_guided.run();

    if (!copter.motors->armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Show: motors disarmed during show");
        _error_start();
        return;
    }

    // the show is over once the show clock passed the choreography duration
    if ((copter.show_manager.elapsed_usec() - _performance_t0_usec) >=
        (int64_t)copter.show_manager.duration_ms() * 1000) {
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

// _send_play_target - evaluate the choreography and command the guided controller
void ModeShow::_send_play_target()
{
    const int64_t show_elapsed_ms = (copter.show_manager.elapsed_usec() - _performance_t0_usec) / 1000;
    if (show_elapsed_ms < 0) {
        return;
    }
    ShowFile::Keyframe kf;
    if (!_player.evaluate((uint32_t)show_elapsed_ms, kf)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: no trajectory data");
        copter.mode_guided.hold_position();
        _error_start();
        return;
    }

    Location target_loc;
    if (!copter.show_manager.show_to_global_location(kf, target_loc)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: no show origin set");
        copter.mode_guided.hold_position();
        _error_start();
        return;
    }
    Location ekf_origin;
    if (!copter.ahrs.get_origin(ekf_origin)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: no EKF origin");
        copter.mode_guided.hold_position();
        _error_start();
        return;
    }
    // NED vector from the EKF origin to the target; alt frames are resolved
    // (the target is ABOVE_HOME, the origin is ABSOLUTE)
    const Vector3f target_ned = ekf_origin.get_distance_NED_alt_frame(target_loc);

    // rotate the choreography velocity into the global frame and scale by
    // the velocity feedforward gain
    const float gain = copter.show_manager.vel_ff_gain();
    const float ori = copter.show_manager.orientation_deg_effective();
    float vn, ve;
    AC_ShowManager::rotate_show_NE_mm(ori, kf.vel_x_mms, kf.vel_y_mms, vn, ve);
    const Vector3f vel_ned(vn * gain * 0.001f, ve * gain * 0.001f, -kf.vel_z_mms * gain * 0.001f);

    const float yaw_rad = radians(ori + kf.yaw_cd * 0.01f);

    if (!copter.mode_guided.set_pos_vel_accel_NED_m(target_ned.topostype(), vel_ned, Vector3f(),
                                                    true, yaw_rad, false, 0.0f, false)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Show: failed to send target");
        copter.mode_guided.hold_position();
        _error_start();
    }
}

// _check_drift - monitor the tracking error against the choreography target
void ModeShow::_check_drift()
{
    Vector3f actual_ned;
    if (!copter.ahrs.get_relative_position_NED_origin_float(actual_ned)) {
        return;
    }
    const int64_t show_elapsed_ms = (copter.show_manager.elapsed_usec() - _performance_t0_usec) / 1000;
    if (show_elapsed_ms < 0) {
        return;
    }
    ShowFile::Keyframe kf;
    if (!_player.evaluate((uint32_t)show_elapsed_ms, kf)) {
        return;
    }
    Location target_loc;
    Location ekf_origin;
    if (!copter.show_manager.show_to_global_location(kf, target_loc) || !copter.ahrs.get_origin(ekf_origin)) {
        return;
    }
    // NED vector from the EKF origin to the target; alt frames are resolved
    // (the target is ABOVE_HOME, the origin is ABSOLUTE)
    const Vector3f target_ned = ekf_origin.get_distance_NED_alt_frame(target_loc);
    const float err_xy = Vector2f{actual_ned.x - target_ned.x, actual_ned.y - target_ned.y}.length();
    const float err_z = fabsf(actual_ned.z - target_ned.z);
    if (err_xy > copter.show_manager.max_xy_err_m() || err_z > copter.show_manager.max_z_err_m()) {
        if (++_drift_counter >= 10) {   // one second of continuous exceedance
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Show: drift exceeded");
            _error_start();
        }
    } else {
        _drift_counter = 0;
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
    if (copter.ap.land_complete) {
        // the RTL has landed; move on to the landed stage.  Without this
        // transition the RTL sub-mode keeps running and its landing
        // detector would disarm the vehicle again on a subsequent re-arm.
        _landed_start();
    }
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
