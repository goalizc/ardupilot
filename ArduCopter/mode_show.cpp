#include "Copter.h"

#if MODE_SHOW_ENABLED

/*
 * Init and run calls for SHOW flight mode
 *
 * The SHOW mode plays a choreography (trajectory + lights) on a GPS time
 * base. P0: the mode is an empty shell that holds position by delegating
 * to the guided controller; the show state machine (P2) replaces this.
 */

// init - initialise show controller
bool ModeShow::init(bool ignore_checks)
{
    // P0: hold the current position through the guided controller until the
    // show state machine is implemented (P2). This mirrors the proven
    // Skybrush behaviour for the non-performing phase of a show.
    if (!copter.mode_guided.init(ignore_checks)) {
        return false;
    }
    return true;
}

// exit - exit show mode
void ModeShow::exit()
{
}

// run - runs the show controller
// should be called at 100hz or more
void ModeShow::run()
{
    // P0: hold position. The show player will feed pos/vel targets here
    // from P3 onwards.
    copter.mode_guided.run();
}

bool ModeShow::allows_arming(AP_Arming::Method method) const
{
    // the show is started from the ground station, so only allow arming
    // from the GCS or scripting
    return AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING;
}

#endif // MODE_SHOW_ENABLED
