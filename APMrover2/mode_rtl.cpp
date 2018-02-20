#include "mode.h"
#include "Rover.h"

bool ModeRTL::_enter()
{
    // refuse RTL if home has not been set
    if (rover.home_is_set == HOME_UNSET) {
        return false;
    }

    // set destination
    set_desired_location(rover.home);

    // RTL never reverses
    rover.set_reverse(false);

    g2.motors.slew_limit_throttle(true);
    return true;
}

void ModeRTL::update()
{
    if (!rover.in_auto_reverse) {
        rover.set_reverse(false);
    }
    calc_lateral_acceleration();
    calc_nav_steer();
    calc_throttle(g.speed_cruise);
}

void ModeRTL::update_navigation()
{
    // no loitering around the wp with the rover, goes direct to the wp position
    if (rover.verify_RTL()) {
        rover.set_mode(rover.mode_hold, MODE_REASON_MISSION_END);
    }
}
