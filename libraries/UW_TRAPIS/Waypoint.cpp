// Waypoint.cpp: implementation of the Waypoint class.
//
// Ravi Patel
// patelr3@uw.edu
//
//////////////////////////////////////////////////////////////////////

//Version History
//	03/03/18: Created

// standard headers
//#include <iostream>						//cout, endl, cerr
//#include <string>						//string
//#include <vector>						//vector
#include <math.h>						//min, max

// local header files
#include "Waypoint.h"		//Waypoint class


// using declaration


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------
////
/// Default constructor.
///
/// Input:			-none
///
/// Output:			-none
///
/// Side-effects:	-none
////
Waypoint::Waypoint()
{
    cur_waypoint_num = STARTING_WAYPOINT; 
}



//-------------------------------------------------------------------------
////
/// Destructor
////
Waypoint::~Waypoint()
{
}

//////////////////////////////////////////////////////////////////////
// Overloaded operators
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Public interface methods
//////////////////////////////////////////////////////////////////////

/// Return Next Waypoint
///	
/// Input:			- mission           = current mission plan 
///                 - cur_loc           = current location of the plane
///                                     (i.e. using trapis coords)
///                 - waypoint_radius   = max radius at which point plane will move to next waypoint
///
/// Output:			- waypoint          = current waypoint (Location)
///                                         -- contains the current waypoint number in loc.options (optional to use)
///
/// Side-effects:	- none
////
Location Waypoint::nextWaypoint(AP_Mission mission, Location cur_loc, uint32_t waypoint_radius, 
                                Location default_loc, FlightMode *control_mode)
{
    // Check if plane is in WSTR
    // If plane is not in WSTR, return last saved waypoint
    // And reset flight plan mission
    if (flight_mode != WSTR) {
        cur_waypoint_num = STARTING_WAYPOINT;
        return loc;
    }

    // Set current control_mode in private field
    flight_mode = *control_mode;
    AP_Mission::Mission_Command cmd; // Saves output from mission.get_next_nav_cmd

    // Waypoint #0 is always the home waypoint
    // Waypoint #1 is skipped due to the takeoff waypoint in sim
    // therefore, if you would like to use this code for actual
    // flight test, use a dummy waypoint for the first one
    // (in place for TAKEOFF)
    if (cur_waypoint_num == STARTING_WAYPOINT) {
        if (mission.get_next_nav_cmd(STARTING_WAYPOINT, cmd)) {
            loc = cmd.content.location;
        }
        else {
            loc = default_loc;
        }
    }

    // Checks if plane has reached the current waypoint
    // If plane has reached, go to next waypoint
    /// get_distance is in Location.h which is included
    if (get_distance(cur_loc, loc) < waypoint_radius) {
        // If there is another waypoint, set plane to that next waypoint
        if (mission.get_next_nav_cmd(cur_waypoint_num + 1, cmd)) {
            cur_waypoint_num++;
            loc = cmd.content.location;
        }

        else if (mission.get_next_nav_cmd(cur_waypoint_num, cmd)) {
            loc = cmd.content.location;
            cur_waypoint_num = STARTING_WAYPOINT;
        }

        // If no waypoints available, set plane to home
        // Plane will continue circling the home waypoint once
        // it reaches the home waypoint
        else {
            loc = default_loc;
            cur_waypoint_num = STARTING_WAYPOINT; // Reset starting waypoint for next test run
        }
    }

    // Using unused loc.options to store the cur_index so that it is accessible in the return type
    loc.options = cur_waypoint_num;

    return loc;
}

void Waypoint::sendMessage()
{
    gcs().send_text(MAV_SEVERITY_INFO, "FlightMode: %i", flight_mode);
}

void Waypoint::getFlightMode(FlightMode *control_mode) {
    flight_mode = *control_mode;

    // Check if plane is in WSTR
    // If plane is not in WSTR, return last saved waypoint
    // And reset flight plan mission
    if (flight_mode != WSTR) {
        cur_waypoint_num = STARTING_WAYPOINT;
    }
}


//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================
