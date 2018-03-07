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
    cur_index = 2; 
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

/// Compute Aileron Deflection
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
Location Waypoint::nextWaypoint(AP_Mission mission, Location cur_loc, uint32_t waypoint_radius)
{
    AP_Mission::Mission_Command cmd;
    // Waypoint #0 is always the home waypoint
    // Waypoint #1 is skipped due to the takeoff waypoint
    // therefore, if you would like to use this code for actual
    // flight test, use a dummy waypoint for the first one
    // (in place for TAKEOFF)
    if (cur_index == 2) {
        mission.get_next_nav_cmd(2, cmd);
        loc = cmd.content.location;
    }
    if (get_distance(cur_loc, loc) < waypoint_radius) {
        // If there is another waypoint, set plane to that next waypoint
        if (mission.get_next_nav_cmd(cur_index + 1, cmd)) {
            cur_index++;
            loc = cmd.content.location;
        }
        // If no waypoints available, set plane to home
        // Plane will continue circling the home waypoint once
        // it reaches the home waypoint

        // TODO: Figure out why mavproxy still prints waypoint 6
        else {
            mission.get_next_nav_cmd(0, cmd); // 0 is home waypoint
            loc = cmd.content.location;
        }
    }
    // Using unused loc.options to store the cur_index so that it is accessible in the return type
    loc.options = cur_index;

    return loc;
}


//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

