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
    cur_index = 1; 
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
/// Input:			- mission       = current mission plan 
///
/// Output:			- waypoint      = current waypoint (Location)
///
/// Side-effects:	- none
////
Location Waypoint::nextWaypoint(AP_Mission mission)
{
    AP_Mission::Mission_Command cmd;
    //cmd.content.location
    // get_next_nav_cmd(cur_index, cmd) finds waypoint at or after "cur_index"
    if (trapis.loc.getDistance(loc) < 100) {
        if (mission.get_next_nav_cmd(cur_index, cmd)) {
            cur_index++;
        }
        else {
            mission.get_next_nav_cmd(0, cmd);
        }
        loc = cmd.content.location;
    }
    return loc;
}


//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

