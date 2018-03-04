// Waypoint.h: interface for the Waypoint class.
//
// Ravi Patel
// patelr3@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_Waypoint_h
#define GUARD_Waypoint_h

/*
// generate dll
#ifdef ORBITCONTROLLERLIBRARY_EXPORTS
#define ORBITCONTROLLERLIBRARY_API __declspec(dllexport)
#else
#define ORBITCONTROLLERLIBRARY_API __declspec(dllimport)
#endif
*/

// Standard headers
//#include <string>			//string
//#include <vector>			//vector
#include <AP_Mission/AP_Mission.h>
#include <AP_Common/Location.h>

// Local header files

//-------------------------------------------------------------------------------
/// Define Waypoint objects which are used to extract waypoints from the flight plan.
///
////
class Waypoint {

public:
    ///////////// Construction/Destruction ///////////////////////////////

    // default constructor
    Waypoint();

    // destructor
    virtual ~Waypoint();

    ///////////// Overload operators /////////////////////////////////////


    ///////////// Public interface methods ///////////////////////////////
    Location nextWaypoint(AP_Mission mission, Location cur_loc, uint32_t waypoint_radius);

    // ====== Get/Set Functions ==========================


    // data members

protected:

    ///////////// Protected data members /////////////////////////////////

    // data members

private:

    ///////////// Private data members ///////////////////////////////////

    // data members
    uint16_t cur_index;
    Location loc;

};
#endif
