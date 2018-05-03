// Waypoint.h: interface for the Waypoint class.
//
// Ravi Patel
// patelr3@uw.edu
//
//
// NOTES:
// The nextWaypoint() method assumes that your first waypoint is a
// dummy waypoint. This code will not command to the first waypoint
// initially. This is so you can assign a TAKEOFF waypoint in simulation
// while still being able to command to certain waypoints in your flight plan
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
#define STARTING_WAYPOINT 2

// Standard headers
//#include <string>			//string
//#include <vector>			//vector
#include <AP_Mission/AP_Mission.h>
#include <AP_Common/Location.h>
#include <../ArduPlane/defines.h>
#include <../ArduPlane/GCS_Plane.h>


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
    Location nextWaypoint(AP_Mission mission, Location cur_loc, uint32_t waypoint_radius, 
                            Location default_loc, FlightMode *control_mode);

    //void sendMessage(GCS_Plane *gcsPtr);
    void sendMessage(GCS_Plane& gcs, Location default_loc);
    // ====== Get/Set Functions ==========================

    void getFlightMode(FlightMode *control_mode);
    // data members

protected:

    ///////////// Protected data members /////////////////////////////////

    // data members

private:

    ///////////// Private data members ///////////////////////////////////

    // data members
    uint16_t cur_waypoint_num;
    uint16_t prev_waypoint_num;
    Location loc;
    FlightMode flight_mode;
    uint8_t flight_plan_existing_counter;

};
#endif
