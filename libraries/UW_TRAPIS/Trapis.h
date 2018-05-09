// AltitudeHold.h: interface for the AltitudeHold class.
//
// Ravi Patel
// patelr3@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_Trapis_h
#define GUARD_Trapis_h

#include "../../ArduPlane/Plane.h"
#include "../AP_AHRS/AP_AHRS.h"         // for AHRS object (for retrieving plane state)
#include "../../ArduPlane/GCS_Plane.h"  // for GCS object (for sending text)
#include "../AP_Common/Location.h"      // for Location object (for storing location information)
#include "../AFSL/AFSL.h"               // for AFSL Library object/functions
#include "../../ArduPlane/Parameters.h" // for GCS Parameters (retrieving parameters from MissionPlanner)
#include "../../ArduPlane/defines.h"    // for Flight Mode object to be sent to waypoint object
#include "../AP_GPS/AP_GPS.h"           // for GPS information on plane
#include "../AP_Mission/AP_Mission.h"   // for mission information
#include "../SRV_Channel/SRV_Channel.h" // for setting plane outputs
#include "../RC_Channel/RC_Channel.h"   // for RC_Channel for getting plane state

// Standard headers
//#include <string>			//string
//#include <vector>			//vector

// Local header files

//-------------------------------------------------------------------------------
/// Define altitude hold objects which are used to represent a altitude hold controller.
///
////
class Trapis {

public:
    ///////////// Construction/Destruction ///////////////////////////////

    // default constructor
    Trapis();

    // destructor
    virtual ~Trapis();

    ///////////// Overload operators /////////////////////////////////////


    ///////////// Public interface methods ///////////////////////////////
    void engageMode(GCS_Plane& gcs, AP_AHRS& ahrs, Parameters& g, FlightMode& control_mode, AP_GPS& gps,
                    AP_Mission mission, Location home, float relative_altitude, int16_t* steering, int16_t* rudder, RC_Channel* channel_rudder);

    // ====== Get/Set Functions ==========================


    // data members

protected:

    ///////////// Protected data members /////////////////////////////////

    // data members

private:

    ///////////// Private data members ///////////////////////////////////
    // AFSL Objects
    struct {
        WingLeveler WL;
        AltitudeHold AH;
        Steer STR;
        Waypoint WP;
    } wstr_state;


    // trapis information to keep hold of
    struct {
        double lat;
        double lng;
        double alt;
        Location loc;
        int16_t waypoint_num;
        int8_t flight_plan_existing_counter;
    } trapis_state;
};
#endif
