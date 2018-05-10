// Trapis.h: interface for the Trapis class. This object handles/stores
// trapis data (trapis coordinates) and controls the plane's behavior
// based on that data.
//
// Ravi Patel
// patelr3@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_Trapis_h
#define GUARD_Trapis_h

#include "../AP_AHRS/AP_AHRS.h"         // for AHRS object (for retrieving plane state)
#include "../../ArduPlane/GCS_Plane.h"  // for GCS object (for sending text)
#include "../AP_Common/Location.h"      // for Location object (for storing location information)
#include "../AFSL/AFSL.h"               // for AFSL Library object/functions
#include "../AP_Terrain/AP_Terrain.h"
#include "../AP_Button/AP_Button.h"
#include "../AP_Stats/AP_Stats.h"
#include "../AP_ICEngine/AP_ICEngine.h"
#include "../AP_Soaring/AP_Soaring.h"
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
/// Define trapis objects that contain trapis data and control plane's behavior
/// for Trapis modes
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
    /// Engage Trapis WSTR mode
    ///	
    /// Input:			- gcs               = ground control station object to send messages to
    ///                 - ahrs              = ahrs object containing sensor, position, velocity, and other information
    ///                 - g                 = parameters object for retrieving ground station parameters
    ///                 - control_mode      = current mode of the plane
    ///                 - gps               = current state of the gps on the plane
    ///                 - mission           = current state of the flight plan and mission on the plane
    ///                 - home              = current home waypoint loaded onto the plane
    ///                 - relative_altitude = relative altitude of the plane
    ///
    /// Output:			- steering          = steering value to be set
    ///                 - rudder            = rudder position value to be set
    ///                 - channel_rudder    = actual rudder state being sent to the plane (this actually controls the plane)
    ///
    /// Side-effects:	- affects plane.steering and plane.rudder fields (see Plane.h for declaration of those fields)
    void engageWSTRMode(GCS_Plane& gcs, AP_AHRS& ahrs, Parameters& g, FlightMode* control_mode, AP_GPS& gps,
                    AP_Mission& mission, Location home, float relative_altitude, int16_t* steering, int16_t* rudder, RC_Channel* channel_rudder);

    /// Engage Trapis WSMP mode
    ///	
    /// Input:			- gcs               = ground control station object to send messages to
    ///                 - g                 = parameters object for retrieving ground station parameters
    ///                 - control_mode      = current mode of the plane
    ///                 - gps               = current state of the gps on the plane
    ///                 - mission           = current state of the flight plan and mission on the plane
    ///                 - home              = current home waypoint loaded onto the plane
    ///
    /// Output:
    ///
    /// Side-effects:
    void engageWSMPMode(GCS_Plane& gcs, Parameters& g, FlightMode* control_mode, AP_GPS& gps, AP_Mission& mission, Location home);

    /// Set Trapis Coordinates
    ///	
    /// Input:			- gcs   = ground control station object to send messages to
    ///                 - Tlat  = Trapis latitude
    ///                 - Tlng  = Trapis longitude
    ///                 - Talt  = Trapis altitude
    ///
    /// Output:			
    ///
    /// Side-effects:	- trapis_state = updates this private field
    void setTrapisCoords(GCS_Plane& gcs, double Tlat, double Tlng, double Talt);

    // ====== Get/Set Functions ==========================


    // data members

protected:

    ///////////// Protected data members /////////////////////////////////

    // data members

private:

    ///////////// Private data members ///////////////////////////////////
    // AFSL Control Surfaces Objects
    struct {
        WingLeveler WL;     // primarily for keeping wings level (ailerons)
        AltitudeHold AH;    // for keeping plane at constant altitude (elevator)
        Steer STR;          // for steering plane to a certain waypoint (rudder)
        Waypoint WP;        // for handling flight plan navigation
    } wstr_state;


    // Contains trapis data
    struct {
        double lat;
        double lng;
        double alt;
        Location loc;
    } trapis_state;
};
#endif // GUARD_Trapis_h
