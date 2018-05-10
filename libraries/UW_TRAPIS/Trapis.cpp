// Trapis.cpp: implementation of the Trapis class. Contains methods
// to engage the trapis WSTR mode and update trapis coordinates
// in the plane.
//
// Ravi Patel
// patelr3@uw.edu
//
//////////////////////////////////////////////////////////////////////

//Version History
//	05/18/18: Created

// standard headers
//#include <iostream>						//cout, endl, cerr
//#include <string>						//string
//#include <vector>						//vector
#include <math.h>						//min, max

// local header files
#include "Trapis.h"		//AltitudeHold class

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
/// Side-effects:	-Initialize all objects with default constructors
////
Trapis::Trapis()
{
    trapis_state.lat = 0;
    trapis_state.lng = 0;
    trapis_state.alt = 0;
}



//-------------------------------------------------------------------------
////
/// Destructor
////
Trapis::~Trapis()
{

}

//////////////////////////////////////////////////////////////////////
// Overloaded operators
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Public interface methods
//////////////////////////////////////////////////////////////////////

/// Engage Trapis mode
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
////
void Trapis::engageWSTRMode(GCS_Plane& gcs, AP_AHRS& ahrs, Parameters& g, FlightMode* control_mode, AP_GPS& gps, 
                        AP_Mission& mission, Location home, float relative_altitude, int16_t* steering, int16_t* rudder, RC_Channel* channel_rudder)
{   
    /// Parameters
    // Set waypoint radius for Waypoint Navigation (using AFSL Library)
    uint16_t wp_rad = g.waypoint_radius;

    // Wing Leveler Gains
    double wl_pro_gain = g.wstr_wl_pro_gain;    // wing leveler proportional gain
    double wl_der_gain = g.wstr_wl_der_gain;    // wing leveler derivative gain

    // Altitude Hold Gains
    double kAlt = g.wstr_ah_pro_gain;   // altitude hold proportional gain

    // Steer Gains
    double kPsi = g.wstr_rd_pro_gain;   // rudder proportional gain
    double kR = g.wstr_rd_der_gain;     // rudder derivative gain

    /// Plane behavior
    // Sets plane position (from Waypoint Navigation perspective) to trapis location/coords
    // if appropriate Mission Planner parameter (WSTR_TRAPIS_LOC) is 1, otherwise uses plane gps coords
    Location plane_location;
    plane_location = g.wstr_trapis_loc == 1 ? trapis_state.loc : gps.location();

    // Retrieve waypoint
    Location waypoint = wstr_state.WP.nextWaypoint(mission, plane_location, wp_rad, home, control_mode);
    wstr_state.WP.sendMessage(gcs, home);  // Sends waypoint navigation information to gcs program

    // Calculate control surface deflections for aileron, elevator, and 
    double dA = wstr_state.WL.computeAileronDeflection(ahrs, wl_pro_gain, wl_der_gain); // rad
    double dE = wstr_state.AH.computeElevatorDeflection(relative_altitude, ahrs, kAlt); // rad
    double dR = wstr_state.STR.computeRudderDeflection(waypoint, plane_location, ahrs, kPsi, kR); // centidegrees

    // Set RC output channels to control surface deflections
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, dA); //centidegrees
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, dE); //centidegrees

    // If wstr_activate is set to 0 in Mission Planner, gives rudder control to pilot (no automatic waypiont navigation)
    if (g.wstr_activate != 1) {
        *steering = *rudder = channel_rudder->get_control_in_zero_dz();
    }
    // If wstr_activate == 1, use full waypoint navigation
    else {
        *steering = *rudder = -dR;
    }

    // Set throttle
    // For use only in simulation
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 25); //percentage
}

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
void Trapis::setTrapisCoords(GCS_Plane& gcs, double Tlat, double Tlng, double Talt)
{

    trapis_state.lat = Tlat;
    trapis_state.lng = Tlng;
    trapis_state.alt = Talt;

    trapis_state.loc.lat = (int32_t)(Tlat * 1e7);
    trapis_state.loc.lng = (int32_t)(Tlng * 1e7);
    trapis_state.loc.alt = (int32_t)(Talt * 100);

    gcs.send_text(MAV_SEVERITY_INFO, "Set GPS to %.6f %.6f",
        Tlat,
        Tlng);
}


//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

