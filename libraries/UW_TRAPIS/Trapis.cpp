// AltitudeHold.cpp: implementation of the AltitudeHold class.
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
/// Side-effects:	-none
////
Trapis::Trapis()
{
    kAlt = 0.5;   // proportional altitude hold
    kTheta = 3;   // proportional pitch gain
    kQ = 0.5;       // derivative pitch gain

                    // initialize integrators
    intAltitude = 0;

    // initialize previous values for input
    last_alt = 0;
    last_q = 0;
    last_theta = 0;
    last_dt = 0;
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

/// Compute Elevator Deflection
///	
/// Input:			- alt         = altitude (m)
///                 - theta       = pitch angle (rad)
///                 - q           = pitch rate (rad/s)
///
/// Output:			- dE = Elevator deflection (rad)
///
/// Side-effects:	- none
////
void Trapis::engageMode(GCS_Plane& gcs, AP_AHRS& ahrs, Parameters& g, FlightMode& control_mode, AP_GPS& gps, 
                        AP_Mission mission, Location home, float relative_altitude, int16_t* steering, int16_t* rudder, RC_Channel* channel_rudder)
{
    // Set waypoint radius for Waypoint Navigation (using AFSL Library)
    uint16_t wp_rad = g.waypoint_radius;

    // Sets plane's location based on Mission Planner parameter WSTR_TRAPIS_LOC
    // When set to 1, uses trapis coords. When set to 0, uses gps coords
    Location plane_location;

    // Sets plane position (from Waypoint Navigation perspective) to trapis location/coords
    // if appropriate Mission Planner parameter (WSTR_TRAPIS_LOC) is 1, otherwise uses plane gps coords
    plane_location = g.wstr_trapis_loc == 1 ? trapis_state.loc : gps.location();

    // Retrieve waypoint
    Location waypoint = wstr_state.WP.nextWaypoint(mission, plane_location, wp_rad, home, &control_mode);
    wstr_state.WP.sendMessage(gcs, home);

    // Calculate control surface deflections with parameters from MissionPlanner
    // Wing Leveler Gains
    double wl_pro_gain = g.wstr_wl_pro_gain;
    double wl_der_gain = g.wstr_wl_der_gain;

    // Altitude Hold Gains
    double kAlt = g.wstr_ah_pro_gain;

    // Steer gains
    double kPsi = g.wstr_rd_pro_gain; // proportional gain
    double kR = g.wstr_rd_der_gain; // derivative gain

    double dA = wstr_state.WL.computeAileronDeflection(ahrs, g.wstr_wl_pro_gain, g.wstr_wl_der_gain); // rad
    double dE = wstr_state.AH.computeElevatorDeflection(relative_altitude, ahrs, g.wstr_ah_pro_gain); // rad
    double dR = wstr_state.STR.computeRudderDeflection(waypoint, plane_location, ahrs, g.wstr_rd_pro_gain, g.wstr_rd_der_gain); // centidegrees

    // Set RC output channels to control surface deflections
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, dA); //centidegrees
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, dE); //centidegrees

    // If wstr_uw_activate is set to 0 in Mission Planner, gives rudder control to Hannah/Pilot
    if (g.wstr_activate != 1) {
        *steering = *rudder = channel_rudder->get_control_in_zero_dz();
    }
    // If wstr_activate == 1, use full waypoint navigation
    else {
        *steering = *rudder = -dR; //Units: centi-degrees
    }


    // set RC channel 3 PWM (throttle)

    // For use only in simulation
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 25); //percentage
}


//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

