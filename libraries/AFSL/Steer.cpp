// Steer.cpp: implementation of the Steer class.
//
// Ryan Grimes
// rjgrimes@uw.edu
//
// Hannah Rotta
// hrotta@uw.edu
//
//////////////////////////////////////////////////////////////////////

//Version History
//	02/21/18: Created

// standard headers
//#include <iostream>						//cout, endl, cerr
//#include <string>						//string
//#include <vector>						//vector
#include <math.h>						//min, max

// local header files
#include "Steer.h"		//InnerLoopController class

#define PI 3.14159

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
Steer::Steer()
{
	kPsi = 3;   // proportional gain
	kR = 0.5;       // derivative gain

	// initialize integrators
	//intAltitude = 0;

	// initialize previous values for input
	last_r = 0;
	last_psi = 0;
    last_nav_bearing = 0;
}



//-------------------------------------------------------------------------
////
/// Destructor
////
Steer::~Steer()
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
/// Input:			- psi           = yaw angle (centidegrees)
///                 - r             = yaw rate (centidegrees/s)
///                 - nav_bearing   = heading to target (centidegrees)
///
/// Output:			- dR = Rudder deflection (rad)
///
/// Side-effects:	- none
////
double Steer::computeRudderDeflection(Location waypoint, Location cur_loc, AP_AHRS& ahrs, double pro_gain, double der_gain)
{
	////
	/// Check input data range (subject to change depending on aircraft specification)
	////

    double off_x = waypoint.lng - cur_loc.lng;
    double off_y = waypoint.lat - cur_loc.lat;
    double bearing = 9000 + atan2f(-off_y, off_x) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;   // centidegrees
    }

    double r = ahrs.get_gyro().z * 180 / PI * 100; // centidegrees/s
    double psi = ahrs.yaw * 180 / PI * 100;

    //convert psi to range of [0, 36000] centidegrees
    if (psi < 0) {
        psi += 36000;
    }

	// invalid state (inertial measurement) input
	//if (r>0.9 || r<-0.9) {
		//r = last_r;
	//}

	////
	/// Rudder Control Interface
	////
    kPsi = pro_gain;
    kR = der_gain;

    double psi_e = bearing - psi;

    if (psi_e < -18000) {
        psi_e = psi_e + 36000;
    }

    if (psi_e > 18000) {
        psi_e = psi_e - 36000;
    }

    double dR = -(psi_e*kPsi); // -r*kR);

   

    // limit rudder deflection to +/- 30 deg

    if (dR < -3000) {
        dR = -3000;
    }
    else if (dR > 3000) {
        dR = 3000;
    }

	// save input information
	last_r = r;
    last_psi = psi;
    last_nav_bearing = bearing;

	return(dR);
}
	

//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

