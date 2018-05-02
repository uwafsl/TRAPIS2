// AltitudeHold.cpp: implementation of the AltitudeHold class.
//
// Ryan Grimes
// rjgrimes@uw.edu
//
// Hannah Rotta
// hrotta@uw.edu
//
//////////////////////////////////////////////////////////////////////

//Version History
//	02/14/18: Created

// standard headers
//#include <iostream>						//cout, endl, cerr
//#include <string>						//string
//#include <vector>						//vector
#include <math.h>						//min, max

// local header files
#include "AltitudeHold.h"		//AltitudeHold class

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
AltitudeHold::AltitudeHold()
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
AltitudeHold::~AltitudeHold()
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
double AltitudeHold::computeElevatorDeflection(double alt, AP_AHRS_DCM& ahrs, double alt_pro_gain)
{
	////
	/// Check input data range (subject to change depending on aircraft specification)
	////

    // get q
    double q = ahrs.get_gyro().y;
    double dt = DT;
    double theta = ahrs.pitch;


	// invalid state (inertial measurement) input
	if (q>0.9 || q<-0.9) {
		q = last_q;
	}
	if (theta>1 || theta<-1) {
        theta = last_theta;
	}
    if (alt>6000 || alt<-100) {
        alt = last_alt;
    }
    if (dt == 0) {
        // invalid time step
        dt = last_dt;
    }

	////
	/// Elevator Control Interface
	////
    // Mission Planner parameter controlling altitude proportional gain
    kAlt = alt_pro_gain;   // proportional altitude hold

    double pi = 3.14159; // constant

    double alt_ref = 100; //(m)
    //double alt = relative_altitude; //(m)

    double alt_e = alt_ref - alt;

    intAltitude += (kAlt / 15)*alt_e*dt;
    // signal saturation
    if (intAltitude < -60) {
        intAltitude = -60;
    }
    else if (intAltitude > 60) {
        intAltitude = 60;
    }

    double theta_cmd = (alt_e*kAlt + intAltitude) * (pi / 180);

    // Limit max/min bank angle
    double theta_cmd_limit = 25 * pi / 180; // 25 deg in radians
    if (theta_cmd < -theta_cmd_limit) {
        theta_cmd = -theta_cmd_limit;
    }
    else if (theta_cmd > theta_cmd_limit) {
        theta_cmd = theta_cmd_limit;
    }

    double theta_e = theta_cmd - theta;
    double dE = -(theta_e*kTheta - q*kQ);

    // limit elevator deflection to +/- 30 deg

    double dE_limit = 30 * pi / 180; // 30 deg in radians
    if (dE < -dE_limit) {
        dE = -dE_limit;
    }
    else if (dE > dE_limit) {
        dE = dE_limit;
    }

	// save input information
	last_q = q;
    last_theta = theta;
    last_alt = alt;
    last_dt = dt;

	return(dE);
}
	

//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

