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
	//intAltitude = 0;

	// initialize previous values for input
    last_alt = 0;
	last_q = 0;
	last_theta = 0;
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
double AltitudeHold::computeElevatorDeflection(double alt, double theta, double q)
{
	////
	/// Check input data range (subject to change depending on aircraft specification)
	////

	// invalid state (inertial measurement) input
	if (q>0.9 || q<-0.9) {
		q = last_q;
	}
	if (theta>1 || theta<-1) {
        theta = last_theta;
	}

	////
	/// Elevator Control Interface
	////
	
    double pi = 3.14159; // constant

    double alt_ref = 100; //(m)
    //double alt = relative_altitude; //(m)

    double alt_e = alt_ref - alt;

    double theta_cmd = (alt_e*kAlt) * (pi / 180);

    double theta_e = theta_cmd - theta;
    double dE = -(theta_e*kTheta - q*kQ);

    // limit elevator deflection to +/- 30 deg

    if (dE < -0.5236) {
        dE = -0.5236;
    }
    else if (dE > 0.5236) {
        dE = 0.5236;
    }

	// save input information
	last_q = q;
    last_theta = theta;
    last_alt = alt;

	return(dE);
}
	

//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

