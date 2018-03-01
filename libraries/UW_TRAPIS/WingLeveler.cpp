// WingLeveler.cpp: implementation of the WingLeveler class.
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
#include "WingLeveler.h"		//InnerLoopController class

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
WingLeveler::WingLeveler()
{
	kPhi = 3;   // proportional gain
	kP = 0.5;       // derivative gain

	// initialize integrators
	//intAltitude = 0;

	// initialize previous values for input
	last_p = 0;
	last_phi = 0;
}



//-------------------------------------------------------------------------
////
/// Destructor
////
WingLeveler::~WingLeveler()
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
/// Input:			- phi       = bank angle (rad)
///                 - p         = roll rate (rad/s)
///
/// Output:			- dA = Aileron deflection (rad)
///
/// Side-effects:	- none
////
double WingLeveler::computeAileronDeflection(double phi, double p)
{
	////
	/// Check input data range (subject to change depending on aircraft specification)
	////

	// invalid state (inertial measurement) input
	if (p>0.9 || p<-0.9) {
		p = last_p;
	}
	if (phi>1 || phi<-1) {
		phi = last_phi;
	}

	////
	/// Aileron Control Interface
	////
	
    double phi_cmd = 0; // desired bank angle is zero for a wing leveler

	double phi_e = phi_cmd - phi;
	double dA = -(phi_e*kPhi - p*kP);

    // limit aileron deflection to +/- 30 deg

    if (dA < -0.5236) {
        dA = -0.5236;
    }
    else if (dA > 0.5236) {
        dA = 0.5236;
    }

	// save input information
	last_p = p;
    last_phi = phi;

	return(dA);
}
	

//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

