// WingLeveler.h: interface for the WingLeveler class.
//
// Ryan Grimes
// rjgrimes@uw.edu
//
// Hannah Rotta
// hrotta@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_WingLeveler_h
#define GUARD_WingLeveler_h

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
#include <AP_AHRS/AP_AHRS_DCM.h>
#include "AFSL_Constants.h"

// Local header files
//#include "ControlSurfaceDeflections.h"		//ControlSurfaceDeflections class

//-------------------------------------------------------------------------------
/// Define WingLeveler objects which are used to represent a wing leveler controller.
///
////
class WingLeveler {

public:
	///////////// Construction/Destruction ///////////////////////////////

	// default constructor
	WingLeveler();
	
	// destructor
	virtual ~WingLeveler();

	///////////// Overload operators /////////////////////////////////////


	///////////// Public interface methods ///////////////////////////////
    double computeAileronDeflection(AP_AHRS_DCM& ahrs, double pro_gain, double der_gain);
	
	// ====== Get/Set Functions ==========================


	// data members

protected:

	///////////// Protected data members /////////////////////////////////

	// data members

	////
	/// Gains
	////
	double kPhi;   // proportional
	double kP;       // derivative
	
private:

	///////////// Private data members ///////////////////////////////////

	// integrator terms
	//double intAltitude;

	// previous values for inputs
	double last_p;
	double last_phi;

};
#endif
