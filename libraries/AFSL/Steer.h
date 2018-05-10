// Steer.h: interface for the Steer class.
//
// Ryan Grimes
// rjgrimes@uw.edu
//
// Hannah Rotta
// hrotta@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_Steer_h
#define GUARD_Steer_h

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>

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

// Local header files
//#include "ControlSurfaceDeflections.h"		//ControlSurfaceDeflections class

//-------------------------------------------------------------------------------
/// Define Steer objects which are used to represent a rudder controller.
///
////
class Steer {

public:
	///////////// Construction/Destruction ///////////////////////////////

	// default constructor
    Steer();
	
	// destructor
	virtual ~Steer();

	///////////// Overload operators /////////////////////////////////////


	///////////// Public interface methods ///////////////////////////////
    double computeRudderDeflection(Location waypoint, Location cur_loc, AP_AHRS& ahrs, double pro_gain, double der_gain);

	
	// ====== Get/Set Functions ==========================


	// data members

protected:

	///////////// Protected data members /////////////////////////////////

	// data members

	////
	/// Gains
	////
	double kPsi;   // proportional
	double kR;       // derivative
	
private:

	///////////// Private data members ///////////////////////////////////

	// integrator terms
	//double intAltitude;

	// previous values for inputs
	double last_r;
	double last_psi;
    double last_nav_bearing;

};
#endif
