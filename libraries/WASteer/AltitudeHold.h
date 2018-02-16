// AltitudeHold.h: interface for the AltitudeHold class.
//
// Ryan Grimes
// rjgrimes@uw.edu
//
// Hannah Rotta
// hrotta@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_AltitudeHold_h
#define GUARD_AltitudeHold_h

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

//-------------------------------------------------------------------------------
/// Define altitude hold objects which are used to represent a altitude hold controller.
///
////
class AltitudeHold {

public:
	///////////// Construction/Destruction ///////////////////////////////

	// default constructor
	AltitudeHold();
	
	// destructor
	virtual ~AltitudeHold();

	///////////// Overload operators /////////////////////////////////////


	///////////// Public interface methods ///////////////////////////////
    double computeElevatorDeflection(double alt, double theta, double q);
	
	// ====== Get/Set Functions ==========================


	// data members

protected:

	///////////// Protected data members /////////////////////////////////

	// data members

	////
	/// Gains
	////
	double kAlt;         // proportional altitude hold
	double kTheta;       // proportional pitch control
    double kQ;           // derivative pitch control
	
private:

	///////////// Private data members ///////////////////////////////////

	// integrator terms
	//double intAltitude;

	// previous values for inputs
	double last_q;
	double last_theta;
    double last_alt;

};
#endif
