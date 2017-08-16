// OuterLoopController.cpp: implementation of the OuterLoopController class.
//
// Christopher Lum
// lum@u.washington.edu
//
// Dai Tsukada
// dat6@uw.edu
//
//////////////////////////////////////////////////////////////////////

//Version History
//  03/31/15: Created

// standard headers
//#include <iostream>                       //cout, endl, cerr
//#include <string>                     //string
//#include <vector>                     //vector
//#include <math.h>                       //min, max

// local header files
#include "OuterLoopController.h"        //InnerLoopController class

// using declaration


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------
////
/// Default constructor.
///
/// Input:          -none
///
/// Output:         -none
///
/// Side-effects:   -none
////
OuterLoopController::OuterLoopController()
{
    Kp_outer = 3;        // outerloopcontroller forward gain
    fadein_tau = 0.005;  // time constant for fade-in activation

    //initialize parameters
    last_fader = 0;
    last_rad_ref = 0;
    last_rad_act = 0;

    //radius derivative values
    dr = 0;
    dr_1 = 0;
    dr_2 = 0;
    dr_3 = 0;
    dr_4 = 0;
    dr_5 = 0;
    dr_count = 0;
}



//-------------------------------------------------------------------------
////
/// Destructor
////
OuterLoopController::~OuterLoopController()
{
}

//////////////////////////////////////////////////////////////////////
// Overloaded operators
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Public interface methods
//////////////////////////////////////////////////////////////////////

/// Compute outer loop signal (heading rate error)
///
/// Input:          - rad_act = actual radius measured by vision system (m)
///                 - rad_ref = reference radius (m)
///
/// Output:         - psiDotErr = heading rate error (rad/s)
///
/// Side-effects:   - none
////
double OuterLoopController::computeOuterLoopSignal(double rad_act, double rad_ref)
{

    double psiDotErr;
    ////
    /// Check input data validity
    ////
    if (rad_act < 0) {
        // disable the outer loop controller if vision system passes invalid value (negative radius)
        //throw std::runtime_error("OuterLoopController Error: Invalid actual radius input");
        //return psiDotErr = 0;

        rad_act = last_rad_act;
    }
    if (rad_ref < 0) {
        // disable the outer loop controller if user input is invalid
        //throw std::runtime_error("OuterLoopController Error: Invalid reference radius input");
        //return psiDotErr = 0;

        rad_ref = last_rad_ref;
    }


    ////
    /// Outer Loop Algorithm
    ////
    //UW_Mode pre_gain = 5e-3 (simulator)
    double pre_gain = 5e-3;  // gain to scale down the radius error into the appropriate order of magnitude
    double r_err = rad_ref - rad_act;

	//UW_Mode dr_gain = 1e-2 (simulator)
    double dr_gain = 1e-2; // gain to scale down the radius derivative error

	double dt = 0.02; //delta-t for computing the radius derivative 

	double der_forget_factor = 0.8; //forgetting factor to reduce derivative input signal when no new radius information
	double pro_forget_factor = 0.98; //forgetting factor to reduce proportional input signal when no new radius information

    //radius derivative, used to maintain a flight path tangential to the desired orbit
    if (last_rad_act == 0){
        //do nothing, can't compute derivative
    }
    else if (dr_count == 0){
        dr_1 = (rad_act - last_rad_act)/dt;

		if (rad_act - last_rad_act == 0){
			dr_1 = dr_5*der_forget_factor;
			r_err = r_err*pro_forget_factor;
		}

        dr_count++;
    }
    else if (dr_count == 1){
        dr_2 = (rad_act - last_rad_act)/dt;

		if (rad_act - last_rad_act == 0){
			dr_2 = dr_1*der_forget_factor;
			r_err = r_err*pro_forget_factor;
		}

        dr_count++;
    }
    else if (dr_count == 2){
        dr_3 = (rad_act - last_rad_act)/dt;

		if (rad_act - last_rad_act == 0){
			dr_3 = dr_2*der_forget_factor;
			r_err = r_err*pro_forget_factor;
		}

        dr_count++;
    }
    else if (dr_count == 3){
        dr_4 = (rad_act - last_rad_act)/dt;

		if (rad_act - last_rad_act == 0){
			dr_4 = dr_3*der_forget_factor;
			r_err = r_err*pro_forget_factor;
		}

        dr_count++;
    }
    else if (dr_count == 4){
        dr_5 = (rad_act - last_rad_act)/dt;

		if (rad_act - last_rad_act == 0){
			dr_5 = dr_4*der_forget_factor;
			r_err = r_err*pro_forget_factor;
		}

        dr_count = 0;
    }
    dr = (dr_1+dr_2+dr_3+dr_4+dr_5)/5; //averages 5 sequential derivatives to reduce noise

	//Ryan Grimes disabled the derivative gain correction sign factor (because he thinks its incorrect)
    //This makes sure the radius derivative gain will have the correct sign
    //if ((r_err<0 && rad_act>last_rad_act) || (r_err>0 && rad_act<last_rad_act))
        //dr = -dr;

	//Ryan Grimes disabled the fade in gain application to the forward gain because of its negative impact on wind drift correction
    //double forward_gain = OuterLoopController::activateController() * pre_gain * Kp_outer;
	double forward_gain = pre_gain * Kp_outer;
	psiDotErr = r_err * forward_gain * -1 + dr_gain*dr;


    // signal saturation
	// Ryan Grimes increased max and min psiDotErr values (was 0.1)
    if (psiDotErr < -0.5) {
        psiDotErr = -0.5;
    } else if (psiDotErr > 0.5) {
        psiDotErr = 0.5;
    }

    // save input information
    last_rad_act = rad_act;
    last_rad_ref = rad_ref;

    return psiDotErr;
}

/// Compute fade-in activation gain for outer loop controller
///
/// Input:          - none
///
/// Output:         - fader = fade-in gain for outer loop controller
///
/// Side-effects:   - none
////

double OuterLoopController::activateController()
{
    ////
    /// Fade-in activation gain for Outer Loop
    ////
    fader = last_fader + fadein_tau * (1 - last_fader); // discrete low pass fliter with alpha = 1
    if (1 - fader < 1e-3){
        fader = 1;
    }
    last_fader = fader;

    return fader;
}


//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

