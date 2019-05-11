// InnerLoopController.cpp: implementation of the InnerLoopController class.
//
// Christopher Lum
// lum@u.washington.edu
//
// Dai Tsukada
// dat6@uw.edu
//
//////////////////////////////////////////////////////////////////////

//Version History
//	03/31/15: Created

// standard headers
//#include <iostream>						//cout, endl, cerr
//#include <string>						//string
//#include <vector>						//vector
#include <math.h>						//min, max

// local header files
#include "InnerLoopController.h"		//InnerLoopController class
#include "ControlSurfaceDeflections.h"	//ControlSurfaceDeflections class

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
InnerLoopController::InnerLoopController()
{
	//kPhi = 3;   // roll loop forward (P - proportional) gain
	//kP = 0.5;       // roll loop damping (D - diferential) gain
	//kR = 1.2;       // yaw damper forward (P - proportional) gain, tune for Pear (was 1)  1.2
	kTheta = 3;   // pitch loop forward (P - proporcional) gain
	kQ = 0.5;     // pitch loop damping (D - diferential) gain
	kAlt = 0.5;   // altitude loop forward (P - proportional) gain

	// initialize integrators
	intYawDamper = 0;
	intAltitude = 0;

	// initialize previous values for input
	last_psiDotErr = 0;
	last_p = 0;
	last_q = 0;
	last_r = 0;
	last_phi = 0;
	last_theta = 0;
	last_uB = 0;
	last_vB = 0;
	last_wB = 0;
	//last_rad_ref = 0;
	last_rad_act = 0; //Ryan Grimes added rad_act information
	last_alt_ref = 0;
	last_alt = 0; 
	last_dt = 0;
}



//-------------------------------------------------------------------------
////
/// Destructor
////
InnerLoopController::~InnerLoopController()
{
}

//////////////////////////////////////////////////////////////////////
// Overloaded operators
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Public interface methods
//////////////////////////////////////////////////////////////////////

/// Compute control input
///	
/// Input:			- psiDotErr = heading rate error computed in outer loop controller (rad/s)
///                 - p         = roll rate (rad/s)
///                 - q         = pitch rate (rad/s)
///                 - r         = yaw rate (rad/s)
///                 - phi       = bank angle (rad)
///                 - theta     = pitch angle (rad)
///                 - psi       = yaw (heading) angle (rad)
///                 - uB        = velocity in x-direction (m/s)
///                 - vB        = velocity in y-direction (m/s)
///                 - wB        = velocity in z-direction (m/s)
///                 - rad_ref   = reference radius (m)
///                 - rad_act   = actual radius (m)
///                 - alt_ref   = reference altitude (m)
///                 - alt       = actual altitude (m)
///                 - dt        = sample time (s)
///
/// Output:			- dA = Aileron deflection (rad)
///                 - dE = Elevator deflection (rad)
///                 - dR = Rudder deflection (rad)
///
/// Side-effects:	- none
////
ControlSurfaceDeflections InnerLoopController::computeControl(double psiDotErr, double p, double q, double r, 
		double phi, double theta, double uB, double vB, double wB, double rad_act, double alt_ref, double alt, double dt, double kR, double kPhi, double kP, double rad_ref, int cont_type, double kR_der)
{
	////
	/// Check input data range (subject to change depending on aircraft specification)
	////
	if (psiDotErr>1 || psiDotErr<-1)
	{
		// invalid signal from outer loop controller
		//throw std::runtime_error("InnerLoopController Error: Invalid outer loop signal");
		//ControlSurfaceDeflections U = ControlSurfaceDeflections();
		//return(U);

		psiDotErr = last_psiDotErr;
	}
	//if(p>0.9 || p<-0.9 || q>0.9 || q<-0.9|| r>0.5 || r<-0.5 
	//	|| phi>1 || phi<-1 || theta>1.2 || theta<-1.2
	//	|| uB>100 || uB<-100 || vB>100 || vB<-100 || wB>30 || wB<-30
	//	|| alt>6000 || alt<-100)
	//{
	//	// invalid state (inertial measurement) input
	//	//throw std::runtime_error("InnerLoopController Error: Invalid state input");
	//	//ControlSurfaceDeflections U = ControlSurfaceDeflections();
	//	//return(U);
	//}

	// invalid state (inertial measurement) input
	if (p>0.9 || p<-0.9) {
		p = last_p;
	}
	if (q>0.9 || q<-0.9) {
		q = last_q;
	}
	if (r>0.5 || r<-0.5) {
		r = last_r;
	}
	if (phi>1 || phi<-1) {
		phi = last_phi;
	}
	if (theta>1.2 || theta<-1.2) {
		theta = last_theta;
	}
	if (uB>100 || uB<-100) {
		uB = last_uB;
	}
	if (vB>100 || vB<-100) {
		vB = last_vB;
	}
	if (wB>30 || wB<-30) {
		wB = last_wB;
	}
	if (alt>6000 || alt<-100) {
		alt = last_alt;
	}

	//if (rad_ref>10000 || rad_ref<0) {
		// invalid reference radius input
		//rad_ref = last_rad_ref;
	//}
	if (rad_act>10000 || rad_act<0) {
		// invalid actual radius input
		rad_act = last_rad_act;
	}
	if (alt_ref>5000 || alt_ref<0) {
		// invalid altitude reference input
		alt_ref = last_alt_ref;
	}
	if (dt == 0) {
		// invalid time step
		dt = last_dt;
	}

	////
	/// Define Constants
	////
	const double g = 9.80665;
	const double pi = 3.14159;

	////
	/// Inner Loop Interface
	////

	//Ryan Grimes reference heading rate to be (vA/rad_act) instead of (vA/rad_ref)
	//Ryan Grimes updated r_ref equation to multiply by cos(phi) instead of dividing by cos(phi)
	double vA = sqrt(uB*uB + vB*vB + wB*wB);
    //Rostyk Svitelskyi set the average speed with reduced correction from IAS (removed in current version)

    //Limiters kR=1.16; Der=0.01; Pro=0.005.
    //double r_check = ((15 + (vA - 15) / 5) / rad_ref);//RS added limit on required psi_dot
    /*if (r_check < 0.1) {
        r_check = 0.1;
    }
    else if (r_check > 0.21) {
        r_check = 0.21;
    }*/
    //double r_filt = r * 0.5 + last_r * 0.5; //Filter for r-controller//1
    double psiDot = 0;
    double dA = 0;
    double dR = 0;
    //RS added control over reduced radius calculation
    switch (cont_type)
    {
    case 0: {psiDot = psiDotErr + ((kR_der) / rad_ref);}
        break;

        //New alg. kR=2; Der =0.005; Pro=0.0003; Int=0.03.
    case 2: {psiDot = psiDotErr + r * cos(phi) / cos(theta);
        } //Measurment//2
        break;

    case 3:{
        dR = -((rad_act - rad_ref )*kR)-((psiDotErr + ((12 + (vA - 12) / 5) / rad_ref))-r)*kR_der;
        if (dR < -0.5236) {
            dR = -0.5236;
        }
        else if (dR > 0.5236) {
            dR = 0.5236;
        }
        else {
            dR = dR;
        }
        dA = -((-(0.08*dR/0.5236)-phi) * kPhi - p * kP);
        goto labp;}
           break;
    }

    {
        if (psiDot < -0.1) {//RS added limit//3
            psiDot = -0.1;
        }
        else if (psiDot > 0.3) {
            psiDot = 0.3;
        }

        /*New alg. Precise psi_dot. kR=1; Der =0.006; Pro=0.0003.
        double psiDot = psiDotErr + r * cos(phi) / cos(theta) + q * sin(phi) / cos(theta);
        */

        /*Original alg. kR=1; Der=0.0005; Pro=0.0005.
        double psiDot = psiDotErr + (vA / rad_act); //Measurment
        */

        ////
        /// Roll Inner Loop
        ////
        double phi_cmd = atan((psiDot*vA) * (1 / g));


        //Ryan Grimes implemented limitations on desired bank angle
        //Rostyk Svitelskyi changed to radians (35 deg = 0.61 rad)
        if (phi_cmd < -0.61) {
            phi_cmd = -0.61;
        }
        else if (phi_cmd > 0.61) {
            phi_cmd = 0.61;
        }

        double phi_e = phi_cmd - phi;
        dA = -(phi_e*kPhi - p * kP);

        // reference yaw rate
        // double r_ref = (vA/rad_act)*cos(phi); //original *
        double r_ref = (kR_der/ rad_act) / cos(phi);
        ////
        /// Yaw Damper
        ////
        double r_e = r_ref - r;

        // was kR/5
        /*intYawDamper += r_e*dt;//4
        // signal saturation
        if (intYawDamper < -0.7) {
            intYawDamper = -0.7;
        } else if (intYawDamper > 0.7) {
            intYawDamper = 0.7;
        }
        double dR = -(r_e*kR + intYawDamper * (kR / 10));
        */
        dR = -(r_e*kR);
    }
    labp:
	////
	/// Altitude Loop
	////
	double alt_e = alt_ref - alt;
	intAltitude += (kAlt/15)*alt_e*dt;
	// signal saturation
	if (intAltitude < -60) {
		intAltitude = -60;
	} else if (intAltitude > 60) {
		intAltitude = 60;
	}
	double theta_cmd = (alt_e*kAlt + intAltitude) * (pi/180);

	// Ryan Grimes implemented limitations on desired pitch
    // 20 deg = 0.35 rad
	if (theta_cmd < -0.35) {
		theta_cmd = -0.35;
	} else if (theta_cmd > 0.35) {
		theta_cmd = 0.35;
	}

	////
	/// Pitch Inner Loop
	////
	double theta_e = theta_cmd - theta;
	double dE = -(theta_e*kTheta - q*kQ);

	// save input information
	last_psiDotErr = psiDotErr;
	last_p = p;
	last_q = q;
	last_r = r;
	last_phi = phi;
	last_theta = theta;
	last_uB = uB;
	last_vB = vB;
	last_wB = wB;
	//last_rad_ref = rad_ref;
	last_rad_act = rad_act; // Ryan Grimes added rad_act information
	last_alt_ref = alt_ref;
	last_alt = alt;
	last_dt = dt;


	// instantiate control surface deflection object (data type to store control inputs)
	ControlSurfaceDeflections U = ControlSurfaceDeflections();

	U.SetAileron(dA);
	U.SetRudder(dR);
	U.SetElevator(dE);

	return(U);
}
	

//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================

