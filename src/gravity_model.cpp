//
//	Gravity Model
//	David Hanley
//	
//	gravity_model.cpp
//  Gravity model header file for the Bretl Group INS 
//  package. This contains a class of potential gravity 
//  vectors. This includes the standard gravity vector 
//  and a simple gravity model described in:
//
//  D. Titterton and J. Weston. "Strapdown Inertial 
//  Navigation Technology." IET Radar, Sonar, Navigation 
//  and Avionics Series. Volume 17. 2nd Ed. 2004.
//
//  Resulting vectors are computed in a NED and ENU 
//  coordinate frame.
//
//	Usage:	
//  ins_ode.cpp
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include "gravity_model.h"
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/

/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
/*--------------- End Namespaces ---------------*/

/*------------------ Pragmas -------------------*/
/*---------------- End Pragmas -----------------*/

/*------------- Function Prototypes ------------*/
/*----------- End Function Prototypes ----------*/
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Preamble ---------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*--------------------------------- Functions ---------------------------------*/
/*-----------------------------------------------------------------------------*/
// The constructor assigns a default identity value to the class gravity vectors 
GravModel::GravModel()
{
    // Define standard gravity
    double std_gravity = 9.80665;

    // Define NED gravity vector given standard gravity
    gravNED(0) = 0.0;
    gravNED(1) = 0.0;
    gravNED(2) = std_gravity;

    // Define ENU gravity vector given standard gravity
    gravENU(0) = 0.0;
    gravENU(1) = 0.0;
    gravENU(2) = -std_gravity;
}

// The parameterized constructor assigns a gravity vector
// based on users location.
// Inputs:  lat = latitude (in radians) of current sensor location
//          height = altitude (in meters) of current sensor location
// Outputs: gravNED and gravENU
GravModel::GravModel(double lat, double height)
{   
    // Mean radius of curvature of Earth and rotation rate
    double R0 = 6371008.00;             // in meters
    double rotEarth = 0.00007292115;    // in rad/s

    // Define magnitude of gravity
    double s2L = pow(sin(lat),2.0);
    double s22L = pow(sin(2.0*lat),2.0);
    double g_norm = 9.780318*(1.0 + 0.0053024*s2L - 0.0000059*s22L);
    double g_normHeight = g_norm/pow(1.0 + height/R0,2.0);

    // Compute gravity with the plumb-bob effect
    gravNED(0) = -(pow(rotEarth,2.0)*(R0 + height)*sin(2.0*lat)/2.0);
    gravNED(1) = 0.0;
    gravNED(2) = g_normHeight - (pow(rotEarth,2.0)*(R0 + height)*(1.0 + cos(2.0*lat))/2.0);

    gravENU(0) = 0.0;
    gravENU(1) = -(pow(rotEarth,2.0)*(R0 + height)*sin(2.0*lat)/2.0);
    gravENU(2) = -g_normHeight + (pow(rotEarth,2.0)*(R0 + height)*(1.0 + cos(2.0*lat))/2.0);
}
/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/ 