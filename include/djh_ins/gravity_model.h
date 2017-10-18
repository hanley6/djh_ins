//
//	Gravity Model
//	David Hanley
//	
//	gravity_model.h
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
//	ins_ode.cpp
//

/*---------------- Include Guard ---------------*/
#ifndef gravity_model_H
#define gravity_model_H
/*-------------- End Include Guard -------------*/

/*------------------ Includes ------------------*/
#include <Eigen/Core>
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
/*--------------- End Namespaces ---------------*/

/*------------------ Globals -------------------*/
/*---------------- End Globals -----------------*/

/*------------------- Classes ------------------*/
class GravModel
{
    public:
        // Constructor
        GravModel();

        // The parameterized constructor assigns a gravity vector
        // based on users location.
        // Inputs:  lat = latitude (in radians) of current sensor location
        //          height = altitude (in meters) of current sensor location
        // Outputs: gravNED and gravENU
        GravModel(double lat, double height);

        // NED Gravity Vector
        Vector3d gravNED;

        // ENU Gravity Vector
        Vector3d gravENU;
    private:
};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/
