//
//	Preintegration 
//	David Hanley
//	
//	preintegration.h
//	Preintegration header file for the Bretl Group INS package. 
//  This contains a class that computes the preintegration intervals 
//  that can be useful in sensor fusion approaches like visual-
//  inertial odometry.
//
//	Usage:
//  djh_ins.cpp
//	

/*---------------- Include Guard ---------------*/
#ifndef preintegration_H
#define preintegration_H
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
class preintegrate
{
    public:
        // Constructor
        preintegrate();

    private:
};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/
