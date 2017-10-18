//
//	Inertial Navigation System Ordinary Differential Equations
//	David Hanley
//	
//	ins_ode.h
//	Inertial navigation system ordinary differential equations
//  header file for the Bretl Group INS package. This contains 
//  a class that describes the equations of motion of an IMU.
//
//	Usage:
//	quaternion_math.cpp	
//	gravity_model.cpp
//	integrator.cpp
//

/*---------------- Include Guard ---------------*/
#ifndef ins_ode_H
#define ins_ode_H
/*-------------- End Include Guard -------------*/

/*------------------ Includes ------------------*/
#include <Eigen/Core>
#include "quaternion_math.h"
#include "gravity_model.h"
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
/*--------------- End Namespaces ---------------*/

/*------------------ Globals -------------------*/
typedef Matrix<double, 10, 1> Vector10d;
/*---------------- End Globals -----------------*/

/*------------------- Classes ------------------*/
class ins_ode
{
    public:
        // Constructor
        ins_ode();

        // The parameterized constructor computes the time derivative of the state 
        // based on gravity inputs
        // Inputs: lat = latitude of sensor (in radians) for gravity calculation
        //         height = height of sensor (in meters) for gravity calculation
        // Outputs: gravity vector based on latitude and height
        ins_ode(double lat, double height);

        // The parameterized constructor computes the time derivative 
        // of the state based on inputs
        // Inputs: state = [quaternion, position, velocity] 
        //         imu = [accelerometers, gyroscopes]
        //         lat = latitude of sensor (in radians) for gravity 
        //               calculation
        //         height = height of sensor (in meters) for gravity 
        //                  calculation
        // Outputs: dstate = time derivative of the state
        ins_ode(Vector10d state, VectorXd imu, double lat, double height);

        // Update the State Derivatives
        void ode_compute(Vector10d state, VectorXd imu);

        // State Derivatives
        Vector10d dstate;
    private:
        // Quaternion 
        QuatMath quaternion;

        // Gravity Vector
        GravModel grav;

        // Angular rate matrix for quaternion ODE
        Matrix4d O;
};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/
