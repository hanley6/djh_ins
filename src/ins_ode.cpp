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
//  gravity_model.cpp
//  integrator.cpp
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include "ins_ode.h"
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
// The constructor assigns a default value to the derivative of the state
ins_ode::ins_ode()
{
    dstate.setZero(10);
}

// The parameterized constructor computes the time derivative of the state 
// based on gravity inputs
// Inputs: lat = latitude of sensor (in radians) for gravity calculation
//         height = height of sensor (in meters) for gravity calculation
// Outputs: gravity vector based on latitude and height
ins_ode::ins_ode(double lat, double height)
                :grav(lat, height)
{
    dstate.setZero(10);
}

// The parameterized constructor computes the time derivative of the state 
// based on inputs
// Inputs: state = [quaternion, position, velocity] 
//         imu = [accelerometers, gyroscopes]
//         lat = latitude of sensor (in radians) for gravity calculation
//         height = height of sensor (in meters) for gravity calculation
// Outputs: dstate = time derivative of the state
//          gravity vector based on latitude and height
ins_ode::ins_ode(Vector10d state, VectorXd imu, double lat, double height)
                :quaternion(state.head(4)),
                grav(lat, height)
{
    /*---------- Initializations ----------*/
    O = quaternion.OmegaMatrix(imu.tail(3));
    /*--------- End Initiaizations --------*/

    /*-------- Equations of Motion --------*/
    // Quaternion Equations of Motion
    dstate.head(4) = 0.5*O*quaternion.quat;
    // Position Equations of Motion
    dstate.segment(4,3) = state.tail(3);
    // Velocity Equations of Motion
    dstate.tail(3) = quaternion.CreateRot()*imu.head(3) + grav.gravNED;
    /*------ End Equations of Motion ------*/
}

// Updates the time derivative of the state based on inputs
// Inputs: state = [quaternion, position, velocity] 
//         imu = [accelerometers, gyroscopes]
//         lat = latitude of sensor (in radians) for gravity calculation
//         height = height of sensor (in meters) for gravity calculation
// Outputs: dstate = time derivative of the state
void ins_ode::ode_compute(Vector10d state, VectorXd imu)
{
    /*---------- Initializations ----------*/
    O = quaternion.OmegaMatrix(imu.tail(3));
    quaternion.quat = state.head(4);
    /*--------- End Initiaizations --------*/

    /*-------- Equations of Motion --------*/
    // Quaternion Equations of Motion
    dstate.head(4) = 0.5*O*quaternion.quat;
    // Position Equations of Motion
    dstate.segment(4,3) = state.tail(3);
    // Velocity Equations of Motion
    dstate.tail(3) = quaternion.CreateRot()*imu.head(3) + grav.gravNED;
    /*------ End Equations of Motion ------*/
}
/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/ 