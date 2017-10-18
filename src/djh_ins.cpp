//
//	INS solution interfaces all INS classes with code intending to
//  use the djh_ins package. Unless ROS is used, in which case 
//  djh_ins_ros interfaces all INS classes with the code using
//  the package.
//	David Hanley
//	
//	djh_ins.h
//
//
//	DJH INS implementation file for the Bretl Group INS package. 
//  This contains a class that is called and used by all codes 
//  using the INS package (except those codes also using ROS).
//
//	Usage:
//  integrator.cpp
//  preintegration.cpp
//  imu_correct.cpp
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include <iostream>
#include "djh_ins.h"
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
// Default constructor initializes all integrator, imu correction, 
// and preintegration classes to their default cases.
djh_ins::djh_ins()
{
    // Set Initial State
    state.setZero(10);
    state(3) = 1.0;
}
        
// Parameterized constructor with WGS 84 Gravity Model and entire 
// IMU model for integration method.
// Inputs:  start_tim = start time in seconds (presumably a Unix time)
//          end_tim = end time in seconds (presumable a Unix time)
//          imu = IMU measurement matrix of the form
//                      [time_1   accel   gyro]
//                      [time_2   accel   gyro]
//                      [...      ...     ... ]
//                And we assume entries are in chronological order
//                i.e. time_1 < time_2 < ...
//          state_start = starting state prior to integration (10 elements: 
//                        quaternion, position, velocity)
//          integration_choice = a string selecting either "Euler", 
//                               "Heun", "RK4", or "Savage" methods
//          lat = latitude of the starting point (for gravity model)
//          height = height of the starting point above sea level 
//                   (for gravity model)
//          scale_g = scale factor errors for IMU gyroscopes
//          scale_a = scale factor errors for IMU accelerometers
//          cross_c_g = cross coupling errors for IMU gyroscopes
//          cross_c_a = cross coupling errors for IMU accelerometers
//          bf_g = fixed gyroscope biases
//          bf_a = fixed accelerometer biases
//          b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: Setup Integrator class, Setup imu_correct class
djh_ins::djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                 string integrate_choice, double lat, double height, Matrix3d scale_g, 
                 Matrix3d scale_a, Matrix3d cross_c_g, Matrix3d cross_c_a, Vector3d bf_g, 
                 Vector3d bf_a, Vector3d b_a, Vector3d b_g)
        :integ(start_tim, end_tim, imu, state_start, integrate_choice, lat, height),
         imu_corr(scale_g, scale_a, cross_c_g, cross_c_a, bf_g, bf_a, b_a, b_g)
{
    state = state_start;
}
        
// Parameterized constructor with standard gravity model and entire
// IMU model.
// Inputs:  start_tim = start time in seconds (presumably a Unix time)
//          end_tim = end time in seconds (presumable a Unix time)
//          imu = IMU measurement matrix of the form
//                      [time_1   accel   gyro]
//                      [time_2   accel   gyro]
//                      [...      ...     ... ]
//                And we assume entries are in chronological order
//                i.e. time_1 < time_2 < ...
//          state_start = starting state prior to integration (10 elements: 
//                        quaternion, position, velocity)
//          integration_choice = a string selecting either "Euler", 
//                               "Heun", "RK4", or "Savage" methods
//          scale_g = scale factor errors for IMU gyroscopes
//          scale_a = scale factor errors for IMU accelerometers
//          cross_c_g = cross coupling errors for IMU gyroscopes
//          cross_c_a = cross coupling errors for IMU accelerometers
//          bf_g = fixed gyroscope biases
//          bf_a = fixed accelerometer biases
//          b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: Setup Integrator class, Setup imu_correct class
djh_ins::djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                 string integrate_choice, Matrix3d scale_g, Matrix3d scale_a, 
                 Matrix3d cross_c_g, Matrix3d cross_c_a, Vector3d bf_g, Vector3d bf_a, 
                 Vector3d b_a, Vector3d b_g)
        :integ(start_tim, end_tim, imu, state_start, integrate_choice),
         imu_corr(scale_g, scale_a, cross_c_g, cross_c_a, bf_g, bf_a, b_a, b_g)     
{
    state = state_start;
}
        
// Parameterized constructor with WGS 84 Gravity Model and bias-only 
// IMU model.
// Inputs:  start_tim = start time in seconds (presumably a Unix time)
//          end_tim = end time in seconds (presumable a Unix time)
//          imu = IMU measurement matrix of the form
//                      [time_1   accel   gyro]
//                      [time_2   accel   gyro]
//                      [...      ...     ... ]
//                And we assume entries are in chronological order
//                i.e. time_1 < time_2 < ...
//          state_start = starting state prior to integration (10 elements: 
//                        quaternion, position, velocity)
//          integration_choice = a string selecting either "Euler", 
//                               "Heun", "RK4", or "Savage" methods
//          lat = latitude of the starting point (for gravity model)
//          height = height of the starting point above sea level 
//                   (for gravity model)
//          b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: Setup Integrator class, Setup imu_correct class
djh_ins::djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                 string integrate_choice, double lat, double height, Vector3d b_a, 
                 Vector3d b_g)
        :integ(start_tim, end_tim, imu, state_start, integrate_choice, lat, height),
         imu_corr(b_a, b_g)
{
    state = state_start;
}
        
// Parameterized constructor with standard gravity model and bias-only 
// IMU model.
// Inputs:  start_tim = start time in seconds (presumably a Unix time)
//          end_tim = end time in seconds (presumable a Unix time)
//          imu = IMU measurement matrix of the form
//                      [time_1   accel   gyro]
//                      [time_2   accel   gyro]
//                      [...      ...     ... ]
//                And we assume entries are in chronological order
//                i.e. time_1 < time_2 < ...
//          state_start = starting state prior to integration (10 elements: 
//                        quaternion, position, velocity)
//          integration_choice = a string selecting either "Euler", 
//                               "Heun", "RK4", or "Savage" methodss
//          b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: Setup Integrator class, Setup imu_correct class
djh_ins::djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                 string integrate_choice, Vector3d b_a, Vector3d b_g)
        :integ(start_tim, end_tim, imu, state_start, integrate_choice),
         imu_corr(b_a, b_g)
{
    state = state_start;
}
        
//-------------------------------------------------------------------------//
//        NOTE! Put Preintegration constructor into this space!            //
//-------------------------------------------------------------------------//
                
// Update IMU error components
// Inputs:  b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: Online IMU bias parameters updated in private class data
void djh_ins::update_djh_ins(Vector3d b_a, Vector3d b_g)
{
    imu_corr.update_imu_param(b_a,b_g);
}

// Correct IMU data and generate updated state estimate using 
// integration
// Inputs:  start_tim = start time in seconds (presumably a Unix time)
//          end_tim = end time in seconds (presumably a Unix time)
//          imu = IMU measurement matrix of the form
//                      [time_1   accel    gyro]
//                      [time_2   accel    gyro]
//                      [...      ...      ... ]
//                And we assume entries are in chronological order
//                i.e. time_1 < time_2 < ...
//          state_start = starting state prior to integration (10 elements:
//                        quaternion, position, velocity)
// Outputs: state = an updated INS state estimate
void djh_ins::djh_ins_solution(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start)
{
    // Correct the IMU data for biases, scale factors, etc., etc.
    imu_corr.imuCorr(imu);

    // Integrate to obtain INS solution
    integ.Update_Integration(start_tim, end_tim, imu_corr.Corr_imu_mat, state_start);

    // Assign integrated state as solution
    state = integ.state;
}

//-------------------------------------------------------------------------//
// Correct IMU data and generate updated result using preintegration
//-------------------------------------------------------------------------//

/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/ 