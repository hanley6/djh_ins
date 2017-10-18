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
//	DJH INS header file for the Bretl Group INS package. 
//  This contains a class that is called and used by all codes 
//  using the INS package (except those codes also using ROS).
//
//	Usage:
//  integrator.cpp
//  preintegration.cpp
//  imu_correct.cpp
//

/*---------------- Include Guard ---------------*/
#ifndef djh_ins_H
#define djh_ins_H
/*-------------- End Include Guard -------------*/

/*------------------ Includes ------------------*/
#include <Eigen/Core>
#include "integrator.h"
#include "imu_correct.h"
#include "preintegration.h"
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
using namespace std;
/*--------------- End Namespaces ---------------*/

/*------------------ Globals -------------------*/
/*---------------- End Globals -----------------*/

/*------------------- Classes ------------------*/
class djh_ins
{
    public:
        // Default constructor initializes all integrator, imu correction, 
        // and preintegration classes to their default cases.
        djh_ins();

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
        djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                string integrate_choice, double lat, double height, Matrix3d scale_g, 
                Matrix3d scale_a, Matrix3d cross_c_g, Matrix3d cross_c_a, Vector3d bf_g, 
                Vector3d bf_a, Vector3d b_a, Vector3d b_g);


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
        djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                string integrate_choice, Matrix3d scale_g, Matrix3d scale_a, 
                Matrix3d cross_c_g, Matrix3d cross_c_a, Vector3d bf_g, Vector3d bf_a, 
                Vector3d b_a, Vector3d b_g);

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
        djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                string integrate_choice, double lat, double height, Vector3d b_a, 
                Vector3d b_g);

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
        djh_ins(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start,
                string integrate_choice, Vector3d b_a, Vector3d b_g);

        //-------------------------------------------------------------------------//
        //        NOTE! Put Preintegration constructor into this space!            //
        //-------------------------------------------------------------------------//
        
        // Update IMU error components
        // Inputs:  b_a = accelerometer biases estimated online
        //          b_g = gyroscope biases estimated online
        // Outputs: Online IMU bias parameters updated in private class data
        void update_djh_ins(Vector3d b_a, Vector3d b_g);

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
        void djh_ins_solution(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start);

        //-------------------------------------------------------------------------//
        // Correct IMU data and generate updated result using preintegration
        //-------------------------------------------------------------------------//

        // This is the 10 dimensional INS State Estimate
        // [quaternion (JPL Format), position, velocity]
        Vector10d state;
    private:
        // The Integrator Class used to compute INS state estimates
        Integrator integ;

        // The IMU Correction Class used to correct IMU measurements 
        // according to a deterministic model
        imu_correct imu_corr;

        // The Preintegration Class used to compute preintegration results
        preintegrate Pinte;
};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/