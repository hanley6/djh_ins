//
//	Integrates inertial navigation ODEs
//	David Hanley
//	
//	integrator.h
//
//
//	INS integration header file for the Bretl Group 
//  INS package. This contains a class that allows a 
//  user to numerically integrate a set of IMU measurements 
//  to estimate an INS state.
//
//	Usage:
//  ins_ode.cpp
//  djh_ins.cpp
//

/*---------------- Include Guard ---------------*/
#ifndef integrator_H
#define integrator_H
/*-------------- End Include Guard -------------*/

/*------------------ Includes ------------------*/
#include <Eigen/Core>
#include <string>
#include "ins_ode.h"
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
using namespace std;
/*--------------- End Namespaces ---------------*/

/*------------------ Globals -------------------*/
/*---------------- End Globals -----------------*/

/*------------------- Classes ------------------*/
class Integrator
{
    public:
        // The constructor assigns a default state vector estimate, choice of 
        // integration, start and end times, and IMU measurement matrix
        Integrator();

        // Parameterized constructor initializes the state, start and end times, IMU 
        // measurement matrix, and choice of integration using inputs and a WGS 84 
        // model of gravity
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
        // Outputs: state = 10 element state estimate of the INS
        //          start_time = start time of integration
        //          end_time = end time of integration
        //          imu_mat = IMU measurment matrix
        //          integrat_flag = flag defining the choice of integrator to use
        Integrator(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start, string integrate_choice, double lat, double height);

        // Parameterized constructor initializes the state, start and end times, IMU 
        // measurement matrix, and choice of integration using inputs and a standard 
        // model of gravity
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
        // Outputs: state = 10 element state estimate of the INS
        //          start_time = start time of integration
        //          end_time = end time of integration
        //          imu_mat = IMU measurement matrix
        //          integrat_flag = flag defining the choice of integrator to use
        Integrator(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start, string integrate_choice);

        // Update_Integration updates the INS solution given a start and end time, 
        // IMU measurements, and a starting state estimate.
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
        // Outputs: state = 10 element state estimate of the INS
        //          start_time = sets start time as hidden variable start time
        //          end_time = sets end time as hidden variable end time
        //          imu_mat = IMU measurement matrix
        void Update_Integration(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start);

        // This is the 10 dimensional INS State Estimate
        // [quaternion (JPL Format), position, velocity]
        Vector10d state;
    private:
        // Starting time for INS integration
        double start_time;
        // Ending time for INS integration
        double end_time;
        // Matrix of IMU measurements
        // This is assumed to be in the format
        // [time_1   accel   gyro]
        // [time_2   accel   gyro]
        // [...      ...     ... ]
        // And we assume entries are in chronological order
        // i.e. time_1 < time_2 < ...
        MatrixXd imu_mat;

        // Flag sets method of integration. Available options are:
        //      Euler
        //      Heun
        //      RK4
        //      Savage (not yet complete)
        string integrat_flag;

        // The diff_eq class sets up the equations of motion for use 
        // in the integration algorithms.
        ins_ode diff_eq;

        // Euler_int performs Euler integration
        // Inputs:  Uses only class parameters
        // Outputs: state = 10 element state estimate of the INS
        void Euler_int();

        // Heun_int performs Heun's integration
        // Inputs:  Uses only class parameters
        // Outputs: state = 10 element state estimate of the INS
        void Heun_int();        

        // RK4_int performs Runge-Kutta integration
        // Inputs:  Uses only class parameters
        // Outputs: state = 10 element state estimate of the INS
        void RK4_int();
              
        // Savage_int performs the INS integration presented by Savage 
        // in:
        // P. G. Savage. "Strapdown Inertial Navigation Integration 
        // Algorithm Design Part 1: Attitude Algorithms." Journal of 
        // Guidance, Control, and Dynamics. Vol 21, No 1. Jan. 1998.
        // And
        // P. G. Savage. "Strapdown Inertial Navigation Integration 
        // Algorithm Design Part 2: Velocity and Position Algorithms."
        // Journal of Guidance, Control, and Dynamics. Vol 21, No 2. 
        // March 1998.
        // Inputs:  Uses only class parameters
        // Outputs: state = 10 element state estimate of the INS 
        void Savage_int();

        // Find the next time in the IMU measurement matrix
        // Inputs:  t = reference time stamp with which to find the next 
        //              time in seconds (presumably a Unix time)
        // Outputs: the next time in seconds (presumably a Unix time) or
        //          1.0 which means that we ran out of IMU entries
        double find_next_time(double t);

        // Find the next IMU measurement to use in integration
        // Inputs:  t = reference time stamp with which to find the next 
        //              IMU measurement entry
        // Outputs: the next IMU measurement entry to use as an Eigen 
        //          row number
        int find_imu_meas(double t);
};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/
