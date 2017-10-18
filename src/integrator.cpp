//
//	Integrates inertial navigation ODEs
//	David Hanley
//	
//	integrator.cpp
//
//
//	INS integration implementation file for the Bretl Group 
//  INS package. This contains a class that allows a 
//  user to numerically integrate a set of IMU measurements 
//  to estimate an INS state.
//
//	Usage:
//  ins_ode.cpp
//  djh_ins.cpp
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include <iostream>
#include "integrator.h"
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
// The constructor assigns a default state vector estimate, choice of 
// integration, start and end times, and IMU measurement matrix
Integrator::Integrator()
{
    // Set Initial State
    state.setZero(10);
    state(3) = 1.0;

    // State default start and end time and IMU measurements
    start_time = 1503618731094;
    end_time = 1503618754408;
    imu_mat.setZero(1,7);

    // State default integrator to use
    integrat_flag = "Euler";
}

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
Integrator::Integrator(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start, string integrate_choice, double lat, double height)
                      :diff_eq(lat,height)
{
    // Set Initial State
    state = state_start;
        
    // Initialize start and end time and IMU mesurements
    start_time = start_tim;
    end_time = end_tim;
    imu_mat = imu;

    // Set integrator to use
    integrat_flag = integrate_choice;
}

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
//          imu_mat = IMU measurment matrix
//          integrat_flag = flag defining the choice of integrator to use
Integrator::Integrator(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start, string integrate_choice)
{
    // Set Initial State
    state = state_start;
        
    // Initialize start and end time and IMU mesurements
    start_time = start_tim;
    end_time = end_tim;
    imu_mat = imu;

    // Set integrator to use
    integrat_flag = integrate_choice;
}

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
void Integrator::Update_Integration(double start_tim, double end_tim, MatrixXd imu, Vector10d state_start)
{
    // Reassign Class Data
    start_time = start_tim;
    end_time = end_tim;
    imu_mat = imu;
    state = state_start;

    // Choose Correct Method of Integration
    if (integrat_flag == "Euler")
    {
       Euler_int();
    }
    else if (integrat_flag == "Heun")
    {
        Heun_int();
    }
    else if (integrat_flag == "RK4")
    {
        RK4_int();
    } 
    else if (integrat_flag == "Savage")
    {
        Savage_int();
    }
    else
    {
        cerr << "\033[1;31m Error: invalid integration flag choice!\n";
    }
}

// Euler_int performs Euler integration
// Inputs:  Uses only class parameters
// Outputs: state = 10 element state estimate of the INS
void Integrator::Euler_int()
{
    // Start time at the class's start time
    double t  = start_time;
    double dt;
    VectorXd mat_meas;
    // Eigen has some rules (I think) about making 
    // statements like x = x + something or nothing
    Vector10d temp_state; 

    while (t < end_time)
    {
        // Compute dt
        if (find_next_time(t) == 1 || find_next_time(t) > end_time)
        {
            dt = end_time - t;
        }
        else
        {
            dt = find_next_time(t) - t;
        }

        // Compute appropriate IMU measurement
        mat_meas = imu_mat.row(find_imu_meas(t));

        // Compute dx/dt
        diff_eq.ode_compute(state, mat_meas.tail(6));

        // Compute next state
        temp_state = state;
        state = diff_eq.dstate*dt + temp_state;

        // Compute next time
        t = t + dt;
    }
}

// Heun_int performs Heun's integration
// Inputs:  Uses only class parameters
// Outputs: state = 10 element state estimate of the INS
void Integrator::Heun_int()
{
    // Start time at the class's start time
    double t  = start_time;
    double dt;
    VectorXd mat_meas;
    // Eigen has some rules (I think) about making 
    // statements like x = x + something or nothing
    Vector10d temp_state;
    // Results of ODE interval calculations
    Vector10d k_1;
    Vector10d k_2;

    while (t < end_time)
    {
        // Compute dt
        if (find_next_time(t) == 1 || find_next_time(t) > end_time)
        {
            dt = end_time - t;
        }
        else
        {
            dt = find_next_time(t) - t;
        }

        // Compute appropriate IMU measurement
        mat_meas = imu_mat.row(find_imu_meas(t));

        // Compute dx/dt
        diff_eq.ode_compute(state, mat_meas.tail(6));
        k_1 = diff_eq.dstate;
        diff_eq.ode_compute(state + dt*k_1, mat_meas.tail(6));
        k_2 = diff_eq.dstate;

        // Compute next state
        temp_state = state;
        state = temp_state + 0.5*dt*(k_1 + k_2);

        // Compute next time
        t = t + dt;
    }
}

// RK4_int performs Runge-Kutta integration
// Inputs:  Uses only class parameters
// Outputs: state = 10 element state estimate of the INS
void Integrator::RK4_int()
{
    // Start time at the class's start time
    double t  = start_time;
    double dt;
    VectorXd mat_meas;
    // Eigen has some rules (I think) about making 
    // statements like x = x + something or nothing
    Vector10d temp_state;
    // Results of ODE interval calculations
    Vector10d k_1;
    Vector10d k_2;
    Vector10d k_3;
    Vector10d k_4;

    while (t < end_time)
    {
        // Compute dt
        if (find_next_time(t) == 1 || find_next_time(t) > end_time)
        {
            dt = end_time - t;
        }
        else
        {
            dt = find_next_time(t) - t;
        }

        // Compute appropriate IMU measurement
        mat_meas = imu_mat.row(find_imu_meas(t));

        // Compute dx/dt
        diff_eq.ode_compute(state, mat_meas.tail(6));
        k_1 = diff_eq.dstate;
        diff_eq.ode_compute(state + 0.5*dt*k_1, mat_meas.tail(6));
        k_2 = diff_eq.dstate;
        diff_eq.ode_compute(state + 0.5*dt*k_2, mat_meas.tail(6));
        k_3 = diff_eq.dstate;
        diff_eq.ode_compute(state + dt*k_3, mat_meas.tail(6));
        k_4 = diff_eq.dstate;

        // Compute next state
        temp_state = state;
        state = temp_state + (dt/6.0)*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4);

        // Compute next time
        t = t + dt;
    }
}

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
void Integrator::Savage_int()
{
    cerr << "\033[1;31m Error: Savage Integration not set up yet!\n";
}

// Find the next time in the IMU measurement matrix
// Inputs:  t = reference time stamp with which to find the next 
//              time in seconds (presumably a Unix time)
// Outputs: the next time in seconds (presumably a Unix time) or
//          1.0 which means that we ran out of IMU entr
double Integrator::find_next_time(double t)
{
    // Initialize our counter and the output to 0
    double next_time = 0.0;
    int count = 0;

    // Iterate through the IMU measurement matrix until a 
    // time is found greater than the current time or we 
    // run out of entries
    while (next_time == 0.0)
    {
        if (imu_mat(count,0) > t && count < imu_mat.rows())
        {
            next_time = imu_mat(count,0);
        }
        else if (count >= imu_mat.rows())
        {
            next_time = 1.0;
        }
        else
        {
            count++;
        }
    }

    // Return the result of the function
    return next_time;
}

// Find the next IMU measurement to use in integration
// Inputs:  t = reference time stamp with which to find the next 
//              IMU measurement entry
// Outputs: the next IMU measurement entry to use as an Eigen 
//          row number
int Integrator::find_imu_meas(double t)
{
    // Initialize counter to size of measurement matrix
    int count = imu_mat.rows() - 1;
    int flag = 0;

    // Iterate backwards through the IMU measurement matrix
    // until a time is found less than or equal to the 
    // current time or we run out of entries
    while (flag == 0)
    {
        if(imu_mat(count,0) <= t || count == 0)
        {
            flag = 1;
        }
        else
        {
            count--;
        }
    }

    // Return the result of the function
    return count;
}
/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/ 