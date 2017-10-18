//
//	Corrects IMU measurements for various biases, scale 
//  factors, etc.
//	David Hanley
//	
//	imu_correct.h
//
//
//	IMU Correction implementation file for the Bretl Group 
//  INS package. This contains a class that allows a 
//  user correct IMU measurements for various biases, scale 
//  factors, etc.
//
//	Usage:
//  
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include <iostream>
#include "imu_correct.h"
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
// Default constructor sets all IMU error parameters to zero
imu_correct::imu_correct()
{
    // Initialize Matrices as zero matrix
    Scale_gyro = MatrixXd::Zero(3,3);
    Scale_accel = MatrixXd::Zero(3,3);
    Cross_coup_gyro = MatrixXd::Zero(3,3);
    Cross_coup_accel = MatrixXd::Zero(3,3);

    // Initialize Fixed Bias Vectors as a zero vector
    Bf_gyro = MatrixXd::Zero(3,1);
    Bf_accel = MatrixXd::Zero(3,1);

    // Initialize online estimate of bias to zero
    biasa = MatrixXd::Zero(3,1);
    biasg = MatrixXd::Zero(3,1);
}

// Parameterized constructor initializes all IMU error parameters
// based on inputs
// Inputs:  scale_g = scale factor errors for IMU gyroscopes
//          scale_a = scale factor errors for IMU accelerometers
//          cross_c_g = cross coupling errors for IMU gyroscopes
//          cross_c_a = cross coupling errors for IMU accelerometers
//          bf_g = fixed gyroscope biases
//          bf_a = fixed accelerometer biases
//          b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: All IMU error parameters set as private class data
imu_correct::imu_correct(Matrix3d scale_g, Matrix3d scale_a, Matrix3d cross_c_g, Matrix3d cross_c_a, Vector3d bf_g, Vector3d bf_a, Vector3d b_a, Vector3d b_g)
{
    // Initialize matrices based on inputs
    Scale_gyro = scale_g;
    Scale_accel = scale_a;
    Cross_coup_gyro = cross_c_g;
    Cross_coup_accel = cross_c_a;

    // Initialize Fixed Bias Vectors Based on Inputs
    Bf_gyro = bf_g;
    Bf_accel = bf_a;

    // Initialize online estimate of bias based on inputs
    biasa = b_a;
    biasg = b_g;
}

// Parameterized constructor initializes all IMU error parameters based on
// inputs
// Inputs:  b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: All IMU error parameters set as private class data
imu_correct::imu_correct(Vector3d b_a, Vector3d b_g)
{
    // Initialize Matrices as zero matrix
    Scale_gyro = MatrixXd::Zero(3,3);
    Scale_accel = MatrixXd::Zero(3,3);
    Cross_coup_gyro = MatrixXd::Zero(3,3);
    Cross_coup_accel = MatrixXd::Zero(3,3);

    // Initialize Fixed Bias Vectors as a zero vector
    Bf_gyro = MatrixXd::Zero(3,1);
    Bf_accel = MatrixXd::Zero(3,1);

    // Initialize online estimate of bias to zero
    biasa = b_a;
    biasg = b_g;
}

// Update IMU error components
// Inputs:  b_a = accelerometer biases estimated online
//          b_g = gyroscope biases estimated online
// Outputs: Online IMU bias parameters updated in private class data
void imu_correct::update_imu_param(Vector3d b_a, Vector3d b_g)
{
    biasa = b_a;
    biasg = b_g;
}

// imuCorr accepts an aggregated n-by-7 IMU matrix and corrects the IMU 
// measurements for fixed scale, bias, and cross-coupling factors. It then 
// adds a bias estimated online.
// Inputs:  agg_mat = Aggregated IMU matrix of the form
//                      [time_1   accel    gyro]
//                      [time_2   accel    gyro]
//                      [...      ...      ... ]
// Outputs: Corr_imu_mat = the aggregated IMU matrix adjusted to eliminate 
//                         IMU errors
void imu_correct::imuCorr(MatrixXd agg_mat)
{
    // Manipulate incoming aggregated matrix
    accel_mat = agg_mat.middleCols(1, 3);
    gyro_mat = agg_mat.middleCols(4,3);
    accel_mat.transposeInPlace();
    gyro_mat.transposeInPlace();

    // Correct Fixed IMU Errors
    tmpg = (MatrixXd::Identity(3,3)+Scale_gyro)*gyro_mat + Cross_coup_gyro*gyro_mat;
    tmpg.colwise() += Bf_gyro;

    tmpa = (MatrixXd::Identity(3,3)+Scale_accel)*accel_mat + Cross_coup_accel*accel_mat;
    tmpa.colwise() += Bf_accel;

    // Correct IMU Errors Estimated Online
    tmpa.colwise() += biasa;
    tmpg.colwise() += biasg;

    // Fix the aggregated matrix
    gyro_mat = tmpg;
    accel_mat = tmpa;
    accel_mat.transposeInPlace();
    gyro_mat.transposeInPlace();
    agg_mat.middleCols(1, 3) = accel_mat;
    agg_mat.middleCols(4, 3) = gyro_mat;
    Corr_imu_mat = agg_mat;
}
/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/ 