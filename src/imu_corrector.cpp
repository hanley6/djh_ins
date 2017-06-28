//
//	IMU Corrector
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	imu_correct.cpp
//	IMU Corrector header file for the Bretl Group 
//  INS package. This corrects IMU measurements for
//  bias.
//
//	Options:
//
//
//	Usage:
//	djh_ins_node.cpp	
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include <iostream>
#include <stdio.h>
#include <string>
#include "imu_corrector.h"
// Include the ROS C++ APIs
#include "ros/ros.h"
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
// The constructs the IMUCorrector object and sets default online computed biases 
// to zero and fixed scale factors, cross-coupling matrices, and biases to values 
// set by an IMU model yaml file.
IMUCorrector::IMUCorrector()
{
    // Initialize Matrices as zero matrix
    Scale_gyro = MatrixXd::Zero(3,3);
    Scale_accel = MatrixXd::Zero(3,3);
    Cross_coup_gyro = MatrixXd::Zero(3,3);
    Cross_coup_accel = MatrixXd::Zero(3,3);

    // Initialize online estimate of bias to zero
    Vector3d biasa = MatrixXd::Zero(3,1);
    Vector3d biasg = MatrixXd::Zero(3,1);

    // Grab parameters from ROS Parameter Server
    ros::param::get("djh_ins_node/S_xg",Scale_gyro(0,0));
    ros::param::get("djh_ins_node/S_yg",Scale_gyro(1,1));
    ros::param::get("djh_ins_node/S_zg",Scale_gyro(2,2));
    ros::param::get("djh_ins_node/S_xa",Scale_accel(0,0));
    ros::param::get("djh_ins_node/S_ya",Scale_accel(1,1));
    ros::param::get("djh_ins_node/S_za",Scale_accel(2,2));
    ros::param::get("djh_ins_node/Bf_xg",Bf_gyro(0));
    ros::param::get("djh_ins_node/Bf_yg",Bf_gyro(1));
    ros::param::get("djh_ins_node/Bf_zg",Bf_gyro(2));
    ros::param::get("djh_ins_node/Bf_xa",Bf_accel(0));
    ros::param::get("djh_ins_node/Bf_ya",Bf_accel(1));
    ros::param::get("djh_ins_node/Bf_za",Bf_accel(2));
    ros::param::get("djh_ins_node/M_xyg",Cross_coup_gyro(0,1));
    ros::param::get("djh_ins_node/M_xzg",Cross_coup_gyro(0,2));
    ros::param::get("djh_ins_node/M_yxg",Cross_coup_gyro(1,0));
    ros::param::get("djh_ins_node/M_yzg",Cross_coup_gyro(1,2));
    ros::param::get("djh_ins_node/M_zxg",Cross_coup_gyro(2,0));
    ros::param::get("djh_ins_node/M_zyg",Cross_coup_gyro(2,1));
    ros::param::get("djh_ins_node/M_xya",Cross_coup_accel(0,1));
    ros::param::get("djh_ins_node/M_xza",Cross_coup_accel(0,2));
    ros::param::get("djh_ins_node/M_yxa",Cross_coup_accel(1,0));
    ros::param::get("djh_ins_node/M_yza",Cross_coup_accel(1,2));
    ros::param::get("djh_ins_node/M_zxa",Cross_coup_accel(2,0));
    ros::param::get("djh_ins_node/M_zya",Cross_coup_accel(2,1));
}

// imu_correct accepts an aggregated n-by-7 IMU matrix and corrects the IMU 
// measurements for fixed scale, bias, and cross-coupling factors. It then 
// adds a bias estimated online.
// Inputs:  agg_mat = Aggregated IMU matrix
// Outputs: agg_corrected = the aggregated IMU matrix adjusted to eliminate 
//                          IMU errors
void IMUCorrector::imu_correct(MatrixXd agg_mat)
{
    // First, create two temporary matrices
    MatrixXd tmpg;
    MatrixXd tmpa;

    // Manipulate incoming aggregated matrix
    MatrixXd accel_mat = agg_mat.middleCols(1, 3);
    MatrixXd gyro_mat = agg_mat.middleCols(4,3);
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
    agg_corrected = agg_mat;
}

// biasCallback is a callback function a ROS subscriber that receives updated 
// IMU bias estimates. These updated bias estimates are saved as the class' 
// private variables which are in turn used in other functions in the class.
// Inputs:  msg = the bias vector from a ROS topic. First three values are the 
//                accelerometer bias estimates. Second three values are the 
//                gyroscope bias estimates. 
// Outputs: biasa = private class Eigen vector for online bias accelerometer 
//                  estimates
//          biasg = private class Eigen vector for online bias gyroscope 
//                  estimates
void IMUCorrector::biasCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Accelerometer bias vector
    biasa[0] = msg->data[0]; // x
    biasa[1] = msg->data[1]; // y
    biasa[2] = msg->data[2]; // z

    // Gyroscope bias vector
    biasg[0] = msg->data[3]; // x
    biasg[1] = msg->data[4]; // y
    biasg[2] = msg->data[5]; // z
}
/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/