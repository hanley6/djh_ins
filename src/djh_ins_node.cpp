//
//	Inertial navigation system node
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	djh_ins_node.cpp
//	Entry point to the inertial navigation system node.
//
//	Options:
//
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
#include <stdio.h>
#include <time.h>
#include <Eigen/Core>
#include "imu_corrector.h"
#include "aggregator.h"
// Include the ROS C++ APIs
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
using namespace Eigen;
/*--------------- End Namespaces ---------------*/

/*------------------ Pragmas -------------------*/
/*---------------- End Pragmas -----------------*/

/*------------- Function Prototypes ------------*/
/*----------- End Function Prototypes ----------*/
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Preamble ---------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Helpers ------------------------------------*/
/*-----------------------------------------------------------------------------*/
// This will get called when a new aggregated IMU packet has arrived 
// on the appropriate topic.
void insCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    /*------- Receive and reform aggregated IMU Matrix -------*/
    // Define a temporary std vector for the aggregated IMU 
    // message data
    vector<double> vec = msg->data;
    // Compute the number of rows in the aggregated matrix
    int sz = vec.size()/7;
    // Create pointer and store memory address of first vector element
    double* ptr = &vec[0];
    // Note: MatrixX7d is defined in aggregator.h
    Map<MatrixX7d>agg_mat(ptr,sz,7);
    /*----- End receive and reform aggregated IMU Matrix -----*/

    /*----------------- Correct the IMU Data -----------------*/
    // Setup the subscriber to bias estimator and create an IMU 
    // correction object once
    static IMUCorrector imu_correct;
    static ros::NodeHandle ns_bias;
    static ros::Subscriber bias_sub = ns_bias.subscribe("/bias_est",1000, &IMUCorrector::biasCallback, &imu_correct);

    // Correct IMU Measurements
    imu_correct.imu_correct(agg_mat);
    agg_mat = imu_correct.agg_corrected;
    /*--------------- End Correct the IMU Data ---------------*/

    /*----- INS Integration -----*/
    // ADD STUFF HERE!!!
    /*--- End INS Integration ---*/
}
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Helpers ----------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*----------------------------------- Main ------------------------------------*/
/*-----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	// Init timer
	clock_t t;

	// Annouce this program to the ROS master as a "node" called "djh_ins_node"
	ros::init(argc, argv, "djh_ins_node");

	// Start the node resource managers (communication, time, etc)
	ros::start();

    // Initialize node handles
    ros::NodeHandle ns;
    ros::NodeHandle agg_receiver;

    // Create an IMU Aggregator Object
    IMU_Aggregator imu_agg;

    // Subscribe to IMU data
    ros::Subscriber sub = ns.subscribe("/imu0",1000, &IMU_Aggregator::imuCallback, &imu_agg);

    // Subscribe to Aggregated IMU data and run INS Code
    ros::Subscriber ins_sub = agg_receiver.subscribe("/aggregate_imu",1000, insCallback);

	//Process ROS callbacks until receiving a SIGINT (ctrl-c)
	ros::spin();

	// Stop the node's resources
	ros::shutdown();

	return 0;
}
/*-----------------------------------------------------------------------------*/
/*--------------------------------- End Main ----------------------------------*/
/*-----------------------------------------------------------------------------*/