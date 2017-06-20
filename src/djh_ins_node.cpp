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
// This will get called when a new message has arrived on the appropriate topic.
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Initialize the measurement vectors
    Vector3d a_sf_i;
    Vector3d w_i;
    Vector3d biasg;
    Vector3d biasa;

    // Fill Accelerometer vector and gyroscope vector
    a_sf_i << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    w_i << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    // State biases (to be sent from another ROS node)
    biasg << 0.0, 0.0, 0.0;
    biasa << 0.0, 0.0, 0.0; 

    // Correct the IMU measurements
    imu_correct(a_sf_i,w_i,biasa,biasg);

    // Print Results
    cout << endl << a_sf_i << endl;

    // INS output is a timestamped position and orientation
    geometry_msgs::PoseStamped msg_out;
    // Time stamp of the INS position should correspond to the IMU reading
    msg_out.header.stamp = msg->header.stamp;

    // Print out the INS Result
    ROS_INFO("-------------------------------------");
    cout << msg_out.header.stamp << endl;
    ROS_INFO("-------------------------------------");
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
    //ros::NodeHandle np;

    // Subscribe to IMU data
    ros::Subscriber sub = ns.subscribe("/imu0",1000, imuCallback);

    // Setup publisher of INS results
    ros::Publisher pub = ns.advertise<std_msgs::String>("chatter",1000);

	// Time initialization
	t = clock();

    // Compute the INS solution
    /* 
        INSERT IMPORTANT STUFF HERE!!!
    */
    /*---------- Temporary ----------*/
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world";
    msg.data = ss.str();
    /*-------- End Temporary --------*/

    // Publish the INS Results
    pub.publish(msg);

	t = clock() - t;
	printf("\n It took %f seconds to do ____ \n",((float)t)/CLOCKS_PER_SEC);

	// Broadcast a simple log message
	ROS_INFO_STREAM("Hello, world!");

	//Process ROS callbacks until receiving a SIGINT (ctrl-c)
	ros::spin();

	// Stop the node's resources
	ros::shutdown();

	return 0;
}
/*-----------------------------------------------------------------------------*/
/*--------------------------------- End Main ----------------------------------*/
/*-----------------------------------------------------------------------------*/