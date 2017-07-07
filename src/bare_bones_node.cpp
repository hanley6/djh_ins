//
//	A bare bones node to show you how to use the INS node
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	bare_bones_node.cpp
//	The entry point to the bare bones node which shows you how to use the INS 
//  node.
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
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "aggregator.h"
// Include the ROS C++ APIs
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
// Create a vector of vectors to save aggregated IMU data 
std::vector<std::vector <double>> data_agg_save;
double start_time;
bool imageFunctionCalled;
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
using namespace cv;
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
// This will get called when a new image has arrived on the appropriate topic
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Define the ROS publisher and the associated message
    static ros::NodeHandle get_ins;
    static ros::NodeHandle ns_bias;
    static ros::Publisher comp_sol = get_ins.advertise<djh_ins::compute_ins>("comp_sol",1000);
    static ros::Publisher bias_sub = ns_bias.advertise<std_msgs::Float64MultiArray>("bias_est",1000);
    static djh_ins::compute_ins msg_out;
    static std_msgs::Float64MultiArray bias_msg;

    // Set Bias message to publish
    bias_msg.data.clear();
    bias_msg.data.push_back(0.0); // Accelerometer x
    bias_msg.data.push_back(0.0); // Accelerometer y
    bias_msg.data.push_back(0.0); // Accelerometer z
    bias_msg.data.push_back(0.0); // Gyroscope x
    bias_msg.data.push_back(0.0); // Gyroscope y
    bias_msg.data.push_back(0.0); // Gyroscope z

    // Throw flag that an INS State estimate is desired
    msg_out.stop_agg = true;
    // Fill Accelerometer vector and gyroscope vector
    msg_out.time_desired = msg->header.stamp.toNSec()/1000000000.0;
    // We are going to publish this message now
    msg_out.header.stamp = ros::Time::now();
    // We will also take this time as the starting time
    if (imageFunctionCalled == false)
    {
        imageFunctionCalled = true;
        start_time = msg->header.stamp.toNSec()/1000000000.0;
        msg_out.start_time = start_time;
    }
    else
    {
        msg_out.start_time = start_time;
    }
    // Set initial state vector to integrate
    msg_out.start_pose.orientation.x = 0.0;
    msg_out.start_pose.orientation.y = 0.0;
    msg_out.start_pose.orientation.z = 0.0;
    msg_out.start_pose.orientation.w = 1.0;
    msg_out.start_pose.position.x = 0.0;
    msg_out.start_pose.position.y = 0.0;
    msg_out.start_pose.position.z = 0.0;
    msg_out.start_vel.x = 0.0;
    msg_out.start_vel.y = 0.0;
    msg_out.start_vel.z = 0.0;
    // Publish the messages now
    comp_sol.publish(msg_out);
    bias_sub.publish(bias_msg);

    // Gets image from CV bridge and places it in "im1"
    Mat im1 = cv_bridge::toCvShare(msg,"bgr8")->image;

    // Displays im1 with the detected keypoints
	cv::imshow("view", im1);
    cv::waitKey(30);
}

// This Callback saves the aggregated IMU data
void saveAggCallback(const djh_ins::compute_ins::ConstPtr& msg)
{
    data_agg_save.push_back(msg->aggregatedMatrix.data);

    cout << "-----------------------------------------------\n";
    cout << "Saved vector now has: " << data_agg_save.size() << " matrices." << endl;
    cout << "-----------------------------------------------\n";
}

// This callback receives the INS result and prints it
void saveINSCallback(const djh_ins::compute_ins::ConstPtr& msg)
{
    // quat, position, velocity
    VectorXd state(10);
    state(0) = msg->start_pose.orientation.x;
    state(1) = msg->start_pose.orientation.y;
    state(2) = msg->start_pose.orientation.z;
    state(3) = msg->start_pose.orientation.w;
    state(4) = msg->start_pose.position.x;
    state(5) = msg->start_pose.position.y;
    state(6) = msg->start_pose.position.z;
    state(7) = msg->start_vel.x;
    state(8) = msg->start_vel.y;
    state(9) = msg->start_vel.z;

    start_time = msg->time_desired;
    cout << "INS Desired time: " << start_time << endl;
    cout << "-----------------------------------------------\n";
    cout << "INS State is now: \n" << state << endl;
    cout << "-----------------------------------------------\n";
}
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Helpers ----------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*----------------------------------- Main ------------------------------------*/
/*-----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	// Annouce this program to the ROS master as a "node" called "djh_ins_node"
	ros::init(argc, argv, "bare_bones_node");

	// Start the node resource managers (communication, time, etc)
	ros::start();

    // Initialize node handles
    ros::NodeHandle sub_camera;
    ros::NodeHandle agg_receiver;
    ros::NodeHandle ins_receiver;

    // Subscribe to Camera data so we can aggragated 
    image_transport::ImageTransport it(sub_camera);
    image_transport::Subscriber sub_im = it.subscribe("cam0/image_raw", 1, imageCallback);

    // Save aggregated IMU data as it is produced
    ros::Subscriber ins_sub = agg_receiver.subscribe("/aggregate_imu",1000, saveAggCallback);
    ros::Subscriber insreceiver_sub = ins_receiver.subscribe("/ins_result",1000, saveINSCallback);

    // Initialize image function called variable to false, since 
    // it hasn't been called yet.
    imageFunctionCalled = false;

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