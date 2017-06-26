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
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "imu_corrector.h"
#include "aggregator.h"
// Include the ROS C++ APIs
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
// Init timer
clock_t t;
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
using namespace Eigen;
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
    static ros::Publisher comp_sol = get_ins.advertise<djh_ins::compute_ins>("comp_sol",1000);
    static djh_ins::compute_ins msg_out;

    // Throw flag that an INS State estimate is desired
    msg_out.stop_agg = true;
    // Fill Accelerometer vector and gyroscope vector
    msg_out.time_desired = msg->header.stamp.toNSec()/1000000000.0;
    // We are going to publish this message now
    msg_out.header.stamp = ros::Time::now();
    // Publish the message now
    comp_sol.publish(msg_out);

    // Gets image from CV bridge and places it in "im1"
    Mat im1 = cv_bridge::toCvShare(msg,"bgr8")->image;

    // Displays im1 with the detected keypoints
	//cv::imshow("view", im1);
    //cv::waitKey(30);
}

// This will get called when a new aggregated IMU packet has arrived 
// on the appropriate topic.
void insCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    t = clock();
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
    t = clock() - t;
    printf("\n It took %f seconds to receive aggregated data\n",((float)t)/CLOCKS_PER_SEC);

    cout << "-----------------------------------------------\n";
    cout << agg_mat << endl;
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
	// Init timer
	clock_t t;

	// Annouce this program to the ROS master as a "node" called "djh_ins_node"
	ros::init(argc, argv, "djh_ins_node");

	// Start the node resource managers (communication, time, etc)
	ros::start();

    // Initialize node handles
    ros::NodeHandle ns;
    ros::NodeHandle sub_camera;
    ros::NodeHandle agg_receiver;
    ros::NodeHandle np;

    IMU_Aggregator imu_agg;

    // Subscribe to IMU data
    ros::Subscriber sub = ns.subscribe("/imu0",1000, &IMU_Aggregator::imuCallback, &imu_agg);
    ros::Subscriber ins_sub = agg_receiver.subscribe("/aggregate_imu",1000, insCallback);
    image_transport::ImageTransport it(sub_camera);
    image_transport::Subscriber sub_im = it.subscribe("cam0/image_raw", 1, imageCallback);

    // Setup publisher of INS results
    ros::Publisher pub = np.advertise<std_msgs::String>("chatter",1000);

	// Time initialization
	t = clock();

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