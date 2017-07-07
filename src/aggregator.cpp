//
//	IMU Aggregator
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	aggregator.cpp
//	IMU Aggregator header file for the Bretl Group 
//  INS package. This aggregates IMU measurements for
//  use by either a navigation function or the INS.
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
#include <algorithm>
#include <vector>
#include "aggregator.h"
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
// Default IMU_Aggregator class constructor
// Inputs:  None.
// Outputs: Default stop_agg and time_desired assignments.
IMU_Aggregator::IMU_Aggregator()
{
    stop_agg = false;
    time_desired = 0.0;
    start_time = 0.0;
    state(0) = 0.0;
    state(1) = 0.0;
    state(2) = 0.0; 
    state(3) = 1.0;
    state(4) = 0.0;
    state(5) = 0.0;
    state(6) = 0.0;
    state(7) = 0.0;
    state(8) = 0.0;
    state(9) = 0.0;
}

// Parameterize IMU_Aggregator class constructor
// Inputs:  stop_agg_change = input true (stop aggregating) 
//                            or false (continue aggregating) 
//          time_desired_change = input double time (in sec) to 
//                                compute INS solution
//          state_change = input INS state
//          start_time_change = input start time for INS state estimation
// Outputs: stop_agg and time_desired assignments.
IMU_Aggregator::IMU_Aggregator(bool stop_agg_change,double time_desired_change,VectorXd state_change,double start_time_change)
{
    stop_agg = stop_agg_change;
    time_desired = time_desired_change;
    state = state_change;
    start_time = start_time_change;
}

// This callback function receives a compute_ins message and assigns 
// the result to stop_agg and time_desired
// Inputs:  compute_ins ROS message
// Outputs: stop_agg and time_desired assignments.
void IMU_Aggregator::comp_ins_solCallback(const djh_ins::compute_ins::ConstPtr& msg)
{
    stop_agg = msg->stop_agg;
    time_desired = msg->time_desired;
    start_time = msg->start_time;
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
}


// aggregate outputs an aggregated matrix of IMU data collected over time.
// Inputs:  Standard ROS IMU message
//          stop_agg = input message sent to INS 
//                     when a new state estimate 
//                     is requested
// Outputs: Matrix of IMU Data in the form ...
//          [timestamp1, accel_x1, accel_y1, accel_z1, gyro_x1, gyro_y1, gyro_z1]
//          [timestamp2, accel_x2, accel_y2, accel_z2, gyro_x2, gyro_y2, gyro_z2]
//          [...       , ...     , ...     , ...     , ...    , ...    , ...    ]
MatrixXd IMU_Aggregator::Aggregate(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Initialize the measurement vectors
    Vector3d a_sf_i;
    Vector3d w_i;
    MatrixXd tmp(1,7);
    MatrixXd Output;
    double timestamp;

    // Fill Accelerometer vector and gyroscope vector
    timestamp = msg->header.stamp.toNSec()/1000000000.0;
    a_sf_i << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    w_i << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    tmp << timestamp, a_sf_i(0), a_sf_i(1), a_sf_i(2), w_i(0), w_i(1), w_i(2);

    // Collect data in aggregator
    AggregatedOutput.conservativeResize(AggregatedOutput.rows()+1,AggregatedOutput.cols());
    AggregatedOutput.row(AggregatedOutput.rows()-1) = tmp;

    // Set Output
    Output = AggregatedOutput;

    // Check if we should reset the aggregator
    if (stop_agg == true)
    {
        // Note we set aggregator to tmp so that we will 
        // have a measurement around the next starting point to use
        AggregatedOutput = tmp;
        // Reset the aggregator flag
        stop_agg = false;
    }

    // Return function result
    return Output;
}

// PrunedIMUData removes IMU data collected at and after the desired state 
// estimate time, saves the removed IMU data for later use in aggregate function,
// and outputs time-correct IMU data.
// Inputs:  Matrix of IMU data (in same form as from aggregate)
//          time_desired = time of desired state estimate
// Outputs: Matrix of IMU data (in same form as from aggregate)
MatrixXd IMU_Aggregator::PrunedIMUData(MatrixXd aggregated_meas)
{
    // Initialize the measurement vectors
    unsigned int numRows = aggregated_meas.rows();
    MatrixX7d Pruned_Data;
    MatrixX7d Save_Data_for_later;
    bool DataToSaveForLater = false;
    
    /*----------------------------------------------------------*/
    // Find IMU measurements to use for later state estimates 
    // and ones to use now
    // Tranforms Eigen Matrix Time Column to a std vector
    vector<double> vec(aggregated_meas.col(0).data(),aggregated_meas.col(0).data()+aggregated_meas.rows());
    // Compute the lower bound of time with respect to time desired
    std::vector<double>::iterator low;
    low = lower_bound(vec.begin(),vec.end(),time_desired);
    // Prune the data based on lower bound
    Pruned_Data.conservativeResize(low-vec.begin(),Pruned_Data.cols());
    Pruned_Data = aggregated_meas.topRows(low-vec.begin());
    // Save the data the isn't less than the time_desired
    if (aggregated_meas(numRows-1,0) >= time_desired)
    {
        Save_Data_for_later.conservativeResize(vec.size()-(low-vec.begin()),Save_Data_for_later.cols());
        Save_Data_for_later = aggregated_meas.bottomRows(vec.size()-(low-vec.begin()));
        DataToSaveForLater = true;
    }
    /*----------------------------------------------------------*/

    // If there is data to save for later, then do so
    // Note AggregatedOutput will be reset here and used 
    // in aggregate function
    if (DataToSaveForLater == true)
    {
        AggregatedOutput = Save_Data_for_later;
    }
    
    // Return function result
    return Pruned_Data;
}

// This will get called when a new IMU message has arrived on the appropriate topic.
void IMU_Aggregator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Matrix for aggregated IMU measurements
    MatrixXd aggregated_meas;
    // Create Publisher as Float64MultiArray
    static ros::NodeHandle agg_topic;
    static ros::Publisher pub_agg = agg_topic.advertise<djh_ins::compute_ins>("aggregate_imu",1000);
    static djh_ins::compute_ins msg0;
    // Create subscriber of ins_compute
    static ros::NodeHandle agg_comp_ins;
    static IMU_Aggregator listener;
    static ros::Subscriber agg_ins_comp = agg_comp_ins.subscribe("/comp_sol",1000,&IMU_Aggregator::comp_ins_solCallback, &listener);

    if (listener.stop_agg == false)
    {

        /*-------------------- Aggregate IMU Data --------------------*/
        aggregated_meas = listener.Aggregate(msg);
        /*------------------ End Aggregate IMU Data ------------------*/
    }
    else
    {
        /*-------------------- Aggregate IMU Data --------------------*/
        aggregated_meas = listener.Aggregate(msg);
        /*------------------ End Aggregate IMU Data ------------------*/

        /*--------------- Prune the result if necessary --------------*/
        aggregated_meas = listener.PrunedIMUData(aggregated_meas);
        /*------------- End Prune the result if necessary ------------*/
    
        /*--------------- Publish Aggregated Result ------------------*/
        // Turn Eigen Matrix to Vector
        vector<double> vec(aggregated_meas.data(), aggregated_meas.data()+aggregated_meas.rows()*aggregated_meas.cols());
        // Clear any old message data (just in case)
        msg0.aggregatedMatrix.data.clear();
        // Assign aggregated data to message
        msg0.aggregatedMatrix.data.insert(msg0.aggregatedMatrix.data.end(),vec.begin(),vec.end());
        // This assigns the time at which a desired INS solution is to be computed
        msg0.time_desired = listener.time_desired;
        // This assigns the stop agg flag to the message
        msg0.stop_agg = listener.stop_agg;
        // Set start time
        msg0.start_time = listener.start_time;
        // This assigns the initial state for the INS computation with the aggregated matrix
        msg0.start_pose.orientation.x = listener.state(0);
        msg0.start_pose.orientation.y = listener.state(1);
        msg0.start_pose.orientation.z = listener.state(2);
        msg0.start_pose.orientation.w = listener.state(3);
        msg0.start_pose.position.x = listener.state(4);
        msg0.start_pose.position.y = listener.state(5);
        msg0.start_pose.position.z = listener.state(6);
        msg0.start_vel.x = listener.state(7);
        msg0.start_vel.y = listener.state(8);
        msg0.start_vel.z = listener.state(9);
        // Publish the Aggregated Results
        pub_agg.publish(msg0);
        /*------------- End Publish Aggregated Result ----------------*/

    }
}
/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/