//
//	Aggregator of IMU Data
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	aggregator.h
//	Aggregator of IMU Data header file for the Bretl Group 
//  INS package. This aggregates the IMU data into a matrix
//  and this matrix is sent to INS for integration.
//
//	Options:
//
//
//	Usage:
//	djh_ins_node.cpp	
//

/*---------------- Include Guard ---------------*/
#ifndef aggregator_H
#define aggregator_H
/*-------------- End Include Guard -------------*/

/*------------------ Includes ------------------*/
#include <Eigen/Core>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "djh_ins/compute_ins.h"
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
/*--------------- End Namespaces ---------------*/

/*------------------ Globals -------------------*/
// Define new matrix type with dynamic number of 
// rows and 7 columns
typedef Matrix<double, Dynamic,7> MatrixX7d;
// Define new vector type with 10 elements
typedef Matrix<double, 10, 1> Vector10d;
/*---------------- End Globals -----------------*/

/*------------------- Classes ------------------*/
class IMU_Aggregator
{
    public:
        // Constructor
        IMU_Aggregator();
        // Parameterized Constructor
        IMU_Aggregator(bool stop_agg_change,double time_desired_change,VectorXd state_change,double start_time_change);
        // Recieve message compute_ins message to stop aggregating IMU 
        // data and compute INS solution at a given time
        void comp_ins_solCallback(const djh_ins::compute_ins::ConstPtr& msg);
        // Flag to stop aggregating IMU Data (true) or not (false)
        bool stop_agg;
        // Desired time for an INS Solution
        double time_desired;
        // This will get called when a new IMU message has arrived on the appropriate topic.
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    private:
        // Use a global aggregated IMU data output with a 
        // dynamic number of rows and 7 columns
        MatrixX7d AggregatedOutput;
        // aggregate outputs an aggregated matrix of IMU 
        // data collected over time.
        // Inputs:  Standard ROS IMU message
        // Outputs: Matrix of IMU Data in the form ...
        //          [timestamp1, accel_x1, accel_y1, accel_z1, gyro_x1, gyro_y1, gyro_z1]
        //          [timestamp2, accel_x2, accel_y2, accel_z2, gyro_x2, gyro_y2, gyro_z2]
        //          [...       , ...     , ...     , ...     , ...    , ...    , ...    ]
        MatrixXd Aggregate(const sensor_msgs::Imu::ConstPtr& msg);
        // PrunedIMUData removes IMU data collected at and 
        // after the desired state estimate time, saves 
        // the removed IMU data for later use in aggregate 
        // function, and outputs time-correct IMU data.
        // Inputs:  Matrix of IMU data (in same form as from aggregate)
        // Outputs: Matrix of IMU data (in same form as from aggregate)
        MatrixXd PrunedIMUData(MatrixXd aggregated_meas);
        // Initial state estimate prior to INS integration
        Vector10d state;
        // The start time for INS integration
        double start_time;
};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/