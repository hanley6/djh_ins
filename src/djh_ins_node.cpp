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
#include "quaternion_math.h"
// Include the ROS C++ APIs
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
typedef Matrix<double, 10, 1> Vector10d;
// Gravity vector
Vector3d grav;
// Init timer
clock_t t;
clock_t t1;
clock_t t2;
clock_t t3;
clock_t t4;
clock_t t5;
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
// This function outputs the time derivative of the state given the state, IMU 
// measurements, and estimated gravtiy vector
// Inputs:  state = [quaternion, position, velocity] of INS,
//          imu = [accelerometer, gyroscope] measurements from IMU
//          g = estimated gravity vector
//          O = Angular Velocity Matrix
// Outputs: dstate = time derivative of the state [quaternion, position, velocity]
Vector10d INSODE(Vector10d state,VectorXd imu,Matrix4d O)
{
    /*---------- Initializations ----------*/
    QuatMath quaternion(state.head(4));
    Vector10d dstate(10);
    /*-------- End Initiaizations --------*/

    /*-------- Equations of Motion --------*/
    // Quaternion Equations of Motion
    dstate.head(4) = 0.5*O*quaternion.quat;
    // Position Equations of Motion
    dstate.segment(4,3) = state.tail(3);
    // Velocity Equations of Motion
    dstate.tail(3) = quaternion.CreateRot()*imu.head(3) + grav;
    /*------ End Equations of Motion ------*/

    /*----------- Return Result -----------*/
    return dstate;
    /*--------- End Return Result ---------*/
}

// Implements standard fourth-order Runge Kutta integration for INS
// Inputs:  state = [quaternion, position, velocity] of INS,
//          imu = [accelerometer, gyroscope] measurements from IMU
//          dt = step size for the integration
//          O = Angular Velocity Matrix
// Outputs: output_state = estimate of [quaternion, position, velocity] 
//                         at time t+dt where starting time is t
Vector10d RK4(Vector10d state, VectorXd imu, double dt, Matrix4d O)
{
    /*---------- Initializations ----------*/
    Vector10d K1(10);
    Vector10d K2(10);
    Vector10d K3(10);
    Vector10d K4(10);
    Vector10d output_state(10);
    /*-------- End Initializations --------*/

    /*------ Runge Kutta Integration ------*/
    K1 = INSODE(state,imu,O);
    K2 = INSODE(state + 0.5*dt*K1,imu,O);
    K3 = INSODE(state + 0.5*dt*K2,imu,O);
    K4 = INSODE(state + dt*K3,imu,O);
    output_state = state + (1.0/6.0)*dt*(K1 + 2.0*K2 + 2.0*K3 + K4);
    // Normalize quaternion result
    QuatMath q2norm(output_state.head(4));
    q2norm.Normalize();
    output_state.head(4) = q2norm.quat;
    /*---- End Runge Kutta Integration ----*/

    /*----------- Return Result -----------*/
    return output_state;
    /*--------- End Return Result ---------*/
}

// This will get called when a new aggregated IMU packet has arrived 
// on the appropriate topic. This computes and publishes an INS result
void insCallback(const djh_ins::compute_ins::ConstPtr& msg)
{   
    /*------- Receive and reform aggregated IMU Matrix -------*/
    // Define a temporary std vector for the aggregated IMU message data
    vector<double> vec = msg->aggregatedMatrix.data;
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
    
    /*------------- Set the initial state vector -------------*/
    // quat, position, velocity
    Vector10d state(10);
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
    /*----------- End Set the initial state vector -----------*/

    /*------------------- INS Integration --------------------*/
    // Iterate through aggregated matrix
    VectorXd IMU_meas(6);
    double dt;
    Vector10d state_result(10);
    QuatMath quaternion;
    Matrix4d O;
    for (int i = 0; i < agg_mat.rows(); i++)
    {
        // The latest accelerometer measurements
        IMU_meas(0) = agg_mat(i,1);
        IMU_meas(1) = agg_mat(i,2);
        IMU_meas(2) = agg_mat(i,3);
        // The latest gyroscope measurements
        IMU_meas(3) = agg_mat(i,4);
        IMU_meas(4) = agg_mat(i,5);
        IMU_meas(5) = agg_mat(i,6);

        // if we are not on the last row
        if (i < agg_mat.rows()-1)
        {
            // but we are the first element
            if (i == 0)
            {
                dt = agg_mat(i+1,0) - msg->start_time;
            }
            // but we are at time greater than or equal to time desired
            else if (agg_mat(i,0) >= msg->time_desired)
            {
                dt = 0.0;
            }
            // but we are at time less than time desired
            else if (agg_mat(i+1,0) < msg->time_desired)
            {
                dt = agg_mat(i+1,0) - agg_mat(i,0);
            }
            // but the next time stamp is greater than time desired 
            // (although not current time stamp)
            else
            {
                dt = msg->time_desired - agg_mat(i,0);
            }
        }
        // but we are the first element
        else if (i == 0)
        {
            dt = msg->time_desired - msg->start_time;
        }
        // Otherwise ...
        else
        {
            dt = msg->time_desired - agg_mat(i,0);
        }

        // Compute the state at the next time step using fourth-order Runge-Kutta
        O = quaternion.OmegaMatrix(IMU_meas.tail(3));
        state_result = RK4(state,IMU_meas,dt,O);
        state = state_result;
    }
    /*----------------- End INS Integration ------------------*/

    /*-------------------- Publish Results -------------------*/
    static ros::NodeHandle ins_result;
    static ros::Publisher pub_insres = ins_result.advertise<djh_ins::compute_ins>("ins_result",1000);
    static djh_ins::compute_ins msg_out;
    // Fill Accelerometer vector and gyroscope vector
    msg_out.time_desired = msg->time_desired;
    // We are going to publish this message now
    msg_out.header.stamp = ros::Time::now();
    // We will also take this time as the starting time
    msg_out.start_time = msg->time_desired;
    // Publish Pose
    msg_out.start_pose.orientation.x = state(0);
    msg_out.start_pose.orientation.y = state(1);
    msg_out.start_pose.orientation.z = state(2);
    msg_out.start_pose.orientation.w = state(3);
    msg_out.start_pose.position.x = state(4);
    msg_out.start_pose.position.y = state(5);
    msg_out.start_pose.position.z = state(6);
    msg_out.start_vel.x = state(7);
    msg_out.start_vel.y = state(8);
    msg_out.start_vel.z = state(9);
    // Publish the messages now
    pub_insres.publish(msg_out);
    /*------------------ End Publish Results -----------------*/
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
	ros::init(argc, argv, "djh_ins_node");

	// Start the node resource managers (communication, time, etc)
	ros::start();

    // Initialize node handles
    ros::NodeHandle ns;
    ros::NodeHandle agg_receiver;

    // Create an IMU Aggregator Object
    IMU_Aggregator imu_agg;

    /*------------ Set Gravity ------------*/
    // Assuming a NED coordinate frame
    // Pull Earth parameters from ROS server
    double latitude;
    double height;
    double R0;
    double rotEarth;
    ros::param::get("djh_ins_node/latitude",latitude);
    ros::param::get("djh_ins_node/altitude",height);
    ros::param::get("djh_ins_node/EarthRad",R0);
    ros::param::get("djh_ins_node/rotEarth",rotEarth);
    // Compute magnitude of gravity
    double s2L = pow(sin(latitude),2.0);
    double s22L = pow(sin(2.0*latitude),2.0);
    double g_norm = 9.780318*(1.0 + 0.0053024*s2L - 0.0000059*s22L);
    double g_normHeight = g_norm/pow(1.0 + height/R0,2.0);
    // Compute gravity with plumb-bob effect
    grav(0) = -(pow(rotEarth,2.0)*(R0 + height)*sin(2.0*latitude)/2.0);
    grav(1) = 0.0;
    grav(2) = g_normHeight - (pow(rotEarth,2.0)*(R0 + height)*(1.0 + cos(2.0*latitude))/2.0);
    /*---------- End Set Gravity ----------*/

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