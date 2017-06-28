//
//	IMU Corrector
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	imu_corrector.h
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

/*------------------ Includes ------------------*/
#include <Eigen/Core>
#include "std_msgs/Float64MultiArray.h"
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
/*--------------- End Namespaces ---------------*/
 
/*------------ Function Prototypes -------------*/
/*---------- End Function Prototypes -----------*/

/*------------------- Classes ------------------*/
class IMUCorrector
{
    public:
        // Constructor
        IMUCorrector();

        // imu_correct accepts an aggregated n-by-7 IMU matrix and corrects the IMU 
        // measurements for fixed scale, bias, and cross-coupling factors. It then 
        // adds a bias estimated online.
        // Inputs:  agg_mat = Aggregated IMU matrix
        // Outputs: agg_corrected = the aggregated IMU matrix adjusted to eliminate 
        //                          IMU errors
        void imu_correct(MatrixXd agg_mat);

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
        void biasCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

        // Public variable that is filled with an aggregated matrix that has been
        // corrected for IMU errors in the function imu_correct
        MatrixXd agg_corrected;
    private:
        // Matrix of scale factor errors for IMU gyroscopes
        Matrix3d Scale_gyro;
        // Matrix of scale factor errors for IMU accelerometers
        Matrix3d Scale_accel;
        // Matrix of cross coupling errors for IMU gyroscopes
        Matrix3d Cross_coup_gyro;
        // Matrix of cross coupling errors for IMU accelerometers
        Matrix3d Cross_coup_accel;
        // Vector of fixed gyroscope biases 
        Vector3d Bf_gyro;
        // Vector of fixed accelerometer biases
        Vector3d Bf_accel;
        // Vector of accelerometer biases estimated online
        Vector3d biasa;
        // Vector of gyroscope biases estimated online
        Vector3d biasg;
};
/*----------------- End Classes ----------------*/