//
//	Corrects IMU measurements for various biases, scale 
//  factors, etc.
//	David Hanley
//	
//	imu_correct.h
//
//
//	IMU Correction header file for the Bretl Group 
//  INS package. This contains a class that allows a 
//  user correct IMU measurements for various biases, scale 
//  factors, etc.
//
//	Usage:
//  
//

/*---------------- Include Guard ---------------*/
#ifndef imu_correct_H
#define imu_correct_H
/*-------------- End Include Guard -------------*/

/*------------------ Includes ------------------*/
#include <Eigen/Core>
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
using namespace std;
/*--------------- End Namespaces ---------------*/

/*------------------ Globals -------------------*/
/*---------------- End Globals -----------------*/

/*------------------- Classes ------------------*/
class imu_correct
{
    public:
        // Default constructor sets all IMU error parameters 
        // to zero
        imu_correct();

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
        imu_correct(Matrix3d scale_g, Matrix3d scale_a, Matrix3d cross_c_g, Matrix3d cross_c_a, Vector3d bf_g, Vector3d bf_a, Vector3d b_a, Vector3d b_g);

        // Parameterized constructor initializes all IMU error parameters based on
        // inputs
        // Inputs:  b_a = accelerometer biases estimated online
        //          b_g = gyroscope biases estimated online
        // Outputs: All IMU error parameters set as private class data
        imu_correct(Vector3d b_a, Vector3d b_g);

        // Update IMU error components
        // Inputs:  b_a = accelerometer biases estimated online
        //          b_g = gyroscope biases estimated online
        // Outputs: Online IMU bias parameters updated in private class data
        void update_imu_param(Vector3d b_a, Vector3d b_g);

        // imuCorr accepts an aggregated n-by-7 IMU matrix and corrects the IMU 
        // measurements for fixed scale, bias, and cross-coupling factors. It then 
        // adds a bias estimated online.
        // Inputs:  agg_mat = Aggregated IMU matrix of the form
        //                      [time_1   accel    gyro]
        //                      [time_2   accel    gyro]
        //                      [...      ...      ... ]
        // Outputs: Corr_imu_mat = the aggregated IMU matrix adjusted to eliminate 
        //                         IMU errors
        void imuCorr(MatrixXd agg_mat);

        // Aggregated IMU matrix adjusted to eliminate IMU errors
        // Matrix is of the form:
        //                 [time_1   accel    gyro]
        //                 [time_2   accel    gyro]
        //                 [...      ...      ... ]
        MatrixXd Corr_imu_mat;
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

        // Some temporary matrices we use when correcting the 
        // errors
        MatrixXd tmpg;
        MatrixXd tmpa;
        MatrixXd accel_mat;
        MatrixXd gyro_mat;
};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/