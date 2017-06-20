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
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
/*--------------- End Namespaces ---------------*/
 
/*------------ Function Prototypes -------------*/
void imu_correct(Vector3d &a_sf_i, Vector3d &w_i, Vector3d biasa, Vector3d biasg);
/*---------- End Function Prototypes -----------*/
