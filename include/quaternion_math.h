//
//	Quaternion math using JPL quaternion convention
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	quaternion_math.h
//	Quaternion math header file for the Bretl Group 
//  INS package. This contains a class of quaternion 
//  math functions using the JPL quaternion convention. 
//  To the best of our knowledge, the Eigen library 
//  uses the Hamilton convention for quaternions. So we 
//  do not use that functionality.
//
//	Options:
//
//
//	Usage:
//	quaternion_math.cpp	
//

/*---------------- Include Guard ---------------*/
#ifndef quaternion_math_H
#define quaternion_math_H
/*-------------- End Include Guard -------------*/

/*------------------ Includes ------------------*/
#include <Eigen/Core>
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace Eigen;
/*--------------- End Namespaces ---------------*/

/*------------------ Globals -------------------*/
/*---------------- End Globals -----------------*/

/*------------------- Classes ------------------*/
class QuatMath
{
    public:
        // Constructor
        QuatMath();

        // Parameterized Constructor
        // Inputs:  quat_inputs = JPL input quaternion of the form 
        //                        [q_x, q_y, q_z, q_w]
        // Outputs: quat = class quaternion in JPL convention
        QuatMath(Vector4d quat_input);

        // This function ensures that the class' quaternion has a norm of 1.
        // Inputs:  None. This function operates on the public quaternion variable
        // Outputs: quat = class quaternion in JPL convention (now with a norm of 1)
        void Normalize();

        // Compute the angle of rotation of the class' quaternion.
        // Inputs:  None. This function operates on the public quaternion variable
        // Outputs: angle = the angle of rotation in radians
        double AngleOfRot();

        // Multiply the class quaternion (q) by an input quaternion (p) in 
        // the form q*p
        // Inputs:  quat_input = Input quaternion (p) that is multiplied
        //          by class quaternion (q) in the form q*p
        // Outputs: output_quat = Output quaternion from multiplication q*p
        Vector4d QuatMultiply(Vector4d quat_input);

        // Compute and output the inverse of the class' quaternion.
        // Inputs:  None. This function operates on the public quaternion variable
        // Outputs: output_quat = the inverse of the class' quaternion
        Vector4d QuatInverse();

        // Output rotation matrix of the class' quaternion
        // Inputs:  None. This function operates on the public quaternion variable.
        // Outputs: C = the 3x3 rotation matrix represented by the class' quaternion.
        Matrix3d CreateRot();

        // Assigns the class' quaternion to be equivalent to a rotation matrix
        // Inputs:  C = a rotation matrix to be converted to a quaternion
        // Outputs: quat = class quaternion in JPL convention 
        void Rot2Quat(Matrix3d C);

        // This function computes a 4x4 matrix of angular velocities 
        // from a vector of angular velocities. This matrix can be used 
        // in a quaternion's equations of motion.
        // Inputs:  w = a three element vector of angular velocity 
        //              [w_x, w_y, w_z]
        // Outputs: O = a 4x4 matrix of angular velocities that can 
        //              be used in quaternion equations of motion.
        Matrix4d OmegaMatrix(Vector3d w);

        // Public JPL Quaternion variable 
        // quat = [q_x, q_y, q_z, q_w]
        Vector4d quat;
    private:

};
/*----------------- End Classes ----------------*/

/*---------------- Include Guard ---------------*/
#endif
/*-------------- End Include Guard -------------*/