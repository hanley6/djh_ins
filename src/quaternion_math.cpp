//
//	Quaternion math using JPL quaternion convention
//	David Hanley
//	
//	quaternion_math.cpp
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
//	djh_ins_node.cpp	
//  ins_ode.cpp
//  

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include "quaternion_math.h"
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
// The constructor assigns a default identity value to the class quaternion 
QuatMath::QuatMath()
{
    quat(0) = 0.0;
    quat(1) = 0.0;
    quat(2) = 0.0;
    quat(3) = 1.0;
}

// The parameterized constructor assigns the input quaternion to the class 
// quaternion
// Inputs:  quat_inputs = JPL input quaternion of the form [q_x, q_y, q_z, q_w]
// Outputs: quat = class quaternion in JPL convention
QuatMath::QuatMath(Vector4d quat_input)
{
    quat = quat_input;
}

// This function ensures that the class' quaternion has a norm of 1.
// Inputs:  None. This function operates on the public quaternion variable
// Outputs: quat = class quaternion in JPL convention (now with a norm of 1)
void QuatMath::Normalize()
{
    /*---------- Initializations ----------*/
    double nm = quat.norm();
    Vector4d output_quat;
    /*-------- End Initializations --------*/

    /*----- Normalize the quaternion ------*/
    output_quat = quat/nm;
    quat = output_quat;
    /*--- End Normalize the quaternion ----*/
}

// Compute the angle of rotation of the class' quaternion.
// Inputs:  None. This function operates on the public quaternion variable
// Outputs: angle = the angle of rotation in radians
double QuatMath::AngleOfRot()
{
    /*---------- Initializations ----------*/
    double angle;
    /*-------- End Initializations --------*/

    /*----- Compute angle of rotation -----*/
    angle = 2.0*acos(quat(3));
    /*--- End Compute angle of rotation ---*/

    /*----------- Output Result -----------*/
    return angle;
    /*--------- End Output Result ---------*/
}

// Multiply the class quaternion (q) by an input quaternion (p) in 
// the form q*p
// Inputs:  quat_input = Input quaternion (p) that is multiplied
//          by class quaternion (q) in the form q*p
// Outputs: output_quat = Output quaternion from multiplication q*p
Vector4d QuatMath::QuatMultiply(Vector4d quat_input)
{
    /*---------- Initializations ----------*/
    Vector4d output_quat;
    /*-------- End Initializations --------*/

    /*------- Multiply Quaternions --------*/
    output_quat(0) =  quat(3)*quat_input(0) + quat(2)*quat_input(1) - quat(1)*quat_input(2) + quat(0)*quat_input(3);
    output_quat(1) = -quat(2)*quat_input(0) + quat(3)*quat_input(1) + quat(0)*quat_input(2) + quat(1)*quat_input(3);
    output_quat(2) =  quat(1)*quat_input(0) - quat(0)*quat_input(1) + quat(3)*quat_input(2) + quat(2)*quat_input(3);
    output_quat(3) = -quat(0)*quat_input(0) - quat(1)*quat_input(1) - quat(2)*quat_input(2) + quat(3)*quat_input(3);
    /*----- End Multiply Quaternions ------*/

    /*----------- Output Result -----------*/
    return output_quat;
    /*--------- End Output Result ---------*/
}

// Compute and output the inverse of the class' quaternion.
// Inputs:  None. This function operates on the public quaternion variable
// Outputs: output_quat = the inverse of the class' quaternion
Vector4d QuatMath::QuatInverse()
{
    /*---------- Initializations ----------*/
    Vector4d output_quat;
    /*-------- End Initializations --------*/

    /*---- Compute Quaternion Inverse -----*/
    output_quat(0) = -quat(0);
    output_quat(1) = -quat(1);
    output_quat(2) = -quat(2);
    output_quat(3) =  quat(3);
    /*-- End Compute Quaternion Inverse ---*/

    /*----------- Output Result -----------*/
    return output_quat;
    /*--------- End Output Result ---------*/
}

// Output rotation matrix of the class' quaternion
// Inputs:  None. This function operates on the public quaternion variable.
// Outputs: C = the 3x3 rotation matrix represented by the class' quaternion.
Matrix3d QuatMath::CreateRot()
{
    /*---------- Initializations ----------*/
    Matrix3d C;
    /*-------- End Initializations --------*/

    /*------ Create Rotation Matrix -------*/
    C(0,0) = 1.0 - 2.0*pow(quat(1),2.0) - 2.0*pow(quat(2),2.0);
    C(0,1) = 2.0*(quat(0)*quat(1) + quat(2)*quat(3));
    C(0,2) = 2.0*(quat(0)*quat(2) - quat(1)*quat(3));
    C(1,0) = 2.0*(quat(0)*quat(1) - quat(2)*quat(3));
    C(1,1) = 1.0 - 2.0*pow(quat(0),2.0) - 2.0*pow(quat(2),2.0);
    C(1,2) = 2.0*(quat(1)*quat(2) + quat(0)*quat(3));
    C(2,0) = 2.0*(quat(0)*quat(2) + quat(1)*quat(3));
    C(2,1) = 2.0*(quat(1)*quat(2) - quat(0)*quat(3));
    C(2,2) = 1.0 - 2.0*pow(quat(0),2.0) - 2.0*pow(quat(1),2.0);
    /*---- End Create Rotation Matrix -----*/

    /*---------- Return Results -----------*/
    return C;
    /*-------- End Return Results ---------*/
}

// Assigns the class' quaternion to be equivalent to a rotation matrix
// Inputs:  C = a rotation matrix to be converted to a quaternion
// Outputs: quat = class quaternion in JPL convention 
void QuatMath::Rot2Quat(Matrix3d C)
{
    /*---------------- Initializations ----------------*/
    double trace;
    /*-------------- End Initializations --------------*/

    /*-------- Compute Trace of Rotation Matrix -------*/
    trace = C(0,0) + C(1,1) + C(2,2);
    /*------ End Compute Trace of Rotation Matrix -----*/

    /*----- Convert Rotation Matrix to Quaternion -----*/
    if (trace != -1.0)
    {
        // Assume q_w is denominator
        quat(3) = sqrt(1.0 + trace)/2.0;
        quat(0) = (C(1,2) - C(2,1))/(4.0*quat(3));
        quat(1) = (C(2,0) - C(0,2))/(4.0*quat(3));
        quat(2) = (C(0,1) - C(1,0))/(4.0*quat(3));
    }
    else if (C(0,0) != 0.0)
    {
        // Assume q_x is denominator
        quat(0) = sqrt(1.0 + 2.0*C(0,0) - trace)/2.0;
        quat(1) = (C(0,1) + C(1,0))/(4.0*quat(0));
        quat(2) = (C(0,2) + C(2,0))/(4.0*quat(0));
        quat(3) = (C(1,2) - C(2,1))/(4.0*quat(0));
    }
    else if (C(1,1) != 0.0)
    {
        // Assume q_y is denominator
        quat(1) = sqrt(1.0 + 2.0*C(1,1) - trace)/2.0;
        quat(0) = (C(0,1) + C(1,0))/(4.0*quat(1));
        quat(2) = (C(1,2) + C(2,1))/(4.0*quat(1));
        quat(3) = (C(2,0) - C(0,2))/(4.0*quat(1));
    }
    else if (C(2,2) != 0.0)
    {
        // Assume q_z is denominator
        quat(2) = sqrt(1.0 + 2.0*C(2,2) - trace)/2.0;
        quat(0) = (C(0,2) + C(2,0))/(4.0*quat(2));
        quat(1) = (C(1,2) + C(2,1))/(4.0*quat(2));
        quat(3) = (C(0,1) - C(1,0))/(4.0*quat(2));
    }
    /*--- End Convert Rotation Matrix to Quaternion ---*/
}

// This function computes a 4x4 matrix of angular velocities 
// from a vector of angular velocities. This matrix can be used 
// in a quaternion's equations of motion.
// Inputs:  w = a three element vector of angular velocity 
//              [w_x, w_y, w_z]
// Outputs: O = a 4x4 matrix of angular velocities that can 
//              be used in quaternion equations of motion.
Matrix4d QuatMath::OmegaMatrix(Vector3d w)
{
    /*---------------- Initializations ----------------*/
    Matrix4d O;
    /*-------------- End Initializations --------------*/

    /*-------- Create Angular Velocity Matrix ---------*/
    O(0,0) =  0.0;
    O(0,1) =  w(2);
    O(0,2) = -w(1);
    O(0,3) =  w(0);
    O(1,0) = -w(2);
    O(1,1) =  0.0;
    O(1,2) =  w(0);
    O(1,3) =  w(1);
    O(2,0) =  w(1);
    O(2,1) = -w(0);
    O(2,2) =  0.0;
    O(2,3) =  w(2);
    O(3,0) = -w(0);
    O(3,1) = -w(1);
    O(3,2) = -w(2);
    O(3,3) =  0.0;
    /*------ End Create Angular Velocity Matrix -------*/

    /*----------------- Return Result -----------------*/
    return O;
    /*--------------- End Return Result ---------------*/
}
/*-----------------------------------------------------------------------------*/
/*------------------------------- End Functions -------------------------------*/
/*-----------------------------------------------------------------------------*/ 