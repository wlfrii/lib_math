/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		continuum_pose.h
 * 
 * @brief 		Design some interfaces for continuum segment kinematics.
 * 
 * @author		Longfei Wang
 * 
 * @date		2019/12/14
 * 
 * @license		
 * 
 * Copyright (C) 2019 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_CONTINUUM_RT_H_LF
#define LIB_MATH_CONTINUUM_RT_H_LF
#include <Eigen/Dense>
#include "pose.h"
#include "continuum_configspc.h"
#include "../matrix/rotation.h"

namespace mmath{
namespace continuum{

/** 
 * @brief Calculating the rotation and position of the end frame of single 
 * continuum segment with respect to its base frame.
 * @tparam Tp    The type of inputs, generally float or double.
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @return rt The rotation and position matrix of single continuum segment with
 * respect to its base frame.
 */
template <typename Tp = double>
void  calcSingleSegmentPose(Tp L, Tp theta, Tp delta, Pose<Tp>& pose)
{
    Eigen::Matrix<Tp, 3, 3> R_t1_2_tb =
            rotByZ<Tp>(-PI / 2 - delta)*rotByY<Tp>(-PI / 2);
    pose.R = R_t1_2_tb * rotByZ<Tp>(theta) * R_t1_2_tb.transpose();
    if (abs(theta) < 1e-5) {
        pose.t = { 0, 0, L };
    }
    else {
        Tp rc = L / theta;
        pose.t = rc * R_t1_2_tb *
                Eigen::Vector<Tp, 3>(sin(theta), 1 - cos(theta), 0);
    }
}


/**
 * @brief Override based on 'void calcSingleSegmentPose()'
 */
template <typename Tp = double>
Pose<Tp> calcSingleSegmentPose(Tp L, Tp theta, Tp delta)
{
    Pose<Tp> pose;
    calcSingleSegmentPose<Tp>(L, theta, delta, pose);
    return pose;
}


/**
 * @brief Override based on 'void calcSingleSegmentPose()'
 */
template <typename Tp = double>
void calcSingleSegmentPose(const ConfigSpc<Tp>& q, Pose<Tp>& pose)
{
    if(q.is_bend){
        calcSingleSegmentPose<Tp>(q.length, q.theta, q.delta, pose);
    }
    else{
        pose.R = rotByZ<Tp>(q.delta);
        pose.t[2] = q.length;
    }
}


/**
 * @brief Override based on 'void calcSingleSegmentPose()'
 */
template <typename Tp = double>
Pose<Tp> calcSingleSegmentPose(const ConfigSpc<Tp>& q)
{
    Pose<Tp> pose;
    calcSingleSegmentPose<Tp>(q, pose);
    return pose;
}


/**
 * @brief Explicit interface
 */
void calcSingleSegmentPosef(float L, float theta, float delta, Pose<float>& pose);
/**
 * @brief Explicit interface
 */
Pose<float> calcSingleSegmentPosef(float L, float theta, float delta);
/**
 * @brief Explicit interface
 */
void calcSingleSegmentPosef(const ConfigSpc<float>& q, Pose<float>& pose);
/**
 * @brief Explicit interface
 */
Pose<float> calcSingleSegmentPosef(const ConfigSpc<float>& q);


/** 
 * @brief Calculating the rotation and position of the end frame of single 
 * continuum segment with a rigid segment, with respect to its base frame.
 * @param L      The length of the segment.
 * @param Lr     The length of the rigid segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @return rt The rotation and position matrix of single continuum segment with
 * respect to its base frame.
 */
template <typename Tp = double>
void calcSingleWithRigidSegmentPose(Tp L, Tp theta, Tp delta, Tp Lr, Pose<Tp>& pose)
{
    pose = calcSingleSegmentPose<Tp>(L, theta, delta);
    pose.t += Lr * pose.R.rightCols(0);
}


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
template <typename Tp = double>
Pose<Tp> calcSingleWithRigidSegmentPose(Tp L, Tp theta, Tp delta, Tp Lr)
{
    Pose<Tp> pose;
    calcSingleWithRigidSegmentPose<Tp>(L, Lr, theta, delta, pose);
    return pose;
}


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
template <typename Tp = double>
void calcSingleWithRigidSegmentPose(const ConfigSpc<Tp>& q, Tp Lr, Pose<Tp>& pose)
{
    calcSingleSegmentPose<Tp>(q, pose);
    pose.t += Lr * pose.R.rightCols(0);
}


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
template <typename Tp = double>
Pose<Tp> calcSingleWithRigidSegmentPose(const ConfigSpc<Tp>& q, Tp Lr)
{
    Pose<Tp> pose;
    calcSingleWithRigidSegmentPose<Tp>(q, Lr, pose);
    return pose;
}


/**
 * @brief Explicit interface
 */
void calcSingleWithRigidSegmentPosef(float L, float theta, float delta, float Lr, Pose<float>& pose);
/**
 * @brief Explicit interface
 */
Pose<float> calcSingleWithRigidSegmentPosef(float L, float theta, float delta, float Lr);
/**
 * @brief Explicit interface
 */
void calcSingleWithRigidSegmentPosef(const ConfigSpc<float>& q, float Lr, Pose<float>& pose);
/**
 * @brief Explicit interface
 */
Pose<float> calcSingleWithRigidSegmentPosef(const ConfigSpc<float>& q, float Lr);


}} // mmath::continuum
#endif // LIB_MATH_CONTINUUM_RT_H_LF
