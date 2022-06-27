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
 * @tparam Tp    The type of outputs, generally float or double.
 * @tparam Tp1   The type of inputs, generally float or double.
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @return rt The rotation and position matrix of single continuum segment with
 * respect to its base frame.
 */
template <typename Tp = double, typename Tp1 = double>
void  calcSingleSegmentPose(Tp1 L, Tp1 theta, Tp1 delta, Pose<Tp>& pose)
{
    Eigen::Matrix<Tp, 3, 3> R_t1_2_tb =
            rotByZ<Tp>(-PI / 2 + delta)*rotByY<Tp>(-PI / 2);
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
template <typename Tp = double, typename Tp1 = double>
Pose<Tp> calcSingleSegmentPose(Tp1 L, Tp1 theta, Tp1 delta)
{
    Pose<Tp> pose;
    calcSingleSegmentPose<Tp>(L, theta, delta, pose);
    return pose;
}


/**
 * @brief Override based on 'void calcSingleSegmentPose()'
 */
template <typename Tp = double, typename Tp1 = double>
void calcSingleSegmentPose(const ConfigSpc<Tp1>& q, Pose<Tp>& pose)
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
template <typename Tp = double, typename Tp1 = double>
Pose<Tp> calcSingleSegmentPose(const ConfigSpc<Tp1>& q)
{
    Pose<Tp> pose;
    calcSingleSegmentPose<Tp>(q, pose);
    return pose;
}


/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
void calcSingleSegmentPosef(Tp1 L, Tp1 theta, Tp1 delta, Pose<float>& pose)
{
    calcSingleSegmentPose<float>(L, theta, delta, pose);
}
/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
Pose<float> calcSingleSegmentPosef(Tp1 L, Tp1 theta, Tp1 delta)
{
    return calcSingleSegmentPose<float>(L, theta, delta);
}
/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
void calcSingleSegmentPosef(const ConfigSpc<Tp1>& q, Pose<float>& pose)
{
    calcSingleSegmentPose<float>(q, pose);
}
/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
Pose<float> calcSingleSegmentPosef(const ConfigSpc<Tp1>& q)
{
    return calcSingleSegmentPose<float>(q);
}


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
template <typename Tp = double, typename Tp1 = double>
void calcSingleWithRigidSegmentPose(Tp1 L, Tp1 theta, Tp1 delta, Tp1 Lr, Pose<Tp>& pose)
{
    pose = calcSingleSegmentPose<Tp>(L, theta, delta);
    Eigen::Vector<Tp, 3> z(pose.R(0,2), pose.R(1,2), pose.R(2,2));
    pose.t += Lr * z;
}


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
template <typename Tp = double, typename Tp1 = double>
Pose<Tp> calcSingleWithRigidSegmentPose(Tp1 L, Tp1 theta, Tp1 delta, Tp1 Lr)
{
    Pose<Tp> pose;
    calcSingleWithRigidSegmentPose<Tp>(L, theta, delta, Lr, pose);
    return pose;
}


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
template <typename Tp = double, typename Tp1 = double>
void calcSingleWithRigidSegmentPose(const ConfigSpc<Tp1>& q, Tp1 Lr, Pose<Tp>& pose)
{
    calcSingleSegmentPose<Tp>(q, pose);
    Eigen::Vector<Tp, 3> z(pose.R(0,2), pose.R(1,2), pose.R(2,2));
    pose.t += Lr * z;
}


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
template <typename Tp = double, typename Tp1 = double>
Pose<Tp> calcSingleWithRigidSegmentPose(const ConfigSpc<Tp1>& q, Tp1 Lr)
{
    Pose<Tp> pose;
    calcSingleWithRigidSegmentPose<Tp>(q, Lr, pose);
    return pose;
}


/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
void calcSingleWithRigidSegmentPosef(Tp1 L, Tp1 theta, Tp1 delta, Tp1 Lr, Pose<float>& pose)
{
    calcSingleWithRigidSegmentPose<float>(L, theta, delta, Lr, pose);
}
/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
Pose<float> calcSingleWithRigidSegmentPosef(Tp1 L, Tp1 theta, Tp1 delta, Tp1 Lr)
{
    return calcSingleWithRigidSegmentPose<float>(L, theta, delta, Lr);
}
/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
void calcSingleWithRigidSegmentPosef(const ConfigSpc<Tp1>& q, Tp1 Lr, Pose<float>& pose)
{
    calcSingleWithRigidSegmentPose<float>(q, Lr, pose);
}
/**
 * @brief Explicit interface
 */
template<typename Tp1 = float>
Pose<float> calcSingleWithRigidSegmentPosef(const ConfigSpc<Tp1>& q, Tp1 Lr)
{
    return calcSingleWithRigidSegmentPose<float>(q, Lr);
}


}} // mmath::continuum
#endif // LIB_MATH_CONTINUUM_RT_H_LF
