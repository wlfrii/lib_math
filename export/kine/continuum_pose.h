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
 * @license		MIT
 * 
 * Copyright (C) 2019-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_CONTINUUM_POSE_H_LF
#define LIB_MATH_CONTINUUM_POSE_H_LF
#include <Eigen/Dense>
#include "pose.h"
#include "continuum_configspc.h"
#include "../matrix/rotation.h"

namespace mmath{
namespace continuum{

/** 
 * @brief Calculating the rotation and position of the end frame of single 
 * continuum segment with respect to its base frame.
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @return The rotation and position matrix of single continuum segment with
 * respect to its base frame.
 */
void calcSingleSegmentPose(kfloat L, kfloat theta, kfloat delta, Pose &pose);


/**
 * @brief Override based on 'void calcSingleSegmentPose()'
 */
Pose calcSingleSegmentPose(kfloat L, kfloat theta, kfloat delta);


/**
 * @brief Override based on 'void calcSingleSegmentPose()'
 */
void calcSingleSegmentPose(const ConfigSpc &q, Pose &pose);


/**
 * @brief Override based on 'void calcSingleSegmentPose()'
 */
Pose calcSingleSegmentPose(const ConfigSpc& q);



/** 
 * @brief Calculating the rotation and position of the end frame of single 
 * continuum segment with a rigid segment, with respect to its base frame.
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @param Lr     The length of the rigid segment.
 * @return The rotation and position matrix of single continuum segment with
 * respect to its base frame.
 */
void calcSingleWithRigidSegmentPose(kfloat L, kfloat theta, kfloat delta,
                                    kfloat Lr, Pose &pose);


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
Pose calcSingleWithRigidSegmentPose(kfloat L, kfloat theta, kfloat delta,
                                    kfloat Lr);


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
void calcSingleWithRigidSegmentPose(const ConfigSpc &q, kfloat Lr, Pose &pose);


/**
 * @brief Override based on 'void calcSingleWithRigidSegmentPose()'
 */
Pose calcSingleWithRigidSegmentPose(const ConfigSpc &q, kfloat Lr);


}} // mmath::continuum
#endif // LIB_MATH_CONTINUUM_POSE_H_LF