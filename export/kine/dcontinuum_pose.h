/**--------------------------------------------------------------------
 *
 *   				   Mathematics extension library
 *
 * Description:
 * This file is part of lib_math. You can redistribute it and or modify
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 *
 * @file 		dcontinuum_pose.h
 *
 * @brief 		Interfaces for derivatives of continuum segment kinematics.
 *
 * @author		Longfei Wang
 *
 * @date		2022/06/27
 *
 * @license
 *
 * Copyright (C) 2019 Longfei Wang.
 *
 * --------------------------------------------------------------------
 * Change History:
 *
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_DCONTINUUM_POSE_H_LF
#define LIB_MATH_DCONTINUUM_POSE_H_LF
#include <Eigen/Dense>
#include "pose.h"
#include "continuum_configspc.h"
#include "../matrix/drotation.h"

namespace mmath{
namespace continuum{


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of single continuum segment, to delta.
 * 
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @param dpose  The derivatives of pose to theta.
 */
void dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta, Pose &dpose);

/**
 * @brief Override based on 'void dSingleSegmentPose2theta()'
 * 
 * @sa dSingleSegmentPose2theta
 */
Pose dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of single continuum segment, to delta.
 * 
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @param dpose  The derivatives of pose to delta.
 */
void dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta, Pose &dpose);

/**
 * @brief Override based on 'void dSingleSegmentPose2delta()'
 * 
 * @sa dSingleSegmentPose2delta
 */
Pose dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of single continuum segment with a rigid segment, to theta.
 * 
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @param Lr     The length of the rigid segment.
 * @param dpose  The derivatives of pose to theta.
 */
void dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose);

/**
 * @brief Override based on 'void dSingleWithRigidSegmentPose2theta()'
 * 
 * @sa dSingleWithRigidSegmentPose2theta
 */
Pose dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of single continuum segment with a rigid segment, to delta.
 * 
 * @param L      The length of the segment.
 * @param theta  The bending angle of the segment.
 * @param delta  The bending direction of the segment.
 * @param Lr     The length of the rigid segment.
 * @param dpose  The derivatives of pose to delta.
 */
void dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose);

/**
 * @brief Override based on 'void dSingleWithRigidSegmentPose2delta()'
 * 
 * @sa dSingleWithRigidSegmentPose2delta
 */
Pose dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr);


}} // mmath::continuum
#endif // LIB_MATH_DCONTINUUM_POSE_H_LF
