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

void dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta, Pose &dpose);

Pose dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta);


void dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta, Pose &dpose);

Pose dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta);


void dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose);

Pose dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr);


void dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose);

Pose dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr);


}} // mmath::continuum
#endif // LIB_MATH_DCONTINUUM_POSE_H_LF
