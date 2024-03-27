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
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with respect to its base frame.
 * 
 * @remark This is the base of overloaded functions.
 *
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [out] pose  The end pose of a single continuum segment w.r.t its base 
 * frame. 
 * 
 * @see mmath::Pose.
 */
void calcSingleSegmentPose(kfloat L, kfloat theta, kfloat delta, Pose &pose);


/**
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with respect to its base frame.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * 
 * @return The end pose of a single continuum segment w.r.t its base frame.
 * 
 * @see mmath::Pose.
 */
Pose calcSingleSegmentPose(kfloat L, kfloat theta, kfloat delta);


/**
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with respect to its base frame.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @param [in] q A ConfiSpc object
 * @param [out] pose  The end pose of a single continuum segment w.r.t its base 
 * frame.
 * 
 * @see mmath::Pose, mmath::continuum::ConfigSpc
 */
void calcSingleSegmentPose(const ConfigSpc &q, Pose &pose);


/**
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with respect to its base frame.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @param [in] q A ConfiSpc object
 * 
 * @return The end pose of a single continuum segment w.r.t its base frame.
 * 
 * @see mmath::Pose, mmath::continuum::ConfigSpc
 */
Pose calcSingleSegmentPose(const ConfigSpc& q);



/** 
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with a rigid segment, with respect to its base frame.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] pose  The end pose of a single continuum segment that with a 
 * rigid segment w.r.t its base frame.
 * 
 * @see mmath::Pose
 */
void calcSingleWithRigidSegmentPose(kfloat L, kfloat theta, kfloat delta,
                                    kfloat Lr, Pose &pose);


/** 
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with a rigid segment, with respect to its base frame.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [in] Lr     The length of the rigid segment.
 * 
 * @return The end pose of a single continuum segment that with a rigid segment 
 * w.r.t its base frame.
 * 
 * @see mmath::Pose
 */
Pose calcSingleWithRigidSegmentPose(kfloat L, kfloat theta, kfloat delta,
                                    kfloat Lr);


/** 
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with a rigid segment, with respect to its base frame.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @param [in] q  A ConfiSpc object
 * @param [in] Lr The length of the rigid segment.
 * @param [out] pose The end pose of a single continuum segment that with a 
 * rigid segment w.r.t its base frame.
 * 
 * @see mmath::Pose, mmath::continuum::ConfigSpc
 */
void calcSingleWithRigidSegmentPose(const ConfigSpc &q, kfloat Lr, Pose &pose);


/**
 * @brief Calculating the rotation and position of the end frame of a single 
 * continuum segment with a rigid segment, with respect to its base frame.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @param [in] q  A ConfiSpc object
 * @param [in] Lr The length of the rigid segment.
 * 
 * @return The end pose of a single continuum segment that with a rigid segment
 * w.r.t its base frame.
 * 
 * @see mmath::Pose, mmath::continuum::ConfigSpc
 */
Pose calcSingleWithRigidSegmentPose(const ConfigSpc &q, kfloat Lr);


}} // mmath::continuum
#endif // LIB_MATH_CONTINUUM_POSE_H_LF