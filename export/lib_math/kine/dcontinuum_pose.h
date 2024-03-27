/**--------------------------------------------------------------------
 *
 *                     Mathematics extension library
 *
 * Description:
 * This file is part of lib_math. You can redistribute it and or modify
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 *
 * @file 	dcontinuum_pose.h
 *
 * @brief 	Interfaces for derivatives of continuum segment kinematics,
 *              which known as Jacobian.
 *
 * @author	Longfei Wang
 *
 * @date	2022/06/27
 *
 * @license     MIT
 *
 * Copyright (C) 2019-Now Longfei Wang.
 *
 * --------------------------------------------------------------------
 * Change History:
 * 2022.11.30 Complete Jacobian for [Jv, Jw].
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
 * @brief Calculating the partial of the pose of the end frame of a single 
 * continuum segment, to delta.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [out] dpose The derivatives of pose to theta.
 * 
 * @see mmath::Pose.
 */
void dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta, Pose &dpose);


/**
 * @brief Calculating the partial of the pose of the end frame of a single 
 * continuum segment, to delta.
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
 * @return The derivatives of pose to theta.
 * 
 * @see mmath::Pose.
 */
Pose dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta);


/**
 * @brief Calculating the partial of the pose of the end frame of a single 
 * continuum segment, to delta.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [out] dpose The derivatives of pose to delta.
 * 
 * @see mmath::Pose.
 */
void dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta, Pose &dpose);


/**
 * @brief Calculating the partial of the pose of the end frame of a single 
 * continuum segment, to delta.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the the void-returned value is 
 * suggested.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * 
 * @return The derivatives of pose to delta.
 */
Pose dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment, to L.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [out] dpose The derivatives of pose to L.
 * 
 * @see mmath::Pose.
 */
void dSingleSegmentPose2L(kfloat L, kfloat theta, kfloat delta, Pose &dpose);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment, to L.
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
 * @return The derivatives of pose to L.
 * 
 * @see mmath::Pose.
 */
Pose dSingleSegmentPose2L(kfloat L, kfloat theta, kfloat delta);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment with a rigid segment, to theta.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] dpose The derivatives of pose to theta.
 * 
 * @see mmath::Pose.
 */
void dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment with a rigid segment, to theta.
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
 * @return The derivatives of pose to theta.
 * 
 * @see mmath::Pose.
 */
Pose dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment with a rigid segment, to delta.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] dpose The derivatives of pose to delta.
 * 
 * @see mmath::Pose.
 */
void dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment with a rigid segment, to delta.
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
 * @return The derivatives of pose to delta.
 * 
 * @see mmath::Pose.
 */
Pose dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment with a rigid segment, to L.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] dpose The derivatives of pose to L.
 * 
 * @see mmath::Pose.
 */
void dSingleWithRigidSegmentPose2L(kfloat L, kfloat theta, kfloat delta,
                                   kfloat Lr, Pose &dpose);


/**
 * @brief Calculating the partial of rotation and position of the end 
 * frame of a single continuum segment with a rigid segment, to L.
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
 * @return The derivatives of pose to L.
 * 
 * @see mmath::Pose.
 */
Pose dSingleWithRigidSegmentPose2L(kfloat L, kfloat theta, kfloat delta,
                                   kfloat Lr);


/*---------------------------------------------------------------------------*/
/*         Calculate the Jacobian w.r.t Velocity and Angular-Velocity        */
/*---------------------------------------------------------------------------*/


/**
 * @brief Calculate the Jabobian of a single segment w.r.t Velocity and Angular
 *        -Velocity.
 *
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [out] Jv    The returned Jacobian w.r.t Velocity, with 
 *                    [Jv_theta(:), Jv_delta(:)].
 * @param [out] Jw    The returned Jacobian w.r.t Angular-Velocity, with 
 *                    [Jw_theta(:), Jw_delta(:)].
 */
void calcSingleSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta,
        Eigen::Matrix<kfloat, 3, 2>& Jv, Eigen::Matrix<kfloat, 3, 2>& Jw);


/**
 * @brief Calculate the Jabobian of a single segment w.r.t Velocity and Angular
 *        -Velocity.
 *
 * @param [in]  q   A ConfiSpc object.
 * @param [out] Jv  The returned Jacobian w.r.t Velocity, with 
 *                  [Jv_theta(:), Jv_delta(:)].
 * @param [out] Jw  The returned Jacobian w.r.t Angular-Velocity, with 
 *                  [Jw_theta(:), Jw_delta(:)].
 */
void calcSingleSegmentJacobian(const ConfigSpc &q,
        Eigen::Matrix<kfloat, 3, 2>& Jv, Eigen::Matrix<kfloat, 3, 2>& Jw);


/**
 * @brief Calculate the Jabobian of a single segment tha has a variable length
 *        w.r.t Velocity and Angular-Velocity.
 *
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [out] Jv    The returned Jacobian w.r.t Velocity, with 
 *                    [Jv_theta(:), Jv_delta(:), Jv_L(:)].
 * @param [out] Jw    The returned Jacobian w.r.t Angular-Velocity, with 
 *                    [Jw_theta(:), Jw_delta(:), Jw_L(:)].
 */
void calcVariableLengthSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta,
        Eigen::Matrix<kfloat, 3, 3>& Jv, Eigen::Matrix<kfloat, 3, 3>& Jw);


/**
 * @brief Calculate the Jabobian of a single segment tha has a variable length
 *        w.r.t Velocity and Angular-Velocity.
 * 
 * @param [in]  q     A ConfiSpc object.
 * @param [out] Jv    The returned Jacobian w.r.t Velocity, with 
 *                    [Jv_theta(:), Jv_delta(:), Jv_L(:)].
 * @param [out] Jw    The returned Jacobian w.r.t Angular-Velocity, with 
 *                    [Jw_theta(:), Jw_delta(:), Jw_L(:)].
 */
void calcVariableLengthSegmentJacobian(const ConfigSpc &q,
        Eigen::Matrix<kfloat, 3, 3>& Jv, Eigen::Matrix<kfloat, 3, 3>& Jw);


/**
 * @brief Calculate the Jabobian of a single segment with a rigid segment w.r.t
 *        Velocity and Angular-Velocity.
 *
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] Jv    The returned Jacobian w.r.t Velocity, with 
 *                    [Jv_theta(:), Jv_delta(:)].
 * @param [out] Jw    The returned Jacobian w.r.t Angular-Velocity, with 
 *                    [Jw_theta(:), Jw_delta(:)].
 */
void calcSingleWithRigidSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta, kfloat Lr,
        Eigen::Matrix<kfloat, 3, 2>& Jv, Eigen::Matrix<kfloat, 3, 2>& Jw);


/**
 * @brief Calculate the Jabobian of a single segment with a rigid segment w.r.t
 *        Velocity and Angular-Velocity.
 * 
 * @param [in] q      A ConfiSpc object.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] Jv    The returned Jacobian w.r.t Velocity, with 
 *                    [Jv_theta(:), Jv_delta(:)].
 * @param [out] Jw    The returned Jacobian w.r.t Angular-Velocity, with 
 *                    [Jw_theta(:), Jw_delta(:)].
 */
void calcSingleWithRigidSegmentJacobian(const ConfigSpc &q, kfloat Lr,
        Eigen::Matrix<kfloat, 3, 2>& Jv, Eigen::Matrix<kfloat, 3, 2>& Jw);


/**
 * @brief Calculate the Jabobian of a single segment tha has a variable length
 *        and followed by a rigid segment, w.r.t Velocity and Angular-Velocity.
 *
 * @param [in] L      The length of the segment.
 * @param [in] theta  The bending angle of the segment.
 * @param [in] delta  The bending direction of the segment.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] Jv    The returned Jacobian w.r.t Velocity, with 
 *                    [Jv_theta(:), Jv_delta(:), Jv_L(:)].
 * @param [out] Jw    The returned Jacobian w.r.t Angular-Velocity, with 
 *                    [Jw_theta(:), Jw_delta(:), Jw_L(:)].
 */
void calcVariableLengthWithRigidSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta, kfloat Lr,
        Eigen::Matrix<kfloat, 3, 3>& Jv, Eigen::Matrix<kfloat, 3, 3>& Jw);


/**
 * @brief Calculate the Jabobian of a single segment tha has a variable length
 *        and followed by a rigid segment, w.r.t Velocity and Angular-Velocity.
 * 
 * @param [in] q      A ConfiSpc object.
 * @param [in] Lr     The length of the rigid segment.
 * @param [out] Jv    The returned Jacobian w.r.t Velocity, with 
 *                    [Jv_theta(:), Jv_delta(:), Jv_L(:)].
 * @param [out] Jw    The returned Jacobian w.r.t Angular-Velocity, with 
 *                    [Jw_theta(:), Jw_delta(:), Jw_L(:)].
 */
void calcVariableLengthWithRigidSegmentJacobian(const ConfigSpc &q, kfloat Lr,
        Eigen::Matrix<kfloat, 3, 3>& Jv, Eigen::Matrix<kfloat, 3, 3>& Jw);

}} // mmath::continuum
#endif // LIB_MATH_DCONTINUUM_POSE_H_LF