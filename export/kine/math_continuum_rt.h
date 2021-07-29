/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		math_continuum_rt.h 
 * 
 * @brief 		Design some interfaces for continuum segment kinematics.
 * 
 * @author		Longfei Wang
 * 
 * @version		1.0.0
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
#include "math_rt.h"
#include "math_configspc.h"

namespace mmath{

/** 
 * @brief Calculation the rotation and position of the end frame of single 
* continuum segment with respect to its base frame.
*
* @param L      The length of the segment.
* @param theta  The bending angle of the segment.
* @param delta  The bending direction of the segment.
* @return rt The rotation and position matrix of single continuum segment with 
* respect to its base frame.
*/
void calcSingleSegmentRT(float L, float theta, float delta, RT& rt);
/**
 * @brief Override based on 'void calcSingleSegmentRT()'
 */
RT   calcSingleSegmentRT(int L, int theta, int delta);
/**
 * @brief Override based on 'void calcSingleSegmentRT()'
 */
void calcSingleSegmentRT(const ConfigSpc& q, RT& rt);
/**
 * @brief Override based on 'void calcSingleSegmentRT()'
 */
RT   calcSingleSegmentRT(const ConfigSpc& q);



/** 
 * @brief Calculation the rotation and position of the end frame of single 
* continuum segment with a rigid segment, with respect to its base frame.
* @param L      The length of the segment.
* @param Lr     The length of the rigid segment.
* @param theta  The bending angle of the segment.
* @param delta  The bending direction of the segment.
* @return rt The rotation and position matrix of single continuum segment with 
* respect to its base frame.
*/
void calcSingleWithRigidSegmentRT(float L, float theta, float delta, float Lr, RT& rt);
/**
 * @brief Override based on 'void calcSingleWithRigidSegmentRT()'
 */
RT   calcSingleWithRigidSegmentRT(float L, float theta, float delta, float Lr);
/**
 * @brief Override based on 'void calcSingleWithRigidSegmentRT()'
 */
void calcSingleWithRigidSegmentRT(const ConfigSpc& q, float Lr, RT& rt);
/**
 * @brief Override based on 'void calcSingleWithRigidSegmentRT()'
 */
RT   calcSingleWithRigidSegmentRT(const ConfigSpc& q, float Lr);

} // mmath
#endif // LIB_MATH_CONTINUUM_RT_H_LF