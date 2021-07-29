/**=====================================================================
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		math_rotation.h 
 * 
 * @biref 		Design some interfaces for calculating rotation matrix.
 * 
 * @author		Longfei Wang
 * 
 * @version		1.0.0
 * 
 * @data		2019/12/14
 * 
 * @license		
 * 
 * Copyright (C) 2019 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 *  Change History :                                  
 *  <Date>     | <Version> |   <Author>    | <Description>       
 * --------------------------------------------------------------------
 *  2019/12/14 | 1.0.0     | Longfei Wang  | Create the file          
 *====================================================================*/
#ifndef LIB_MATH_ROTATION_H_LF
#define LIB_MATH_ROTATION_H_LF
#include <Eigen/Dense>
#include <type_traits>
#include <cmath>

namespace mmath{

/** @brief Return a rotation matrix with rotated by x-axis by radian.
 */
template<typename T = float, typename T1 = double>
Eigen::Matrix<T, 3, 3> rotByX(const T1 radian)
{
	if (!std::is_arithmetic<T1>::value)
		std::abort();

	Eigen::Matrix<T, 3, 3> rot;
	rot << static_cast<T>(1), static_cast<T>(0), static_cast<T>(0),
		static_cast<T>(0), static_cast<T>(cos(radian)), static_cast<T>(-sin(radian)),
		static_cast<T>(0), static_cast<T>(sin(radian)), static_cast<T>(cos(radian));
	return rot;
}

/** @brief Return a rotation matrix with rotated by y-axis by radian.
 */
template<typename T = float, typename T1 = double>
Eigen::Matrix<T, 3, 3> rotByY(const T1 radian)
{
	if (!std::is_arithmetic<T1>::value)
		std::abort();

	Eigen::Matrix<T, 3, 3> rot;
	rot << static_cast<T>(cos(radian)), static_cast<T>(0), static_cast<T>(sin(radian)),
		static_cast<T>(0), static_cast<T>(1), static_cast<T>(0),
		static_cast<T>(-sin(radian)), static_cast<T>(0), static_cast<T>(cos(radian));
	return rot;
}


/** @brief Return a rotation matrix with rotated by z-axis by radian.
 */
template<typename T = float, typename T1 = double>
Eigen::Matrix<T, 3, 3> rotByZ(const T1 radian)
{
	if (!std::is_arithmetic<T1>::value)
		std::abort();

	Eigen::Matrix<T, 3, 3> rot;
	rot << static_cast<T>(cos(radian)), static_cast<T>(-sin(radian)), static_cast<T>(0),
		static_cast<T>(sin(radian)), static_cast<T>(cos(radian)), static_cast<T>(0),
		static_cast<T>(0), static_cast<T>(0), static_cast<T>(1);
	return rot;
}


/** @brief Return the rotation matrix based on the direction vector z.
*/
Eigen::Matrix3f createRotMatByVecZ(Eigen::Vector3f z);


} // mmath
#endif // LIB_MATH_ROTATION_H_LF