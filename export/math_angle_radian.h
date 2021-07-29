/**=====================================================================
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		math_angle_radian.h 
 * 
 * @biref 		Design some interfaces for angle and radian conversion.
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
#ifndef LIB_MATH_ANGLE_RADIAN_H_LF
#define LIB_MATH_ANGLE_RADIAN_H_LF
#include <type_traits>
#include <cstdlib>

namespace mmath{

const double PI = 3.14159265358979323846;

/** @brief Convert the angle with unit of degree to angle with unit of radian.
 */
template<typename T1 = double, typename T2 = double>
inline T1 deg2rad(const T2 degree)
{
	if (!std::is_arithmetic<T2>::value)
		std::abort();

	return static_cast<T1>(degree / 180.0 * PI);
}

/** @brief Convert the angle with unit of radian to angle with unit of degree.
 */
template<typename T1 = double, typename T2 = double>
inline T1 rad2deg(const T2 radian)
{
	if (!std::is_arithmetic<T2>::value)
		std::abort();

	return static_cast<T1>(radian / PI * 180.0);
}

} // mmath
#endif // LIB_MATH_ANGLE_RADIAN_H_LF