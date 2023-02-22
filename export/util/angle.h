/**--------------------------------------------------------------------
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
 * @brief 		Design some interfaces for angle and radian conversion.
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
 * 2021/07/29 Complete the doxygen comments.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_ANGLE_RADIAN_H_LF
#define LIB_MATH_ANGLE_RADIAN_H_LF
#include <type_traits>
#include <cstdlib>

namespace mmath{

const double PI = 3.14159265358979323846;

/**
 * @brief Convert the angle with unit of degree to angle with unit of radian.
 * 
 * @tparam T1 
 * @tparam T2 
 * @param degree The angle value described by degree.
 * @return T1 
 */
template<typename T1 = double, typename T2 = double>
inline T1 deg2rad(const T2 degree)
{
	if (!std::is_arithmetic<T2>::value)
		std::abort();

	return static_cast<T1>(degree / 180.0 * PI);
}


/**
 * @brief Convert the angle with unit of degree to angle with unit of radian.
 * 
 * @tparam T1 
 * @param degree The angle value described by degree.
 * @return float
 */
template<typename T1 = double>
inline float deg2radf(const T1 degree)
{
	return deg2rad<float, T1>(degree);
}


/**
 * @brief Convert the angle with unit of radian to angle with unit of degree.
 * 
 * @tparam T1 
 * @tparam T2 
 * @param radian The angle value described by radian.
 * @return T1 
 */
template<typename T1 = double, typename T2 = double>
inline T1 rad2deg(const T2 radian)
{
	if (!std::is_arithmetic<T2>::value)
		std::abort();

	return static_cast<T1>(radian / PI * 180.0);
}


/**
 * @brief Convert the angle with unit of radian to angle with unit of degree.
 * 
 * @tparam T1 
 * @param radian The angle value described by radian.
 * @return float
 */
template<typename T1 = double>
inline float rad2degf(const T1 degree)
{
	return rad2deg<float, T1>(degree);
}


} // mmath
#endif // LIB_MATH_ANGLE_RADIAN_H_LF
