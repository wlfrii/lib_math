/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		drotation.h
 * 
 * @brief 		Interfaces for calculating derivatives of rotation matrix.
 * 
 * @author		Longfei Wang
 * 
 * @date		2022/06/27
 * 
 * @license		MIT
 * 
 * Copyright (C) 2019-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * 2022/06/27  Complete the file.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_DROTATION_H_LF
#define LIB_MATH_DROTATION_H_LF
#include <Eigen/Dense>
#include <type_traits>
#include <cmath>
#include "rotation.h"
#include "skew.h"

namespace mmath{

/**
 * @brief Return the derivatives of rotation matrix with rotated by x-axis by
 * radian.
 * 
 * @tparam T     Type of return matrix.
 * @tparam T1    Type of input value. 
 * @param radian The angle value described by radian.
 * @return  A rotation matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> dRotByX(const T1 radian)
{
	if (!std::is_arithmetic<T1>::value)
		std::abort();

    Eigen::Vector3d x(1, 0, 0);
    return skewSymmetric<T>(x) * rotByX<T>(radian);
}


/**
 * @brief Return the derivatives of rotation matrix with rotated by x-axis by
 * radian.
 * 
 * @tparam T     Type of return matrix.
 * @tparam T1    Type of input value. 
 * @param radian The angle value described by radian.
 * @return  A rotation matrix.
 */
template<typename T1 = double>
Eigen::Matrix3f dRotByXf(const T1 radian)
{
    return dRotByX<float, T1>(radian);
}


/**
 * @brief Return the derivatives of rotation matrix with rotated by y-axis by
 * radian.
 * 
 * @tparam T     Type of return matrix.
 * @tparam T1    Type of input value. 
 * @param radian The angle value described by radian.
 * @return  A rotation matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> dRotByY(const T1 radian)
{
	if (!std::is_arithmetic<T1>::value)
        std::abort();

    Eigen::Vector3d y(0, 1, 0);
    return skewSymmetric<T>(y) * rotByY<T>(radian);
}


/**
 * @brief Return the derivatives of rotation matrix with rotated by y-axis by
 * radian.
 * 
 * @tparam T     Type of return matrix.
 * @tparam T1    Type of input value. 
 * @param radian The angle value described by radian.
 * @return  A rotation matrix.
 */
template<typename T1 = double>
Eigen::Matrix3f dRotByYf(const T1 radian)
{
    return dRotByY<float, T1>(radian);
}


/**
 * @brief Return the derivatives of rotation matrix with rotated by z-axis by
 * radian.
 * 
 * @tparam T     Type of return matrix.
 * @tparam T1    Type of input value. 
 * @param radian The angle value described by radian.
 * @return  A rotation matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> dRotByZ(const T1 radian)
{
	if (!std::is_arithmetic<T1>::value)
		std::abort();

    Eigen::Vector3d z(0, 0, 1);
    return skewSymmetric<T>(z) * rotByZ<T>(radian);
}


/**
 * @brief Return the derivatives of rotation matrix with rotated by z-axis by
 * radian.
 * 
 * @tparam T1    Type of input value. 
 * @param radian The angle value described by radian.
 * @return  A rotation matrix.
 */
template<typename T1 = double>
Eigen::Matrix3f dRotByZf(const T1 radian)
{
    return dRotByZ<float, T1>(radian);
}

} // mmath
#endif // LIB_MATH_DROTATION_H_LF
