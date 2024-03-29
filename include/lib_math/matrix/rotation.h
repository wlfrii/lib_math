/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		rotation.h 
 * 
 * @brief 		Design some interfaces for calculating rotation matrix.
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
 * 2022/06/06  Complete the doxygen comments.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_ROTATION_H_LF
#define LIB_MATH_ROTATION_H_LF
#include <Eigen/Dense>
#include <type_traits>
#include <cmath>
#include "../util/angle.h"

namespace mmath{

/**
 * @brief Return a rotation matrix that rotated by x-axis by radian.
 * 
 * @tparam T     The arithmetic class type of return matrix.
 * @tparam T1    The arithmetic class type of input value. 
 * @param [in] radian The angle value described by radian.
 * 
 * @return  A rotation matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> rotByX(const T1 radian) {
	if (!std::is_arithmetic<T1>::value){
		std::abort();
	}
	Eigen::Matrix<T, 3, 3> rot;
	rot << static_cast<T>(1), static_cast<T>(0), static_cast<T>(0),
		static_cast<T>(0), static_cast<T>(cos(radian)), static_cast<T>(-sin(radian)),
		static_cast<T>(0), static_cast<T>(sin(radian)), static_cast<T>(cos(radian));
	return rot;
}


/**
 * @brief Return a rotation matrix that rotated by x-axis by radian.
 * 
 * @remark This is a partial explicity function of mmath::rotByX().
 * 
 * @tparam T1    The arithmetic class type of input value. 
 * @param [in] radian The angle value described by radian.
 * 
 * @return  A rotation matrix.
 * 
 * @see mmath::rotByX().
 */
template<typename T1 = double>
Eigen::Matrix3f rotByXf(const T1 radian) {
    return rotByX<float, T1>(radian);
}


/**
 * @brief Return a rotation matrix that rotated by y-axis by radian.
 * 
 * @tparam T     The arithmetic class type of return matrix.
 * @tparam T1    The arithmetic class type of input value. 
 * @param [in] radian The angle value described by radian.
 * 
 * @return  A rotation matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> rotByY(const T1 radian) {
	if (!std::is_arithmetic<T1>::value){
		std::abort();
	}
	Eigen::Matrix<T, 3, 3> rot;
	rot << static_cast<T>(cos(radian)), static_cast<T>(0), static_cast<T>(sin(radian)),
		static_cast<T>(0), static_cast<T>(1), static_cast<T>(0),
		static_cast<T>(-sin(radian)), static_cast<T>(0), static_cast<T>(cos(radian));
	return rot;
}


/**
 * @brief Return a rotation matrix that rotated by y-axis by radian.
 * 
 * @remark This is a partial explicity function of mmath::rotByY().
 * 
 * @tparam T1    The arithmetic class type of input value. 
 * @param [in] radian The angle value described by radian.
 * 
 * @return  A rotation matrix.
 * 
 * @see mmath::rotByY().
 */
template<typename T1 = double>
Eigen::Matrix3f rotByYf(const T1 radian) {
    return rotByY<float, T1>(radian);
}


/**
 * @brief Return a rotation matrix that rotated by z-axis by radian.
 * 
 * @tparam T     The arithmetic class type of return matrix.
 * @tparam T1    The arithmetic class type of input value. 
 * @param [in] radian The angle value described by radian.
 * 
 * @return  A rotation matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> rotByZ(const T1 radian) {
	if (!std::is_arithmetic<T1>::value){
		std::abort();
	}
	Eigen::Matrix<T, 3, 3> rot;
	rot << static_cast<T>(cos(radian)), static_cast<T>(-sin(radian)), static_cast<T>(0),
		static_cast<T>(sin(radian)), static_cast<T>(cos(radian)), static_cast<T>(0),
		static_cast<T>(0), static_cast<T>(0), static_cast<T>(1);
	return rot;
}


/**
 * @brief Return a rotation matrix that rotated by z-axis by radian.
 * 
 * @remark This is a partial explicity function of mmath::rotByZ().
 * 
 * @tparam T1    The arithmetic class type of input value. 
 * @param [in] radian The angle value described by radian.
 * 
 * @return  A rotation matrix.
 * 
 * @see mmath::rotByZ().
 */
template<typename T1 = double>
Eigen::Matrix3f rotByZf(const T1 radian) {
    return rotByZ<float, T1>(radian);
}


/**
 * @brief Create a rotation matrix by a 3D vector that be the z-axis.
 * 
 * @note A x-axis [1, 0, 0] is supposed to composed the rotation matrix.
 * 
 * @tparam T     The arithmetic class type of return matrix.
 * @tparam T1    The arithmetic class type of input value. 
 * @param [in] vec_z The 3D vector denotes z-axis
 * 
 * @return  A rotation matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> createRotMatByVecZ(const Eigen::Vector<T1, 3>& vec_z) {
	Eigen::Vector<T, 3> z(vec_z[0], vec_z[1], vec_z[2]);
	z /= z.norm();

	Eigen::Vector<T, 3> x(1, 0, 0);
	auto y = z.cross(x);
	y /= y.norm();
	x = y.cross(z);
	x /= x.norm();

	Eigen::Matrix<T, 3, 3> mat;
	mat.col(0) = x;
	mat.col(1) = y;
	mat.col(2) = z;
	return mat;
}

} // mmath
#endif // LIB_MATH_ROTATION_H_LF
