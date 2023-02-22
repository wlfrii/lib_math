/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		skew.h 
 * 
 * @brief 		Design some interfaces for matrix/vector operation.
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
 * 2022/06/06 Templated the functions.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_SKEW_H_LF
#define LIB_MATH_SKEW_H_LF
#include <Eigen/Dense>

namespace mmath{

/**
 * @brief Return the skew-symmetric matrix based on the input vector.
 * 
 * @tparam T  Type of return matrix.
 * @tparam T1  Type of input vector. 
 * @param vec A 3D column vector.
 * @return A skew symmetric matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> skewSymmetric(Eigen::Vector<T1, 3> &vec)
{
    Eigen::Matrix<T, 3, 3> mat;
	mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
	return mat;
}


/**
 * @brief Return the skew-symmetric matrix based on the input vector.
 * 
 * @tparam T  Type of return matrix.
 * @tparam T1  Type of input vector.
 * @param vec A 3D row vector.
 * @return A skew symmetric matrix.
 */
template<typename T = double, typename T1 = double>
Eigen::Matrix<T, 3, 3> skewSymmetric(Eigen::RowVector<T1, 3> &vec)
{
	return skewSymmetric<T>(vec.transpose());
}


/**
 * @brief Return the skew-symmetric matrix based on the input vector.
 * 
 * @tparam T  Type of input vector.
 * @param vec A 3D column vector.
 * @return A skew symmetric matrix.
 */
template<typename T = double>
Eigen::Matrix3f skewSymmetricf(Eigen::Vector<T, 3> &vec)
{
	return skewSymmetric<float>(vec);
}


/**
 * @brief Return the skew-symmetric matrix based on the input vector.
 * 
 * @tparam T  Type of input vector.
 * @param vec A 3D row vector.
 * @return A skew symmetric matrix.
 */
template<typename T = double>
Eigen::Matrix3f skewSymmetricf(Eigen::RowVector<T, 3> &vec)
{
	return skewSymmetric<float>(vec);
}


} // mmath
#endif // LIB_MATH_SKEW_H_LF
