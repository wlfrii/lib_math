/**=====================================================================
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		math_utility.h 
 * 
 * @biref 		Design some interfaces for matrix/vector operation.
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
#ifndef LIB_MATH_UTILITY_H_LF
#define LIB_MATH_UTILITY_H_LF
#include <Eigen/Dense>
#include <type_traits>
#include <cmath>
#include <vector>

namespace mmath{


/** @brief Return the absoluted Matrix.
* M  represents the row of the matrix.
* N  represents the column of the matrix.
*/
template<typename T, int M, int N>
Eigen::Matrix<T, M, N> absMat(const Eigen::Matrix<T, M, N>& mat)
{
	Eigen::Matrix<T, M, N> abs_mat = mat;
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			abs_mat(i, j) = std::abs(abs_mat(i, j));
		}
	}
	return abs_mat;
}


/** @brief Return the skew-symmetric matrix based on the input vector.
*/
Eigen::Matrix3f skewSymmetric(Eigen::Vector3f &vec);
Eigen::Matrix3f skewSymmetric(Eigen::RowVector3f &vec);


/** @brief Return a vector contain 'num' numeral between 'start' and 'end'.
 */
template<typename T1, typename T2, typename T3>
std::vector<T3>& linespace(const T1 start, const T2 end, const int num, std::vector<T3> &output)
{
	if (!std::is_arithmetic<T1>::value || !std::is_arithmetic<T2>::value)
		std::abort();

	if (num == 1)
	{
		output.push_back(static_cast<T3>(start));
		return output;
	}
	else if (num == 2)
	{
		output.push_back(static_cast<T3>(start));
		output.push_back(static_cast<T3>(end));
		return output;
	}

	float step = (static_cast<T3>(end) - static_cast<T3>(start)) / static_cast<T3>(num - 1);
	for (int i = 0; i < num; ++i)
		output.push_back(static_cast<T3>(start + i * step));

	return output;
}


} // mmath
#endif // LIB_MATH_UTILITY_H_LF