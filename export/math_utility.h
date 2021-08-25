/**--------------------------------------------------------------------
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
 * @brief 		Design some interfaces for matrix/vector operation.
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
 * 2021.7.29 Complete the doxygen comments.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_UTILITY_H_LF
#define LIB_MATH_UTILITY_H_LF
#include <Eigen/Dense>
#include <type_traits>
#include <cmath>
#include <vector>

namespace mmath{

/**
 * @brief Return the absoluted Matrix.
 * 
 * @tparam T The type of the matrix, e.g, float, double
 * @tparam M Represents the row of the matrix.
 * @tparam N Represents the column of the matrix.
 * @param mat 
 * @return Eigen::Matrix<T, M, N> 
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


/**
 * @brief Return the skew-symmetric matrix based on the input vector.
 * 
 * @param vec The input 3D vector.
 * @return 	Eigen::Matrix3f 
 */
Eigen::Matrix3f skewSymmetric(Eigen::Vector3f &vec);
/**
 * @brief Return the skew-symmetric matrix based on the input vector.
 * 
 * @param vec The input 3D vector.
 * @return Eigen::Matrix3f 
 */
Eigen::Matrix3f skewSymmetric(Eigen::RowVector3f &vec);


/**
 * @brief Return a vector contain 'num' numeral between 'start' and 'end'.
 * 
 * @tparam T 
 * @param start  	The start value.
 * @param end  		The end value.
 * @param num 		The number of values in the range [start, end].
 * @param output 	The output values, start <= values <= end.
 * @return std::vector<T>& 
 */
template<typename T = double>
std::vector<T>& linespace(double start, double end, int num, std::vector<T> &output)
{
	if (!std::is_arithmetic<T>::value)
		std::abort();
	output.clear();

	if (num == 1)
	{
		output.push_back(static_cast<T>(start));
		return output;
	}
	else if (num == 2)
	{
		output.push_back(static_cast<T>(start));
		output.push_back(static_cast<T>(end));
		return output;
	}

	float step = static_cast<T>(end - start) / static_cast<T>(num - 1);
	for (int i = 0; i < num-1; ++i)
		output.push_back(static_cast<T>(start + i * step));
	output.push_back(static_cast<T>(end));

	return output;
}

/**
 * @brief Return a vector that the difference between each adjacent values is 'gap'
 * 
 * @tparam T 
 * @param start   	The start value.
 * @param gap  		The gap between adjacent values.
 * @param end  		The end value.
 * @param output 	The output values, start : gap : end.
 * @return std::vector<T>& 
 */
template<typename T = double>
std::vector<T>& linespace(double start, double gap, double end, std::vector<T> &output)
{
	if (!std::is_arithmetic<T>::value)
		std::abort();

	output.clear();
	while(abs(start - end) > 1e-7){
		output.push_back(static_cast<T>(start));
		start += gap;
	}
	output.push_back(static_cast<T>(end));
	
	return output;
}

} // mmath
#endif // LIB_MATH_UTILITY_H_LF