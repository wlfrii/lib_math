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
#ifndef LIB_MATH_MAT_H_LF
#define LIB_MATH_MAT_H_LF
#include <Eigen/Dense>

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

} // math
#endif // LIB_MATH_MAT_H_LF
