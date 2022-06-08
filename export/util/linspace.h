/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		linspace.h 
 * 
 * @brief 		Design some interfaces for matrix/vector operation.
 * 
 * @author		Longfei Wang
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
 * 2021/07/29 Complete the doxygen comments.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_LINSPACE_H_LF
#define LIB_MATH_LINSPACE_H_LF
#include <Eigen/Dense>
#include <type_traits>
#include <cmath>
#include <vector>

namespace mmath{

/**
 * @brief Return a vector contain 'num' numeral between 'start' and 'end'.
 * 
 * @tparam T     The type of value stored in returned vector.
 * @param start  The start value.
 * @param end  	 The end value.
 * @param num 	 The number of values in the range [start, end].
 * @param output The output values, start <= values <= end.
 * @return  A vector stores linspaced values. 
 */
template<typename T = double>
std::vector<T>& linspace(double start, double end, int num, std::vector<T> &output)
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
 * @tparam T     The type of value stored in returned vector.
 * @param start  The start value.
 * @param gap  	 The gap between adjacent values.
 * @param end  	 The end value.
 * @param output The output values, start : gap : end.
 * @return  A vector stores linspaced values. 
 */
template<typename T = double>
std::vector<T>& linspace(double start, double gap, double end, std::vector<T> &output)
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
#endif // LIB_MATH_LINSPACE_H_LF
