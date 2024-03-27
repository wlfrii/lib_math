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
 * @remark This is the base of overloaded functions.
 * 
 * @tparam T     The type of value stored in returned vector, should be the 
 *               arithmatic class.
 * @param [in] start The start value.
 * @param [in] end   The end value.
 * @param [in] num 	 The number of values in the range [start, end].
 * @param [out] output A vector stores the linspaced values, where the values
 * meet the condition, start <= values <= end.
 */
template<typename T = double>
void linspaceN(double start, double end, int num, std::vector<T> &output) {
	if (!std::is_arithmetic<T>::value){
		std::abort();
	}
	assert(start < end);
	output.clear();

	if (num == 1) {
		output.push_back(static_cast<T>(start));
		return;
	}
	else if (num == 2) {
		output.push_back(static_cast<T>(start));
		output.push_back(static_cast<T>(end));
		return;
	}

	float step = static_cast<T>(end - start) / static_cast<T>(num - 1);
	for (int i = 0; i < num-1; ++i) {
		output.push_back(static_cast<T>(start + i * step));
	}
	output.push_back(static_cast<T>(end));
}


/**
 * @brief Return a vector contain 'num' numeral between 'start' and 'end'.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @tparam T     The type of value stored in returned vector, should be the 
 *               arithmatic class.
 * @param [in] start The start value.
 * @param [in] end   The end value.
 * @param [in] num 	 The number of values in the range [start, end].
 * 
 * @return A vector stores the linspaced values, where the values
 * meet the condition, start <= values <= end.
 */
template<typename T = double>
std::vector<T> linspaceN(double start, double end, int num) {
	std::vector<T> output;
	linspaceN(start, end, num, output);
	return output;
}


/**
 * @brief Return an arithmetic sequence vector with a difference of 'gap'.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @tparam T     The type of value stored in returned vector, should be the 
 *               arithmatic class.
 * @param [in] start  The start value.
 * @param [in] gap    The gap between adjacent values.
 * @param [in] end    The end value.
 * @param [out] output A vector stores the linspaced values = start : gap : end.
 */
template<typename T = double>
void linspace(double start, double gap, double end, std::vector<T> &output)
{
	if (!std::is_arithmetic<T>::value){
		std::abort();
	}
	assert(start < end);

	output.clear();
	while(start - end < 0){
		output.push_back(static_cast<T>(start));
		start += gap;
	}
	output.push_back(static_cast<T>(end));
}


/**
 * @brief Return an arithmetic sequence vector with a difference of 'gap'.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the base function only in what argument(s) it accepts and the returned
 * value. To improve efficiency, the function with the void-returned value is 
 * suggested.
 * 
 * @tparam T     The type of value stored in returned vector, should be the 
 *               arithmatic class.
 * @param [in] start  The start value.
 * @param [in] gap    The gap between adjacent values.
 * @param [in] end    The end value.
 * 
 * @return output A vector stores the linspaced values = start : gap : end.
 */
template<typename T = double>
std::vector<T> linspace(double start, double gap, double end)
{
	std::vector<T> output;
	linspace(start, gap, end, output);
	return output;
}

} // mmath
#endif // LIB_MATH_LINSPACE_H_LF
