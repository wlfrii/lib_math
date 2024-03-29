/**--------------------------------------------------------------------
 *
 *   				   Mathematics extension library
 *
 * Description:
 * This file is part of lib_math. You can redistribute it and or modify
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 *
 * @file 		math_precision.h
 *
 * @brief 		Control the precision for kinematics-related interfaces.
 *
 * @author		Longfei Wang
 *
 * @date		2022/06/27
 *
 * @license     MIT
 *
 * Copyright (C) 2019-Now Longfei Wang.
 *
 * --------------------------------------------------------------------
 * Change History:
 *
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_PRECISION_H_LF
#define LIB_MATH_PRECISION_H_LF

namespace mmath{

/** To use double precision when using ./kine, define this
 * LIB_MATH_USE_DOUBLE
 * macro in your preprocessing list before include <lib_math/lib_math.h>.
 */
#ifdef LIB_MATH_USE_DOUBLE
using kfloat = double;
#else
using kfloat = float;
#endif

} //namespace::mmath
#endif // LIB_MATH_KINE_PRECISION_H_LF
