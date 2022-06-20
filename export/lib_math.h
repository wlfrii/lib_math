/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is header file of lib_math. You can redistribute it and or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		lib_math.h 
 * 
 * @brief 		The header file for the library
 * 
 * @author		Longfei Wang
 * 
 * @version		1.2.0
 * 
 * @date		2020/07/04
 * 
 * @license		
 * 
 * Copyright (C) 2019 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * #v1.1 Complete kinematics related utilities.
 * #v1.2 Templated most of the interfaces.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_LIB_LF
#define LIB_MATH_LIB_LF

// Tiny utilities
#include "util/angle.h"
#include "util/linspace.h"

// Matrix related utilities
#include "matrix/mat.h"
#include "matrix/rotation.h"
#include "matrix/skew.h"

// Kinematics related utilities
#include "kine/pose.h"
#include "kine/continuum_configspc.h"
#include "kine/continuum_pose.h"

// Curve related utilities
#include "curve/line_2d.h"
#include "curve/gauss_curve_2d.h"


// Some explicit template class
namespace mmath {

using Posef = Pose<float>;

using Linef = Line<float>;
using GaussianCurvef = GaussianCurve<float>;

namespace continuum {

using ConfigSpcf = ConfigSpc<float>;

}}

#endif // LIB_MATH_LIB_LF
