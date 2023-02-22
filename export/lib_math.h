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
#include "matrix/drotation.h"

// Kinematics related utilities
#include "kine/kine_precision.h"
#include "kine/pose.h"
#include "kine/continuum_configspc.h"
#include "kine/continuum_pose.h"
#include "kine/dcontinuum_pose.h"

// Curve related utilities
#include "curve/line_2d.h"
#include "curve/gauss_curve_2d.h"


// Include some useful function for time counting.
// Although timer counting is not really realted to math, this file contains
// several pretty useful function for test the time comsumption.
#include "timer/timer.h"


// Some explicit template class
namespace mmath {

using Linef = Line<float>;
using GaussianCurvef = GaussianCurve<float>;

}

#endif // LIB_MATH_LIB_LF
