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
 * @date		2020/07/04
 * 
 * @license		MIT
 * 
 * Copyright (C) 2019-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * #v1.1 Complete kinematics related utilities.
 * #v1.2 Templated most of the interfaces.
 * #v1.3 Fixed several bug and integrate timer.
 * #v1.4 All the comments are updated.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_LIB_LF
#define LIB_MATH_LIB_LF

/** Precision */
#include "lib_math/math_precision.h"

/** Tiny utilities */
#include "lib_math/util/angle.h"
#include "lib_math/util/linspace.h"

/** Matrix related utilities */
#include "lib_math/matrix/mat.h"
#include "lib_math/matrix/rotation.h"
#include "lib_math/matrix/skew.h"
#include "lib_math/matrix/drotation.h"

/** Kinematics related utilities */
#include "lib_math/kine/pose.h"
#include "lib_math/kine/continuum_configspc.h"
#include "lib_math/kine/continuum_pose.h"
#include "lib_math/kine/dcontinuum_pose.h"

/** Curve related utilities */
#include "lib_math/curve/line_2d.h"
#include "lib_math/curve/gauss_curve_2d.h"


/** Include some useful function for time counting. <p>
 * Although timer counting is not really realted to math, this file contains <p>
 * several pretty useful function for test the time comsumption. */
#include "lib_math/timer/timer.h"


/** Camera projection */
#include "lib_math/cam/camera_projector.h"

// Some explicit template class
namespace mmath {

using Linef = Line<float>;
using GaussianCurvef = GaussianCurve<float>;

}

#endif // LIB_MATH_LIB_LF
