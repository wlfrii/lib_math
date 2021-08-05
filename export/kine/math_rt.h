/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		math_rt.h 
 * 
 * @brief 		Define a description for 3D transformation.
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
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_RT_H_LF
#define LIB_MATH_RT_H_LF
#include <Eigen/Dense>

namespace mmath{

using EMat3f = Eigen::Matrix3f;
using EMat4f = Eigen::Matrix4f;
using EVec3f = Eigen::Vector3f;

/** 
 * @brief A class designed to describe the transformation.
 * This class can furtherly simplify the calculation of pose-trasform.
 * There are three members include in this class:
 *   R	--  denotes the rotation or orientation.
 *   t  --  denotes the translation or position.
 *   T  --  denotes the transformation, detemined by R and t.
 */
class RT
{
public:
	explicit RT(const EMat3f& R = EMat3f::Identity(), const EVec3f& t = {0, 0, 0});
	explicit RT(const EMat3f& R);
	explicit RT(float tx, float ty, float tz);
	explicit RT(float data[16], bool is_row_fisrt = true);

	RT& operator= (const RT& rt);
	RT  operator* (const RT& rt);
	RT& operator*= (const RT& rt);

	/**
	 * @brief Return the inverse of the RT.
	 * 
	 * @return RT 
	 */
    RT inverse();

	/**
	 * @brief Return the RT info for print/std::out.
	 * 
	 * @return char* 
	 */
	char* to_c_str() const;

	EMat3f R;
	EVec3f t;
	EMat4f T;
};

} // mmath
#endif // LIB_MATH_RT_H_LF