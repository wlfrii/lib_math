/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		math_configspc.h 
 * 
 * @brief 		Define a cofiguration space for single continuum segment.
 * 
 * @author		Longfei Wang
 * 
 * @version		1.0.0
 * 
 * @date		2020/03/38
 * 
 * @license		
 * 
 * Copyright (C) 2019 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_CONTINUUM_CONFIGSPC_H_LF
#define LIB_MATH_CONTINUUM_CONFIGSPC_H_LF
#include <vector>

namespace mmath{
namespace continuum{
/** 
 * @brief A class stores the configuration of each continuum segment.
 * 
 * There are four member included in this class.
 * theta    --  the bending angle
 * delta    --  the bending direction
 * length   --  the length of the segment
 * is_bend  --  sepcifiy the bending/rigid segment, the default value is
 *              bending segment. If rigid segment is set, the kinematics
 *              of this segment will only consider a rotation alone z.
 */ 
class ConfigSpc
{
public:
	ConfigSpc(float theta, float delta, float len, bool bend = false)
		: theta(theta), delta(delta), length(len), is_bend(bend)
	{}

	ConfigSpc() : theta(0), delta(0), length(0), is_bend(false)
	{}

	float theta;
	float delta;
	float length;
	bool is_bend;
};

/**
 * @brief Configurations of multi continuum.
 */
using ConfigSpcs = std::vector<ConfigSpc>;


/**
 * @brief A class stores the configuration values based on joint space of two segment continuum.
 */
class JointSpc
{
public:
	JointSpc()
		: L(0), phi(0)
		, theta1(0), delta1(0)
		, theta2(0), delta2(0)
	{}

	float L;
	float phi;
	float theta1;
	float delta1;
	float theta2;
	float delta2;
};

}} // mmath::continuum
#endif // LIB_MATH_CONFIGSPC_H_LF