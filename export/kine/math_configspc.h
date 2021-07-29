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
#ifndef LIB_MATH_CONFIGSPC_H_LF
#define LIB_MATH_CONFIGSPC_H_LF

namespace mmath{

/** 
 * @brief A class that store the configuration of each continuum segment.
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

} // mmath
#endif // LIB_MATH_CONFIGSPC_H_LF