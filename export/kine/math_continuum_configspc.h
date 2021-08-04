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
 * 2021.8.3 Add ConfigSpcs, and task space.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_CONTINUUM_CONFIGSPC_H_LF
#define LIB_MATH_CONTINUUM_CONFIGSPC_H_LF
#include <array>
#include <stdint.h>
#include "math_rt.h"

namespace mmath{
namespace continuum{


enum SegmentType{
	SEGMENT_TROCAR_TO_SEG1BASE,
	SEGMENT_SEG1BASE_TO_SEG1END,
	SEGMENT_SEG1END_TO_SEG2BASE,
	SEGMENT_SEG2BASE_TO_SEG2END,
	SEGMENT_SEG2END_TO_GRIPPER,
	SEGMENT_TYPE_NONE
};


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
	ConfigSpc(SegmentType type = SEGMENT_TYPE_NONE, float theta = 0, 
		float delta = 0, float len = 0, bool bend = false);
	
	void clear();

	SegmentType type;
	float theta;
	float delta;
	float length;
	bool is_bend;
};

/**
 * @brief Configurations of twe-segment continuum robot.
 */
const int MAX_SECTION_COUNT = 5;
template <uint8_t N = MAX_SECTION_COUNT>
class ConfigSpcs
{ 	
public:
	ConfigSpcs() : _count(0)
	{}

	ConfigSpc& operator[] (uint8_t idx)
	{
		return _config_spcs[idx];
	}

	const ConfigSpc& operator[] (uint8_t idx) const
	{
		return _config_spcs[idx];
	}

	bool push_back(const ConfigSpc& config_spc)
	{
		if(_count >= N){
			return false;
		}
		_config_spcs[_count++] = config_spc;
		return true;
	}
	
	int size() const
	{
		return _count;
	}

	void clear()
	{
		for(int i = 0; i < count; i++){
        	_config_spcs[i].clear();
    	}
	}

private:
	ConfigSpc	_config_spcs[N];
	uint8_t 	_count;
};

/**
 * @brief A class stores the RT of each segment based on task space of
 * continuum robot.
 */
template<uint8_t N = 5>
class TaskSpc
{
public:
	TaskSpc() {}

	void clear();

	//!< Stores each segment transformation for the whole rotot
	std::array<RT, SEGMENT_TYPE_COUNT> T_segments;

	//!< The transformation form robot end to base
	RT T_trocar_to_gripper;

private:
	RT rt[N];

};

}} // mmath::continuum
#endif // LIB_MATH_CONFIGSPC_H_LF