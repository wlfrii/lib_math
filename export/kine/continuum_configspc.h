/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		continuum_configspc.h
 * 
 * @brief 		Define a cofiguration space for single continuum segment.
 * 
 * @author		Longfei Wang
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
#include <cstdint>
#include <cstdio>
#include "kine_precision.h"

namespace mmath{
namespace continuum{


/** 
 * @brief A class stores the configuration of each continuum segment.
 * 
 * There are four members included in this class.
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
    ConfigSpc(kfloat theta = 0, kfloat delta = 0, kfloat len = 0, bool bend = false)
        : theta(theta), delta(delta), length(len), is_bend(bend)
    {}
	

    /**
     * @brief Clear current configure value.
     */
    void clear()
    {
        *this = ConfigSpc();
    }


    /**
     * @brief Return a string a current configure value.
     */
    const char* info() const
    {
        static char info[64];
        sprintf(info, "theta:%f,delta:%f,length:%f,is_bend:%d",
                theta, delta, length, is_bend);
        return info;
    }

    kfloat theta;
    kfloat delta;
    kfloat length;
	bool is_bend;
};



}} // mmath::continuum
#endif // LIB_MATH_CONFIGSPC_H_LF
