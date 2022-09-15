/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		pose.h 
 * 
 * @brief 		Define a description for 3D rigid transformation.
 * 
 * @author		Longfei Wang
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
 * 2022/06/06 Rename the file as pose and templated this class.
 * 2022/06/27 Consider the usage of this class are not sensitive to precision,
 * remove the templated-class-type, then controlling the precesion by
 * LIB_MATH_KINE_DOUBLE.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_POSE_H_LF
#define LIB_MATH_POSE_H_LF
#include <Eigen/Dense>
#include "kine_precision.h"

namespace mmath{

/** 
 * @brief A class designed to describe the transformation (spatial pose).
 * This class can furtherly simplify the calculation of pose-trasform.
 * There are two members include in this class:
 *   R	--  denotes the rotation or orientation.
 *   t  --  denotes the translation or position.
 */
class Pose
{
public:
    /**
     * @brief Construct a new Pose object.
     * 
     * @tparam Tp1 The type of input matrix.
     * @param T    The transform matrix.
     */
    template<typename Tp1 = kfloat>
    explicit Pose(const Eigen::Matrix<Tp1, 4, 4>& T);


    /**
     * @brief Construct a new Pose object.
     * 
     * @tparam Tp1  The type of rotation matrix.
     * @tparam Tp2  The type of translation vector.
     * @param R  The rotation matrix.
     * @param t  The translation vector.
     */
    template<typename Tp1 = kfloat, typename Tp2 = kfloat>
    explicit Pose(const Eigen::Matrix<Tp1, 3, 3>& R =
            Eigen::Matrix<Tp1, 3, 3>::Identity(),
                  const Eigen::Vector<Tp2, 3>& t = {0, 0, 0});
    

    /**
     * @brief Construct a new Pose object.
     * 
     * @tparam Tp1  The type of inputs.
     * @param tx  The x value of translation vector.
     * @param ty  The y value of translation vector.
     * @param tz  The z value of translation vector.
     */
    template<typename Tp1 = kfloat>
    explicit Pose(Tp1 tx, Tp1 ty, Tp1 tz);
    
    
    /**
     * @brief Construct a new Pose object
     * 
     * @tparam Tp1  The type of input array.
     * @param data  The input array[16]
     * @param is_row_fisrt  Flag of row first/column first.
     */
    template<typename Tp1 = kfloat>
    explicit Pose(Tp1 data[16], bool is_row_fisrt = true);


    /**
     * @brief operator =
     * @param pose
     * @return
     */
    Pose& operator= (const Pose& pose);

    /**
     * @brief operator *
     * @param pose
     * @return
     */
    Pose  operator* (const Pose& pose);
    const Pose  operator* (const Pose& pose);

    /**
     * @brief operator *=
     * @param pose
     * @return
     */
    Pose& operator*= (const Pose& pose);


    /**
     * @brief Transform a given point or vector
     * @param p 
     * @return Eigen::Vector<kfloat, 3> 
     */
    Eigen::Vector<kfloat, 3> operator*(const Eigen::Vector<kfloat, 3>& p);


    /**
     * @brief Return quaterion.
     * 
     * @return Eigen::Quaternion<kfloat>
     */
    Eigen::Quaternion<kfloat> q() const;


    /**
     * @brief Return transform matrix.
     * 
     * @return Eigen::Matrix<kfloat, 4, 4>
     */
    Eigen::Matrix<kfloat, 4, 4> T() const;


	/**
     * @brief Return the inverse of the Pose.
	 * 
     * @return Pose object
	 */
    Pose inverse();


    /**
     * @brief Add a increment.
     * @param dq  The increment of rotation/orientation
     * @param dt  The increament of translation/position
     * @sa decrease()
     */
    void increase(const Eigen::Quaternion<kfloat>& dq,
                         const Eigen::Vector<kfloat, 3>& dt);


    /**
     * @brief Minus a increment.
     * @param dq  The increment of rotation/orientation
     * @param dt  The increament of translation/position
     * @sa increase()
     */
    void decrease(const Eigen::Quaternion<kfloat>& dq,
                         const Eigen::Vector<kfloat, 3>& dt);


	/**
     * @brief Return the Pose info for print/std::out.
	 * 
	 * @return char* 
	 */
    char *info() const;


    Eigen::Matrix<kfloat, 3, 3> R; // The rotation/orientation matrix
    Eigen::Vector<kfloat, 3>    t; // The translation/position vector
};


/* ------------------------------------------------------------------- */
/*                          Pose Implementation                        */
/* ------------------------------------------------------------------- */

template<typename Tp1>
Pose::Pose(const Eigen::Matrix<Tp1, 4, 4> &T)
    : t(Eigen::Vector<kfloat, 3>(T(0, 3), T(1, 3), T(2, 3)))
{
    R << T(0, 0), T(0, 1), T(0, 2),
            T(1, 0), T(1, 1), T(1, 2),
            T(2, 0), T(2, 1), T(2, 2);
}


template<typename Tp1, typename Tp2>
Pose::Pose(const Eigen::Matrix<Tp1, 3, 3> &R, const Eigen::Vector<Tp2, 3> &t)
    : t(Eigen::Vector<kfloat, 3>(t(0), t(1), t(2)))
{
    this->R << R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2);
}


template<typename Tp1>
Pose::Pose(Tp1 tx, Tp1 ty, Tp1 tz)
    : R(Eigen::Matrix<kfloat, 3, 3>::Identity())
    , t(Eigen::Vector<kfloat, 3>(tx, ty, tz))
{
}


template<typename Tp1>
Pose::Pose(Tp1 data[], bool is_row_fisrt)
{
    if (is_row_fisrt) {
        R << data[0], data[1], data[2],
                data[4], data[5], data[6],
                data[8], data[9], data[10];
        t = Eigen::Vector<kfloat, 3>(data[3], data[7], data[11]);
    }
    else{
        R << data[0], data[4], data[8],
                data[1], data[5], data[9],
                data[2], data[6], data[10];
        t = Eigen::Vector<kfloat, 3>(data[12], data[13], data[14]);
    }
}

} // mmath
#endif // LIB_MATH_RT_H_LF
