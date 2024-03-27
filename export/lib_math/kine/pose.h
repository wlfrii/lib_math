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
 * @license		MIT
 * 
 * Copyright (C) 2019-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                
 * 
 * 2022/12/06 Add a function check the orthogonalization of rotation matrix.
 * 2022/06/27 Consider the usage of this class are not sensitive to precision,
 * remove the templated-class-type, then controlling the precesion by
 * LIB_MATH_KINE_DOUBLE.
 * 2022/06/06 Rename the file as pose and templated this class.
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_POSE_H_LF
#define LIB_MATH_POSE_H_LF
#include <Eigen/Dense>
#include <iostream>
#include "../math_precision.h"


namespace mmath{

/** 
 * @brief A class designed to describe the transformation (spatial pose).
 * 
 * @details
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
     * @tparam Tp1  The arithmetic class type of rotation matrix.
     * @tparam Tp2  The arithmetic class type of translation vector.
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
     * @remark This is an overloaded construction functions.
     * 
     * @tparam Tp1 The arithmetic class type of input matrix.
     * @param T    The transformation matrix.
     */
    template<typename Tp1 = kfloat>
    explicit Pose(const Eigen::Matrix<Tp1, 4, 4>& T);
    

    /**
     * @brief Construct a new Pose object.
     * 
     * @remark This is an overloaded construction functions.
     * 
     * @tparam Tp1  The arithmetic class type of inputs.
     * @param tx  The x value of translation vector.
     * @param ty  The y value of translation vector.
     * @param tz  The z value of translation vector.
     */
    template<typename Tp1 = kfloat>
    explicit Pose(Tp1 tx, Tp1 ty, Tp1 tz);
    
    
    /**
     * @brief Construct a new Pose object.
     * 
     * @remark This is an overloaded construction functions.
     * 
     * @tparam Tp1  The arithmetic class type of input array.
     * @param data  The input array[16]
     * @param is_row_fisrt  The flag to specified whether the given data is
     *                      row first/column first array, where true/1 specifies
     *                      row first arrar whereas false/0 specifies column
     *                      first.
     */
    template<typename Tp1 = kfloat>
    explicit Pose(Tp1 data[16], bool is_row_fisrt = true);


    /**
     * @brief Construct a new Pose object.
     * 
     * @remark This is an overloaded construction functions.
     * 
     * @tparam Tp1 The arithmetic class type of input array.
     * @param m11 T(1,1), where T denote a 4x4 transformation matrix.
     * @param m12 T(1,2), where T denote a 4x4 transformation matrix.
     * @param m13 T(1,3), where T denote a 4x4 transformation matrix.
     * @param m14 T(1,4), where T denote a 4x4 transformation matrix.
     * @param m21 T(2,1), where T denote a 4x4 transformation matrix.
     * @param m22 T(2,2), where T denote a 4x4 transformation matrix.
     * @param m23 T(2,3), where T denote a 4x4 transformation matrix.
     * @param m24 T(2,4), where T denote a 4x4 transformation matrix.
     * @param m31 T(3,1), where T denote a 4x4 transformation matrix.
     * @param m32 T(3,2), where T denote a 4x4 transformation matrix.
     * @param m33 T(3,3), where T denote a 4x4 transformation matrix.
     * @param m34 T(3,4), where T denote a 4x4 transformation matrix.
     * 
     * @note The last row of an transformation matrix is no needed, since it 
     * is always a [0, 0, 0, 1] row-vector for a transformation matrix.
     */
    template<typename Tp1 = kfloat>
    explicit Pose(Tp1 m11, Tp1 m12, Tp1 m13, Tp1 m14,
                  Tp1 m21, Tp1 m22, Tp1 m23, Tp1 m24,
                  Tp1 m31, Tp1 m32, Tp1 m33, Tp1 m34);


    /**
     * @brief Copy a pose from 4x4 matrix.
     * 
     * @tparam Tp1  The arithmetic class type of inputs.
     * @param [in] pose The pose with class Eigen::Matrix<Tp1, 4, 4>.
     * 
     * @return This object.
     */
    template<typename Tp1 = kfloat>
    Pose& operator= (const Eigen::Matrix<Tp1, 4, 4>& pose);


    /**
     * @brief Copy a pose from another Pose object.
     * 
     * @param [in] pose The another Pose object.
     * 
     * @return This object.
     */
    Pose& operator= (const Pose& pose);


    /**
     * @brief Multiplication with another pose.
     * 
     * @details This is a right multiplication and with return a new Pose
     * object, which is "new_pose = (*this) * pose".
     * 
     * @param [in] pose The another Pose object.
     * 
     * @return A new Pose object.
     */
    Pose  operator* (const Pose& pose) const;
    

    /**
     * @brief Multiplication with another pose.
     * 
     * @details This is a right multiplication, which is
     * "(*this) = (*this) * pose".
     * 
     * @param [in] pose The another Pose object.
     * 
     * @return This object.
     */
    Pose& operator*= (const Pose& pose);


    /**
     * @brief Transform a given point or vector.
     * 
     * @details This is a "matrix * vection" multiplication, which is 
     * "ret = this->R * p + this.t".
     * 
     * @param [in] p 
     * 
     * @return A new point/vector, an object of class Eigen::Vector<kfloat, 3>.
     */
    Eigen::Vector<kfloat, 3> operator*(const Eigen::Vector<kfloat, 3>& p);


    /**
     * @brief For std::cout operation to print the member information.
     * 
     * @note This function is a friend function of class Pose.
     * 
     * @param [in] os The std::ostream object.
     * @param [in] pose The Pose opject.
     * 
     * @return The std::ostream object.
     */
    friend std::ostream& operator<< (std::ostream& os, const Pose& pose);


    /**
     * @brief Return a rotation represented by a quaternion.
     * 
     * @return A quaternion, an object of class Eigen::Quaternion<kfloat>.
     */
    Eigen::Quaternion<kfloat> q() const;


    /**
     * @brief Return the transform matrix.
     * 
     * @return A transform, an object of class Eigen::Matrix<kfloat, 4, 4>.
     */
    Eigen::Matrix<kfloat, 4, 4> T() const;


	/**
     * @brief Return the inverse of the Pose.
	 * 
     * @return A new Pose object.
	 */
    Pose inverse() const;


    /**
     * @brief Check whether the vector of rotation matrix is unit orthogonal.
     * 
     * @return Return whether current rotation matric is orthogonal
     *      @retval true The rotation matric is orthogonal
     *      @retval false The rotation matric is not orthogonal
     */
    bool isUnitOrthogonal() const;


    /**
     * @brief Unit orthogonalize the rotation matrix. 
     * 
     * @see mmath::Pose::isUnitOrthogonal() for checking the orthogonality.
     */
    void unitOrthogonalize();


    /**
     * @brief Add a increment to current pose object.
     * 
     * @param [in] dq  The increment of rotation/orientation.
     * @param [in] dt  The increament of translation/position.
     * 
     * @sa mmath::Pose::decrease().
     */
    void increase(const Eigen::Quaternion<kfloat>& dq,
                  const Eigen::Vector<kfloat, 3>& dt);


    /**
     * @brief Minus a increment to current pose object.
     * 
     * @param [in] dq  The increment of rotation/orientation.
     * @param [in] dt  The increament of translation/position.
     * 
     * @sa mmath::Pose::increase().
     */
    void decrease(const Eigen::Quaternion<kfloat>& dq,
                  const Eigen::Vector<kfloat, 3>& dt);


	/**
     * @brief Return the Pose info for print/std::out.
	 * 
	 * @return A character array that stores current member function.
	 */
    char *info() const;


    Eigen::Matrix<kfloat, 3, 3> R; //!< The rotation/orientation matrix
    Eigen::Vector<kfloat, 3>    t; //!< The translation/position vector
};


/* ------------------------------------------------------------------- */
/*                          Pose Implementation                        */
/* ------------------------------------------------------------------- */


template<typename Tp1, typename Tp2>
Pose::Pose(const Eigen::Matrix<Tp1, 3, 3> &R, const Eigen::Vector<Tp2, 3> &t)
    : t(Eigen::Vector<kfloat, 3>(t(0), t(1), t(2))) {
    this->R << R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2);
}


template<typename Tp1>
Pose::Pose(const Eigen::Matrix<Tp1, 4, 4> &T)
    : t(Eigen::Vector<kfloat, 3>(T(0, 3), T(1, 3), T(2, 3))) {
    R << T(0, 0), T(0, 1), T(0, 2),
            T(1, 0), T(1, 1), T(1, 2),
            T(2, 0), T(2, 1), T(2, 2);
}


template<typename Tp1>
Pose::Pose(Tp1 tx, Tp1 ty, Tp1 tz)
    : R(Eigen::Matrix<kfloat, 3, 3>::Identity())
    , t(Eigen::Vector<kfloat, 3>(tx, ty, tz)) {
}


template<typename Tp1>
Pose::Pose(Tp1 data[16], bool is_row_fisrt) {
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


template<typename Tp1>
Pose::Pose(Tp1 m11, Tp1 m12, Tp1 m13, Tp1 m14,
           Tp1 m21, Tp1 m22, Tp1 m23, Tp1 m24,
           Tp1 m31, Tp1 m32, Tp1 m33, Tp1 m34) {
    this->R << m11, m12, m13, m21, m22, m23, m31, m32, m33;
    this->t = Eigen::Vector<kfloat, 3>(m14, m24, m34);
}


template<typename Tp1>
Pose& Pose::operator=(const Eigen::Matrix<Tp1, 4, 4>& pose) {
    *this = Pose(pose);
    return *this;
}

} // mmath
#endif // LIB_MATH_RT_H_LF
