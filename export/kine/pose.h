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
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_POSE_H_LF
#define LIB_MATH_POSE_H_LF
#include <Eigen/Dense>

namespace mmath{

/** 
 * @brief A class designed to describe the transformation (spatial pose).
 * This class can furtherly simplify the calculation of pose-trasform.
 * There are three members include in this class:
 *   R	--  denotes the rotation or orientation.
 *   t  --  denotes the translation or position.
 */
template <typename Tp = double>
class Pose
{
public:
    /**
     * @brief Construct a new Pose object.
     * 
     * @tparam Tp1 The type of input matrix.
     * @param T    The transform matrix.
     */
    template<typename Tp1 = double>
    explicit Pose(const Eigen::Matrix<Tp1, 4, 4>& T);


    /**
     * @brief Construct a new Pose object.
     * 
     * @tparam Tp1  The type of rotation matrix.
     * @tparam Tp2  The type of translation vector.
     * @param R  The rotation matrix.
     * @param t  The translation vector.
     */
    template<typename Tp1 = double, typename Tp2 = double>
    explicit Pose(const Eigen::Matrix<Tp1, 3, 3>& R =
            Eigen::Matrix<Tp1, 3, 3>::Identity(),
                  const Eigen::Vector<Tp2, 3>& t = {0, 0, 0});
    

    /**
     * @brief Construct a new Pose object.
     *
     * @tparam Tp1  The type of quaterion.
     * @tparam Tp2  The type of translation vector.
     * @param q  The quaterion.
     * @param t  The translation vector.
     */
    template<typename Tp1 = double, typename Tp2 = double>
    explicit Pose(const Eigen::Quaternion<Tp1>& q,
                  const Eigen::Vector<Tp2, 3>& t = {0, 0, 0});
    

    /**
     * @brief Construct a new Pose object.
     * 
     * @tparam Tp1  The type of inputs.
     * @param tx  The x value of translation vector.
     * @param ty  The y value of translation vector.
     * @param tz  The z value of translation vector.
     */
    template<typename Tp1 = double>
    explicit Pose(Tp1 tx, Tp1 ty, Tp1 tz);
    
    
    /**
     * @brief Construct a new Pose object
     * 
     * @tparam Tp1  The type of input array.
     * @param data  The input array[16]
     * @param is_row_fisrt  Flag of row first/column first.
     */
    template<typename Tp1 = double>
    explicit Pose(Tp1 data[16], bool is_row_fisrt = true);


    /**
     * @brief Return quaterion.
     * 
     * @return Eigen::Quaternion<Tp>
     */
    inline Eigen::Quaternion<Tp> q() const;


    /**
     * @brief Return transform matrix.
     * 
     * @return Eigen::Matrix4f 
     */
    inline Eigen::Matrix<Tp, 4, 4> T() const;


	/**
     * @brief Return the inverse of the Pose.
	 * 
     * @return Pose<Tp>
	 */
    inline Pose<Tp> inverse();


    /**
     * @brief Add a increment.
     * @param dq  The increment of rotation/orientation
     * @param dt  The increament of translation/position
     * @sa decrease()
     */
    inline void increase(const Eigen::Quaternion<Tp>& dq,
                         const Eigen::Vector<Tp, 3>& dt);


    /**
     * @brief Minus a increment.
     * @param dq  The increment of rotation/orientation
     * @param dt  The increament of translation/position
     * @sa increase()
     */
    inline void decrease(const Eigen::Quaternion<Tp>& dq,
                         const Eigen::Vector<Tp, 3>& dt);



	/**
     * @brief Return the Pose info for print/std::out.
	 * 
	 * @return char* 
	 */
    char *info() const;


    Eigen::Matrix<Tp, 3, 3> R;
    Eigen::Vector<Tp, 3>    t;
};


/* ------------------------------------------------------------------- */
/*                          Pose Implementation                        */
/* ------------------------------------------------------------------- */

template<typename Tp>
template<typename Tp1>
Pose<Tp>::Pose(const Eigen::Matrix<Tp1, 4, 4> &T)
    : t(Eigen::Vector<Tp, 3>(T(0, 3), T(1, 3), T(2, 3)))
{
    R << T(0, 0), T(0, 1), T(0, 2),
            T(1, 0), T(1, 1), T(1, 2),
            T(2, 0), T(2, 1), T(2, 2);
}


template<typename Tp>
template<typename Tp1, typename Tp2>
Pose<Tp>::Pose(const Eigen::Matrix<Tp1, 3, 3> &R, const Eigen::Vector<Tp2, 3> &t)
    : t(Eigen::Vector<Tp, 3>(t(0), t(1), t(2)))
{
    this->R << R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2);
}


template<typename Tp>
template<typename Tp1, typename Tp2>
Pose<Tp>::Pose(const Eigen::Quaternion<Tp1> &q, const Eigen::Vector<Tp2, 3> &t)
    : t(Eigen::Vector<Tp, 3>(t(0), t(1), t(2)))
{
    Eigen::Matrix<Tp1, 3, 3> R_ = q.toRotationMatrix();
    R << R_(0, 0), R_(0, 1), R_(0, 2),
            R_(1, 0), R_(1, 1), R_(1, 2),
            R_(2, 0), R_(2, 1), R_(2, 2);
}


template<typename Tp>
template<typename Tp1>
Pose<Tp>::Pose(Tp1 tx, Tp1 ty, Tp1 tz)
    : R(Eigen::Matrix<Tp, 3, 3>::Identity())
    , t(Eigen::Vector<Tp, 3>(tx, ty, tz))
{
}


template<typename Tp>
template<typename Tp1>
Pose<Tp>::Pose(Tp1 data[], bool is_row_fisrt)
{
    if (is_row_fisrt) {
        R << data[0], data[1], data[2],
                data[4], data[5], data[6],
                data[8], data[9], data[10];
        t = Eigen::Vector<Tp, 3>(data[3], data[7], data[11]);
    }
    else{
        R << data[0], data[4], data[8],
                data[1], data[5], data[9],
                data[2], data[6], data[10];
        t = Eigen::Vector<Tp, 3>(data[12], data[13], data[14]);
    }
}


template<typename Tp>
inline Eigen::Quaternion<Tp> Pose<Tp>::q() const
{
    return Eigen::Quaternion<Tp>(R);
}


template<typename Tp>
inline Eigen::Matrix<Tp, 4, 4> Pose<Tp>::T() const
{
    Eigen::Matrix<Tp, 4, 4> T;
    T.topLeftCorner(3, 3) = R;
    T.topRightCorner(3, 1) = t;
    return T;
}


template<typename Tp>
inline Pose<Tp> Pose<Tp>::inverse()
{
    return Pose<Tp>(R.transpose(), -R.transpose()*t);
}


template<typename Tp>
void Pose<Tp>::increase(const Eigen::Quaternion<Tp> &dq,
                        const Eigen::Vector<Tp, 3> &dt)
{
    Eigen::Quaternion<Tp> q = this->q();
    q.coeffs() += dq.coeffs();
    q.normalize();
    R = q.toRotationMatrix();
    t += dt;
}


template<typename Tp>
void Pose<Tp>::decrease(const Eigen::Quaternion<Tp> &dq,
                        const Eigen::Vector<Tp, 3> &dt)
{
    Eigen::Quaternion<Tp> q = this->q();
    q.coeffs() -= dq.coeffs();
    q.normalize();
    R = q.toRotationMatrix();
    t -= dt;
}


template<typename Tp>
char* Pose<Tp>::info() const
{
    static char tmp_cstr[200];
    sprintf(tmp_cstr, "row-1st, R=[%f,%f,%f,%f,%f,%f,%f,%f,%f],t=[%f,%f,%f]",
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2), t[0], t[1], t[2]);
    return tmp_cstr;
}

} // mmath
#endif // LIB_MATH_RT_H_LF
