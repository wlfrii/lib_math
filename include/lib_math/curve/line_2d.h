/**--------------------------------------------------------------------
 *
 *   				   Mathematics extension library
 *
 * Description:
 * This file is part of lib_math. You can redistribute it and or modify
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 *
 * @file 		line_2d.h
 *
 * @brief 		Include some line or curve fitting interfaces.
 *
 * @author		Longfei Wang
 *
 * @date		2022/06/08
 *
 * @license     MIT
 *
 * Copyright (C) 2019-Now Longfei Wang.
 *
 * --------------------------------------------------------------------
 * Change History:
 *
 * 2022/06/08 Create this file and code some previous works in C++.
 *
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_LINE_2D_H_LF
#define LIB_MATH_LINE_2D_H_LF
#include <Eigen/Dense>

namespace mmath {

/**
 * @brief A class to represent a 2D line.
 * 
 * @tparam Tp 
 * 
 * @see mmath::fitLine(), mmath::ransacFitLineBias().
 */
template <typename Tp = double>
struct Line
{
public:
    /**
     * @brief Construct a new Line object
     * 
     * @param k The slope of a 2D line.
     * @param b The bias of a 2D line.
     */
    explicit Line(Tp k = 0, Tp b = 0) : k(k), b(b) {}

    /**
     * @brief Calculate the distance from a given point to this line.
     * 
     * @remark This is an overloaded member functions.
     * 
     * @tparam Tp1 Should be arithmetic class type.
     * @tparam Tp2 Should be arithmetic class type.
     * @param [in] pt The given point.
     * 
     * @return The distance between 'pt' and this line object.
     */
    template<typename Tp1 = double, typename Tp2>
    Tp1 distanceTo(const Eigen::Vector<Tp2, 2>& pt){
        return static_cast<Tp1>((k * pt[0] - pt[1] + b) / sqrt(k*k + 1));
    }

    /**
     * @brief Calculate the distance from a given point to this line.
     * 
     * @remark This is an overloaded member functions.
     * 
     * @tparam Tp1 Should be arithmetic class type.
     * @tparam Tp2 Should be arithmetic class type.
     * @param [in] x The given point's x coordinate.
     * @param [in] y The given point's y coordinate.
     * 
     * @return The distance between '(x, y)' and this line object.
     */
    template<typename Tp1 = double, typename Tp2>
    Tp1 distanceTo(Tp2 x, Tp2 y){
        return static_cast<Tp1>((k * x - y + b) / sqrt(k*k + 1));
    }


    Tp k; ///< The slope of the 2D line.
    Tp b; ///< The bias of the 2D line.
};


/**
 * @brief  Fit 2D line based on 2D points.
 * 
 * @remark This is an overloaded functions.
 * 
 * @tparam Tp1 Should be arithmetic class type.
 * @tparam Tp2 Should be arithmetic class type.
 * @param [in] xs A set of x coordinate of 2D points.
 * @param [in] ys A set of y coordinate of 2D points.
 * 
 * @return A Line<Tp1> object.
 * 
 * @see mmath::Line. 
 */
template<typename Tp1 = double, typename Tp2>
Line<Tp1> fitLine(const std::vector<Tp2>& xs, const std::vector<Tp2>& ys) {
    if(xs.size() != ys.size()) std::abort();

    Eigen::MatrixXd A;
    A.resize(xs.size(), 2);
    Eigen::VectorXd B;
    B.resize(xs.size());
    for(size_t i = 0; i < xs.size(); i++){
        A(i, 0) = xs[i];
        A(i, 1) = 1;
        B[i] = ys[i];
    }
    Eigen::Vector<Tp1, 2> kb = (A.transpose()*A).inverse() * A.transpose() * B;

    return Line<Tp1>(kb[0], kb[1]);
}


/**
 * @brief  Fit 2D line based on 2D points.
 * 
 * @remark This is an overloaded functions.
 * 
 * @tparam Tp1 Should be arithmetic class type.
 * @tparam Tp2 Should be arithmetic class type.
 * @param [in] pts A set of 2D points.
 * 
 * @return A Line<Tp1> object.
 * 
 * @see mmath::Line. 
 */
template<typename Tp1 = double, typename Tp2>
Line<Tp1> fitLine(const std::vector<Eigen::Vector<Tp2, 2>>& pts) {
    Eigen::MatrixXd A;
    A.resize(pts.size(), 2);
    Eigen::VectorXd B;
    B.resize(pts.size());
    for(size_t i = 0; i < pts.size(); i++){
        A(i, 0) = pts[i][0];
        A(i, 1) = 1;
        B[i] = pts[i][1];
    }
    Eigen::Vector<Tp1, 2> kb = (A.transpose()*A).inverse() * A.transpose() * B;

    return Line<Tp1>(kb[0], kb[1]);
}


/**
 * @brief  Fit the bias of a 2D line based on 2D points with a specified slope.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @tparam Tp1  Should be arithmetic class type.
 * @tparam Tp2  Should be arithmetic class type.
 * @param [in] k  The specified slope of 2D Line.
 * @param [in] xs A set of x coordinate of 2D points.
 * @param [in] ys A set of y coordinate of 2D points.
 * @param [in] iterations The max iteration times, with default value 100.
 * @param [in] thresh The threshold to exclude outlier 2D points, with default 
 *                    value 2.
 * 
 * @return The bias of a 2D line.
 */
template<typename Tp1 = double, typename Tp2>
Tp1 ransacFitLineBias(Tp1 k, const std::vector<Tp2>& xs, 
                      const std::vector<Tp2>& ys,
                      uint16_t iterations = 100, float thresh = 2) {
    if(xs.size() != ys.size()) std::abort();

    int max_inlier_num = 0;
    Tp1 b = 0;
    Line<Tp1> line(k, b);

    for(uint16_t i = 0; i < iterations; i++){
        uint16_t idx = rand() % xs.size();

        Tp1 x = xs[idx];
        Tp1 y = ys[idx];
        line.b = y - k*x;
        int count = 0;
        for(size_t j = 0; j < xs.size(); j++){
            float dis = line.distanceTo(xs[j], ys[j]);
            if(abs(dis) < thresh) {
                count++;
            }
        }

        if(count > max_inlier_num){
            max_inlier_num = count;
            b = line.b;
        }
    }

    return b;
}


/**
 * @brief  Fit the bias of a 2D line based on 2D points with a specified slope.
 * 
 * @remark This is the base of overloaded functions.
 * 
 * @tparam Tp1  Should be arithmetic class type.
 * @tparam Tp2  Should be arithmetic class type.
 * @param [in] k  The specified slope of 2D Line.
 * @param [in] pts A set of 2D points.
 * @param [in] iterations The max iteration times, with default value 100.
 * @param [in] thresh The threshold to exclude outlier 2D points, with default 
 *                    value 2.
 * 
 * @return The bias of a 2D line.
 */
template<typename Tp1 = double, typename Tp2>
Tp1 ransacFitLineBias(Tp1 k, const std::vector<Eigen::Vector<Tp2, 2>>& pts,
                      uint16_t iterations = 100, float thresh = 2)
{
    int max_inlier_num = 0;
    Tp1 b = 0;
    Line<Tp1> line(k, b);

    for(uint16_t i = 0; i < iterations; i++){
        uint16_t idx = rand() % pts.size();

        Tp1 x = pts[idx][0];
        Tp1 y = pts[idx][1];
        line.b = y - k*x;
        int count = 0;
        for(size_t j = 0; j < pts.size(); j++){
            float dis = line.distanceTo(pts[j]);
            if(abs(dis) < thresh) {
                count++;
            }
        }

        if(count > max_inlier_num){
            max_inlier_num = count;
            b = line.b;
        }
    }

    return b;
}


} // mmath
#endif // LIB_MATH_CURVE_2D_H_LF
