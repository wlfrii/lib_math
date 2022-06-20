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
 * @license
 *
 * Copyright (C) 2022 Longfei Wang.
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
 * @brief A structure to represent a line.
 * 
 * @tparam Tp 
 */
template <typename Tp = double>
struct Line
{
    Line(Tp k = 0, Tp b = 0) : k(k), b(b) {}
    Tp k;
    Tp b;
};


/**
 * @brief
 *
 * @tparam Tp1
 * @tparam Tp2
 * @tparam Tp3
 * @param x     x coordinate of 2D point.
 * @param y     y coordinate of 2D point.
 * @param kb    A line object
 * @return Tp1  Distance between the given 2D point and the given line
 */
template<typename Tp1 = double, typename Tp2, typename Tp3>
Tp1 disFromPt2Line(Tp2 x, Tp2 y, const Line<Tp3>& l)
{
    return static_cast<Tp1>((l.k * x - y + l.b) / sqrt(l.k*l.k + 1));
}


/**
 * @brief
 *
 * @tparam Tp1
 * @tparam Tp2
 * @tparam Tp3
 * @param pts
 * @param kb
 * @return Tp1
 */
template<typename Tp1 = double, typename Tp2, typename Tp3>
Tp1 disFromPt2Line(const Eigen::Vector<Tp2, 2> &pts, const Line<Tp3>& l)
{
    return static_cast<Tp1>((l.k * pts[0] - pts[1] + l.b) / sqrt(l.k*l.k + 1));
}


/**
 * @brief  Fit 2D line based on 2D points.
 * 
 * @tparam Tp1 
 * @tparam Tp2 
 * @param xs 
 * @param ys 
 * @return Line<Tp1> 
 */
template<typename Tp1 = double, typename Tp2>
Line<Tp1> fitLine(const std::vector<Tp2>& xs, const std::vector<Tp2>& ys)
{
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
 * @brief 
 * 
 * @tparam Tp1 
 * @tparam Tp2 
 * @param pts 
 * @return Line<Tp1> 
 */
template<typename Tp1 = double, typename Tp2>
Line<Tp1> fitLine(const std::vector<Eigen::Vector<Tp2, 2>>& pts)
{
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
 * @brief 
 * 
 * @tparam Tp1 
 * @tparam Tp2 
 * @param k 
 * @param xs 
 * @param ys 
 * @param iterations 
 * @param thresh 
 * @return Tp1 
 */
template<typename Tp1 = double, typename Tp2>
Tp1 ransacFitLineBias(Tp1 k, const std::vector<Tp2>& xs, const std::vector<Tp2>& ys,
                      uint16_t iterations = 100, float thresh = 2)
{
    if(xs.size() != ys.size()) std::abort();

    int max_inlier_num = 0;
    Tp1 b = 0;

    for(uint16_t i = 0; i < iterations; i++){
        uint16_t idx = rand() % xs.size();

        Tp1 x = xs[idx];
        Tp1 y = ys[idx];
        Tp1 b_random = y - k*x;
        int count = 0;
        for(size_t j = 0; j < xs.size(); j++){
            float dis = disFromPt2Line(xs[j], ys[j], Line<Tp1>(k, b_random));
            if(abs(dis) < thresh) {
                count++;
            }
        }

        if(count > max_inlier_num){
            max_inlier_num = count;
            b = b_random;
        }
    }

    return b;
}


/**
 * @brief 
 * 
 * @tparam Tp1 
 * @tparam Tp2 
 * @param k 
 * @param pts 
 * @param iterations 
 * @param thresh 
 * @return Tp1 
 */
template<typename Tp1 = double, typename Tp2>
Tp1 ransacFitLineBias(Tp1 k, const std::vector<Eigen::Vector<Tp2, 2>>& pts,
                      uint16_t iterations = 100, float thresh = 2)
{
    int max_inlier_num = 0;
    Tp1 b = 0;

    for(uint16_t i = 0; i < iterations; i++){
        uint16_t idx = rand() % pts.size();

        Tp1 x = pts[idx][0];
        Tp1 y = pts[idx][1];
        Tp1 b_random = y - k*x;
        int count = 0;
        for(size_t j = 0; j < pts.size(); j++){
            float dis = disFromPt2Line(pts[j], Line<Tp1>(k, b_random));
            if(abs(dis) < thresh) {
                count++;
            }
        }

        if(count > max_inlier_num){
            max_inlier_num = count;
            b = b_random;
        }
    }

    return b;
}


} // mmath
#endif // LIB_MATH_CURVE_2D_H_LF
