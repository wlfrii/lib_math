/**--------------------------------------------------------------------
 *
 *   				   Mathematics extension library
 *
 * Description:
 * This file is part of lib_math. You can redistribute it and or modify
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 *
 * @file 		gauss_curve_2d.h
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
#ifndef LIB_MATH_GAUSS_CURVE_2D_H_LF
#define LIB_MATH_GAUSS_CURVE_2D_H_LF
#include <Eigen/Dense>

namespace mmath{

/**
 * @brief A struct to represent Gaussian curve.
 *
 * @note The gaussian formula is supposed to be
 *  g(x) = a*exp(-1/2 * ((x-mu)/sigma)^2).
 * 
 * @tparam Tp The arithmetic class type.
 * 
 * @seet mmath::fitGuassianCurve();
 */
template <typename Tp = double>
struct GaussianCurve
{
    explicit GaussianCurve(Tp a = 0, Tp mu = 0, Tp sigma = 0)
        : a(a), mu(mu), sigma(sigma) {}

    Tp a;       ///< The coefficient of Gaussian function.
    Tp mu;      ///< The mean value of Gaussian function.
    Tp sigma;   ///< The standard derivation of Gaussian function.


    /**
     * @brief Compute the Gaussian value at a given position.
     * 
     * @tparam Tp1 The arithmetic class type.
     * @tparam Tp2 The arithmetic class type.
     * @param [in] x  The value of a given position.
     * 
     * @return The Gaussian value at 'x' position, an object of class Tp1.
     */
    template<typename Tp1 = double, typename Tp2>
    Tp1 valueAt(Tp2 x){
        return a * exp(-0.5*pow((x - mu) / sigma, 2));
    }


    /**
     * @brief Compute the Jocabian of Gaussian curve at a given position.
     * 
     * @tparam Tp1 The arithmetic class type.
     * @tparam Tp2 The arithmetic class type.
     * @param [in] x  The value of a given position.
     * 
     * @return The Jocabian value corresponding to the member [a, mu, sigma] at 
     *         'x' position.
     */
    template<typename Tp1 = double, typename Tp2>
    Eigen::Vector<Tp1, 3> JocabianAt(Tp2 x){
        Tp1 f = this->valueAt(x);
        Eigen::Vector<Tp1, 3> J;
        J[0] = f / a;
        J[1] = f * (x - mu) / (sigma * sigma);
        J[2] = f * pow(x - mu, 2) / pow(sigma, 3);
        return J;
    }
};


/**
 * @brief Fit GaussianCurve using Netwon-Gaussian method.
 * 
 * @remark This is an overloaded functions.
 * 
 * @tparam Tp The arithmetic class type.
 * @tparam Tp1 The arithmetic class type.
 * @param [in] xs  A set of x coordinates.
 * @param [in] ys  A set of y coordinates.
 * @param [in] max_iterations  The max iteration times, with default value 100.
 *
 * @return A GaussianCurve<Tp> object.
 * 
 * @see mmath::GaussianCurve. 
 */
template <typename Tp = double, typename Tp1>
GaussianCurve<Tp> fitGuassianCurve(const std::vector<Tp1>& xs,
                                   const std::vector<Tp1>& ys,
                                   uint16_t max_iterations = 100) {
    if(xs.size() != ys.size()) std::abort();

    // Given a intial guess
    GaussianCurve<Tp> gauss;
    double a = 0, mu = 0;
    int id = 0;
    for(size_t i = 0; i < xs.size(); i++){
        if(ys[i] > a){
            id = i;
            a = ys[i];
            mu = xs[i];
        }
    }
    gauss.a = a;
    gauss.mu = mu;
    gauss.sigma = (xs[id+1] - mu) / sqrt(-2.0 * log(ys[id+1] / a));

    // Fit by Newton-Gaussian method
    double cost_prev = 1e32;
    Eigen::Vector3d J, dx, b;
    Eigen::Matrix3d H;
    for(int j = 0; j < max_iterations; j++){
        H = Eigen::Matrix3d::Zero();
        b = Eigen::Vector3d::Zero();
        J = Eigen::Vector3d::Zero();
        double cost = 0;

        for(size_t i = 0; i < xs.size(); i++){
            double x = xs[i];
            double err = ys[i] - gauss.valueAt(x);
            J = gauss.JocabianAt(x);

            H += J*J.transpose();
            b += err*J;
            cost += err*err;
        }

        dx = H.ldlt().solve(b); // faster
        //dx = H.inverse() * b;

        if(cost >= cost_prev){
            // printf("Iterations: %d, cost: %f, cos_prev: %f\n", j, cost, cost_prev);
            break;
        }

        gauss.a += dx[0];
        gauss.mu += dx[1];
        gauss.sigma += dx[2];

        cost_prev = cost;
    }

    return gauss;
}


/**
 * @brief Fit GaussianCurve using Netwon-Gaussian method.
 * 
 * @remark This is an overloaded functions.
 * 
 * @tparam Tp The arithmetic class type.
 * @tparam Tp1 The arithmetic class type.
 * @param [in] pts  A set of (x, y) coordinates.
 * @param [in] max_iterations  The max iteration times, with default value 100.
 *
 * @return A GaussianCurve<Tp> object.
 * 
 * @see mmath::GaussianCurve. 
 */
template <typename Tp = double, typename Tp1 = double>
GaussianCurve<Tp> fitGuassianCurve(const std::vector<Eigen::Vector<Tp1, 2>>& pts,
                                   uint16_t max_iterations = 100) {
    std::vector<Tp1> xs(pts.size()), ys(pts.size());
    for(size_t i = 0; i < pts.size(); i++){
        xs[i] = pts[i][0];
        ys[i] = pts[i][1];
    }
    return fitGuassianCurve(xs, ys, max_iterations);
}


} // namespace::mmath
#endif // LIB_MATH_GAUSS_CURVE_2D_H_LF
