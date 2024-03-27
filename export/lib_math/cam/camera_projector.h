/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		camera_projector.h
 * 
 * @brief 		Design a class for pin-hole projection.
 * 
 * @author		Longfei Wang
 * 
 * @date		2021/04/01
 * 
 * @license		MIT
 * 
 * Copyright (C) 2019-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_CAMERA_PROJECTOR_H_LF
#define LIB_MATH_CAMERA_PROJECTOR_H_LF
#include <Eigen//Dense>
#include "../math_precision.h"

namespace mmath{

/** A cam namespace to specify left and right camera */
namespace cam
{
/** Specify the camera index */
using ID = bool;
extern ID LEFT;
extern ID RIGHT;
};


/**
 * @brief The CameraProjector class is designed for the conversion between 3D 
 * point w.r.t Camera frame and the 2D point w.r.t image frame.
 * 
 * @note There are several definitions should be clarified first. 
 * - GLOBAL camera frame: equals to camera frame for monocular, or indicates the 
 * frame in the middle between left and right cameras for binocular.
 * - GLOBAL imaging frame: equals to image frame for monocular, or indicates the 
 * frame in the middle between left and right imaging for binocular.
 * - SPECIFIED camera frame: equals to camera frame for monocular, or indicates  
 * the left or right camera frame for binocular.
 * - SPECIFIED imaging frame: equals to image frame for monocular, or indicates  
 * the left or right image frame for binocular.
 */
class CameraProjector
{
public:
    /**
     * @brief Construct a new Camera Projector object.
     * 
     * @param fxy The focal length of camera. For binocular, this value should
     *            be the stereo-rectified value.
     * @param cx  The x coordinta of camera optical axis. For binocular, this 
     *            value should be the stereo-rectified value.
     * @param cy  The y coordinta of camera optical axis. For binocular, this 
     *            value should be the stereo-rectified value.
     * @param t   The distance between stereo cameras. When no distance or zero
     *            value is given, the camera is supposed to be a monocular.
     */
    CameraProjector(kfloat fxy, kfloat cx, kfloat cy, kfloat t = 0);
    ~CameraProjector();


    /**
     * @brief Projecting 3D point to 2D w.r.t GLOBAL imaging frame.
     * 
     * @remark This is the base of overloaded member functions.
     * 
     * @param [in]  x  X coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  y  Y coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  z  Z coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [out] pt2D A 2D point w.r.t GLOBAL imaging frame.
     */
    void cvt3Dto2D(kfloat x, kfloat y, kfloat z, 
                   Eigen::Vector<kfloat, 2>& pt2D) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t GLOBAL imaging frame.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in]  x  X coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  y  Y coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  z  Z coordinate of the 3D point w.r.t GLOBAL camera frame.
     * 
     * @return A 2D point w.r.t GLOBAL imaging frame.
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(kfloat x, kfloat y, kfloat z) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t GLOBAL imaging frame.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. 
     * 
     * @param [in]  pt3D A 3D point w.r.t GLOBAL camera frame.
     * @param [out] pt2D A 2D point w.r.t GLOBAL imaging frame.
     */
    void cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D,
                   Eigen::Vector<kfloat, 2>& pt2D) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t GLOBAL imaging frame.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in]  pt3D A 3D point w.r.t GLOBAL camera frame.
     * 
     * @return A 2D point w.r.t GLOBAL imaging frame.
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t SPECIFIED imaging frame.
     * 
     * @remark This is the base of overloaded member functions.
     * 
     * @param [in]  x  X coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  y  Y coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  z  Z coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  id  Specify the camera index for binocular.
     * @param [out] pt2D A 2D point w.r.t SPECIFIED imaging frame.
     */
    void cvt3Dto2D(kfloat x, kfloat y, kfloat z, cam::ID id,
                   Eigen::Vector<kfloat, 2>& pt2D) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t SPECIFIED imaging frame.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in]  x  X coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  y  Y coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  z  Z coordinate of the 3D point w.r.t GLOBAL camera frame.
     * @param [in]  id  Specify the camera index for binocular.
     * 
     * @return A 2D point w.r.t SPECIFIED imaging frame.
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(kfloat x, kfloat y, kfloat z,
                                       cam::ID id) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t SPECIFIED imaging frame.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. 
     * 
     * @param [in]  pt3D  A 3D point w.r.t GLOBAL camera frame.
     * @param [in]  id    Specify the camera index for binocular.
     * @param [out] pt2D  A 2D point w.r.t SPECIFIED imaging frame.
     */
    void cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D, cam::ID id,
                   Eigen::Vector<kfloat, 2>& pt2D) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t SPECIFIED imaging frame.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in]  pt3D  A 3D point w.r.t GLOBAL camera frame.
     * @param [in]  id    Specify the camera index for binocular.
     * 
     * @return A 2D point w.r.t SPECIFIED imaging frame.
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D,
                                       cam::ID id) const;


    /**
     * @brief Lifting 2D point that w.r.t GLOBAL imaging frame to 3D.
     * 
     * @remark This is the base of overloaded member functions.
     * 
     * @param [in] u  U coordinate of the 2D point w.r.t GLOBAL imaging frame.
     * @param [in] v  V coordinate of the 2D point w.r.t GLOBAL imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * @param [out] pt3D  A 3D point w.r.t GLOBAL camera frame.
     */
    void cvt2Dto3D(kfloat u, kfloat v, kfloat depth,
                  Eigen::Vector<kfloat, 3>& pt3D) const;


    /**
     * @brief Lifting 2D point that w.r.t GLOBAL imaging frame to 3D.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in] u  U coordinate of the 2D point w.r.t GLOBAL imaging frame.
     * @param [in] v  V coordinate of the 2D point w.r.t GLOBAL imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * 
     * @return A 3D point w.r.t GLOBAL camera frame
     */
    Eigen::Vector<kfloat, 3> cvt2Dto3D(kfloat u, kfloat v, kfloat depth) const;


    /**
     * @brief Lifting 2D point that w.r.t GLOBAL imaging frame to 3D
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. 
     * 
     * @param [in] pt2D  A 2D point w.r.t GLOBAL imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * @param [out] pt3D  A 3D point w.r.t GLOBAL camera frame.
     */
    void cvt2Dto3D(const Eigen::Vector<kfloat, 2>& pt2D, kfloat depth,
                   Eigen::Vector<kfloat, 3>& pt3D) const;


    /**
     * @brief Lifting 2D point that w.r.t GLOBAL imaging frame to 3D.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in] pt2D  A 2D point w.r.t GLOBAL imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * 
     * @return A 3D point w.r.t GLOBAL camera frame.
     */
    Eigen::Vector<kfloat, 3> cvt2Dto3D(const Eigen::Vector<kfloat, 2>& pt2D,
                                       kfloat depth) const;


    /**
     * @brief Lifting 2D point that w.r.t SPECIFIED imaging frame to 3D.
     * 
     * @remark This is the base of overloaded member functions.
     * 
     * @param [in] u U coordinate of the 2D point w.r.t SPECIFIED imaging frame.
     * @param [in] v V coordinate of the 2D point w.r.t SPECIFIED imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * @param [in] id    Specify the camera index for binocular.
     * @param [out] pt3D  A 3D point w.r.t GLOBAL camera frame.
     */
    void cvt2Dto3D(kfloat u, kfloat v, kfloat depth, cam::ID id, 
                   Eigen::Vector<kfloat, 3>& pt3D) const;

    /**
     * @brief Lifting 2D point that w.r.t SPECIFIED imaging frame to 3D.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in] u U coordinate of the 2D point w.r.t SPECIFIED imaging frame.
     * @param [in] v V coordinate of the 2D point w.r.t SPECIFIED imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * @param [in] id    Specify the camera index for binocular.
     * 
     * @return A 3D point w.r.t GLOBAL camera frame.
     */
    Eigen::Vector<kfloat, 3> cvt2Dto3D(kfloat u, kfloat v, kfloat depth,
                                       cam::ID id) const;


    /**
     * @brief Lifting 2D point that w.r.t SPECIFIED imaging frame to 3D.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. 
     * 
     * @param [in] pt2D  A 2D point w.r.t SPECIFIED imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * @param [in] id    Specify the camera index for binocular.
     * @param [out] pt3D  A 3D point w.r.t GLOBAL camera frame.
     */
    void cvt2Dto3D(const Eigen::Vector<kfloat, 2>& pt2D, kfloat depth, 
                   cam::ID id, Eigen::Vector<kfloat, 3>& pt3D) const;


    /**
     * @brief Lifting 2D point that w.r.t SPECIFIED imaging frame to 3D.
     * 
     * @remark This is an overloaded member function, provided for convenience. 
     * It differs from the base function only in what argument(s) it accepts 
     * and the returned value. To improve efficiency, the member function with 
     * the void-returned value is suggested.
     * 
     * @param [in] pt2D  A 2D point w.r.t SPECIFIED imaging frame.
     * @param [in] depth Depth of the given 2D point w.r.t GLOBAL camera frame.
     * @param [in] id    Specify the camera index for binocular.
     * 
     * @return A 3D point w.r.t GLOBAL camera frame
     */
    Eigen::Vector<kfloat, 3> cvt2Dto3D(const Eigen::Vector<kfloat, 2>& pt2D,
                                       kfloat depth, cam::ID id) const;


    const float fxy; //!< Focal length
    const float cx;  //!< x coordinates of optical axis in imaging plane
    const float cy;  //!< y coordinates of optical axis in imaging plane
    const float t;   //!< Distance between binocular's optical axis
};

} // mmath
#endif // LIB_MATH_CAMERA_PROJECTOR_H_LF
