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

/** @brief cam namespace
 */
namespace cam
{
/** @brief Specify the camera index
 */
using ID = bool;
extern ID LEFT;
extern ID RIGHT;
};


/**
 * @brief The CameraProjector class designed to the conversion between 3D point
 * w.r.t Camera frame and the 2D point w.r.t image frame.
 */
class CameraProjector
{
public:
    /**
     * @brief Construct a new Camera Projector object
     * 
     * @param fxy  The focal length of camera. For binocular, this value should
     * be the stereo-rectified focal length value.
     * @param cx  The x coordinta of camera optical axis
     * @param cy  The y coordinta of camera optical axis
     * @param t   The distance between stereo cameras
     */
    CameraProjector(kfloat fxy, kfloat cx, kfloat cy, kfloat t = 0);
    ~CameraProjector();


    /**
     * @brief Projecting 3D point to 2D w.r.t GLOBAL imaging frame
     * @param x  X coordinate of the 3D point
     * @param y  Y coordinate of the 3D point
     * @param z  Z coordinate of the 3D point
     * @return 2D point w.r.t GLOBAL imaging frame
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(kfloat x, kfloat y, kfloat z) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t GLOBAL imaging frame
     * @param pt3D
     * @param id  Specify the camera index for binocular
     * @return 2D point w.r.t GLOBAL imaging frame
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t SPECIFIED imaging frame
     * @param x  X coordinate of the 3D point
     * @param y  Y coordinate of the 3D point
     * @param z  Z coordinate of the 3D point
     * @param id  Specify the camera index for binocular
     * @return 2D point w.r.t SPECIFIED imaging frame
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(kfloat x, kfloat y, kfloat z,
                                       cam::ID id) const;


    /**
     * @brief Projecting 3D point to 2D w.r.t SPECIFIED imaging frame
     * @param pt3D
     * @param id  Specify the camera index for binocular
     * @return 2D point w.r.t SPECIFIED imaging frame
     */
    Eigen::Vector<kfloat, 2> cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D,
                                       cam::ID id) const;


    /**
     * @brief lifting 2D point that w.r.t GLOBAL camera frame to 3D
     * @param u  U coordinate of the 2D point
     * @param v  V coordinate of the 2D point
     * @param depth   Depth of the given 2D point
     * @return 3D point w.r.t GLOBAL camera frame
     */
    Eigen::Vector<kfloat, 3> cvt2Dto3D(kfloat u, kfloat v, kfloat depth) const;


    /**
     * @brief lifting 2D point that w.r.t GLOBAL camera frame to 3D
     * @param pt2D  2D Points w.r.t specifed camera
     * @param depth  Depth of the given 2D point
     * @return 3D point w.r.t GLOBAL camera frame
     */
    Eigen::Vector<kfloat, 3> cvt2Dto3D(const Eigen::Vector<kfloat, 2>& pt2D,
                                       kfloat depth) const;


    /**
     * @brief lifting 2D point that w.r.t SPECIFIED camera frame to 3D
     * @param u  U coordinate of the 2D point
     * @param v  V coordinate of the 2D point
     * @param depth   Depth of the given 2D point
     * @param id  Specify the camera index for binocular
     * @return 3D point w.r.t SPECIFIED camera frame
     */
    Eigen::Vector<kfloat, 3> cvt2Dto3D(kfloat u, kfloat v, kfloat depth,
                                       cam::ID id) const;


    /**
     * @brief lifting 2D point that w.r.t SPECIFIED camera frame to 3D
     * @param pt2D  2D Points w.r.t specifed camera
     * @param depth  Depth of the given 2D point
     * @param id  Specify the camera index for binocular
     * @return 3D point w.r.t SPECIFIED camera frame
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
