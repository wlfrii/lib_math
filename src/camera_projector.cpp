#include "../export/lib_math/cam/camera_projector.h"

namespace mmath{

namespace cam {
ID LEFT = 0;
ID RIGHT = 1;
};

CameraProjector::CameraProjector(kfloat fxy, kfloat cx, kfloat cy, kfloat t)
    : fxy(fxy)
    , cx(cx)
    , cy(cy)
    , t(t)
{

}


CameraProjector::~CameraProjector()
{

}


void CameraProjector::cvt3Dto2D(kfloat x, kfloat y, kfloat z, 
        Eigen::Vector<kfloat, 2>& pt2D) const {
    pt2D[0] = (x / z) * fxy + cx;
    pt2D[1] = (y / z) * fxy + cy;
}


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        kfloat x, kfloat y, kfloat z) const {
    Eigen::Vector<kfloat, 2> pt2D;
    cvt3Dto2D(x, y, z, pt2D);
    return pt2D;
}


void CameraProjector::cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D,
        Eigen::Vector<kfloat, 2>& pt2D) const {
    cvt3Dto2D(pt3D[0], pt3D[1], pt3D[2], pt2D);
}


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        const Eigen::Vector<kfloat, 3> &pt3D) const {
    Eigen::Vector<kfloat, 2> pt2D;
    cvt3Dto2D(pt3D, pt2D);
    return pt2D;
}


void CameraProjector::cvt3Dto2D(kfloat x, kfloat y, kfloat z, cam::ID id,
        Eigen::Vector<kfloat, 2>& pt2D) const {
    kfloat x_new = id == cam::LEFT ? x + t/2.f : x - t/2.f;
    pt2D[0] = (x_new / z) * fxy + cx;
    pt2D[1] = (y / z) * fxy + cy;
}


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        kfloat x, kfloat y, kfloat z, cam::ID id) const {
    Eigen::Vector<kfloat, 2> pt2D;
    cvt3Dto2D(x, y, z, id, pt2D);
    return pt2D;
}


void CameraProjector::cvt3Dto2D(const Eigen::Vector<kfloat, 3>& pt3D, 
        cam::ID id, Eigen::Vector<kfloat, 2>& pt2D) const {
    cvt3Dto2D(pt3D[0], pt3D[1], pt3D[2], id, pt2D);
}


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        const Eigen::Vector<kfloat, 3> &pt3D, cam::ID id) const {
    Eigen::Vector<kfloat, 2> pt2D;
    cvt3Dto2D(pt3D, id), pt2D;
    return pt2D;
}


void CameraProjector::cvt2Dto3D(kfloat u, kfloat v, kfloat depth,
    Eigen::Vector<kfloat, 3>& pt3D) const {
    pt3D[0] = (u - cx) / fxy * depth;
    pt3D[1] = (v - cy) / fxy * depth;
    pt3D[2] = depth;
}


Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        kfloat u, kfloat v, kfloat depth) const {
    Eigen::Vector<kfloat, 3> pt3D;
    cvt2Dto3D(u, v, depth, pt3D);
    return pt3D;
}


void CameraProjector::cvt2Dto3D(const Eigen::Vector<kfloat, 2>& pt2D, 
        kfloat depth, Eigen::Vector<kfloat, 3>& pt3D) const {
    cvt2Dto3D(pt2D[0], pt2D[1], depth, pt3D);
}


Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        const Eigen::Vector<kfloat, 2>& pt2D, kfloat depth) const {
    Eigen::Vector<kfloat, 3> pt3D;
    cvt2Dto3D(pt2D, depth, pt3D);
    return pt3D;
}


void CameraProjector::cvt2Dto3D(kfloat u, kfloat v, kfloat depth, cam::ID id, 
        Eigen::Vector<kfloat, 3>& pt3D) const {
    kfloat x = (u - cx) / fxy * depth;
    pt3D[0] = id == cam::LEFT ? x - t/2.f : x + t/2.f;
    pt3D[1] = (v - cy) / fxy * depth;
    pt3D[2] = depth;
}


Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        kfloat u, kfloat v, kfloat depth, cam::ID id) const {
    kfloat x = (u - cx) / fxy * depth;
    kfloat y = (v - cy) / fxy * depth;
    return id == cam::LEFT ?
                Eigen::Vector<kfloat, 3>(x - t/2.f, y, depth) :
                Eigen::Vector<kfloat, 3>(x + t/2.f, y, depth);
}


void CameraProjector::cvt2Dto3D(const Eigen::Vector<kfloat, 2>& pt2D, 
        kfloat depth, cam::ID id, Eigen::Vector<kfloat, 3>& pt3D) const {
    cvt2Dto3D(pt2D[0], pt2D[1], depth, id, pt3D);
}

Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        const Eigen::Vector<kfloat, 2> &pt2D, kfloat depth, cam::ID id) const
{
    return cvt2Dto3D(pt2D[0], pt2D[1], depth, id);
}

} // mmath
