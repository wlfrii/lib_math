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


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        kfloat x, kfloat y, kfloat z) const
{
    kfloat u = (x / z) * fxy + cx;
    kfloat v = (y / z) * fxy + cy;
    return Eigen::Vector<kfloat, 2>(u, v);
}


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        const Eigen::Vector<kfloat, 3> &pt3D) const
{
    return cvt3Dto2D(pt3D[0], pt3D[1], pt3D[2]);
}


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        kfloat x, kfloat y, kfloat z, cam::ID id) const
{
    kfloat x_new = id == cam::LEFT ? x + t/2.f : x - t/2.f;
    kfloat u = (x_new / z) * fxy + cx;
    kfloat v = (y / z) * fxy + cy;
    return Eigen::Vector<kfloat, 2>(u, v);
}


Eigen::Vector<kfloat, 2> CameraProjector::cvt3Dto2D(
        const Eigen::Vector<kfloat, 3> &pt3D, cam::ID id) const
{
    return cvt3Dto2D(pt3D[0], pt3D[1], pt3D[2], id);
}


Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        kfloat u, kfloat v, kfloat depth) const
{
    kfloat x = (u - cx) / fxy * depth;
    kfloat y = (v - cy) / fxy * depth;
    return Eigen::Vector<kfloat, 3>(x, y, depth);
}


Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        const Eigen::Vector<kfloat, 2> &pt2D, kfloat depth) const
{
    return cvt2Dto3D(pt2D[0], pt2D[1], depth);
}


Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        kfloat u, kfloat v, kfloat depth, cam::ID id) const
{
    kfloat x = (u - cx) / fxy * depth;
    kfloat y = (v - cy) / fxy * depth;
    return id == cam::LEFT ?
                Eigen::Vector<kfloat, 3>(x - t/2.f, y, depth) :
                Eigen::Vector<kfloat, 3>(x + t/2.f, y, depth);
}


Eigen::Vector<kfloat, 3> CameraProjector::cvt2Dto3D(
        const Eigen::Vector<kfloat, 2> &pt2D, kfloat depth, cam::ID id) const
{
    return cvt2Dto3D(pt2D[0], pt2D[1], depth, id);
}

} // mmath
