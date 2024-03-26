#include <catch2/catch.hpp>
#include <lib_math/lib_math.h>

TEST_CASE("Test cam", "[projector]")
{
    Eigen::Vector<mmath::kfloat, 2> pt2D;
    Eigen::Vector<mmath::kfloat, 3> pt3D;

    // Binocular
    mmath::kfloat fxy = 1100, cx = 960, cy = 540, t = 4;
    mmath::CameraProjector camproj(fxy, cx, cy, t);

    // Global point
    float x = 20, y = 30, z = 30, u = 0, v = 0;
    // To global imaging plane
    pt2D = camproj.cvt3Dto2D(x, y, z);
    u = (x / z) * fxy + cx;
    CHECK(pt2D[0] == Approx(u).margin(1e-7));
    v = (y / z) * fxy + cy;
    CHECK(pt2D[1] == Approx(v).margin(1e-7));
    // To global camera frame
    pt3D = camproj.cvt2Dto3D(u, v, z);
    CHECK(pt3D[0] == Approx(x).margin(1e-7));
    CHECK(pt3D[1] == Approx(y).margin(1e-7));
    CHECK(pt3D[2] == Approx(z).margin(1e-7));

    // To left imaging plane
    pt2D = camproj.cvt3Dto2D(
        Eigen::Vector<mmath::kfloat, 3>(x, y, z), mmath::cam::LEFT);
    u = ((x + t / 2) / z) * fxy + cx;
    CHECK(pt2D[0] == Approx(u).margin(1e-7));
    v = (y / z) * fxy + cy;
    CHECK(pt2D[1] == Approx(v).margin(1e-7));
    // To global camera frame
    pt3D = camproj.cvt2Dto3D(u, v, z, mmath::cam::LEFT);
    CHECK(pt3D[0] == Approx(x).margin(1e-7));
    CHECK(pt3D[1] == Approx(y).margin(1e-7));
    CHECK(pt3D[2] == Approx(z).margin(1e-7));

    // To right imaging plane
    pt2D = camproj.cvt3Dto2D(
        Eigen::Vector<mmath::kfloat, 3>(x, y, z), mmath::cam::RIGHT);
    u = ((x - t / 2) / z) * fxy + cx;
    CHECK(pt2D[0] == Approx(u).margin(1e-7));
    v = (y / z) * fxy + cy;
    CHECK(pt2D[1] == Approx(v).margin(1e-7));
    // To global camera frame
    pt3D = camproj.cvt2Dto3D(u, v, z, mmath::cam::RIGHT);
    CHECK(pt3D[0] == Approx(x).margin(1e-7));
    CHECK(pt3D[1] == Approx(y).margin(1e-7));
    CHECK(pt3D[2] == Approx(z).margin(1e-7));


    // 2D point
    u = 1000, v = 800;
    float d = 30;   
    // Assume (u, v) belongs to global, to global camera frame
    pt3D = camproj.cvt2Dto3D(u, v, d);
    float val = (u - cx) / fxy * d;
    CHECK(pt3D[0] == Approx(val).margin(1e-7));
    val = (v - cy) / fxy * d;
    CHECK(pt3D[1] == Approx(val).margin(1e-7));
    CHECK(pt3D[2] == Approx(d).margin(1e-7));
    // To global imaging plane
    pt2D = camproj.cvt3Dto2D(pt3D);
    CHECK(pt2D[0] == Approx(u).margin(1e-7));
    CHECK(pt2D[1] == Approx(v).margin(1e-7));

    // Assume (u, v) belongs to left, to global camera frame
    pt3D = camproj.cvt2Dto3D(Eigen::Vector<mmath::kfloat, 2>(u, v), 
                             d, mmath::cam::LEFT);
    val = (u - cx) / fxy * d - t / 2;
    CHECK(pt3D[0] == Approx(val).margin(1e-7));
    val = (v - cy) / fxy * d;
    CHECK(pt3D[1] == Approx(val).margin(1e-7));
    CHECK(pt3D[2] == Approx(d).margin(1e-7));
    // To left imaging plane
    pt2D = camproj.cvt3Dto2D(pt3D, mmath::cam::LEFT);
    CHECK(pt2D[0] == Approx(u).margin(1e-7));
    CHECK(pt2D[1] == Approx(v).margin(1e-7));

    // Assume (u, v) belongs to right, to global camera frame
    pt3D = camproj.cvt2Dto3D(Eigen::Vector<mmath::kfloat, 2>(u, v), 
                             d, mmath::cam::RIGHT);
    val = (u - cx) / fxy * d + t / 2;
    CHECK(pt3D[0] == Approx(val).margin(1e-7));
    val = (v - cy) / fxy * d;
    CHECK(pt3D[1] == Approx(val).margin(1e-7));
    CHECK(pt3D[2] == Approx(d).margin(1e-7));
    // To right imaging plane
    pt2D = camproj.cvt3Dto2D(pt3D, mmath::cam::RIGHT);
    CHECK(pt2D[0] == Approx(u).margin(1e-7));
    CHECK(pt2D[1] == Approx(v).margin(1e-7));

    // Monocular
    mmath::CameraProjector mono_camproj(fxy, cx, cy);
    // To global imaging plane
    pt2D = camproj.cvt3Dto2D(x, y, z);
    val = (x / z) * fxy + cx;
    CHECK(pt2D[0] == Approx(val).margin(1e-7));
    val = (y / z) * fxy + cy;
    CHECK(pt2D[1] == Approx(val).margin(1e-7));

    // To (fake) left imaging plane
    pt2D = mono_camproj.cvt3Dto2D(
        Eigen::Vector<mmath::kfloat, 3>(x, y, z), mmath::cam::LEFT);
    val = (x / z) * fxy + cx;
    CHECK(pt2D[0] == Approx(val).margin(1e-7));
    val = (y / z) * fxy + cy;
    CHECK(pt2D[1] == Approx(val).margin(1e-7));

    // To (fake) right imaging plane
    pt2D = mono_camproj.cvt3Dto2D(
        Eigen::Vector<mmath::kfloat, 3>(x, y, z), mmath::cam::LEFT);
    val = (x / z) * fxy + cx;
    CHECK(pt2D[0] == Approx(val).margin(1e-7));
    val = (y / z) * fxy + cy;
    CHECK(pt2D[1] == Approx(val).margin(1e-7));   
}

