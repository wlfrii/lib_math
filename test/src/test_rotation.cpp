#include <catch2/catch.hpp>
#include <lib_math/lib_math.h>

TEST_CASE("Test rotation", "[rotation]")
{
    Eigen::Matrix3f rot;
    float radian = mmath::deg2radf(30);
    rot = mmath::rotByXf(radian);
    CHECK(rot(1,1) == Approx(0.866025).margin(1e-7));
    CHECK(rot(1,2) == Approx(-0.5).margin(1e-7));
    CHECK(rot(2,1) == Approx(-rot(1,2)).margin(1e-7));
    CHECK(rot(2,2) == Approx(rot(1,1)).margin(1e-7));

    rot = mmath::rotByYf(radian);
    CHECK(rot(0,0) == Approx(0.866025).margin(1e-7));
    CHECK(rot(0,2) == Approx(0.5).margin(1e-7));
    CHECK(rot(2,0) == Approx(-rot(0,2)).margin(1e-7));
    CHECK(rot(2,2) == Approx(rot(0,0)).margin(1e-7));

    rot = mmath::rotByZf(radian);
    CHECK(rot(0,0) == Approx(0.866025).margin(1e-7));
    CHECK(rot(0,1) == Approx(-0.5).margin(1e-7));
    CHECK(rot(1,0) == Approx(-rot(0,1)).margin(1e-7));
    CHECK(rot(1,1) == Approx(rot(0,0)).margin(1e-7));
}


TEST_CASE("Test drotation", "[drotation]")
{
    Eigen::Matrix3f drot;
    float radian = mmath::deg2radf(30);
    drot = mmath::dRotByXf(radian);
    CHECK(drot(0,0) == Approx(0).margin(1e-7));
    CHECK(drot(1,1) == Approx(-sin(radian)).margin(1e-7));
    CHECK(drot(1,2) == Approx(-cos(radian)).margin(1e-7));
    CHECK(drot(2,1) == Approx(-drot(1,2)).margin(1e-7));
    CHECK(drot(2,2) == Approx(drot(1,1)).margin(1e-7));

    drot = mmath::dRotByYf(radian);
    CHECK(drot(1,1) == Approx(0).margin(1e-7));
    CHECK(drot(0,0) == Approx(-sin(radian)).margin(1e-7));
    CHECK(drot(0,2) == Approx(cos(radian)).margin(1e-7));
    CHECK(drot(2,0) == Approx(-drot(0,2)).margin(1e-7));
    CHECK(drot(2,2) == Approx(drot(0,0)).margin(1e-7));

    drot = mmath::dRotByZf(radian);
    CHECK(drot(2,2) == Approx(0).margin(1e-7));
    CHECK(drot(0,0) == Approx(-sin(radian)).margin(1e-7));
    CHECK(drot(0,1) == Approx(-cos(radian)).margin(1e-7));
    CHECK(drot(1,0) == Approx(-drot(0,1)).margin(1e-7));
    CHECK(drot(1,1) == Approx(drot(0,0)).margin(1e-7));
}


TEST_CASE("Test drotation2delta", "[drotation]")
{
    Eigen::Matrix3f drot;
    float delta = mmath::deg2radf(30);
    Eigen::Matrix3f R_t1_2_tb =
            mmath::rotByZ<float>(-mmath::PI / 2)*
            mmath::rotByZ<float>(delta)*
            mmath::rotByY<float>(-mmath::PI / 2);
    CHECK(R_t1_2_tb(0,1) == Approx(cos(delta)).margin(1e-7));
    CHECK(R_t1_2_tb(0,2) == Approx(-sin(delta)).margin(1e-7));
    CHECK(R_t1_2_tb(1,1) == Approx(-R_t1_2_tb(0,2)).margin(1e-7));
    CHECK(R_t1_2_tb(1,2) == Approx(R_t1_2_tb(0,1)).margin(1e-7));
    CHECK(R_t1_2_tb(2,0) == Approx(1).margin(1e-7));

    Eigen::Matrix3f dR_t1_2_tb =
            mmath::rotByZ<float>(-mmath::PI / 2)*
            mmath::dRotByZ<float>(delta)*
            mmath::rotByY<float>(-mmath::PI / 2);
    CHECK(dR_t1_2_tb(0,1) == Approx(-sin(delta)).margin(1e-7));
    CHECK(dR_t1_2_tb(0,2) == Approx(-cos(delta)).margin(1e-7));
    CHECK(dR_t1_2_tb(1,1) == Approx(-dR_t1_2_tb(0,2)).margin(1e-7));
    CHECK(dR_t1_2_tb(1,2) == Approx(dR_t1_2_tb(0,1)).margin(1e-7));
    CHECK(dR_t1_2_tb(2,0) == Approx(0).margin(1e-7));

    Eigen::Matrix3f dR_te_2_te =dR_t1_2_tb.transpose();
    CHECK(dR_te_2_te(1,0) == Approx(-sin(delta)).margin(1e-7));
    CHECK(dR_te_2_te(2,0) == Approx(-cos(delta)).margin(1e-7));
    CHECK(dR_te_2_te(1,1) == Approx(-dR_te_2_te(2,0)).margin(1e-7));
    CHECK(dR_te_2_te(2,1) == Approx(dR_te_2_te(1,0)).margin(1e-7));
    CHECK(dR_te_2_te(0,2) == Approx(0).margin(1e-7));
}
