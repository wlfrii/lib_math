#include <catch2/catch.hpp>
#include <lib_math/lib_math.h>

TEST_CASE("Test continuum pose", "[continuum]")
{
    float L = 30;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::calcSingleSegmentPose(L, theta, delta);

    float val = powf(cos(delta), 2)*cos(theta) + powf(sin(delta), 2);
    CHECK(pose.R(0,0) == Approx(val).margin(1e-6));
    val = sin(delta)*cos(delta)*(cos(theta) - 1);
    CHECK(pose.R(0,1) == Approx(val).margin(1e-6));
    val = cos(delta)*sin(theta);
    CHECK(pose.R(0,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    val = powf(sin(delta), 2)*cos(theta) + powf(cos(delta), 2);
    CHECK(pose.R(1,1) == Approx(val).margin(1e-6));
    val = sin(delta) * sin(theta);
    CHECK(pose.R(1,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(cos(theta)).margin(1e-6));

    val = L / theta * cos(delta) * (1 - cos(theta));
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = L / theta * sin(delta) * (1 - cos(theta));
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    val = L / theta * sin(theta);
    CHECK(pose.t[2] == Approx(val).margin(1e-6));
}


TEST_CASE("Test continuum pose with Lr", "[continuum]")
{
    float L = 30;
    float Lr = 10;
    float theta = mmath::deg2radf(30);
    float delta = mmath::deg2radf(50);
    mmath::Pose pose =
            mmath::continuum::calcSingleWithRigidSegmentPose(L, theta, delta, Lr);

    float val = powf(cos(delta), 2)*cos(theta) + powf(sin(delta), 2);
    CHECK(pose.R(0,0) == Approx(val).margin(1e-6));
    val = sin(delta)*cos(delta)*(cos(theta) - 1);
    CHECK(pose.R(0,1) == Approx(val).margin(1e-6));
    val = cos(delta)*sin(theta);
    CHECK(pose.R(0,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    val = powf(sin(delta), 2)*cos(theta) + powf(cos(delta), 2);
    CHECK(pose.R(1,1) == Approx(val).margin(1e-6));
    val = sin(delta) * sin(theta);
    CHECK(pose.R(1,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(cos(theta)).margin(1e-6));

    val = L / theta * cos(delta) * (1 - cos(theta)) + Lr * cos(delta)*sin(theta);
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = L / theta * sin(delta) * (1 - cos(theta)) + Lr * sin(delta)*sin(theta);
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    val = L / theta * sin(theta) + Lr * cos(theta);
    CHECK(pose.t[2] == Approx(val).margin(1e-6));
}


TEST_CASE("Test continuum dpose", "[continuum]")
{
    float L = 30;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::dSingleSegmentPose2theta(L, theta, delta);

    float val = powf(cos(delta), 2)*cos(theta) + powf(sin(delta), 2);
    CHECK(pose.R(0,0) == Approx(val).margin(1e-6));
    val = sin(delta)*cos(delta)*(cos(theta) - 1);
    CHECK(pose.R(0,1) == Approx(val).margin(1e-6));
    val = cos(delta)*sin(theta);
    CHECK(pose.R(0,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    val = powf(sin(delta), 2)*cos(theta) + powf(cos(delta), 2);
    CHECK(pose.R(1,1) == Approx(val).margin(1e-6));
    val = sin(delta) * sin(theta);
    CHECK(pose.R(1,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(cos(theta)).margin(1e-6));

    val = L / theta * cos(delta) * (1 - cos(theta));
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = L / theta * sin(delta) * (1 - cos(theta));
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    val = L / theta * sin(theta);
    CHECK(pose.t[2] == Approx(val).margin(1e-6));
}
