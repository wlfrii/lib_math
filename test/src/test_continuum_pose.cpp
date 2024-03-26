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


TEST_CASE("Test continuum dpose2theta", "[continuum]")
{
    float L = 30;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::dSingleSegmentPose2theta(L, theta, delta);

    float val = -powf(cos(delta), 2)*sin(theta);
    CHECK(pose.R(0,0) == Approx(val).margin(1e-6));
    val = -sin(delta)*cos(delta)*sin(theta);
    CHECK(pose.R(0,1) == Approx(val).margin(1e-6));
    val = cos(delta)*cos(theta);
    CHECK(pose.R(0,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    val = -powf(sin(delta), 2)*sin(theta);
    CHECK(pose.R(1,1) == Approx(val).margin(1e-6));
    val = sin(delta) * cos(theta);
    CHECK(pose.R(1,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(-sin(theta)).margin(1e-6));

    val = L * cos(delta) * (theta*sin(theta) - 1 + cos(theta)) / (theta*theta);
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = L * sin(delta) * (theta*sin(theta) - 1 + cos(theta)) / (theta*theta);
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    val = L * (theta*cos(theta) - sin(theta)) / (theta*theta);
    CHECK(pose.t[2] == Approx(val).margin(1e-6));
}


TEST_CASE("Test continuum dpose2theta with Lr", "[continuum]")
{
    float L = 30;
    float Lr = 10;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::dSingleWithRigidSegmentPose2theta(
                L, theta, delta, Lr);

    float val = -powf(cos(delta), 2)*sin(theta);
    CHECK(pose.R(0,0) == Approx(val).margin(1e-6));
    val = -sin(delta)*cos(delta)*sin(theta);
    CHECK(pose.R(0,1) == Approx(val).margin(1e-6));
    val = cos(delta)*cos(theta);
    CHECK(pose.R(0,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    val = -powf(sin(delta), 2)*sin(theta);
    CHECK(pose.R(1,1) == Approx(val).margin(1e-6));
    val = sin(delta) * cos(theta);
    CHECK(pose.R(1,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(-sin(theta)).margin(1e-6));

    val = L * cos(delta) * (theta*sin(theta) - 1 + cos(theta)) / (theta*theta) +
            Lr * cos(delta) * cos(theta);
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = L * sin(delta) * (theta*sin(theta) - 1 + cos(theta)) / (theta*theta) +
            Lr * sin(delta) * cos(theta);
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    val = L * (theta*cos(theta) - sin(theta)) / (theta*theta) - Lr*sin(theta);
    CHECK(pose.t[2] == Approx(val).margin(1e-6));
}


TEST_CASE("Test continuum dpose2delta", "[continuum]")
{
    float L = 30;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::dSingleSegmentPose2delta(L, theta, delta);

    float val = 2*cos(delta)*(-sin(delta))*cos(theta) + 2*sin(delta)*cos(delta);
    CHECK(pose.R(0,0) == Approx(val).margin(1e-6));
    val = (cos(delta)*cos(delta) - sin(delta)*sin(delta) )*(cos(theta) - 1);
    CHECK(pose.R(0,1) == Approx(val).margin(1e-6));
    val = -sin(delta)*sin(theta);
    CHECK(pose.R(0,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    val = 2*sin(delta)*cos(delta)*cos(theta) - 2*cos(delta)*sin(delta);
    CHECK(pose.R(1,1) == Approx(val).margin(1e-6));
    val = cos(delta) * sin(theta);
    CHECK(pose.R(1,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(0).margin(1e-6));

    val = L / theta * (-sin(delta)) * (1 - cos(theta));
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = L / theta * cos(delta) * (1 - cos(theta));
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    CHECK(pose.t[2] == Approx(0).margin(1e-6));
}


TEST_CASE("Test continuum dpose2L", "[continuum]")
{
    float L = 30;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::dSingleSegmentPose2L(L, theta, delta);

    CHECK(pose.R(0,0) == Approx(0).margin(1e-6));
    CHECK(pose.R(0,1) == Approx(0).margin(1e-6));
    CHECK(pose.R(0,2) == Approx(0).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    CHECK(pose.R(1,1) == Approx(0).margin(1e-6));
    CHECK(pose.R(1,2) == Approx(0).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(0).margin(1e-6));

    mmath::kfloat val = 1.0 / theta * cos(delta) * (1 - cos(theta));
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = 1.0 / theta * sin(delta) * (1 - cos(theta));
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    val = 1.0 / theta * sin(theta);
    CHECK(pose.t[2] == Approx(val).margin(1e-6));
}


TEST_CASE("Test continuum dpose2delta with Lr", "[continuum]")
{
    float L = 30;
    float Lr = 10;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::dSingleWithRigidSegmentPose2delta(
                L, theta, delta, Lr);

    mmath::kfloat val = 2*cos(delta)*(-sin(delta))*cos(theta) + 
                        2*sin(delta)*cos(delta);
    CHECK(pose.R(0,0) == Approx(val).margin(1e-6));
    val = (cos(delta)*cos(delta) - sin(delta)*sin(delta) )*(cos(theta) - 1);
    CHECK(pose.R(0,1) == Approx(val).margin(1e-6));
    val = -sin(delta)*sin(theta);
    CHECK(pose.R(0,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    val = 2*sin(delta)*cos(delta)*cos(theta) - 2*cos(delta)*sin(delta);
    CHECK(pose.R(1,1) == Approx(val).margin(1e-6));
    val = cos(delta) * sin(theta);
    CHECK(pose.R(1,2) == Approx(val).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(0).margin(1e-6));

    val = L/theta * (-sin(delta)) * (1-cos(theta)) + Lr*(-sin(delta))*sin(theta);
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = L/theta * cos(delta) * (1-cos(theta)) + Lr*cos(delta)*sin(theta);
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    CHECK(pose.t[2] == Approx(0).margin(1e-6));
}


TEST_CASE("Test continuum dpose2L with Lr", "[continuum]")
{
    mmath::kfloat L = 30;
    mmath::kfloat Lr = 10;
    mmath::kfloat theta = mmath::deg2rad(30);
    mmath::kfloat delta = mmath::deg2rad(50);
    mmath::Pose pose = mmath::continuum::dSingleWithRigidSegmentPose2L(
                L, theta, delta, Lr);

    CHECK(pose.R(0,0) == Approx(0).margin(1e-6));
    CHECK(pose.R(0,1) == Approx(0).margin(1e-6));
    CHECK(pose.R(0,2) == Approx(0).margin(1e-6));

    CHECK(pose.R(1,0) == Approx(pose.R(0,1)).margin(1e-6));
    CHECK(pose.R(1,1) == Approx(0).margin(1e-6));
    CHECK(pose.R(1,2) == Approx(0).margin(1e-6));

    CHECK(pose.R(2,0) == Approx(-pose.R(0,2)).margin(1e-6));
    CHECK(pose.R(2,1) == Approx(-pose.R(1,2)).margin(1e-6));
    CHECK(pose.R(2,2) == Approx(0).margin(1e-6));

    mmath::kfloat val = 1.0 / theta * cos(delta) * (1 - cos(theta)) + 
                        Lr * cos(delta)*sin(theta);
    CHECK(pose.t[0] == Approx(val).margin(1e-6));
    val = 1.0 / theta * sin(delta) * (1 - cos(theta)) + Lr * sin(delta)*sin(theta);
    CHECK(pose.t[1] == Approx(val).margin(1e-6));
    val = 1.0 / theta * sin(theta) + Lr * cos(theta);
    CHECK(pose.t[2] == Approx(val).margin(1e-6));
}


TEST_CASE("Test continuum pose*=", "[continuum]")
{
    mmath::kfloat phi = -0.284696;
    mmath::kfloat L = 1.08657;
    mmath::continuum::ConfigSpc config1(0, phi, L, false);
    auto pose1 = mmath::continuum::calcSingleSegmentPose(config1);
    CHECK(pose1.R(0,0) == Approx(0.959747).margin(1e-6));
    CHECK(pose1.R(0,1) == Approx(0.280866).margin(1e-6));
    CHECK(pose1.R(1,0) == Approx(-pose1.R(0,1)).margin(1e-6));
    CHECK(pose1.R(1,1) == Approx(pose1.R(0,0)).margin(1e-6));
    CHECK(pose1.R(2,2) == Approx(1).margin(1e-6));
    CHECK(pose1.t[2] == Approx(L).margin(1e-6));

    mmath::kfloat theta = 0.169437;
    mmath::kfloat delta = -1.879464;
    mmath::kfloat length = 28.012699;
    mmath::continuum::ConfigSpc config2(theta, delta, length, true);
    auto pose2 = mmath::continuum::calcSingleSegmentPose(config2);
    CHECK(pose2.R(0,0) == Approx(0.998678).margin(1e-6));
    CHECK(pose2.R(0,1) == Approx(-0.0041447).margin(1e-6));
    CHECK(pose2.R(0,2) == Approx(-0.0512273).margin(1e-6));
    CHECK(pose2.R(1,0) == Approx(pose2.R(0,1)).margin(1e-6));
    CHECK(pose2.R(1,1) == Approx(0.987002).margin(1e-6));
    CHECK(pose2.R(1,2) == Approx(-0.160658).margin(1e-6));
    CHECK(pose2.R(2,0) == Approx(-pose2.R(0,2)).margin(1e-6));
    CHECK(pose2.R(2,1) == Approx(-pose2.R(1,2)).margin(1e-6));
    CHECK(pose2.R(2,2) == Approx(0.98568).margin(1e-6));
    CHECK(pose2.t[0] == Approx(-0.719228).margin(1e-6));
    CHECK(pose2.t[1] == Approx(-2.25563).margin(1e-6));
    CHECK(pose2.t[2] == Approx(27.8789).margin(1e-6));

    mmath::Pose pose3 = pose1 * pose2;
    CHECK(pose3.R(0,0) == Approx(0.957315).margin(1e-6));
    CHECK(pose3.R(0,1) == Approx(0.273237).margin(1e-6));
    CHECK(pose3.R(0,2) == Approx(-0.0942885).margin(1e-6));
    CHECK(pose3.R(1,0) == Approx(-0.284472).margin(1e-6));
    CHECK(pose3.R(1,1) == Approx(0.948436).margin(1e-6));
    CHECK(pose3.R(1,2) == Approx(-0.139803).margin(1e-6));
    CHECK(pose3.R(2,0) == Approx(0.0512273).margin(1e-6));
    CHECK(pose3.R(2,1) == Approx(0.160658).margin(1e-6));
    CHECK(pose3.R(2,2) == Approx(0.98568).margin(1e-6));
    CHECK(pose3.t[0] == Approx(-1.32381).margin(1e-6));
    CHECK(pose3.t[1] == Approx(-1.96283).margin(1e-6));
    CHECK(pose3.t[2] == Approx(28.9654).margin(1e-6));

    pose1 *= pose2;
    CHECK(pose1.R(0,0) == Approx(0.957315).margin(1e-6));
    CHECK(pose1.R(0,1) == Approx(0.273237).margin(1e-6));
    CHECK(pose1.R(0,2) == Approx(-0.0942885).margin(1e-6));
    CHECK(pose1.R(1,0) == Approx(-0.284472).margin(1e-6));
    CHECK(pose1.R(1,1) == Approx(0.948436).margin(1e-6));
    CHECK(pose1.R(1,2) == Approx(-0.139803).margin(1e-6));
    CHECK(pose1.R(2,0) == Approx(0.0512273).margin(1e-6));
    CHECK(pose1.R(2,1) == Approx(0.160658).margin(1e-6));
    CHECK(pose1.R(2,2) == Approx(0.98568).margin(1e-6));
    CHECK(pose1.t[0] == Approx(-1.32381).margin(1e-6));
    CHECK(pose1.t[1] == Approx(-1.96283).margin(1e-6));
    CHECK(pose1.t[2] == Approx(28.9654).margin(1e-6));
}


TEST_CASE("Test continuum Jacobian", "[continuum]")
{
    float L = 30;
    float Lr = 10;
    float theta = mmath::deg2rad(30);
    float delta = mmath::deg2rad(50);


    Eigen::Matrix<mmath::kfloat, 3, 3> Jv, Jw;
    mmath::continuum::calcVariableLengthSegmentJacobian(
                L, theta, delta, Jv, Jw);

    mmath::kfloat val = 0;
    val = L*cos(delta)*(theta*sin(theta)+cos(theta) - 1) / (theta*theta);
    CHECK(Jv(0, 0) == Approx(val).margin(1e-6));

    val = L*sin(delta)*(cos(theta)-1)/theta;
    CHECK(Jv(0, 1) == Approx(val).margin(1e-6));

    val = L*sin(delta)*(theta*sin(theta)+cos(theta) - 1) / (theta*theta);
    CHECK(Jv(1, 0) == Approx(val).margin(1e-6));
    val = -L*cos(delta)*(cos(theta) - 1)/theta;
    CHECK(Jv(1, 1) == Approx(val).margin(1e-6));
    val = L*(theta*cos(theta) - sin(theta)) / (theta*theta);
    CHECK(Jv(2, 0) == Approx(val).margin(1e-6));
    CHECK(Jv(2, 1) == Approx(0).margin(1e-6));


    mmath::continuum::calcVariableLengthWithRigidSegmentJacobian(
                L, theta, delta, Lr, Jv, Jw);

    val = L*cos(delta)*(theta*sin(theta)+cos(theta) - 1) / (theta*theta) +
            Lr*cos(theta)*cos(delta);
    CHECK(Jv(0, 0) == Approx(val).margin(1e-6));

    val = L*sin(delta)*(cos(theta)-1)/theta - Lr*sin(theta)*sin(delta);
    CHECK(Jv(0, 1) == Approx(val).margin(1e-6));

    val = L*sin(delta)*(theta*sin(theta)+cos(theta) - 1) / (theta*theta) +
            Lr*cos(theta)*sin(delta);
    CHECK(Jv(1, 0) == Approx(val).margin(1e-6));
    val = -L*cos(delta)*(cos(theta) - 1)/theta + Lr*sin(theta)*cos(delta);
    CHECK(Jv(1, 1) == Approx(val).margin(1e-6));
    val = L*(theta*cos(theta) - sin(theta)) / (theta*theta) - Lr*sin(theta);
    CHECK(Jv(2, 0) == Approx(val).margin(1e-6));
    CHECK(Jv(2, 1) == Approx(0).margin(1e-6));

    CHECK(Jv(0, 2) == Approx(cos(delta)*(1 - cos(theta))/theta).margin(1e-6));
    CHECK(Jv(1, 2) == Approx(sin(delta)*(1 - cos(theta))/theta).margin(1e-6));
    CHECK(Jv(2, 2) == Approx(sin(theta)/theta).margin(1e-6));

    CHECK(Jw(0, 0) == Approx(-sin(delta)).margin(1e-6));
    CHECK(Jw(0, 1) == Approx(-sin(theta)*cos(delta)).margin(1e-6));
    CHECK(Jw(1, 0) == Approx(cos(delta)).margin(1e-6));
    CHECK(Jw(1, 1) == Approx(-sin(theta)*sin(delta)).margin(1e-6));
    CHECK(Jw(2, 0) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 1) == Approx(1 - cos(theta)).margin(1e-6));

    CHECK(Jw(0, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(1, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 2) == Approx(0).margin(1e-6));


    theta = 0.f;
    mmath::continuum::calcVariableLengthSegmentJacobian(
                L, theta, delta, Jv, Jw);

    val = L*cos(delta)*0.5;
    CHECK(Jv(0, 0) == Approx(val).margin(1e-6));
    CHECK(Jv(0, 1) == Approx(0).margin(1e-6));
    val = L*sin(delta)*0.5;
    CHECK(Jv(1, 0) == Approx(val).margin(1e-6));
    CHECK(Jv(1, 1) == Approx(0).margin(1e-6));
    CHECK(Jv(2, 0) == Approx(0).margin(1e-6));
    CHECK(Jv(2, 1) == Approx(0).margin(1e-6));

    CHECK(Jv(0, 2) == Approx(0).margin(1e-6));
    CHECK(Jv(1, 2) == Approx(0).margin(1e-6));
    CHECK(Jv(2, 2) == Approx(1).margin(1e-6));

    CHECK(Jw(0, 0) == Approx(-sin(delta)).margin(1e-6));
    CHECK(Jw(0, 1) == Approx(0).margin(1e-6));
    CHECK(Jw(1, 0) == Approx(cos(delta)).margin(1e-6));
    CHECK(Jw(1, 1) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 0) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 1) == Approx(0).margin(1e-6));

    CHECK(Jw(0, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(1, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 2) == Approx(0).margin(1e-6));


    mmath::continuum::ConfigSpc q(0.1, 0.1, 20);

    mmath::continuum::calcVariableLengthWithRigidSegmentJacobian(q, 10, Jv, Jw);
    CHECK(Jv(0, 0) == Approx(0).margin(1e-6));
    CHECK(Jv(0, 1) == Approx(0).margin(1e-6));
    CHECK(Jv(0, 2) == Approx(0).margin(1e-6));
    CHECK(Jv(1, 0) == Approx(0).margin(1e-6));
    CHECK(Jv(1, 1) == Approx(0).margin(1e-6));
    CHECK(Jv(1, 2) == Approx(0).margin(1e-6));
    CHECK(Jv(2, 0) == Approx(0).margin(1e-6));
    CHECK(Jv(2, 1) == Approx(0).margin(1e-6));
    CHECK(Jv(2, 2) == Approx(1).margin(1e-6));
    CHECK(Jw(0, 0) == Approx(0).margin(1e-6));
    CHECK(Jw(0, 1) == Approx(0).margin(1e-6));
    CHECK(Jw(0, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(1, 0) == Approx(0).margin(1e-6));
    CHECK(Jw(1, 1) == Approx(0).margin(1e-6));
    CHECK(Jw(1, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 0) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 1) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 2) == Approx(0).margin(1e-6));


    q = mmath::continuum::ConfigSpc(0.816344, -1.516124, 25.989237, 1);
    mmath::continuum::calcVariableLengthWithRigidSegmentJacobian(
                q, 30, Jv, Jw);
    CHECK(Jv(0, 0) == Approx(1.71887130341869).margin(1e-6));
    CHECK(Jv(0, 1) == Approx(31.8436451577274).margin(1e-6));
    CHECK(Jv(0, 2) == Approx(0.0210931071586948).margin(1e-6));
    CHECK(Jv(1, 0) == Approx(-31.4081813901534).margin(1e-6));
    CHECK(Jv(1, 1) == Approx(1.74270287024721).margin(1e-6));
    CHECK(Jv(1, 2) == Approx(-0.385425095179947).margin(1e-6));
    CHECK(Jv(2, 0) == Approx(-28.4712407349619).margin(1e-6));
    CHECK(Jv(2, 1) == Approx(0).margin(1e-6));
    CHECK(Jv(2, 2) == Approx(0.892573165460230).margin(1e-6));

    CHECK(Jw(0, 0) == Approx(0.998505840575140).margin(1e-6));
    CHECK(Jw(0, 1) == Approx(-0.0398169703077830).margin(1e-6));
    CHECK(Jw(0, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(1, 0) == Approx(0.0546450943574414).margin(1e-6));
    CHECK(Jw(1, 1) == Approx(0.727558033778273).margin(1e-6));
    CHECK(Jw(1, 2) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 0) == Approx(0).margin(1e-6));
    CHECK(Jw(2, 1) == Approx(0.315110288907621).margin(1e-6));
    CHECK(Jw(2, 2) == Approx(0).margin(1e-6));

    q = mmath::continuum::ConfigSpc(1.237475, 0.113694, 19.4, 1);
    Eigen::Matrix<mmath::kfloat, 3, 2> Jv2, Jw2;
    mmath::continuum::calcSingleWithRigidSegmentJacobian(q, 5.71, Jv2, Jw2);
    CHECK(Jv2(0, 0) == Approx(8.10612376841146).margin(1e-6));
    CHECK(Jv2(0, 1) == Approx(-1.80878099695414).margin(1e-6));
    CHECK(Jv2(1, 0) == Approx(0.925609319341581).margin(1e-6));
    CHECK(Jv2(1, 1) == Approx(15.8405952974746).margin(1e-6));
    CHECK(Jv2(2, 0) == Approx(-12.2377834948133).margin(1e-6));
    CHECK(Jv2(2, 1) == Approx(0).margin(1e-6));

    CHECK(Jw2(0, 0) == Approx(-0.113449217316130).margin(1e-6));
    CHECK(Jw2(0, 1) == Approx(-0.938860014790518).margin(1e-6));
    CHECK(Jw2(1, 0) == Approx(0.993543796261824).margin(1e-6));
    CHECK(Jw2(1, 1) == Approx(-0.107205071631614).margin(1e-6));
    CHECK(Jw2(2, 0) == Approx(0).margin(1e-6));
    CHECK(Jw2(2, 1) == Approx(0.672816648889328).margin(1e-6));
}
