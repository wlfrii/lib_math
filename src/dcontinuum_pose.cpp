#include "../export/kine/dcontinuum_pose.h"

namespace mmath{
namespace continuum{

void dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta, Pose &dpose)
{
    Eigen::Matrix<kfloat, 3, 3> R_t1_2_tb =
            rotByZ<kfloat>(-PI / 2 + delta)*rotByY<kfloat>(-PI / 2);
    dpose.R = R_t1_2_tb * dRotByZ<kfloat>(theta) * R_t1_2_tb.transpose();
    if (abs(theta) < 1e-5) {
        dpose.t = { 0, 0, 0 };
    }
    else {
        dpose.t = L * R_t1_2_tb * Eigen::Vector<kfloat, 3>(
                    (theta*cos(theta) - sin(theta)) / (theta*theta),
                    (theta*sin(theta) - 1 + cos(theta)) / (theta*theta),
                    0);
    }
}


Pose dSingleSegmentPose2theta(kfloat L, kfloat theta, kfloat delta)
{
    Pose pose;
    dSingleSegmentPose2theta(L, theta, delta, pose);
    return pose;
}


void dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta, Pose &dpose)
{
    Eigen::Matrix<kfloat, 3, 3> R_t1_2_tb =
            rotByZ<kfloat>(-PI / 2 + delta)*rotByY<kfloat>(-PI / 2);
    Eigen::Matrix<kfloat, 3, 3> dR_t1_2_tb =
            rotByZ<kfloat>(-PI / 2)*dRotByZ<kfloat>(delta)*rotByY<kfloat>(-PI / 2);

    dpose.R = dR_t1_2_tb * rotByZ<kfloat>(theta) * R_t1_2_tb.transpose() +
            R_t1_2_tb * rotByZ<kfloat>(theta) * dR_t1_2_tb.transpose();
    if (abs(theta) < 1e-5) {
        dpose.t = { 0, 0, 0 };
    }
    else {
        kfloat rc = L / theta;
        dpose.t = rc * dR_t1_2_tb *
                Eigen::Vector<kfloat, 3>(sin(theta), 1 - cos(theta), 0);
    }
}


Pose dSingleSegmentPose2delta(kfloat L, kfloat theta, kfloat delta)
{
    Pose pose;
    dSingleSegmentPose2delta(L, theta, delta, pose);
    return pose;
}


void dSingleSegmentPose2L(kfloat L, kfloat theta, kfloat delta, Pose &dpose)
{
    Eigen::Matrix<kfloat, 3, 3> R_t1_2_tb =
            rotByZ<kfloat>(-PI / 2 + delta)*rotByY<kfloat>(-PI / 2);
    dpose.R = Eigen::Matrix3f::Zero();
    if (abs(theta) < 1e-5) {
        dpose.t = Eigen::Vector<kfloat, 3>(0, 0, 1);
    }
    else {
        kfloat rc = 1.0 / theta;
        dpose.t = rc * R_t1_2_tb *
                Eigen::Vector<kfloat, 3>(sin(theta), 1 - cos(theta), 0);
    }
}


Pose dSingleSegmentPose2L(kfloat L, kfloat theta, kfloat delta)
{
    Pose pose;
    dSingleSegmentPose2L(L, theta, delta, pose);
    return pose;
}


void dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose)
{
    dSingleSegmentPose2theta(L, theta, delta, dpose);
    if (abs(theta) > 1e-5) {
        dpose.t += Lr * Eigen::Vector<kfloat, 3>(
                    cos(delta)*cos(theta), sin(delta)*cos(theta), -sin(theta));
    }
}


Pose dSingleWithRigidSegmentPose2theta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr)
{
    Pose pose;
    dSingleWithRigidSegmentPose2theta(L, theta, delta, Lr, pose);
    return pose;
}


void dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr, Pose &dpose)
{
    dSingleSegmentPose2delta(L, theta, delta, dpose);
    if (abs(theta) > 1e-5) {
        dpose.t += Lr * Eigen::Vector<kfloat, 3>(
                    -sin(delta)*sin(theta), cos(delta)*sin(theta), 0);
    }
}


Pose dSingleWithRigidSegmentPose2delta(kfloat L, kfloat theta, kfloat delta,
                                       kfloat Lr)
{
    Pose pose;
    dSingleWithRigidSegmentPose2delta(L, theta, delta, Lr, pose);
    return pose;
}


void dSingleWithRigidSegmentPose2L(kfloat L, kfloat theta, kfloat delta,
                                   kfloat Lr, Pose &dpose)
{
    dSingleSegmentPose2L(L, theta, delta, dpose);
    if (abs(theta) > 1e-5) {
        dpose.t += Lr * Eigen::Vector<kfloat, 3>(
                    cos(delta)*sin(theta), sin(delta)*sin(theta), cos(theta));
    }
}


Pose dSingleWithRigidSegmentPose2L(kfloat L, kfloat theta, kfloat delta,
                                   kfloat Lr)
{
    Pose pose;
    dSingleWithRigidSegmentPose2L(L, theta, delta, Lr, pose);
    return pose;
}

}} // mmath::continuum
