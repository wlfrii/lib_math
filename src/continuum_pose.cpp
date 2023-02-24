#include "../export/lib_math/kine/continuum_pose.h"
#include <iostream>
namespace mmath{
namespace continuum{

void calcSingleSegmentPose(kfloat L, kfloat theta, kfloat delta, Pose &pose)
{
    Eigen::Matrix<kfloat, 3, 3> R_t1_2_tb =
            rotByZ<kfloat>(-PI / 2 + delta)*rotByY<kfloat>(-PI / 2);
    pose.R = R_t1_2_tb * rotByZ<kfloat>(theta) * R_t1_2_tb.transpose();
    if (abs(theta) < 1e-5) {
        pose.t = Eigen::Vector<kfloat, 3>(0, 0, L);
    }
    else {
        kfloat rc = L / theta;
        pose.t = rc * R_t1_2_tb *
                Eigen::Vector<kfloat, 3>(sin(theta), 1 - cos(theta), 0);
    }
}


Pose calcSingleSegmentPose(kfloat L, kfloat theta, kfloat delta)
{
    Pose pose;
    calcSingleSegmentPose(L, theta, delta, pose);
    return pose;
}


void calcSingleSegmentPose(const ConfigSpc &q, Pose &pose)
{
    if(q.is_bend){
        calcSingleSegmentPose(q.length, q.theta, q.delta, pose);
    }
    else{
        pose.R = rotByZ<kfloat>(q.delta);
        pose.t = Eigen::Vector<kfloat, 3>(0, 0, q.length);
    }
}


Pose calcSingleSegmentPose(const ConfigSpc& q)
{
    Pose pose;
    calcSingleSegmentPose(q, pose);
    return pose;
}


void calcSingleWithRigidSegmentPose(kfloat L, kfloat theta, kfloat delta,
                                    kfloat Lr, Pose &pose)
{
    pose = calcSingleSegmentPose(L, theta, delta);
    pose.t += Lr * pose.R.rightCols(1);
}


Pose calcSingleWithRigidSegmentPose(kfloat L, kfloat theta, kfloat delta,
                                    kfloat Lr)
{
    Pose pose;
    calcSingleWithRigidSegmentPose(L, theta, delta, Lr, pose);
    return pose;
}


void calcSingleWithRigidSegmentPose(const ConfigSpc &q, kfloat Lr, Pose &pose)
{
    calcSingleSegmentPose(q, pose);
    pose.t += Lr * pose.R.rightCols(1);
}


Pose calcSingleWithRigidSegmentPose(const ConfigSpc &q, kfloat Lr)
{
    Pose pose;
    calcSingleWithRigidSegmentPose(q, Lr, pose);
    return pose;
}

}} // mmath::continuum