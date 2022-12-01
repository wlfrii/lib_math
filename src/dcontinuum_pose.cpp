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


/*---------------------------------------------------------------------------*/
/*         Calculate the Jacobian w.r.t Velocity and Angular-Velocity        */
/*---------------------------------------------------------------------------*/


void calcSingleSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta,
        Eigen::Matrix<kfloat, 3, 2>& Jv, Eigen::Matrix<kfloat, 3, 2>& Jw)
{
    if(abs(theta) <= 1e-5) {
        Jv << L * cos(delta) * 0.5, 0,
                L * sin(delta) * 0.5, 0,
                0, 0;
        Jw << -sin(delta), 0,
                cos(delta), 0,
                0, 0;
    }
    else{
        Jv(0, 0) = L*cos(delta)*(theta*sin(theta) + cos(theta) - 1)/theta*theta;
        Jv(0, 1) = L*sin(delta)*(cos(theta) - 1)/theta;
        Jv(1, 0) = L*sin(delta)*(theta*sin(theta) + cos(theta) - 1)/theta*theta;
        Jv(1, 1) = -L*cos(delta)*(cos(theta) - 1)/theta;
        Jv(2, 0) = L*(theta*cos(theta) - sin(theta))/theta*theta;
        Jv(2, 1) = 0;

        Jw << -sin(delta), -sin(theta)*cos(delta),
                cos(delta), -sin(theta)*sin(delta),
                0, 1 - cos(theta);
    }
}


void calcVariableLengthSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta,
        Eigen::Matrix<kfloat, 3, 3>& Jv, Eigen::Matrix<kfloat, 3, 3>& Jw)
{
    Eigen::Matrix<kfloat, 3, 2> Jv1, Jw1;
    calcSingleSegmentJacobian(L, theta, delta, Jv1, Jw1);
    Jv.block(0, 0, 3, 2) = Jv1;
    Jw.block(0, 0, 3, 2) = Jw1;
    Jw.col(2) = Eigen::Vector<kfloat, 3>(0, 0, 0);
    if(abs(theta) <= 1e-5) {
        Jv.col(2) = Eigen::Vector<kfloat, 3>(0, 0, 1);
    }
    else{
        Jv.col(2) = Eigen::Vector<kfloat, 3>(
                    cos(delta)*(1 - cos(theta)) / theta,
                    sin(delta)*(1 - cos(theta)) / theta,
                    sin(theta) / theta);
    }
}


void calcSingleWithRigidSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta, kfloat Lr,
        Eigen::Matrix<kfloat, 3, 2>& Jv, Eigen::Matrix<kfloat, 3, 2>& Jw)
{
    calcSingleSegmentJacobian(L, theta, delta, Jv, Jw);
    if(abs(theta) <= 1e-5) {
        Jv(0, 0) += Lr*cos(delta);
        Jv(1, 0) += Lr*sin(delta);
    }
    else{
        Jv(0, 0) += Lr*cos(theta)*cos(delta);
        Jv(0, 1) += -Lr*sin(theta)*sin(delta);
        Jv(1, 0) += Lr*cos(theta)*sin(delta);
        Jv(1, 1) += Lr*sin(theta)*cos(delta);
        Jv(2, 0) += -Lr*sin(theta);
    }
}


void calcVariableLengthWithRigidSegmentJacobian(
        kfloat L, kfloat theta, kfloat delta, kfloat Lr,
        Eigen::Matrix<kfloat, 3, 3>& Jv, Eigen::Matrix<kfloat, 3, 3>& Jw)
{
    Eigen::Matrix<kfloat, 3, 2> Jv1, Jw1;
    calcSingleWithRigidSegmentJacobian(L, theta, delta, Lr, Jv1, Jw1);
    Jv.block(0, 0, 3, 2) = Jv1;
    Jw.block(0, 0, 3, 2) = Jw1;
    Jw.col(2) = Eigen::Vector<kfloat, 3>(0, 0, 0);
    if(abs(theta) <= 1e-5) {
        Jv.col(2) = Eigen::Vector<kfloat, 3>(0, 0, 1);
    }
    else{
        Jv.col(2) = Eigen::Vector<kfloat, 3>(
                    cos(delta)*(1 - cos(theta)) / theta,
                    sin(delta)*(1 - cos(theta)) / theta,
                    sin(theta) / theta);
    }
}

}} // mmath::continuum
