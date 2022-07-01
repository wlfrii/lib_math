#include "../export/kine/pose.h"

namespace mmath{

Pose &Pose::operator=(const Pose &pose)
{
    this->R = pose.R;
    this->t = pose.t;
    return *this;
}


Pose Pose::operator*(const Pose &pose)
{
    Pose ret;
    ret.R = this->R * pose.R;
    ret.t = this->R * pose.t + this->t;
    return ret;
}


Pose &Pose::operator*=(const Pose &pose)
{
    this->t += this->R * pose.t;
    this->R *= pose.R;
    return *this;
}


Eigen::Quaternion<kfloat> Pose::q() const
{
    return Eigen::Quaternion<kfloat>(R);
}


Eigen::Matrix<kfloat, 4, 4> Pose::T() const
{
    Eigen::Matrix<kfloat, 4, 4> T = Eigen::Matrix<kfloat, 4, 4>::Identity();
    T.topLeftCorner(3, 3) = R;
    T.topRightCorner(3, 1) = t;
    return T;
}


Pose Pose::inverse()
{
    Eigen::Matrix<kfloat, 4, 4> T;
    T.topLeftCorner(3, 3) = R.transpose();
    T.topRightCorner(3, 1) = -R.transpose()*t;
    return Pose(T);
}


void Pose::increase(const Eigen::Quaternion<kfloat> &dq,
                    const Eigen::Vector<kfloat, 3> &dt)
{
    Eigen::Quaternion<kfloat> q = this->q();
    q.coeffs() += dq.coeffs();
    q.normalize();
    R = q.toRotationMatrix();
    t += dt;
}


void Pose::decrease(const Eigen::Quaternion<kfloat> &dq,
                    const Eigen::Vector<kfloat, 3> &dt)
{
    Eigen::Quaternion<kfloat> q = this->q();
    q.coeffs() -= dq.coeffs();
    q.normalize();
    R = q.toRotationMatrix();
    t -= dt;
}


char* Pose::info() const
{
    static char tmp_cstr[200];
    sprintf(tmp_cstr, "row-1st, R=[%f,%f,%f,%f,%f,%f,%f,%f,%f],t=[%f,%f,%f]",
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2), t[0], t[1], t[2]);
    return tmp_cstr;
}

} // mmath
