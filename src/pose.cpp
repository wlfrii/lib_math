#include "../export/kine/pose.h"
#include <iomanip>

namespace mmath{

Pose& Pose::operator=(const Pose &pose)
{
    this->R = pose.R;
    this->t = pose.t;
    return *this;
}


Pose Pose::operator*(const Pose &pose) const
{
    Pose ret;
    ret.R = this->R * pose.R;
    ret.t = this->R * pose.t + this->t;
    return ret;
}


Pose& Pose::operator*=(const Pose &pose)
{
    this->t += this->R * pose.t;
    this->R *= pose.R;
    return *this;
}


Eigen::Vector<kfloat, 3> Pose::operator*(const Eigen::Vector<kfloat, 3>& p)
{
    Eigen::Vector<kfloat, 3> ret;
    ret = this->R * p + this->t;
    return ret;
}


std::ostream& operator<<(std::ostream &os, const Pose &pose)
{
    int w = 12;
    for(uint8_t i = 0; i < 3; i++) {
        os << std::setw(w) << pose.R(i, 0) << " "
           << std::setw(w) << pose.R(i, 1) << " "
           << std::setw(w) << pose.R(i, 2) << " "
           << std::setw(w) << pose.t[i];
        if(i < 2) os << "\n";
    }    
    return os;
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


Pose Pose::inverse() const
{
    Pose pose;
    pose.R = this->R.transpose();
    pose.t = -this->R.transpose()*this->t;
    return pose;
}


bool Pose::isUnitOrthogonal() const
{
    Pose pose = this->inverse() * (*this);
    auto& RR = pose.R;
    auto& tt = pose.t;
    
    auto isEqual = [](float val1, float val2) -> bool {
        return abs(val1 - val2) < 1e-5;
    };
    bool flag = 
        isEqual(RR(0,0), 1) && isEqual(RR(1,1), 1) && isEqual(RR(2,2), 1)
        && isEqual(RR(0,1), 0) && isEqual(RR(0,2), 0)
        && isEqual(RR(1,0), 0) && isEqual(RR(1,2), 0) 
        && isEqual(RR(2,0), 0) && isEqual(RR(2,1), 0)
        && isEqual(tt[0], 0) && isEqual(tt[1], 0) && isEqual(tt[2], 0);
        
    return flag;
}


void Pose::unitOrthogonalize()
{
    Eigen::Quaternion<kfloat> q = this->q();
    q.normalize();
    R = q.toRotationMatrix();
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
