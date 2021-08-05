#include "../../export/kine/math_rt.h"

namespace mmath{

RT::RT(const EMat3f& R/* = EMat3f::Identity()*/, const EVec3f& t/* = {0,0,0}*/)
    : R(R)
    , t(t)
{}

RT::RT(const EMat3f& R)
    : R(R)
    , t(Eigen::Vector3f(0, 0, 0))
{}

RT::RT(float tx, float ty, float tz)
    : R(EMat3f::Identity())
    , t(EVec3f(tx, ty, tz))
{}

RT::RT(float data[16], bool is_row_fisrt/* = true*/)
{
    if (is_row_fisrt) {
        R << data[0], data[1], data[2], data[4], data[5], data[6], data[8], data[9], data[10];
        t << data[3], data[7], data[11];
    }
    else{
        R << data[0], data[4], data[8], data[1], data[5], data[9], data[2], data[6], data[10];
        t << data[12], data[13], data[14];
    }
}

RT& RT::operator= (const RT& rt)
{
    this->R = rt.R;
    this->t = rt.t;
    return *this;
}

RT RT::operator* (const RT& rt)
{
    RT ret;
    ret.R = this->R * rt.R;
    ret.t = this->R * rt.t + this->t;
    return ret;
}

RT& RT::operator*= (const RT& rt)
{
    this->t = this->R * rt.t + this->t;
    this->R = this->R * rt.R;
    return *this;
}

RT RT::inverse()
{
    RT inv;
    inv.R = R.transpose();
    inv.t = -R.transpose() * t;
    return inv;
}

char* RT::to_c_str() const
{
    static char tmp_cstr[200];
    sprintf(tmp_cstr, "row-1st, R=[%f,%f,%f,%f,%f,%f,%f,%f,%f],t=[%f,%f,%f]", R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), t[0], t[1], t[2]);
    return tmp_cstr;
}

} // mmath