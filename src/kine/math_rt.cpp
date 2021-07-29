#include "../../export/kine/math_rt.h"

namespace mmath{

RT::RT(const EMat3f &R/* = EMat3f::Identity()*/, const EVec3f &t/* = {0,0,0}*/)
    : R(R)
    , p(t)
{}

RT::RT(float px, float py, float pz)
    : R(EMat3f::Identity())
    , p(EVec3f(px, py, pz))
{}

RT::RT(float pts[16], bool is_row_fisrt/* = true*/)
{
    if (is_row_fisrt) {
        R << pts[0], pts[1], pts[2], pts[4], pts[5], pts[6], pts[8], pts[9], pts[10];
        p << pts[3], pts[7], pts[11];
    }
    else{
        R << pts[0], pts[4], pts[8], pts[1], pts[5], pts[9], pts[2], pts[6], pts[10];
        p << pts[12], pts[13], pts[14];
    }
}

RT& RT::operator= (const RT& rt)
{
    this->R = rt.R;
    this->p = rt.p;
    return *this;
}

RT RT::inverse()
{
    RT inv;
    inv.R = R.transpose();
    inv.p = -R.transpose() * p;
    return inv;
}

char* RT::to_c_str() const
{
    static char tmp_cstr[200];
    sprintf(tmp_cstr, "row-1st, R=[%f,%f,%f,%f,%f,%f,%f,%f,%f],t=[%f,%f,%f]", R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), p[0], p[1], p[2]);
    return tmp_cstr;
}

} // mmath