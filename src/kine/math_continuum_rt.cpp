#include "../../export/kine/math_continuum_rt.h"
#include "../../export/math_rotation.h"

namespace mmath{

extern const double PI;

/* Calulate RT of a single continuum segment */
void calcSingleSegmentRT(float L, float theta, float delta, RT& rt)
{
	Eigen::Matrix3f R_t1_2_tb = rotByZ<float>(-PI / 2 - delta)*rotByY<float>(-PI / 2);
	rt.R = R_t1_2_tb * rotByZ<float>(theta) * R_t1_2_tb.transpose();
	if (abs(theta) < 1e-5) {
		rt.p = { 0, 0, L };
	}
	else {
		float rc = L / theta;
		rt.p = rc * R_t1_2_tb * Eigen::Vector3f(sin(theta), 1 - cos(theta), 0);
	}
}

RT calcSingleSegmentRT(float L, float theta, float delta)
{
	RT rt;
	calcSingleSegmentRT(L, theta, delta, rt);
	return rt;
}

void calcSingleSegmentRT(const ConfigSpc& q, RT& rt)
{
	calcSingleSegmentRT(q.length, q.theta, q.delta, rt);
}

RT calcSingleSegmentRT(const ConfigSpc& q)
{
	return calcSingleSegmentRT(q.length, q.theta, q.delta);
}


/* Calulate RT of a single continuum segment followed with a rigid segment */
void calcSingleWithRigidSegmentRT(float L, float theta, float delta, float Lr, RT& rt)
{
	rt = calcSingleSegmentRT(L, theta, delta);
	rt.p += Lr * rt.R.rightCols(0);
}

RT calcSingleWithRigidSegmentRT(float L, float theta, float delta, float Lr)
{
	RT rt;
	calcSingleWithRigidSegmentRT(L, Lr, theta, delta, rt);
	return rt;
}

void calcSingleWithRigidSegmentRT(const ConfigSpc& q, float Lr, RT& rt)
{
	calcSingleWithRigidSegmentRT(q.length, q.theta, q.delta, Lr, rt);
}

RT calcSingleWithRigidSegmentRT(const ConfigSpc& q, float Lr)
{
	return calcSingleWithRigidSegmentRT(q.length, q.theta, q.delta, Lr);
}

} // mmath