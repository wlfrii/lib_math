#include "../export/math_rotation.h"

namespace mmath{

Eigen::Matrix3f createRotMatByVecZ(Eigen::Vector3f z)
{
	z /= z.norm();

	Eigen::Vector3f x(1, 0, 0);
	auto y = z.cross(x);
	y /= y.norm();
	x = y.cross(z);
	x /= x.norm();

	Eigen::Matrix3f mat;
	mat.col(0) = x;
	mat.col(1) = y;
	mat.col(2) = z;
	return mat;
};

} // mmath