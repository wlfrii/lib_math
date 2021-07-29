#include "../export/math_utility.h"

namespace mmath{

Eigen::Matrix3f skewSymmetric(Eigen::Vector3f &vec)
{
	Eigen::Matrix3f mat;
	mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
	return mat;
}
Eigen::Matrix3f skewSymmetric(Eigen::RowVector3f &vec)
{
	Eigen::Matrix3f mat;
	mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
	return mat;
}

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