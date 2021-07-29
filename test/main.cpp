#include <lib_math/lib_math.h>

int main()
{
	int a = 10;
	int *p = &a;
	float t1 = mmath::deg2rad<float>(1); // 
	
	Eigen::Matrix3f res = mmath::rotByX<float>(1)*mmath::rotByY<float>(2)*mmath::rotByZ<float>(3);

	std::vector<float> tt;;
	tt = mmath::linespace(1, 2, 100, tt);

	return 0;
}