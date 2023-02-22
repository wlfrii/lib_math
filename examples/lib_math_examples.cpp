#include <lib_math/lib_math.h>

void repeatCalcPose(uint16_t times);

int main()
{
    int count = 1;
    printf("====================== lib_math examples ======================\n");


    printf("Calculation %02d: ZYX Eular angles to rotation matrix.\n", count++);
    auto time_start = mmath::timer::getCurrentTimePoint();
    float alpha = 0.4;
    float beta  = 0.5;
    float gamma = 0.6;
    Eigen::Matrix3f R;
    R = mmath::rotByZf(gamma) * mmath::rotByYf(beta) * mmath::rotByXf(alpha);
    float ms = mmath::timer::getDurationSince(time_start);
    printf("When alpha=%.3f, beta=%.3f, gamma=%.3f\nRotation matrix =\n", alpha, beta, gamma);
    std::cout << R << std::endl;
    printf("[%.4f ms] elapsed.\n\n", ms);


    printf("Calculation %02d: The end pose of single continuum followed a rigid stem.\n", count++);
    time_start = mmath::timer::getCurrentTimePoint();
    float L = 30;
    float Lr = 10;
    float theta = mmath::deg2radf(30);
    float delta = mmath::deg2radf(50);
    mmath::Pose pose =
            mmath::continuum::calcSingleWithRigidSegmentPose(L, theta, delta, Lr);
    printf("When L=%.3f, theta=%.3f, delta=%.3f, Lr=%.3f\nEnd pose =\n", L, theta, delta, Lr);
    std::cout << pose << std::endl;
    printf("[%.4f ms] elapsed.\n\n", ms);


    uint16_t times = 10000;
    float time_toler = 0.01;
    printf("Calculation %02d: Repeat calc the end pose of single continuum [%d] times, "
           "if the time comsumption is greater than [%.3f ms], "
           "the overall elapsed time will be printed\n",
            count++, times, time_toler);
    MMATH_TIMER_COUNT_VOID_FUNC_TIME(repeatCalcPose(times), time_toler, "\n");

    return 0;
}


void repeatCalcPose(uint16_t times)
{
    for(uint16_t i = 0; i < times; i++) {
        float L = 30;
        float Lr = 10;
        float theta = mmath::deg2radf(30);
        float delta = mmath::deg2radf(50);
        mmath::Pose pose =
                mmath::continuum::calcSingleWithRigidSegmentPose(L, theta, delta, Lr);
    }
}