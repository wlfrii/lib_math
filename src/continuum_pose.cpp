#include "../export/kine/continuum_pose.h"

namespace mmath{
namespace continuum{

void calcSingleSegmentPosef(float L, float theta, float delta, Pose<float>& pose)
{
    calcSingleSegmentPose<float>(L, theta, delta, pose);
}


Pose<float> calcSingleSegmentPosef(float L, float theta, float delta)
{
    return calcSingleSegmentPose<float>(L, theta, delta);
}


void calcSingleSegmentPosef(const ConfigSpc<float>& q, Pose<float>& pose)
{
    calcSingleSegmentPose<float>(q, pose);
}


Pose<float> calcSingleSegmentPosef(const ConfigSpc<float>& q)
{
    return calcSingleSegmentPose<float>(q);
}


void calcSingleWithRigidSegmentPosef(float L, float theta, float delta, float Lr, Pose<float>& pose)
{
    calcSingleWithRigidSegmentPose<float>(L, Lr, theta, delta, pose);
}


Pose<float> calcSingleWithRigidSegmentPosef(float L, float theta, float delta, float Lr)
{
    return calcSingleWithRigidSegmentPose<float>(L, theta, delta, Lr);
}


void calcSingleWithRigidSegmentPosef(const ConfigSpc<float>& q, float Lr, Pose<float>& pose)
{
    calcSingleWithRigidSegmentPose<float>(q, Lr, pose);
}


Pose<float> calcSingleWithRigidSegmentPosef(const ConfigSpc<float>& q, float Lr)
{
    return calcSingleWithRigidSegmentPose<float>(q, Lr);
}


}} // mmath::continuum
