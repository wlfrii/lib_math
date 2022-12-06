#include <catch2/catch.hpp>
#include <lib_math/lib_math.h>

TEST_CASE("Test pose", "[kine]")
{
    mmath::Pose pose(
                0.000292, 0.074980, 0.997186, -16.154986,
                0.997980,0.063391,-0.004474, -2.498408,
                0.063548,0.995169, -0.074848, 38.223778);
    REQUIRE(pose.isUnitOrthogonal() == false);
    pose.unitOrthogonalize();
    REQUIRE(pose.isUnitOrthogonal() == true);
}
