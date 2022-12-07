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


TEST_CASE("Test dpose", "[kine]")
{
    float alpha = 0.1;
    float beta  = 0.2;
    float gamma = 0.3;

    // dR to alpha
    Eigen::Matrix3f dR = mmath::rotByZf(gamma) * mmath::rotByYf(beta)
            * mmath::dRotByXf(alpha);
    CHECK(dR(0, 0) == Approx(0).margin(1e-6));
    float val = sin(gamma)*sin(alpha) + cos(gamma)*sin(beta)*cos(alpha);
    CHECK(dR(0, 1) == Approx(val).margin(1e-6));
    val = sin(gamma)*cos(alpha) - cos(gamma)*sin(beta)*sin(alpha);
    CHECK(dR(0, 2) == Approx(val).margin(1e-6));

    CHECK(dR(1, 0) == Approx(0).margin(1e-6));
    val = -cos(gamma)*sin(alpha) + sin(gamma)*sin(beta)*cos(alpha);
    CHECK(dR(1, 1) == Approx(val).margin(1e-6));
    val = -cos(gamma)*cos(alpha) - sin(gamma)*sin(beta)*sin(alpha);
    CHECK(dR(1, 2) == Approx(val).margin(1e-6));

    CHECK(dR(2, 0) == Approx(0).margin(1e-6));
    val = cos(beta)*cos(alpha);
    CHECK(dR(2, 1) == Approx(val).margin(1e-6));
    val = -cos(beta)*sin(alpha);
    CHECK(dR(2, 2) == Approx(val).margin(1e-6));

    // dR to beta
    dR = mmath::rotByZf(gamma) * mmath::dRotByYf(beta)
            * mmath::rotByXf(alpha);
    val = -cos(gamma)*sin(beta);
    CHECK(dR(0, 0) == Approx(val).margin(1e-6));
    val = cos(gamma)*cos(beta)*sin(alpha);
    CHECK(dR(0, 1) == Approx(val).margin(1e-6));
    val = cos(gamma)*cos(beta)*cos(alpha);
    CHECK(dR(0, 2) == Approx(val).margin(1e-6));

    val = -sin(gamma)*sin(beta);
    CHECK(dR(1, 0) == Approx(val).margin(1e-6));
    val = sin(gamma)*cos(beta)*sin(alpha);
    CHECK(dR(1, 1) == Approx(val).margin(1e-6));
    val = sin(gamma)*cos(beta)*cos(alpha); 
    CHECK(dR(1, 2) == Approx(val).margin(1e-6));

    val = -cos(beta);
    CHECK(dR(2, 0) == Approx(val).margin(1e-6));
    val = -sin(beta)*sin(alpha);
    CHECK(dR(2, 1) == Approx(val).margin(1e-6));
    val = -sin(beta)*cos(alpha);
    CHECK(dR(2, 2) == Approx(val).margin(1e-6));

    // dR to gamma
    dR = mmath::dRotByZf(gamma) * mmath::rotByYf(beta)
            * mmath::rotByXf(alpha);
    val = -sin(gamma)*cos(beta);
    CHECK(dR(0, 0) == Approx(val).margin(1e-6));
    val = -cos(gamma)*cos(alpha) - sin(gamma)*sin(beta)*sin(alpha);
    CHECK(dR(0, 1) == Approx(val).margin(1e-6));
    val = cos(gamma)*sin(alpha) - sin(gamma)*sin(beta)*cos(alpha);
    CHECK(dR(0, 2) == Approx(val).margin(1e-6));

    val = cos(gamma)*cos(beta);
    CHECK(dR(1, 0) == Approx(val).margin(1e-6));
    val = -sin(gamma)*cos(alpha) + cos(gamma)*sin(beta)*sin(alpha);
    CHECK(dR(1, 1) == Approx(val).margin(1e-6));
    val = sin(gamma)*sin(alpha) + cos(gamma)*sin(beta)*cos(alpha);
    CHECK(dR(1, 2) == Approx(val).margin(1e-6));
    
    CHECK(dR(2, 0) == Approx(0).margin(1e-6));
    CHECK(dR(2, 1) == Approx(0).margin(1e-6));
    CHECK(dR(2, 2) == Approx(0).margin(1e-6));
}