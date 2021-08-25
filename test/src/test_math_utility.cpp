#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <lib_math/lib_math.h>
#include <vector>

TEST_CASE("Linespace are tested", "[math utility]")
{
    std::vector<int> pts;
    mmath::linespace(0, 150, 31, pts);
    REQUIRE(pts.size() == 31);
    CHECK(pts[0] == 0);
    CHECK(pts[1] == 5);
    CHECK(pts[2] == 10);
    CHECK(pts[3] == 15);
    CHECK(pts[10] == 50);
    CHECK(pts[30] == 150);

    std::vector<double> L_range;
    mmath::linespace(0, 5.0, 150.0, L_range);
    REQUIRE(L_range.size() == 31);
    CHECK(L_range[0] == Approx(0).margin(1e-7));
    CHECK(L_range[1] == Approx(5).margin(1e-7));
    CHECK(L_range[2] == Approx(10).margin(1e-7));
    CHECK(L_range[3] == Approx(15).margin(1e-7));
    CHECK(L_range[10] == Approx(50).margin(1e-7));
    CHECK(L_range[30] == Approx(150).margin(1e-7));

    std::vector<double> phi_range;
    mmath::linespace(0, mmath::deg2rad(4), 2.0*mmath::PI, phi_range);
    REQUIRE(phi_range.size() == 91);
    CHECK(phi_range[0] == Approx(0).margin(1e-7));
    CHECK(phi_range[1] == Approx(0.06981317).margin(1e-7));
    CHECK(phi_range[2] == Approx(0.13962634).margin(1e-7));
    CHECK(phi_range[3] == Approx(0.20943951).margin(1e-7));
    CHECK(phi_range[10] == Approx(0.69813170).margin(1e-7));
    CHECK(phi_range[30] == Approx(2.09439510).margin(1e-7));
    CHECK(phi_range[90] == Approx(6.28318531).margin(1e-7));

    std::vector<double> theta1_range;
    mmath::linespace(0, mmath::deg2rad(3), mmath::PI / 2.0, theta1_range);
    REQUIRE(theta1_range.size() == 31);
    CHECK(theta1_range[0] == Approx(0).margin(1e-7));
    CHECK(theta1_range[1] == Approx(0.05235988).margin(1e-7));
    CHECK(theta1_range[2] == Approx(0.10471976).margin(1e-7));
    CHECK(theta1_range[3] == Approx(0.15707963).margin(1e-7));
    CHECK(theta1_range[10] == Approx(0.52359878).margin(1e-7));
    CHECK(theta1_range[30] == Approx(1.57079633).margin(1e-7));

    std::vector<double> theta2_range;
    mmath::linespace(0, mmath::deg2rad(6), mmath::PI*2.0 / 3.0, theta2_range);
    REQUIRE(theta2_range.size() == 21);
    CHECK(theta2_range[0] == Approx(0).margin(1e-7));
    CHECK(theta2_range[1] == Approx(0.10471976).margin(1e-7));
    CHECK(theta2_range[2] == Approx(0.20943951).margin(1e-7));
    CHECK(theta2_range[3] == Approx(0.31415927).margin(1e-7));
    CHECK(theta2_range[10] == Approx(1.04719755).margin(1e-7));
    CHECK(theta2_range[20] == Approx(2.09439510).margin(1e-7));
}


