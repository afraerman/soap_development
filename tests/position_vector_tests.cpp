#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "../matrix.h"
#include "../position_vector.h"

using Catch::Approx;

TEST_CASE("PositionVector dot product works correctly", "[position vector]")
{
	PositionVector v1({1.0, 2.0, 3.0});
	PositionVector v2({4.0, 5.0, 6.0});

	double result = v1.dot(v2);

	REQUIRE(result == Approx(32.0));
}

TEST_CASE("PositionVector cross-product", "[position vector]")
{
	PositionVector v1({1.0 ,2.0, 3.0});
	PositionVector v2({4.0, 5.0, 6.0});

	PositionVector result = v1.cross(v2);

	REQUIRE(result[0] == Approx(-3.0));
	REQUIRE(result[1] == Approx(6.0));
	REQUIRE(result[2] == Approx(-3.0));
}

TEST_CASE("PositionVector outer-product", "[position vector]")
{
	PositionVector v1({1.0, 2.0, 3.0});
	PositionVector v2({4.0, 5.0, 6.0});

	Matrix result = v1.outer(v2);

	REQUIRE(result[0][0] == Approx(4.0));
	REQUIRE(result[0][1] == Approx(5.0));
	REQUIRE(result[0][2] == Approx(6.0));
	REQUIRE(result[1][0] == Approx(8.0));
	REQUIRE(result[1][1] == Approx(10.0));
	REQUIRE(result[1][2] == Approx(12.0));
	REQUIRE(result[2][0] == Approx(12.0));
	REQUIRE(result[2][1] == Approx(15.0));
	REQUIRE(result[2][2] == Approx(18.0));
}

TEST_CASE("PositionVector skew", "[position vector]")
{
	PositionVector v1({1.0, 2.0, 3.0});

	Matrix result = v1.skew();

	REQUIRE(result[0][0] == Approx(0.0));
	REQUIRE(result[0][1] == Approx(-3.0));
	REQUIRE(result[0][2] == Approx(2.0));
	REQUIRE(result[1][0] == Approx(3.0));
	REQUIRE(result[1][1] == Approx(0.0));
	REQUIRE(result[1][2] == Approx(-1.0));
	REQUIRE(result[2][0] == Approx(-2.0));
	REQUIRE(result[2][1] == Approx(1.0));
	REQUIRE(result[2][2] == Approx(0.0));
}


TEST_CASE("PositionVector solve", "[position vector]")
{
	Matrix a(3, 3, {{2.0, 3.0, 1.0}, {4.0, 10.0, 7.0}, {6.0, 25.0, 29.0}});
	PositionVector b({29.0, 118.0, 375.0});

	solve(a, b);

	REQUIRE(b[0] == Approx(3.0));
	REQUIRE(b[1] == Approx(5.0));
	REQUIRE(b[2] == Approx(8.0));
}

TEST_CASE("Matrix - PositionVector multiplication", "[matrix][position vector]")
{
	Matrix a(2, 2, {{1.0, 2.0}, {3.0, 4.0}});
	PositionVector x({5.0, 6.0});

	PositionVector result = mul(a, x);

	REQUIRE(result[0] == Approx(17.0));
	REQUIRE(result[1] == Approx(39.0));
}