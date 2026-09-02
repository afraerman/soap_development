#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "../matrix.h"
#include "../position_vector.h"
#include "../quaternion.h"

using Catch::Approx;

TEST_CASE("Quaternion to Matrix", "[quaternion]")
{
	Quaternion q1(-0.801707, -0.116075, 0.322711, -0.48954);
	Matrix result = q1.to_matrix();

	REQUIRE(result[0][0] == Approx(0.312416));
	REQUIRE(result[0][1] == Approx(-0.859853));
	REQUIRE(result[0][2] == Approx(-0.403793));
	REQUIRE(result[1][0] == Approx(0.710018));
	REQUIRE(result[1][1] == Approx(0.493754));
	REQUIRE(result[1][2] == Approx(-0.502077));
	REQUIRE(result[2][0] == Approx(0.631086));
	REQUIRE(result[2][1] == Approx(-0.129844));
	REQUIRE(result[2][2] == Approx(0.764768));
}

TEST_CASE("Matrix to quatrenion", "[quatrenion]")
{
	Matrix m(3, 3, {{0.312416, -0.859853, -0.403793}, {0.710018, 0.493754, -0.502077}, {0.631086, -0.129844, 0.764768}});
	Quaternion result(m);

	REQUIRE(((result.get_w() == Approx(-0.801707)) || (result.get_w() == Approx(0.801707))));
	REQUIRE(((result.get_x() == Approx(-0.116075)) || (result.get_x() == Approx(0.116075))));
	REQUIRE(((result.get_y() == Approx(0.322711)) || (result.get_y() == Approx(-0.322711))));
	REQUIRE(((result.get_z() == Approx(-0.48954)) || (result.get_z() == Approx(0.48954))));
}

TEST_CASE("Quaternion rotation of a vector", "[quaternion][position vector]")
{
	PositionVector v({1.0, 2.0, 3.0});
	Quaternion q(0.707107, 0.707107, 0.0, 0.0);

	PositionVector result = q * v;

	REQUIRE(result[0] == Approx(1.0));
	REQUIRE(result[1] == Approx(-3.0));
	REQUIRE(result[2] == Approx(2.0));
}

TEST_CASE("Quaternion multiplication", "[quaternion]")
{
	Quaternion q1(-0.801707, -0.116075, 0.322711, -0.48954);
	Quaternion q2( 0.644785,  0.435331, 0.316734, -0.542603);

	Quaternion result = q1 * q2;
	Quaternion answer(-0.834237, -0.443901, -0.321943, -0.0578904);

	for (int i = 0; i < 4; i++)
	{
		REQUIRE(result[i] == Approx(answer[i]));
	}
}

TEST_CASE("Inverse of a quaternion", "[quaternion]")
{
	Quaternion q(-0.801707, -0.116075, 0.322711, -0.48954);

	Quaternion result = q.get_inverse();
	Quaternion answer(-0.801708, 0.116075, -0.322711, 0.48954);

	for (int i = 0; i < 4; i++)
	{
		REQUIRE(result[i] == Approx(answer[i]));
	}
}

