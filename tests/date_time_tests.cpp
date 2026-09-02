#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include<SpiceUsr.h>
#include "date_time.h"

using Catch::Approx;

TEST_CASE("Date time subtraction", "[date time]")
{
	Time t1(2019, 1, 1, 0, 0, 0.0);
	Time t2(2019, 1, 1, 0, 0, 2.0);

	double result = t2 - t1;
	REQUIRE(result == Approx(2.0));

	Time t3(2019, 1, 1, 0, 0, 0.0);
	Time t4(2020, 1, 1, 0, 0, 10.0);

	result = t4 - t3;
	REQUIRE(result == Approx(31536010.0));
}

TEST_CASE("Date time addition", "[date time]")
{
	Time t1(2019, 12, 31, 23, 59, 59.0);
	double dt = 10.0;

	Time result = t1 + dt;
	REQUIRE(result == Time(2020, 1, 1, 0, 0, 9.0));
}

