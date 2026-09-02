// tests/matrix_test.cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include<vector>
#include "../matrix.h"

using Catch::Approx;

TEST_CASE("Matrix addition", "[matrix]") {
    // arrange
    Matrix a(2, 2, {{1.0, 2.0}, {3.0, 4.0}});
    Matrix b(2, 2, {{5.0, 6.0}, {7.0, 8.0}});

    // act
    Matrix result = a + b;

    // assert
    REQUIRE(result[0][0] == Approx(6.0));
    REQUIRE(result[0][1] == Approx(8.0));
    REQUIRE(result[1][0] == Approx(10.0));
    REQUIRE(result[1][1] == Approx(12.0));
}

TEST_CASE("Matrix multiplied by scalar", "[matrix]") {
    Matrix a(2, 2, {{1.0, 2.0}, {3.0, 4.0}});
    const int scalar = 10;

    Matrix result = a * scalar;

    //assert
    REQUIRE(result[0][0] == Approx(10.0));
    REQUIRE(result[0][1] == Approx(20.0));
    REQUIRE(result[1][0] == Approx(30.0));
    REQUIRE(result[1][1] == Approx(40.0));
}

TEST_CASE("Matrix multiplication", "[matrix]")
{
    Matrix m1(2, 2, {{1.0, 2.0}, {3.0, 4.0}});
    Matrix m2(2, 2, {{5.0, 6.0}, {7.0, 8.0}});

    Matrix result = m1 * m2;

    REQUIRE(result[0][0] == Approx(19.0));
    REQUIRE(result[0][1] == Approx(22.0));
    REQUIRE(result[1][0] == Approx(43.0));
    REQUIRE(result[1][1] == Approx(50.0));
}

TEST_CASE("Matrix LU-decomposition", "[matrix]")
{
    Matrix m(3, 3, {{2.0, 3.0, 1.0}, {4.0, 10.0, 7.0}, {6.0, 25.0, 29.0}});
    Matrix answer(3, 3, {{2.0, 3.0, 1.0}, {4.0, 10.0, 7.0}, {6.0, 25.0, 29.0}});

    Matrix a(3,3), b(3,3);

    m.LUdecompose();

    a[0][0] = 1.0;     a[0][1] = 0.0;     a[0][2] = 0.0;
    a[1][0] = m[1][0]; a[1][1] = 1.0;     a[1][2] = 0.0;
    a[2][0] = m[2][0]; a[2][1] = m[2][1]; a[2][2] = 1.0;

    b[0][0] = m[0][0]; b[0][1] = m[0][1]; b[0][2] = m[0][2];
    b[1][0] = 0.0;     b[1][1] = m[1][1]; b[1][2] = m[1][2];
    b[2][0] = 0.0;     b[2][1] = 0.0;     b[2][2] = m[2][2];

    Matrix result = a * b;

    auto indx = m.getIndx();
    std::vector<int> idx(3);

    if (indx[0] == 0)
    {
        if (indx[1] == 1) { idx[0] = 0; idx[1] = 1; idx[2] = 2; }
        else { idx[0] = 0; idx[1] = 2; idx[2] = 1; }
    }
    else if (indx[0] == 1)
    {
        if (indx[1] == 1) { idx[0] = 1; idx[1] = 0; idx[2] = 2; }
        else { idx[0] = 1; idx[1] = 2; idx[2] = 0; }
    }
    else
    {
        if (indx[1] == 1) { idx[0] = 2; idx[1] = 1; idx[2] = 0; }
        else { idx[0] = 2; idx[1] = 0; idx[2] = 0; }
    }

    REQUIRE(result[0][0] == Approx(answer[idx[0]][0]));
    REQUIRE(result[0][1] == Approx(answer[idx[0]][1]));
    REQUIRE(result[0][2] == Approx(answer[idx[0]][2]));
    REQUIRE(result[1][0] == Approx(answer[idx[1]][0]));
    REQUIRE(result[1][1] == Approx(answer[idx[1]][1]));
    REQUIRE(result[1][2] == Approx(answer[idx[1]][2]));
    REQUIRE(result[2][0] == Approx(answer[idx[2]][0]));
    REQUIRE(result[2][1] == Approx(answer[idx[2]][1]));
    REQUIRE(result[2][2] == Approx(answer[idx[2]][2]));
}

