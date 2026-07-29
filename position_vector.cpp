#include "stdafx.h"

/*
 * This file contains methods of class PositionVector
 *
 * PositionVector - any-dimensional vectors with most
 * of linear algebra operations supported
 * 
*/

/**
 * Default constructor
 * 
 * creates a 3d PositionVector with coordinates (1, 0, 0)
 */
PositionVector::PositionVector()
{
	values = std::vector<double>{ 1.0, 0.0, 0.0 };
}

/**
 * Constructor
 * 
 * @param v Coordinates of PositionVector
 */
PositionVector::PositionVector(const std::vector<double>& v)
{
	values = v;
}

/**
 * Dot-product of PositionVectors
 * 
 * @param v PositionVector to have a dot-product with
 * @return dot-product or 0.0 if PositionVector of different dimensions. Prints an error message to console.
 */
double PositionVector::dot(const PositionVector& v) const
{
	if (values.size() != v.values.size())
	{
		std::cout << "Can not multiply vectors with dimentions " << values.size() << " and " << v.values.size() << std::endl;
		return 0.0;
	}
	double d = 0;
	for (int i = 0; i < (int)values.size(); i++) d += values[i] * v.values[i];
	return d;
}

/**
 * Cross-product of POsitionVectors
 * 
 * @param v PositionVector to have a cross-product with
 * @return cross-product or PositionVector with (0.0, 0.0, 0.0) coords if any of two vectors don't match dimension 3.
 */
PositionVector PositionVector::cross(const PositionVector& v) const
{
	if ((values.size() != v.values.size()) || (values.size() != 3) || (v.values.size() != 3))
	{
		std::cout << "Can not multiply vectors with dimentions " << values.size() << " and " << v.values.size() << std::endl;
		return PositionVector({0.0, 0.0, 0.0});
	}
	std::vector<double> v1{ v.values[2] * values[1] - v.values[1] * values[2],
							v.values[0] * values[2] - v.values[2] * values[0],
							v.values[1] * values[0] - v.values[0] * values[1] };
	return PositionVector(v1);
}


/**
 * Outer-product
 * 
 * @param v PositionVector to have an outer-product with
 * @return outer-product
 */
Matrix PositionVector::outer(const PositionVector& v) const
{
	int rows = (int)values.size();
	int columns = (int)v.values.size();

	std::vector<std::vector<double>> m(rows, std::vector<double>(columns));

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
			m[i][j] = values[i] * v.values[j];
	}
	return Matrix(rows, columns, m);
}

/**
 * skew-symmetric matrix. Matrix representation of cross-product
 * 
 * @return skew-symmetric Matrix or zero-matrix if vector isn't 3d
 */
Matrix PositionVector::skew() const
{
	if (values.size() != 3)
	{
		std::cout << "Can not skew vector with length " << values.size() << std::endl;
		return Matrix();
	}
	std::vector<std::vector<double>> m(3, std::vector<double>(3));
	m[0][0] = 0.0;
	m[0][1] = -1.0 * values[2];
	m[0][2] = values[1];
	m[1][0] = values[2];
	m[1][1] = 0.0;
	m[1][2] = -1.0 * values[0];
	m[2][0] = -1.0 * values[1];
	m[2][1] = values[0];
	m[2][2] = 0.0;
	return Matrix(3, 3, m);
}

/**
 * Euclidean norm
 * 
 * @return euclidean norm of PositionVector
 */
double PositionVector::norm() const
{
	double n = 0;
	for (int i = 0; i < (int)values.size(); i++) n += values[i] * values[i];
	return sqrt(n);
}

/**
 * Dimension
 * 
 * @return dimension of PositionVector
 */
int PositionVector::length() const
{
	return (int)values.size();
}

std::vector<double> PositionVector::getValues() const
{
	return values;
}

/**
 * Solve matrix equation M*x = V
 * 
 * @param m Matrix
 * @param v Right-side PositionVector
 * 
 * @return none. Solution is stored in v.
 */
void solve(Matrix& m, PositionVector& v)
{
	std::vector<double> values = v.getValues();
	if ((values.size() != m.getRows()) || (m.getColumns() != m.getRows()))
		std::cout << "Can not solve for vector (" << values.size() << ") and matrix (" << m.getRows() << ", " << m.getColumns() << ")" << std::endl;
	else
	{
		int N = (int)values.size();
		
		m.LUdecompose();

		for (int i = 1; i < N; i++)
		{
			for (int j = 0; j < i; j++)
			{
				values[i] -= m[i][j] * values[j];
			}
		}

		values[N - 1] = values[N - 1] / m[N - 1][N - 1];
		for (int i = N - 2; i > -1; i--)
		{
			for (int j = i + 1; j < N; j++)
			{
				values[i] -= m[i][j] * values[j];
			}
			values[i] /= m[i][i];
		}
	}
	v = PositionVector(values);
}

PositionVector PositionVector::operator+(const PositionVector& v) const
{
	if (values.size() != v.values.size())
	{
		std::cout << "Can not add vectors with dimentions (" << values.size() << ") and (" << v.values.size() << ")" << std::endl;
		return PositionVector();
	}
	std::vector<double> vv(values.size());
	for (int i = 0; i < values.size(); i++)
	{
		vv[i] = values[i] + v.values[i];
	}
	return PositionVector(vv);
}

void PositionVector::operator+=(const PositionVector& v)
{
	if (values.size() != v.values.size())
	{
		std::cout << "Can not add vectors with dimentions (" << values.size() << ") and (" << v.values.size() << ")" << std::endl;
		return;
	}
	for (int i = 0; i < values.size(); i++)
	{
		values[i] += v.values[i];
	}
	return;
}

PositionVector PositionVector::operator-(const PositionVector& v) const
{
	if (values.size() != v.values.size())
	{
		std::cout << "Can not add vectors with dimentions (" << values.size() << ") and (" << v.values.size() << ")" << std::endl;
		return PositionVector();
	}
	std::vector<double> vv(values.size());
	for (int i = 0; i < values.size(); i++)
	{
		vv[i] = values[i] - v.values[i];
	}
	return PositionVector(vv);
}

PositionVector PositionVector::operator*(double x) const
{
	std::vector<double> vv(values.size());
	for (int i = 0; i < values.size(); i++) vv[i] = values[i] * x;
	return PositionVector(vv);
}

/*
PositionVector PositionVector::operator*(const boost::math::quaternion<double>& q) const
{
	if (values.size() != 3)
	{
		std::cout << "Cannot multiply vector of dimension " << values.size() << " with a quaternion" << std::endl;
		return *this;
	}
	std::vector<double> rv(values.size());
	boost::math::quaternion<double> vv{0.0, values[0], values[1], values[2]};
	vv = q * vv * (1.0 / q);
	rv[0] = vv.R_component_2();
	rv[1] = vv.R_component_3();
	rv[2] = vv.R_component_4();
	return PositionVector(rv);
}
*/

PositionVector PositionVector::operator/(double x) const
{
	std::vector<double> vv(values.size());
	for (int i=0; i < values.size(); i++) vv[i] = values[i] / x;
	return PositionVector(vv);
}

double& PositionVector::operator[](int i)
{
	return values[i];
}

const double& PositionVector::operator[](int i) const
{
	return values[i];
}

double PositionVector::sum() const
{
	double s = 0.0;
	for (auto i: values) { s += i; }
	return s;
}

std::string PositionVector::toString() const
{
	std::string s = "";
	for (auto i: values) { s = s + std::to_string(i) + " "; }
	return s;
}

PositionVector operator*(const double x, const PositionVector& p)
{
	std::vector<double> vv(p.values.size());
	for (int i = 0; i < p.values.size(); i++) vv[i] = p.values[i] * x;
	return PositionVector(vv);
}

std::ostream& operator<<(std::ostream& os, const PositionVector& v)
{
	for (int i = 0; i < (int)v.values.size() - 1; i++)
	{
		os << v.values[i] << '\t';
	}
	os << v.values[v.values.size() - 1];
	return os;
}

double absolute_deviation(const PositionVector& p1, const PositionVector& p2)
{
	return sqrt(pow(p1.values[0] - p2.values[0], 2) + pow(p1.values[1] - p2.values[1], 2) + pow(p1.values[2] - p2.values[2], 2));
}

PositionVector pow(const std::vector<double>& v, int exp)
{
	std::vector<double> vv = v;
	for (int i = 0; i < vv.size(); i++)
	{
		vv[i] = pow(vv[i], exp);
	}
	return PositionVector(vv);
}

/**
 * Matrix - PositionVector multiplication
 * 
 * @param m Matrix to multiply
 * @param v PositionVector to multiply matrix m on
 * @return multiplication or default PositionVector
 */
PositionVector mul(const Matrix& m, const PositionVector& v)
{
	if (m.getColumns() != v.length())
	{
		std::cout << "Can not multiply matrix (" << m.getRows() << ", " << m.getColumns() << ") and vector (" << v.length() << ")" << std::endl;
		return PositionVector();
	}
	std::vector<double> vv(m.getRows(), 0);
	for (int i = 0; i < m.getRows(); i++)
	{
		for (int j = 0; j < m.getColumns(); j++)
		{
			vv[i] += m.getValues()[i][j] * v.getValues()[j];
		}
	}
	return PositionVector(vv);
}