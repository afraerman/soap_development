#include "stdafx.h"

Matrix::Matrix()
{
	number_of_columns = 3;
	number_of_rows = 3;
	values = std::vector<std::vector<double>>(number_of_rows, std::vector<double>(number_of_columns, 0));
}

Matrix::Matrix(int rows, int columns)
{
	number_of_rows = rows;
	number_of_columns = columns;
	values = std::vector<std::vector<double>>(rows, std::vector<double>(columns, 0));
}

Matrix::Matrix(int r, int c, const std::vector<std::vector<double>> &d)
{
	number_of_rows = r;
	number_of_columns = c;
	values = d;
}

// Matrix from Euler angles
Matrix::Matrix(const std::vector<double>& angles)
{
	double s_psi = std::sin(angles[0]);
	double c_psi = std::cos(angles[0]);
	double s_theta = std::sin(angles[1]);
	double c_theta = std::cos(angles[1]);
	double s_phi = std::sin(angles[2]);
	double c_phi = std::cos(angles[2]);

	number_of_rows = 3;
	number_of_columns = 3;
	values = std::vector<std::vector<double>>(3, std::vector<double>(3, 0.));
	values[0][0] = c_phi * c_psi - s_phi * c_theta * s_psi;
	values[0][1] = -1.0 * c_phi * s_psi - s_phi * c_theta * c_psi;
	values[0][2] = s_phi * s_theta;

	values[1][0] = s_phi * c_psi + c_phi * c_theta * s_psi;
	values[1][1] = -1.0 * s_phi * s_psi + c_phi * c_theta * c_psi;
	values[1][2] = -1.0 * c_phi * s_theta;

	values[2][0] = s_theta * s_psi;
	values[2][1] = s_theta * c_psi;
	values[2][2] = c_theta;

}

/*
Matrix::Matrix(const boost::math::quaternion<double>& quat)
{
	double w, x, y, z;
	w = quat.R_component_1();
	x = quat.R_component_2();
	y = quat.R_component_3();
	z = quat.R_component_4();

	number_of_rows = 3;
	number_of_columns = 3;
	values = std::vector<std::vector<double>>(3, std::vector<double>(3));
	values[0][0] = 1.0 - 2.0 * y*y - 2.0 * z*z;
	values[0][1] = 2.0*x*y - 2.0*z*w;
	values[0][2] = 2.0*x*z + 2.0*y*w;
	values[1][0] = 2.0*x*y + 2.0*z*w;
	values[1][1] = 1.0 - 2.0*x*x - 2.0*z*z;
	values[1][2] = 2.0*y*z - 2.0*x*w;
	values[2][0] = 2.0*x*z - 2.0*y*w;
	values[2][1] = 2.0*y*z + 2.0*x*w;
	values[2][2] = 1.0 - 2.0*x*x - 2.0*y*y;
}
*/

Matrix Matrix::inverse() const
{
	if (number_of_columns != number_of_rows)
	{
		std::cerr << "\033[31m#251 Can't inverse non-square matrix: \033[0m" << std::endl;
		std::cerr << *this << std::endl;
		throw std::runtime_error("");
	}
	for (int i = 0; i < number_of_rows; i++)
	{
		if (values[i][i] == 0.0)
		{
			std::cerr << "\033[31m#252 Can't inverse matrix with 0 on the diagonal\033[0m" << std::endl;
			std::cerr << *this << std::endl;
			throw std::runtime_error("");
		}
		for (int j = 0; j < number_of_columns; j++)
		{
			if ((i != j) and values[i][j] != 0.0)
			{
				std::cerr << "\033[31m#251 Can't inverse non-square matrix\033[0m" << std::endl;
				std::cerr << *this << std::endl;
				throw std::runtime_error("");
			}
		}
	}
	std::vector<std::vector<double>> m(number_of_rows, std::vector<double>(number_of_columns, 0.0));
	m[0][0] = 1.0 / values[0][0]; m[1][1] = 1.0 / values[1][1]; m[2][2] = 1.0 / values[2][2];

	return Matrix(number_of_rows, number_of_columns, m);
}

Matrix Matrix::transpose() const
{
	int rows = number_of_rows;
	int columns = number_of_columns;
	std::vector<std::vector<double>> m(columns, std::vector<double>(rows));

	for (int i = 0; i < columns; i++)
	{
		for (int j = 0; j < rows; j++)
		{
			m[i][j] = values[j][i];
		}
	}
	return Matrix(columns, rows, m);
}

void Matrix::LUdecompose()
{
	if (number_of_rows != number_of_columns)
	{
		std::cerr << "\033[31m#261 Can't LUDecompose non-square matrix\033[0m" << std::endl;
		throw std::runtime_error("");
	}
	const double tiny = 1.0e-40;
	int i, imax, j, k;
	double big, tmp;
	_ludecompose_d = 1;
	_ludecompose_indx = std::vector<double>(number_of_rows);
	std::vector<double> vv(number_of_rows);
	for (i = 0; i < number_of_rows; i++)
	{
		big = 0.0;
		for (j = 0; j < number_of_rows; j++)
		{
			tmp = std::fabs(values[i][j]);
			if (tmp > big) big = tmp;
		}
		if (big == 0.0)
		{
			std::cerr << "\033[31m#262 Singular matrix in LUdecompose\033[0m" << std::endl;
			throw std::runtime_error("");
		}
		vv[i] = 1.0 / big;
	}
	for (k = 0; k < number_of_rows; k++)
	{
		big = 0.0;
		for (i = k; i < number_of_rows; i++)
		{
			tmp = vv[i] * std::fabs(values[i][k]);
			if (tmp > big)
			{
				big = tmp;
				imax = i;
			}
		}
		if (k != imax)
		{
			for (j = 0; j < number_of_rows; j++)
			{
				tmp = values[imax][j];
				values[imax][j] = values[k][j];
				values[k][j] = tmp;
			}
			_ludecompose_d = -1 * _ludecompose_d;
			vv[imax] = vv[k];
		}
		_ludecompose_indx[k] = imax;
		if (values[k][k] == 0.0) values[k][k] = tiny;

		for (i = k + 1; i < number_of_rows; i++)
		{
			values[i][k] /= values[k][k];
			tmp = values[i][k];
			for (j = k + 1; j < number_of_rows; j++)
			{
				values[i][j] -= tmp * values[k][j];
			}
		}
	}
}

int Matrix::getD()
{
	return _ludecompose_d;
}
std::vector<double> Matrix::getIndx()
{
	return _ludecompose_indx;
}

Matrix Matrix::operator*(const Matrix &m) const
{
	if (number_of_columns != m.number_of_rows)
	{
		std::cout << "Can not multiply matrices with dimentions (" << number_of_rows << ", " << number_of_columns << ") and";
		std::cout << "(" << m.number_of_rows << ", " << m.number_of_columns << ")" << std::endl;
		return Matrix();
	}
	Matrix new_m = Matrix(number_of_rows, m.number_of_columns);
	for (int i = 0; i < number_of_rows; i++)
	{
		for (int j = 0; j < m.number_of_columns; j++)
		{
			for (int k = 0; k < number_of_columns; k++)
			{
				new_m[i][j] += values[i][k] * m.values[k][j];
			}
		}
	}
	return new_m;
}

Matrix Matrix::operator*(double a) const
{
	std::vector<std::vector<double>> m(number_of_rows, std::vector<double>(number_of_columns));

	for (int i = 0; i < number_of_rows; i++)
	{
		for (int j = 0; j < number_of_columns; j++)
		{
			m[i][j] = values[i][j] * a;
		}
	}
	return Matrix(number_of_rows, number_of_columns, m);
}

/*
PositionVector Matrix::operator*(PositionVector &v) const
{
	if (number_of_columns != v.length())
	{
		std::cout << "Can not multiply matrix (" << number_of_rows << ", " << number_of_columns << ") and vector (" << v.length() << ")" << std::endl;
		return;
	}
	std::vector<double> vv(number_of_rows, 0);
	for (int i = 0; i < number_of_rows; i++)
	{
		for (int j = 0; j < number_of_columns; j++)
		{
			vv[i] += values[i][j] * v.getValues[j];
		}
	}
	return PositionVector(vv);
}
*/

Matrix Matrix::operator+(const Matrix& m) const
{
	if ((number_of_rows != m.getRows()) || (number_of_columns != m.getColumns()))
	{
		std::cout << "Can not multiply martices with dimentions (" << number_of_rows << ", " << number_of_columns << ") and (" << m.getRows() << ", " << m.getColumns() << ")" << std::endl;
		return Matrix();
	}
	std::vector<std::vector<double>> new_m(number_of_rows, std::vector<double>(number_of_columns));
	for (int i = 0; i < number_of_rows; i++)
	{
		for (int j = 0; j < number_of_columns; j++)
		{
			new_m[i][j] = values[i][j] + m.getValues()[i][j];
		}
	}
	return Matrix(number_of_rows, number_of_columns, new_m);
}


/*
boost::math::quaternion<double> Matrix::to_quat() const
{
	double w, x, y, z, tr;
	if ((number_of_columns != 3) || (number_of_rows != 3))
	{
		std::cout << "Indefinite quaternion transformations for matrix with shape (" << number_of_rows << ", " << number_of_columns << ")" << std::endl;
		return boost::math::quaternion<double>(1.0, 0.0, 0.0, 0.0);
	}
	tr = values[0][0] + values[1][1] + values[2][2];
	w = std::sqrt(1.0 + tr) / 2.0;
	x = (values[2][1] - values[1][2]) / 4.0 / w;
	y = (values[0][2] - values[2][0]) / 4.0 / w;
	z = (values[1][0] - values[0][1]) / 4.0 / w;
	
	return boost::math::quaternion<double>(w, x, y, z);
}
*/

int Matrix::getColumns() const
{
	return number_of_columns;
}
int Matrix::getRows() const
{
	return number_of_rows;
}
std::vector<std::vector<double>> Matrix::getValues() const
{
	return values;
}

std::vector<double>& Matrix::operator[](int i)
{
	return values[i];
}

std::vector<double> Matrix::operator[](int i) const
{
	return values[i];
}

std::ostream& operator<<(std::ostream& os, const Matrix& m)
{
	int rows = m.getRows();
	int columns = m.getColumns();

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			os << m[i][j] << '\t';
		}
		os << std::endl;
	}
	return os;
}

bool signature_alignment(const Matrix& m1, const Matrix& m2)
{
	if ((m1.getColumns() != m2.getColumns()) || (m1.getRows() != m2.getRows()))
	{
		std::cout << "Can't compare signatures of matrices with dimensions (" << m1.getRows() << ", " << m1.getColumns() << ") and (" << m2.getRows() << ", " << m2.getColumns() << ")" << std::endl;
		return false;
	}
	for (int i = 0; i < m1.getRows(); i++)
	{
		for (int j = 0; j < m1.getColumns(); j++)
		{
			if (m1[i][j] * m2[i][j] < 0) return false;
		}
	}
	return true;
}

Matrix::~Matrix() {}