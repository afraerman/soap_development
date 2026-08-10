#pragma once
class Matrix
{
private:
	int number_of_columns;
	int number_of_rows;
	std::vector<std::vector<double>> values;

public:
	Matrix();
	Matrix(int rows, int columns);
	Matrix(int rows, int columns , const std::vector<std::vector<double>>& values);
	Matrix(const std::vector<double>& angles);
	//Matrix(const boost::math::quaternion<double>&); // local-to-global

	void LUdecompose();
	
	/// works only for diagonal matrices
	Matrix inverse() const;
	Matrix transpose() const;
	Matrix operator*(const Matrix &m) const;
	Matrix operator*(double a) const;
	Matrix operator+(const Matrix &m) const;

	//boost::math::quaternion<double> to_quat() const;

	int getRows() const;
	int getColumns() const;
	std::vector<std::vector<double>> getValues() const;

	std::vector<double>& operator[](int);
	std::vector<double> operator[](int) const;

	friend std::ostream& operator<<(std::ostream&, const Matrix&);
	friend bool signature_alignment(const Matrix&, const Matrix&);
	~Matrix();
};