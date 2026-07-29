#pragma once
class PositionVector
{
private:
	std::vector<double> values;
public:
	PositionVector();
	PositionVector(const std::vector<double>&);

	double dot(const PositionVector&) const;
	PositionVector cross(const PositionVector&) const;
	Matrix outer(const PositionVector&) const;
	Matrix skew() const;
	double norm() const;
	int length() const;
	std::vector<double> getValues() const;

	PositionVector operator+(const PositionVector&) const;
	void operator+=(const PositionVector&);
	PositionVector operator-(const PositionVector&) const;
	PositionVector operator*(double) const;
	// PositionVector operator*(const boost::math::quaternion<double>&) const;
	PositionVector operator/(double) const;
	double& operator[](int);
	const double& operator[](int) const;

	std::string toString() const;
	double sum() const;

	friend void solve(Matrix& m, PositionVector& v); 
	friend double absolute_deviation(const PositionVector& p1, const PositionVector& p2);
	friend PositionVector operator*(const double, const PositionVector&);
	friend std::ostream& operator<<(std::ostream&, const PositionVector&);
};

PositionVector pow(const std::vector<double>&, int);
PositionVector mul(const Matrix& m, const PositionVector& v);