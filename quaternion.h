#pragma once

class Quaternion
{
private:
	double w_, x_, y_, z_;
public:
	Quaternion();
	Quaternion(double w, double x, double y, double z);
	Quaternion(const PositionVector& v);
	Quaternion(const Matrix& m);
	Quaternion(const PositionVector& v0, const PositionVector& v1);

	double get_scalar() const;
	PositionVector get_vector() const;

	double get_w() const;
	double get_x() const;
	double get_y() const;
	double get_z() const;

	void set_w(const double);
	void set_x(const double);
	void set_y(const double);
	void set_z(const double);

	Matrix to_matrix() const;

	Quaternion get_inverse() const;

	Quaternion operator*(const Quaternion& q) const;
	Quaternion operator-(const Quaternion& q) const;
	PositionVector operator*(const PositionVector& v) const;
	double operator[](const int i) const;
	bool operator==(const Quaternion& q) const;
	friend std::ostream& operator<<(std::ostream&, const Quaternion&);
	~Quaternion();
};