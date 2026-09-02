#include<stdafx.h>

Quaternion::Quaternion()
{
	w_ = 1.0;
	x_ = 0.0;
	y_ = 0.0;
	z_ = 0.0;
}
Quaternion::Quaternion(double w, double x, double y, double z)
{
	w_ = w;
	x_ = x;
	y_ = y;
	z_ = z;
}
Quaternion::Quaternion(const PositionVector& v)
{
	if ((v.norm() > 1.0) || (v.length() != 3))
	{
		std::cout << "Can't create quaternion from vector" << v << std::endl;
		w_ = 1.0;
		x_ = 0.0;
		y_ = 0.0;
		z_ = 0.0;
	}
	else
	{
		x_ = v[0];
		y_ = v[1];
		z_ = v[2];
		w_ = std::sqrt(1.0 - v.norm()*v.norm());
	}
}
Quaternion::Quaternion(const Matrix& m)
{
	double tr;
	if ((m.getColumns() != 3) || (m.getRows() != 3))
	{
		std::cout << "Indefinite quaternion transformations for matrix with shape (" << m.getRows() << ", " << m.getColumns() << ")" << std::endl;
		w_ = 1.0;
		x_ = 0.0;
		y_ = 0.0;
		z_ = 0.0;
	}
	else
	{
		tr = m[0][0] + m[1][1] + m[2][2];
		w_ = std::sqrt(1.0 + tr) / 2.0;
		x_ = (m[2][1] - m[1][2]) / 4.0 / w_;
		y_ = (m[0][2] - m[2][0]) / 4.0 / w_;
		z_ = (m[1][0] - m[0][1]) / 4.0 / w_;
	}
}
Quaternion::Quaternion(const PositionVector& v0, const PositionVector& v1)
{
	if (v0.length() != 3 || v1.length() != 3)
	{
		std::cout << "Unknown Quaternion dfinition from vectors with dimensions of " << v0.length() << " and " << v1.length() << std::endl;
		w_ = 1.0;
		x_ = 0.0;
		y_ = 0.0;
		z_ = 0.0;
	}
	else
	{
		PositionVector a = v0 / v0.norm();
		PositionVector b = v1 / v1.norm();

		if (std::fabs((a-b).norm()) < 1e-5)
		{
			std::cerr << "\033[31m#32 Attempt to find a quaternion for antiparallel vectors\033[0m" << std::endl;
			throw std::runtime_error("");
		}

		double w, q_norm;
		w = 1.0 + a.dot(b);
		a = a.cross(b);

		q_norm = std::sqrt(w*w + a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
		a = a / q_norm;
		
		w_ = w / q_norm;
		x_ = a[0];
		y_ = a[1];
		z_ = a[2];
	}
}

double Quaternion::get_scalar() const { return w_; }
PositionVector Quaternion::get_vector() const { return PositionVector({x_, y_, z_}); }
double Quaternion::get_w() const { return w_; }
double Quaternion::get_x() const { return x_; }
double Quaternion::get_y() const { return y_; }
double Quaternion::get_z() const { return z_; }

void Quaternion::set_w(double w) { w_ = w; }
void Quaternion::set_x(double x) { x_ = x; }
void Quaternion::set_y(double y) { y_ = y; }
void Quaternion::set_z(double z) { z_ = z; }

Matrix Quaternion::to_matrix() const
{
	return Matrix(3, 3, {
		{1.0 - 2.0 * y_*y_ - 2.0 * z_*z_, 	2.0*x_*y_ - 2.0*z_*w_, 			2.0*x_*z_ + 2.0*y_*w_},
		{2.0*x_*y_ + 2.0*z_*w_, 			1.0 - 2.0*x_*x_ - 2.0*z_*z_, 	2.0*y_*z_ - 2.0*x_*w_},
		{2.0*x_*z_ - 2.0*y_*w_,				2.0*y_*z_ + 2.0*x_*w_,			1.0 - 2.0*x_*x_ - 2.0*y_*y_}
		}
	);
}

Quaternion Quaternion::get_inverse() const
{
	return Quaternion(w_, -1.0*x_, -1.0*y_, -1.0*z_);
}

Quaternion Quaternion::operator*(const Quaternion&q) const
{
	double w, x, y, z;
	w = w_ * q.get_w() - x_ * q.get_x() - y_ * q.get_y() - z_ * q.get_z();
	x = w_ * q.get_x() + x_ * q.get_w() + y_ * q.get_z() - z_ * q.get_y();
	y = w_ * q.get_y() + y_ * q.get_w() - x_ * q.get_z() + z_ * q.get_x();
	z = w_ * q.get_z() + z_ * q.get_w() + x_ * q.get_y() - y_ * q.get_x();
	return Quaternion(w, x, y, z);
}
Quaternion Quaternion::operator-(const Quaternion& q) const
{
	return Quaternion(w_ - q.get_w(), x_ - q.get_x(), y_ - q.get_y(), z_ - q.get_z());
}
PositionVector Quaternion::operator*(const PositionVector&v) const
{
	if (v.length() != 3)
	{
		std::cout << "Can not rotate vector with " << v.length() << " dimensions" << std::endl;
	}
	PositionVector phi = this->get_vector();
	return (2.0 * w_*w_ - 1.0) * v + 2.0 * phi.dot(v) * phi + 2.0 * w_ * phi.cross(v);
}

double Quaternion::operator[](const int i) const
{
	switch (i){
		case 0: return w_;
		case 1: return x_;
		case 2: return y_;
		case 3: return z_;
		default: {std::cerr << "\033[31m#27Index " << i << " out of range for quaternion\033[0m" << std::endl; throw std::runtime_error(""); }
}

bool Quaternion::operator==(const Quaternion&q) const
{
	if ((w_ == q.get_w()) && (x_ == q.get_x()) && (y_ == q.get_y()) && (z_ == q.get_z()))
		return true;
	return false;
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
	os << q.get_w() << '\t' << q.get_x() << '\t' << q.get_y() << '\t' << q.get_z();
	return os;
}
Quaternion::~Quaternion() {}