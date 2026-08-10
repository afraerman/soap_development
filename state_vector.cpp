#include "stdafx.h"

StateVector::StateVector()
{
	pos = PositionVector();
	vels = PositionVector();
}

StateVector::StateVector(const PositionVector& p, const PositionVector& v)
{
	pos = p;
	vels = v;
}

StateVector::StateVector(const std::vector<double>& p, const std::vector<double>& v)
{
	pos = PositionVector(p);
	vels = PositionVector(v);
}

StateVector::StateVector(const std::vector<double>&  st)
{
	pos = PositionVector({st[0], st[1], st[2]});
	vels = PositionVector({st[3], st[4], st[5]});
}

void StateVector::setPosition(const PositionVector& p)
{
	pos = p;
}
void StateVector::setVelocity(const PositionVector& v)
{
	vels = v;
}
void StateVector::setPosition(const std::vector<double>& p)
{
	pos = PositionVector(p);
}
void StateVector::setVelocity(const std::vector<double>& v)
{
	vels = PositionVector(v);
}

PositionVector StateVector::getPosition() const
{
	return pos;
}
PositionVector StateVector::getVelocity() const
{
	return vels;
}

std::ostream& operator<<(std::ostream& os, const StateVector& st)
{
	os << st.getPosition() << '\t' << st.getVelocity();
	return os;
}

StateVector::~StateVector() {}