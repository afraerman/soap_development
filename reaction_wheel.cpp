#include "stdafx.h"

ReactionWheel::ReactionWheel() : AttitudeController()
{
	angular_velocity = 0.0;
    limit = 1e5;
    alpha = 30.0 * M_PI / 180.0;
    beta = 65.0 * M_PI / 180.0;
    inertia = Matrix(3, 3, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});
    normal = PositionVector({0.8660254038, 0.2113091309, 0.4531538935});
    apex = "X";
    dump_speed = 100.0;
}

ReactionWheel::ReactionWheel(const PositionVector& loc, double a, double b, const PositionVector& norm, const Matrix& in, double angvel, double lim, double m, std::string apx, double ds, double as):
AttitudeController(loc, angvel*in[2][2], m)
{
	alpha = a;
	beta = b;
	inertia = in;
	angular_velocity = angvel;
	limit = lim;
	apex = apx;
    dump_speed = ds;
    if (as == 0.0)
    {
        acc_speed = ds;
    }
    else
    {
        acc_speed = as;
    }
}

double ReactionWheel::getAlpha() const
{
    return alpha;
}
double ReactionWheel::getBeta() const
{
    return beta;
}
double ReactionWheel::getAngularVelocity() const
{
    return angular_velocity;
}
double ReactionWheel::getLimit() const
{
    return limit;
}
PositionVector ReactionWheel::getNormal() const
{
    return normal;
}
PositionVector ReactionWheel::getAngles() const
{
    double cos_alpha = std::cos(alpha);
	double sin_alpha = std::sin(alpha);
    double cos_beta = std::cos(beta);
	double sin_beta = std::sin(beta);
    return PositionVector({sin_alpha, cos_alpha, sin_beta, cos_beta});
}
Matrix ReactionWheel::getInertiaTensor() const
{
    return inertia;
}
std::string ReactionWheel::getApex() const
{
    return apex;
}
double ReactionWheel::getDumpSpeed() const
{
    return dump_speed;
}
double ReactionWheel::getAccSpeed() const
{
    return acc_speed;
}


void ReactionWheel::setAlpha(double a)
{
    alpha = a;
}
void ReactionWheel::setBeta(double b)
{
    beta = b;
}
void ReactionWheel::setAngles(double a, double b)
{
    alpha = a;
    beta = b;
}
void ReactionWheel::setAngularVelocity(double angvel)
{
    angular_velocity = angvel;
}
void ReactionWheel::setNormal(const PositionVector& norm)
{
    normal = norm;
}
void ReactionWheel::setInertia(const Matrix& in)
{
    inertia = in;
}
void ReactionWheel::setLimit(double lim)
{
    limit = lim;
}
ReactionWheel::~ReactionWheel()
{
}