#include "stdafx.h"

Polygon::Polygon()
{
	position = PositionVector();
	normal = PositionVector();
	area = 1.0;
	reflectivity_factor = 0.5;
	specularity_factor = 0.5;
	rotation_axis_index = -1;
}

Polygon::Polygon(const PositionVector& p, const PositionVector& n, double a)
{
	position = p;
	normal = n;
	area = a;
	reflectivity_factor = 0.5;
	specularity_factor = 0.5;
	rotation_axis_index = -1;
}

Polygon::Polygon(const PositionVector& p, const PositionVector& n, double a, double rf, double sf, int rai)
{
	position = p;
	normal = n;
	area = a;
	reflectivity_factor = rf;
	specularity_factor = sf;
	rotation_axis_index = rai;
}

void Polygon::setReflectivityFactor(double rf)
{
	reflectivity_factor = rf;
}

void Polygon::setSpecularityFactor(double sp)
{
	specularity_factor = sp;
}

void Polygon::setArea(double a)
{
	area = a;
}

void Polygon::setNormal(const PositionVector& n)
{
	normal = n;
}

void Polygon::setPosition(const PositionVector& pos)
{
	position = pos;
}

PositionVector Polygon::getPosition() const
{
	return position;
}

PositionVector Polygon::getNormal() const
{
	return normal;
}

double Polygon::getArea() const
{
	return area;
}

double Polygon::getReflectivityFactor() const
{
	return reflectivity_factor;
}

double Polygon::getSpecularityFactor() const
{
	return specularity_factor;
}

int Polygon::getRotationAxisIndex() const
{
	return rotation_axis_index;
}