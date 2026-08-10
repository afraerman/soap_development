#include "stdafx.h"

PositionVector location;
PositionVector limit;

PositionVector AttitudeController::getLimit() const
{
	return limit;
}

PositionVector AttitudeController::getLocation() const
{
	return location;
}

double AttitudeController::getStoredMomentum() const
{
	return stored_momentum;
}

double AttitudeController::getMass() const
{
	return mass;
}

bool AttitudeController::requireDischarge() const
{
	return need_for_discharge;
}

void AttitudeController::setLimit(PositionVector lim)
{
	limit = lim;
}

void AttitudeController::setLocation(PositionVector loc)
{
	location = loc;
}

void AttitudeController::setStoredMomentum(double st_mom)
{
	stored_momentum = st_mom;
}

void AttitudeController::setMass(double m)
{
	mass = m;
}

void AttitudeController::requestDischarge()
{
	need_for_discharge = true;
}

void AttitudeController::discharge()
{
	stored_momentum = 0.0;
	need_for_discharge = false;
}

AttitudeController::AttitudeController()
{
	location = PositionVector();
	stored_momentum = 0.0;
	limit = PositionVector();
	mass = 1.0;
	need_for_discharge = false;
}

AttitudeController::AttitudeController(const PositionVector& loc, double st_mom, const PositionVector& lim, double m)
{
	location = loc;
	stored_momentum = st_mom;
	limit = lim;
	mass = m;
	need_for_discharge = false;
}

AttitudeController::AttitudeController(const PositionVector& loc, double st_mom, double m)
{
	location = loc;
	stored_momentum = st_mom;
	mass = m;
	limit = PositionVector(std::vector<double>{-1.0, -1.0, -1.0});
	need_for_discharge = false;
}

AttitudeController::~AttitudeController() {};