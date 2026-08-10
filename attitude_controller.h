#pragma once
class AttitudeController
{
private:
	double mass;
	PositionVector location;
	PositionVector limit;
	double stored_momentum;
	bool need_for_discharge;

public:
	AttitudeController();
	AttitudeController(const PositionVector& loc, double st_mom,
						const PositionVector& lim = PositionVector(std::vector<double>{0.0, 0.0, 0.0}), double m = 0.0);

	AttitudeController(const PositionVector& loc, double st_mom, double m = 0.0);
	
	double getMass() const;
	bool requireDischarge() const;
	PositionVector getLocation() const;
	double getStoredMomentum() const;
	PositionVector getLimit() const;

	void setMass(double mass);
	void setLocation(PositionVector location);
	void setStoredMomentum(double stored_momentum);
	void setLimit(PositionVector limit);
	void requestDischarge();
	void discharge();

	~AttitudeController();
};