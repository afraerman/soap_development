#pragma once
class StateVector
{
private:
	PositionVector pos;
	PositionVector vels;
public:
	StateVector();
	StateVector(const PositionVector& position, const PositionVector& velocity);
	StateVector(const std::vector<double>& position, const std::vector<double>& velocity);
	StateVector(const std::vector<double>&);

	void setPosition(const PositionVector&);
	void setPosition(const std::vector<double>&);
	void setVelocity(const PositionVector&);
	void setVelocity(const std::vector<double>&);

	PositionVector getPosition() const;
	PositionVector getVelocity() const;

	StateVector operator-(const StateVector& st2) const;
	double norm();

	friend std::ostream& operator<<(std::ostream&, const StateVector&);

	~StateVector();
};