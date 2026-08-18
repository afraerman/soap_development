#pragma once

class ReactionWheel: public AttitudeController
{

private:
    double angular_velocity;
    double limit;
    double alpha;
    double beta;
    double dump_speed;
    double acc_speed;

    Matrix inertia;
    PositionVector normal;
    std::string apex;

public:
	ReactionWheel();
	ReactionWheel(const PositionVector& loc,double alpha, double beta, const PositionVector& norm, const Matrix& in, double angvel, double lim, double m, std::string apex="X", double dump_speed=100.0, double acc_speed=0.0);

	double getAlpha() const;
    double getBeta() const;
    double getAngularVelocity() const;
    double getLimit() const;
    double getDumpSpeed() const;
    double getAccSpeed() const;
    
    // {sin(alpha),  cos(alpha),  sin(beta),  cos(beta)}
    PositionVector getAngles() const;
    PositionVector getNormal() const;

    Matrix getInertiaTensor() const;

    std::string getApex() const;

    void setAlpha(double a);
    void setBeta(double b);
    void setAngles(double a, double b);
    void setAngularVelocity(double vel);
    void setNormal(const PositionVector& nor);
    void setInertia(const Matrix& in);
    void setLimit(double lim);

    ~ReactionWheel();

};