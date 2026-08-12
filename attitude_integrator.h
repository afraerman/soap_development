#pragma once

/**
 * This header file contains AttitudeIntegrator class definition.
 * Integration method is based on article by Zachary R. Manchester and Mason A. Peck
 * "Quaternion Variational Integrators for Spacecraft Dynamics"
 */

class AttitudeIntegrator: public virtual Integrator
{
private:
	int ntrial;
	double tolx;
	double tolf;
	PositionVector initial_momentum;
	PositionVector momentum;
	PositionVector phi; // parametrisation of a quaternion
	PositionVector real_scan_phi;
	Matrix jacobian;
	
	PositionVector target_momentum;
	//boost::math::quaternion<double> target_attitude;
	Quaternion target_attitude;
	//boost::math::quaternion<double> control_quat;
	Quaternion control_quat;
	//boost::math::quaternion<double> real_scan_quat;
	Quaternion real_scan_quat;
	Time target_time;
	PositionVector correction;
	PositionVector accumulated_correction;
	bool target_acquired = false;
	PositionVector backup_phi;
	PositionVector backup_momentum;
	PositionVector backup_sat_momentum;
	double gap, t;
	int end_of_current_target, elapsed_time;
	std::vector<char> control_order;
	std::string mode;

	/// @brief Jacobian of linearized equations. See equation (50) in article
	/// @param I Inertia tensor
	/// @param rho Gyrostats kinetic momentum
	/// @param step integration step
	void compute_jacobian(const Matrix& I, const PositionVector& rho, double step);

	/// @brief Kinetic momentum of satellite. See equation (49) in article
	/// @param I Inertia tensor
	/// @param rho Gyrostats kinetic momentum
	/// @param step integration step
	/// @param operation + or -
	void compute_momentum(const Matrix& I, const PositionVector& rho, double step, char operation);

	/// @brief Check if current time is scan attitude
	/// @param sat Satellite
	/// @param time current date-time
	/// @return true if current time is scan else false 
	bool scan_time(const Satellite& sat, const Time& time) const;

	/// @brief Check if current time is stop attitude
	/// @param sat Satellite
	/// @param time current date-time
	/// @return true if current time is stop attitude else false
	bool stop_time(const Satellite& sat, const Time& time) const;


	/// @brief Check if current time is dump-time
	/// @param sat Satellite
	/// @param time current date-time
	/// @return true if current time is dump-time else false
	bool dump_time(const Satellite& sat, const Time& time) const;

	/// @brief Check if current time is slew-time
	/// @param sat Satellite
	/// @param time current date-time
	/// @return true if current time is slew-time else false
	bool slew_time(const Satellite& sat, const Time& time) const;


	/// @brief Acquire new target
	/// @param sat Satellite
	/// @param time current date-time
	/// @param step integration step
	/// @param torque vector of applied torques
	/// @return 0 if target acquired successful, 1 if target failed (impossible)
	int acquire_target(Satellite& sat, Time& time, const double step, const PositionVector& torque);

protected:
	/// @brief Integration method of variational integrator described in the article.
	///
	/// Performs one integration step
	/// @param sat Satellite
	/// @param t current date-time
	/// @param t_final integration step
	/// @param from_the_start boolean parameter if integration continues or starts (if autostep)
	void integrationMethod(Satellite& sat, Time& t, double t_final, bool from_the_start);
	double comparison(const Satellite& sat1, const Satellite& sat2);

public:
	/// @brief Default constructor
	AttitudeIntegrator();
	
	/// @brief Full Attitude integrator constructor
	/// @param sat Satellite
	/// @param time current date-time
	/// @param interval integration interval [sec]
	/// @param step integration step [sec]. If <=0 -> autostep = true
	/// @param output_step output ephemeris step [sec]
	/// @param autostep Automatic (true) or fixed (false) integration step
	/// @param tolerance Loerance of calculations (if autostep)
	/// @param ntrial Number of iterations in Newton-Rapson method (4 is enough)
	/// @param tolx Tolerance in point calculation (for Newton-Rapson method)
	/// @param tolf Tolerance in function calculation (fro Newton-Rapson method)
	AttitudeIntegrator(Satellite* sat, Time* time, double interval, double step = -1.0, double output_step = 0.0, double autostep = true, double tolerance = 1e-5, int ntrial=4, double tolx=1e-15, double tolf=1e-15);
	
	

	~AttitudeIntegrator();
};