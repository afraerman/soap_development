#pragma once
class OrbitIntegrator : public virtual Integrator
{
private:
	Time end_time;
	Matrix c;
	Matrix alpha;
	Matrix a_arr;
	std::vector<double> substeps;
	Matrix initial_alpha;
	std::vector<double> alpha1;
	std::vector<double> alpha2;
	PositionVector initial_forces;
	PositionVector current_position;
	int number_of_equations;
	int number_of_iterations;
	int nor;

	/// Auxiliary function for Everhart method
	void generateSubsteps(double T);
	
	/// Auxiliary function for Everhart method
	void generateCMatrix();
	
	/// Auxiliary function for Everhart method
	void initializeCoefficients(Time& t, bool from_the_start);
	
	/// Auxiliary function for Everhart method
	void updateAlpha(Satellite&, const Time&, int);
	
	/// Auxiliary function for Everhart method
	void updateCoefficients(Satellite&, const Time&, int);

	/// Auxiliary function for Everhart method
	void interpolateCoefficients();

	int getNextState(Satellite&, Time&);

protected:
	/// @brief Everhart method of integrating second-order equations of motion
	/// @param satellite Satellite
	/// @param time Current date-time
	/// @param step Final point of integration [sec]
	/// @param from_the_start Continuation (false) of integration or new start (true)
	///
	/// from_the_start is used to correctly reset values when autostep is ON
	///
	/// All updated values are stored in satellite StateVector
	void integrationMethod(Satellite& satellite, Time& time, double step, bool from_the_start);
	double comparison(const Satellite& sat1, const Satellite& sat2);

public:
	/// @brief Default orbit intergrator constructor
	OrbitIntegrator();

	/// @brief Full orbit integrator constructor
	/// @param satellite Satellite
	/// @param time Current date-time
	/// @param interval Integration interval [sec]
	/// @param step Integration step [sec]. If negative -> autostep true
	/// @param output_step Step of output ephemeris [sec]
	/// @param autostep Automatic (true) or fixed (false) integration step
	/// @param tolerance Tolerance of calculation (if autostep)
	/// @param number_of_equations Dimention of space (usually 3d)
	/// @param number_of_iterations Iterations of Everhart method (2 is enough)
	/// @param nor Actual tolerance of Everhart method (7, 11, 15, 19, 23, 27)
	OrbitIntegrator(Satellite* satellite, Time* time, double interval, double step = -1.0, double output_step = 0.0, bool autostep = true, double tolerance = 1e-5,
		int number_of_equations = 3, int number_of_iterations = 2, int nor = 7);

	~OrbitIntegrator();
};