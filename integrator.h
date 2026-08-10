#pragma once
class Integrator
{
private:
	Satellite* satellite;
	Time* time;
	double interval;
	double tolerance;
	bool autostep;
	double step;
	double output_step;
	double elapsed_time;

	bool from_the_start;
	
	void make_ephemeris_header(std::ofstream&);
	void make_telemetry_header(std::ofstream&);

	virtual double comparison(const Satellite& sat1, const Satellite& sat2) { return 0.0; };
	virtual void integrationMethod(Satellite& satellite, Time& time, double step, bool from_the_start) {};

protected:
	bool integration_error = false;
	//bool trial_mode = false;
	
public:
	Integrator();

	/// @brief Full base-class integrator constructor
	/// @param satellite Satellite
	/// @param time Current date-time
	/// @param interval Integration interval [sec]
	/// @param step Integration step [sec]. If <= 0 -> autostep = true
	/// @param output_step Step of output ephemeris [sec]
	/// @param autostep Automatic (true) or fixed (false) integration step
	/// @param tolerance Tolerance of calculations (if autostep)
	Integrator(Satellite* satellite, Time* time, double interval, double step = -1.0, double output_step = 0.0, bool autostep = true, double tolerance = 1e-5);

	double getInterval();
	Satellite* getSatellite();
	Time* getTime();
	void autoStepOn();
	void setStep(double step);
	void setOutputStep(double output_step);
	void setInterval(double interval);
	
	void integrate(std::string savefilename = "");

	~Integrator();
};