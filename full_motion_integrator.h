#pragma once
class FullMotionIntegrator : public OrbitIntegrator, public AttitudeIntegrator
{
private:
	void integrationMethod(Satellite& sat, Time& t, double step, bool from_the_start)
	{
		OrbitIntegrator::integrationMethod(sat, t, step, from_the_start);
		AttitudeIntegrator::integrationMethod(sat, t, step, from_the_start);
	}
	double comparison(const Satellite& sat1, const Satellite& sat2)
	{
		return OrbitIntegrator::comparison(sat1, sat2) + AttitudeIntegrator::comparison(sat1, sat2);
	}
public:
	FullMotionIntegrator(Satellite* satellite, Time* time, double interval, double step = -1.0, double output_step = 0.0, bool autostep = true, double tolerance = 1e-5, bool screen_check = true,
		int number_of_equations = 3, int number_of_iterations = 2, int nor = 7, int ntrial = 4, double tolx = 1e-20, double tolf = 1e-20) :
		Integrator(satellite, time, interval, step, output_step, autostep, tolerance, screen_check),
		OrbitIntegrator(satellite, time, interval, step, output_step, autostep, tolerance, number_of_equations, number_of_iterations, nor),
		AttitudeIntegrator(satellite, time, interval, step, output_step, autostep, tolerance, ntrial, tolx, tolf) {};
};