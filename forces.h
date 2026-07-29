#pragma once
class Forces
{
private:
	static PositionVector forces;
	
	// ------------- Earth gravity ---------------- //
	static std::string egmfilename;
	
	static int gravity_order;
	static Matrix order;
	static Matrix Ps;
	static Matrix ml;
	static std::vector<double> Cnm;
	static std::vector<double> Snm;
	static bool no_coefficients;

	/// @brief Stores Cnm and Snm coefficients
	/// @return 1 if an error arises
	static int getGravityCoefficients();

	/// recursive function for Holmes method
	static double a(int n, int m);
	
	/// recursive function for Holmes method
	static double b(int n, int m);
	
	/// recursive function for Holmes method
	static double f(int n, int m);
	
	/// @brief Generate all Pnm(cos(theta)) functions
	/// @param cost Cos(theta) theta - zenit distance
	/// @param pnm_order Highest order of Pnm (n <= pnm_order, m <= pnm_order)
	static void allPs(double cost, int pnm_order);

	/// @brief All sin(ml) and cos(ml)
	/// @param sinl sin(lambda) lambda - longitude
	/// @param cosl cos(lambda)
	/// @param pnm_order Highest order of m
	static void allml(double sinl, double cosl, int pnm_order);

	/// @brief Holmes algorithm of computing gravity acceleration using modified forward column method
	/// @param pos GCRF satellite position (km)
	/// @param time Current date-time
	static void earthGravityForce(const PositionVector& pos, const Time& time);

	static void centralForce(const Satellite& sat, const Time& time);
	
	// -----------------------OUTER BODIES GRAVITY----------// 

	static std::vector<const SpiceChar*> bodies;
	static std::vector<double> gms;

	/// @brief Acceleration from outer bodies
	/// @param pos GCRF satellite position (km)
	/// @param time current date-time
	///
	/// uses CSPICE ephemris
	static void outerBodiesGravityForce(const PositionVector& pos, const Time& time);

	// ------------------SOLAR PRESSURE ----------------------------- //

	/// @brief Acceleration from solar pressure
	/// @param sat Satellite
	/// @param time Current date-time
	///
	/// uses CSPICE ephemris
	static void solarPressureForce(Satellite& sat, const Time& time);
	static void srpForce(Satellite& sat, const Time& time);
	
	static void solarPressureGmat(Satellite& sat, const Time& time);

	static PositionVector solar_pressure_force;
	static bool solar_pressure_calculated;

	// ----------------- GENERAL RELATIVITY EFFECTS ----------------- //

public:
	static void setGravityOrder(const int);
	static void setEGMfile(const std::string& filename);

	static void setSolarPressureForce(const PositionVector&);
	static void setSolarPressureCalculated(const bool);
	
	/// @brief Get vector summ of all accelerations influencing satellite motion
	/// @param sat Satellite
	/// @param time Current date-time
	/// @return PositionVector of total acceleration
	static PositionVector allForces(Satellite& sat, const Time& time);
	static void checkCoeffs();
	static bool account_for_earth_gravity;
	static bool account_for_outer_gravity;
	static bool account_for_solar_pressure;
	static bool account_for_solar_pressure_gmat;
	static bool account_for_relativity;

	static void checkFilenames();
	static void checkPnm();
	static void check2000nan();
};