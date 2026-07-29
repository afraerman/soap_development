#pragma once
class Torques
{
private:
	static PositionVector torques;
	static PositionVector uncompensated_torque;

	static std::string igrffilename;
	static int magnetic_order;
	static bool no_magnetic_coefficients;
	static std::vector<double> Gnm;
	static std::vector<double> Hnm;

	/// @brief Load Gnm and Hnm coeffs
	/// @param time current date-time
	static void get_magnetic_coefficients(const Time& time);
	static Matrix order;
	static Matrix Ps_magnetic;
	static Matrix ml_magnetic;

	/// recursive coeff for Pnm with Schmidt normalization
	static double a_magnetic(int n, int m);
	/// recursive coeff for Pnm with Schmidt normalization
	static double b_magnetic(int n, int m);
	/// recursive coeff for P'nm with Schmidt normalization
	static double f_magnetic(int n, int m);

	/// @brief calculate all Pnm with Schmidt normalization
	/// @param cost cos(theta), theta - zenit distance
	/// @param sint sin(theta)
	/// @param pnm_order highest order and harmonic
	static void allPs_magnetic(double cost, double sint, int pnm_order);

	/// @brief calculate all sin(ml), cos(ml)
	/// @param sinl sin(l), l - longitude
	/// @param cosl cos(l)
	/// @param pnm_order highest order m 
	static void allml_magnetic(double sinl, double cosl, int pnm_order);

	/// @brief Vector of magnetic induction according to IGRF-13 model
	/// @param pos GCRF satellite position
	/// @param time current date-time
	/// @return PositionVector of magnetic induction
	static PositionVector magnetic_field(const PositionVector& pos, const Time& time);
	static bool no_magnetic_field_warning;

	/// @brief Torque from Earth in point model
	/// @param sat Satellite
	/// @param time current date-time
	static void earth_torque(const Satellite& sat, const Time& time);

	/// @brief Torque from solar pressure
	/// @param sat Satellite
	/// @param time current date-time
	///
	/// uses CSPICE ephemris
	static void solar_torque(Satellite& sat, const Time& time);

	static void srpTorque(Satellite& sat, const Time& time);

	// NOT IN USE
	static void magnetic_torque(const Satellite& sat);

	static std::string thrustersfilename;
	static std::string magnfilename;

	static PositionVector center_of_pressure;
	
public:
	static bool account_for_earth_torque;
	static bool account_for_solar_pressure;
	static bool account_for_magnetic_torque;

	/// @brief Vector summ of all torques 
	/// @param sat Satellite
	/// @param time Current date-time
	/// @return PositionVector of all torques
	static PositionVector allTorques(Satellite& sat, const Time& time);

	/// set path to magnetic coefficients
	static void setIGRFfile(const std::string&);
	
	static double testMagnetic(const Time& time);
	
	/// @brief Vector of magnetic induction according to IGRF-13 model
	/// @param pos GCRF satellite position (km)
	/// @param time Current date-time
	/// @return PositionVector of magnetic induction [nT]
	static PositionVector getMagneticField(const PositionVector& pos, const Time& time);

	static void setUncompensatedTorque(const PositionVector& t);
	static PositionVector getUncompensatedTorque();

	static void get_thrusters_activation_times();
	static std::vector<Time> thrusters_activation_times;
	static std::vector<PositionVector> thrusters_activation_torques;
	static void get_magnets_activation_times();
	static std::vector<Time> magn_activation_times;
	static std::vector<PositionVector> magn_activation_torques;
	static std::vector<PositionVector> magnetic_field_data;

	static PositionVector getCenterOfPressure();
};