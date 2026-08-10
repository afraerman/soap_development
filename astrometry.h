#pragma once
class Astrometry
{
private:
	static std::string eopfilename;
	static std::string tlsfilename;
	static std::string ephfilename;
	static std::string gmfilename;
	static int order;
	static int _mjd;
	static Matrix duts;
	static Matrix xps;
	static Matrix yps;

	static Matrix rc2t;
	static Matrix rc2t1;
	static Matrix rt2c;

	/// @brief Performs a Lagrange interpolation
	/// @param m Matrix of known poinst (column 0) and values (column 1)
	/// @param x point of interpolation
	/// @return interpolated value at point x [double]
	static double lagrange_interpol(const Matrix&, double x);
public:
	static bool no_ephemeris;

	static Matrix getRc2t();
	static Matrix getRc2t1();
	static Matrix getRt2c();

	static void setEOPfile(const std::string& filename);
	static void setTLSfile(const std::string& filename);
	static void setEPHEMfile(const std::string& filename);
	static void setGMfile(const std::string& filename);

	/// @brief Percentage of Sun not covered by Earth from the satellite point of view
	/// @param sun_pos GCRF PositionVector of the Sun
	/// @param sat_pos GCRF PositionVector of satellite
	/// @return percentage of Sun visible from the satellite [double]
	static double eclipse_factor(const PositionVector& sun_pos, const PositionVector& sat_pos);
	
	/// @brief Initialize CSpice ephemeris kernels
	static void get_ephemeris();
 
	/// @brief Earth Orientation Parameters 
	///
	/// Find values of UT1 - UTC (dut) and pole coordinates (xp, yp) around given date
	/// @param time Date and time to find EOPs around
	///
	/// uses SOFA  iauCal2jd
	static void EOP(const Time& time);

	/// @brief Creates rotation matricies from GCRF to ITRF for 
	///
	/// coordinates (accelerations) and velocities
	/// @param time Date and time to find matricies on
	static void rotationMatrices(const Time&);

	/// @brief Finds GCRF coordinates of Earth rotation axis
	/// @param t Date and time to find axis coordinates on
	/// @return PositionVector of GCRF coordinates of Earth rotation axis
	static PositionVector get_rotation_axis(const Time&);

	static void check_filenames();
};