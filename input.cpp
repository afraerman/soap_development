#include "stdafx.h"

inline constexpr std::uint32_t fnv1a(const char* str, std::uint32_t hash = 2166136261UL)
{
	return *str ? fnv1a(str + 1, (hash ^ *str) * 16777619ULL) : hash;
}

void Input::require_array_of_doubles(const Json::Value& node, const std::string& path, size_t expected_size)
{
	if (!node.isArray())
	{
		throw std::runtime_error(path + " must be a JSON array");
	}
	if (node.size() != expected_size)
	{
		throw std::runtime_error(path + " must contain exactly " + std::to_string(expected_size) + " elements");
	}

	for (Json::ArrayIndex i = 0; i < expected_size; i++)
	{
		if (node[i].isNull())
		{
			throw std::runtime_error(path + "[" + std::to_string(i) + "] is null");
		}
		if (!node[i].isNumeric())
		{
			throw std::runtime_error(path + "[" + std::to_string(i) + "] is not a number");
		}
	}
}

std::vector<double> Input::get_doubles(const Json::Value& node, const std::string& path, size_t expected_size)
{
	require_array_of_doubles(node, path, expected_size);
	std::vector<double> result;
	result.reserve(expected_size);
	for (Json::ArrayIndex i = 0; i < expected_size; i++)
	{
		result.push_back(node[i].asDouble());
	}
	return result;
}

std::vector<Json::Value> Input::get_parameters(const Json::Value& node, const std::string& path, size_t expected_size)
{
	if (!node.isArray())
	{
		throw std::runtime_error(path + " must be a JSON array");
	}
	if (node.size() != expected_size)
	{
		throw std::runtime_error(path + " must contain exactly " + std::to_string(expected_size) + " elements");
	}
	for (Json::ArrayIndex i = 0; i < expected_size; i++)
	{
		if (node[i].isNull()) throw std::runtime_error(path + "[" + std::to_string(i) + "] is null");
	}

	std::vector<Json::Value> parameters;
	parameters.reserve(expected_size);
	for (Json::ArrayIndex i = 0; i < expected_size; i++)
	{
		parameters.push_back(node[i]);
	}
	return parameters;
}

int Input::read_multiple_satellites_filenames(const std::string& filename, std::vector<std::string>& filenames)
{
	std::ifstream input(filename);
	if (!(input.is_open()))
	{
		return 1;
	}

	Json::Value data;
	input >> data;

	auto fnames = data["filenames"];
	for (auto fname: fnames)
	{
		filenames.push_back(fname.asString());
	}

	input.close();
	return 0;
}

int Input::read_input_file(const std::string& filename, Satellite* sat, Time* time, double& interval, double& step, double& ost)
{
	std::stringstream ss_str, ss_value;
	std::string str, name, middle, value;
	bool state_not_given = true, time_not_given = true, interval_not_given = true, step_not_given = true;
	
	double x, y, z, vx, vy, vz, seconds, w, wx, wy, wz;
	double xx, xy, xz, yx, yy, yz, zx, zy, zz;
	double nx, ny, nz, area, spec, refl;
	double mass;
	double magnx, magny, magnz;
	int year, month, day, hours, minutes, n, turn;
	std::string fname;
	
	std::ifstream input(filename);

	if (!(input.is_open()))
	{
		std::cerr << "\033[31m#01 No such input file or directory: " << filename << "\033[0m" << std::endl;
		return 1;
	}

	getline(input, str);
	while (str != "META_START")
	{
		getline(input, str);
	}
	while (getline(input, str))
	{	
		if (str == "META_START") continue;
		if (str == "") continue;
		if (str == "META_STOP") break;

		ss_str << str;
		getline(ss_str, name, ' ');
		getline(ss_str, middle, '=');
		getline(ss_str, value);
		ss_value << value;
		
		switch (fnv1a(name.c_str()))
		{
		// --------------------- SATELLITE PARAMETERS --------------- //
		
		/**
		 * Position and velocity components
		 * x y z vx vy vz [km km km km/s km/s km/s]
		 * */
		case fnv1a("StateVector"):
		{
			ss_value >> x >> y >> z >> vx >> vy >> vz;
			sat->setState(StateVector(std::vector<double>{x, y, z}, std::vector<double>{vx, vy, vz}));
			state_not_given = false;
			break;
		}

		/**
		 * Date-time in format YYYY-MM-DDTHH:MM:SS
		 * 
		 * Can use any other separator instead of '-' 'T' ':' 
		 */
		case fnv1a("StartTime"):
		{
			char tmp;

			ss_value >> year >> tmp >> month >> tmp >> day >> tmp >> hours >> tmp >> minutes >> tmp >> seconds;

			time->setTime(Time(year, month, day, hours, minutes, seconds));
			time_not_given = false;
			break;
		}

		/// w x y z components of unit-quaternion
		case fnv1a("Quaternion"):
		{
			ss_value >> w >> x >> y >> z;
			//boost::math::quaternion<double> quat{ w, x, y, z };
			Quaternion quat(w, x, y, z);
			sat->setQuaternion(quat);
			break;
		}
		
		/**
		 * Angular velocity
		 * wx wy wz [rad/sec rad/sec rad/sec]
		 */
		case fnv1a("AngularVelocity"):
		{
			ss_value >> wx >> wy >> wz;
			sat->setAngularVelocity(std::vector<double>{wx, wy, wz});
			break;
		}

		/**
		 * Inertia tensor. Although it must be diagonal still
		 * 
		 * Ixx Ixy Ixz Iyx Iyy Iyz Izx Izy Izz [kg * m^2]
		 *
		 * */
		case fnv1a("InertiaTensor"):
		{
			ss_value >> xx >> xy >> xz >> yx >> yy >> yz >> zx >> zy >> zz;
			sat->setInertiaTensor(Matrix(3, 3, std::vector<std::vector<double>>{ {xx, xy, xz}, { yx, yy, yz }, { zx, zy, zz }}));
			break;
		}

		/// mass [kg]
		case fnv1a("Mass"):
		{
			ss_value >> mass;
			sat->setMass(mass);
			break;
		}

		/**
		 * Number of polygons. Then for each polygon give a row containig:
		 * * location of center: x y z [m] [double]
		 * * unit-normal: x y z [double];
		 * * Area [m^2] [double]; 
		 * * reflectivity factor [double];
		 * * specularity factor [double]
		 *
		 * all coordinates are given in satellite reference frame
		 */
		case fnv1a("Polygons"):
		{
			ss_value >> n;
			std::vector<Polygon> polygons(n);

			for (int i = 0; i < n; i++)
			{
				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> x >> y >> z >> nx >> ny >> nz >> area >> refl >> spec;
				polygons[i] = Polygon(std::vector<double>{x, y, z}, std::vector<double>{nx, ny, nz}, area, refl, spec);
			}
			sat->setPolygons(polygons);
			break;
		}

		/**
		 * VTK polygons file
		 * 
		 */
		case fnv1a("vtk_file"):
		{
			std::string vtk_filename;
			ss_value >> vtk_filename;
			std::vector<Polygon> polygons;
			if (read_vtk_file(vtk_filename, polygons))
			{
				std::cerr << "\033[31mSmth went wrong while reading vtk file. Polygons not included\033[0m" << std::endl;
			}
			else
			{
				sat->setPolygons(polygons);
			}
			break;
		}

		/**
		 * Number of polygons. Then for each polygon give a row containig:
		 * * location of center: x y z [m] [double]
		 * * unit-normal: x y z [double];
		 * * rotation axis index: 0 - Ox, 1 - Oy, 2 - Oz [int]
		 * * Area [m^2] [double]; 
		 * * reflectivity factor [double];
		 * * specularity factor [double]
		 *
		 * all coordinates are given in satellite reference frame 
		 * rotation axis index must be integer
		 */
		case fnv1a("SolarPanels"):
		{
			int rai;
			ss_value >> n;
			std::vector<Polygon> polygons(n);
			for (int i = 0; i < n; i++)
			{
				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> x >> y >> z >> nx >> ny >> nz >> rai >> area >> refl >> spec;
				polygons[i] = Polygon(std::vector<double>{x, y, z}, std::vector<double>{nx, ny, nz}, area, refl, spec, rai);
			}
			sat->setSolarPanels(polygons);
			break;
		}

		/**
		 * These are independent gyrostats.
		 * number of gyrostats. Then for each gyrostat give two rows contatinig:
		 * I
		 * * location of center: x y z [m] [double]
		 * * intial angular velocity [rad/sec] [double]
		 * * mass [kg] [double]
		 * II
		 * * Inertia tensor: Ixx Iyy Izz [kg m^2] [double]
		 * * angular velocity limits: lim_x lim_y lim_z [rad/sec] [double]
		 * 
		 * Two of three limits should be 0.0 therefore non-zero limit
		 * is used to determine gyrostat axis
		 */
		case fnv1a("Gyrostats"):
		{
			ss_value >> n;
			std::vector<AttitudeController> gyrostats(n);
			PositionVector coords, limits, tmp;
			Matrix inertia, ir;
			double angular_momentum = 0.0, angvel;

			for (int i = 0; i < n; i++)
			{
				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> coords[0] >> coords[1] >> coords[2] >> angvel >> mass;

				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> ir[0][0] >> ir[1][1] >> ir[2][2] >> limits[0] >> limits[1] >> limits[2];

				for (int j = 0; j < 3; j++) if (limits[j]) angular_momentum = ir[j][j] * angvel;
				
				limits[0] = ir[0][0] * limits[0];
				limits[1] = ir[1][1] * limits[1];
				limits[2] = ir[2][2] * limits[2];

				inertia = inertia + ir + coords.skew() * coords.skew() * (-1.0 * mass);
				gyrostats[i] = AttitudeController(coords, angular_momentum, limits, mass);
			}
			sat->setInertiaTensor(sat->getInertiaTensor() + inertia);
			sat->setGyrostats(gyrostats);
			break;
		}

		/**
		 * Block of exactly 4 reaction wheels. but still first input being
		 * Number of reaction wheels in block (must be 4 or 8). Then describe in 4 rows:
		 * I
		 * * location relative to center of mass: x y z [m] [double]
		 * II - angles
		 * * alpha [degrees] [double]
		 * * beta [degrees] [double]
		 * III
		 * * inertia tensor in primary axes: I11 I22 I33^ [kg * m^2] [double]
		 * * mass [kg] [double]
		 * IV
		 * * angular velocity: wx wy wz [rad/sec] [double]
		 * * it's limit: limit [rad/sec] [double]
		 * 
		 * ^I33 is alongside rotation axis
		 */
		case fnv1a("ReactionWheelsBlock"):
		{
			ss_value >> n;
			if (n != 4)
			{
				std::cerr << "\033[31mThere must be exeactly 4 reaction wheels in one block\033[0m" << std::endl;
				return 1;
			}

			std::vector<ReactionWheel> reaction_wheels(n);
			PositionVector location, inertia_diag, limits, z_axis;
			PositionVector angvel({0.0, 0.0, 0.0, 0.0});
			Matrix inertia, ir, sat_inertia;
			double angular_momentum = 0.0, limit, mass, alpha, beta, tmp;
			std::string apex;

			
			// location relative to satellite center of mass
			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> location[0] >> location[1] >> location[2];
			
			// alpha and beta angles in degrees
			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> alpha >> beta;
			
			alpha *= M_PI / 180.0;
			beta *= M_PI / 180.0;

			// inertia tensor in primary axes and mass
			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> inertia_diag[0] >> inertia_diag[1] >> inertia_diag[2] >> mass;

			// angular velocity and it's limit
			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> angvel[0] >> angvel[1] >> angvel[2] >> angvel[3] >> limit >> apex;

			limit *= inertia_diag[2]; // angular momentum limit

			z_axis[0] = std::sin(alpha);
			z_axis[1] = std::cos(alpha);
			z_axis[2] = std::cos(alpha);
			for (int i = 0; i < 4; i++)
			{
				if (i == 0)
				{
					z_axis[1] *= -1.0*std::cos(beta);
					z_axis[2] *= -1.0*std::sin(beta);
				}
				if (i == 1)
				{
					tmp = location[1];
					location[1] = -1.0 * location[2];
					location[2] = tmp;

					z_axis[1] *= std::sin(beta);
					z_axis[2] *= -1.0*cos(beta);
				}
				if (i == 2)
				{
					location[1] *= -1.0;
					location[2] *= -1.0;

					z_axis[1] *= std::cos(beta);
					z_axis[2] *= std::sin(beta);
				}
				if (i == 3)
				{
					tmp = location[1];
					location[1] = location[2];
					location[2] = -1.0 * tmp;

					z_axis[1] *= -1.0*std::sin(beta);
					z_axis[2] *= std::cos(beta);
				}

				reaction_wheels[i] = ReactionWheel(location, alpha, beta, z_axis, inertia, angvel[i], limit, mass, apex);
				// converge inertia to satellite axes
				ir[0][0] = z_axis[1] * z_axis[1];
				ir[0][1] = z_axis[0] * z_axis[0];
				ir[0][2] = 0.0;

				ir[1][0] = z_axis[0]*z_axis[0]*z_axis[2]*z_axis[2];
				ir[1][1] = z_axis[1]*z_axis[1]*z_axis[2]*z_axis[2];
				ir[1][2] = (z_axis[0]*z_axis[0] - z_axis[1]*z_axis[1])*(z_axis[0]*z_axis[0] - z_axis[1]*z_axis[1]);

				ir[2][0] = z_axis[0] * z_axis[0];
				ir[2][1] = z_axis[1] * z_axis[1];
				ir[2][2] = z_axis[2] * z_axis[2];

				inertia_diag = mul(ir, inertia_diag);

				double parallel_axis_theorem = mass * location.norm() * location.norm();

				inertia[0][0] = inertia_diag[0] + parallel_axis_theorem;
				inertia[1][1] = inertia_diag[1] + parallel_axis_theorem;
				inertia[2][2] = inertia_diag[2] + parallel_axis_theorem;
				
				sat_inertia = sat_inertia + inertia + location.skew() * location.skew() * (-1.0 * mass);
			}
			sat->setInertiaTensor(sat->getInertiaTensor() + sat_inertia);
			sat->setReactionWheelsBlock(reaction_wheels);
			sat->setMass(sat->getMass() + 4.0 * mass);
			break;
		}

		/**
		 * Number of magnetroquers. Then for each magnetroquer give 1 row:
		 * * limits: limx limy limz [A * m^2] [double]
		 * * initial magnetic torque: initial_torque [A * m^2] [doble]
		 * * mass [kg];
		 */
		case fnv1a("Magnetorquers"):
		{
			ss_value >> n;
			std::vector<AttitudeController> magnetorquers(n);
			PositionVector limits;
			double initial_moment;
			for (int i = 0; i < n; i++)
			{
				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> limits[0] >> limits[1] >> limits[2] >> initial_moment >> mass;
				magnetorquers[i] = AttitudeController(PositionVector(), initial_moment, limits, mass);
			}
			sat->setMagnetorquers(magnetorquers);
			break;
		}

		/**
		 * Number of thrusters. Then for each thruster give 2 rows:
		 * I:
		 * * position: posx posy posz [m] [double]
		 * * mass [kg] [double]
		 * II:
		 * * Isp: Isp_x Isp_y Isp_z [s] [double]^
		 * 
		 * ^Isp should be non-zero in exactly one component
		 */
		case fnv1a("Thrusters"):
		{
			ss_value >> n;
			std::vector<AttitudeController> thrusters(n);
			PositionVector position, limits;
			double initial_moment, total_mass = 0.0;
			for (int i = 0; i < n; i++)
			{
				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> position[0] >> position[1] >> position[2] >> mass;

				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> limits[0] >> limits[1] >> limits[2];
				limits = limits * 9.80665; // [s] -> [m/s]
				thrusters[i] = AttitudeController(position, 0.0, limits, mass);
				total_mass += mass;
			}
			sat->setThrusters(thrusters);
			sat->setMass(sat->getMass() + total_mass);
			break;
		}


		/*
		case fnv1a("MagneticMomentum"):
		{
			ss_value >> magnx >> magny >> magnz;
			sat->setMagneticMomentum(PositionVector(std::vector<double>{magnx, magny, magnz}));
			break;
		}
		*/

		// ---------------------  ATTITUDE MODES ------------------ //

		/**
		 * Parameters of constant attitude mode: 3 rows
		 * I
		 * * unit-quaternion to maintain: w x y z [double]
		 * II
		 * * date-time to start: @see StartTime format
		 * III
		 * * date-time to stop: @see StartTime format
		 */
		case fnv1a("StopMotion"):
		{
			char tmp;

			ss_value >> w >> x >> y >> z;

			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> year >> tmp >> month >> tmp >> day >> tmp >> hours >> tmp >> minutes >> tmp >> seconds;
			Time stopmotion_start = Time(year, month, day, hours, minutes, seconds);

			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> year >> tmp >> month >> tmp >> day >> tmp >> hours >> tmp >> minutes >> tmp >> seconds;
			Time stopmotion_stop = Time(year, month, day, hours, minutes, seconds);

			//sat->setStopMotion(stopmotion_start, stopmotion_stop, boost::math::quaternion<double>{w, x, y, z});
			sat->setStop(stopmotion_start, stopmotion_stop);
			sat->disableCounterrotation();
			break;
		}

		/**
		 * Parameter of scan attitude mode: 3 rows
		 * I
		 * * angular velocity to maintain: wx wy wz [rad/sec] [double]
		 * II
		 * * date-time to start: @see StartTime format
		 * III
		 * * date-time to stop: @see StartTime format
		 */
		case fnv1a("Scan"):
		{
			ss_value >> wx >> wy >> wz;
			
			char tmp;
			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> year >> tmp >> month >> tmp >> day >> tmp >> hours >> tmp >> minutes >> tmp >> seconds;
			Time scan_start = Time(year, month, day, hours, minutes, seconds);
			
			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> year >> tmp >> month >> tmp >> day >> tmp >> hours >> tmp >> minutes >> tmp >> seconds;
			Time scan_stop = Time(year, month, day, hours, minutes, seconds);

			sat->setScan(scan_start, scan_stop, PositionVector({ wx, wy, wz }));

			break;
		}

		/**
		 * Pulse engine parameters. 3 rows required:
		 * I
		 * * location in satellite frame: x y z [m] (for now there can be only one pulse engine)
		 * II
		 * * Force of pulse engines: fx fy fz [N]
		 * III
		 * * Date-time of pulse: @see StartTime format
		 */
		case fnv1a("PulseEngine"):
		{
			ss_value >> x >> y >> z;
			sat->setPulseEngineLocation(PositionVector({x, y, z}));

			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> x >> y >> z;

			char tmp;
			ss_str.clear();
			getline(input, str);
			ss_str << str;
			ss_str >> year >> tmp >> month >> tmp >> day >> tmp >> hours >> tmp >> minutes >> tmp >> seconds;
			Time pulse_time = Time(year, month, day, hours, minutes, seconds);

			sat->setPulse(pulse_time, PositionVector({x, y, z}));

			break;
		}

		/**
		 * Order in which control systems will be applyed to perform corrections
		 * r - reaction wheels block
		 * g - gyrostats
		 * m - magnetorquers
		 * 0 - thrusters
		 * 
		 * default - 0 0
		 * 
		 * NOTE: there can't be both "gyrostats" and "reaction wheels block" on the same satellite
		 */
		case fnv1a("ControlOrder"):
		{
			char first, second;
			ss_value >> first >> second;
			Control::setControlOrder(first, second);
			break;
		}

		// --------------------- INTEGRATION PARAMETERS ----------- //

		/// Integration interval [sec] [double]
		case fnv1a("Interval"):
		{
			double interv;
			ss_value >> interv;
			interval = interv;
			interval_not_given = false;
			break;
		}
		
		/// Integration step [sec] [double]
		case fnv1a("Step"):
		{
			double st;
			ss_value >> st;
			step = st;
			step_not_given = false;
			ost = st;
			break;
		}

		/// The highest order M (gravity potential)
		case fnv1a("GravityOrder"):
		{
			ss_value >> n;
			Forces::setGravityOrder(n);
			break;
		}

		/// NOT IN USE!!!
		case fnv1a("OuterBodies"):
		{
			std::string name;
			double gm;

			ss_value >> n;
			std::vector<const char*> names(n);
			std::vector<double> gms(n);

			for (int i = 0; i < n; i++)
			{
				ss_str.clear();
				getline(input, str);
				ss_str << str;
				ss_str >> name >> gm;
				names[i] = name.c_str();
				//std::cout << names[i] << std::endl;
				gms[i] = gm;
			}
			//std::cout << names[0] << names[1] << names[2] << std::endl;
			//Forces::setExternalBodies(names, gms);
			break;
		}

		/// Step of output ephemeris [sec] [double]
		case fnv1a("OutputStep"):
		{
			double st;
			ss_value >> st;
			ost = st;
			break;
		}
		// --------------------- SELECTORS ------------------------ // 

		/// 1 or 0
		case fnv1a("GravityForce"):
		{
			ss_value >> turn;
			Forces::account_for_earth_gravity = (bool)turn;
			break;
		}

		case fnv1a("OuterGravityForce"):
		{
			ss_value >> turn;
			Forces::account_for_outer_gravity = (bool)turn;
			break;
		}

		case fnv1a("SolarPressureForce"):
		{
			ss_value >> turn;
			Forces::account_for_solar_pressure = (bool)turn;
			break;
		}

		case fnv1a("SolarPressureGmat"):
		{
			ss_value >> turn;
			Forces::account_for_solar_pressure_gmat = (bool)turn;
			break;
		}
		
		case fnv1a("GravityTorque"):
		{
			ss_value >> turn;
			Torques::account_for_earth_torque = (bool)turn;
			break;
		}

		case fnv1a("SolarPressureTorque"):
		{
			ss_value >> turn;
			Torques::account_for_solar_pressure = (bool)turn;
			break;
		}

		case fnv1a("MagneticTorque"):
		{
			ss_value >> turn;
			Torques::account_for_magnetic_torque = (bool)turn;
			break;
		}

		// ----------------------- FILES --------------------------- //
		/// Path to EGM file (cnm and snm coeffs)
		case fnv1a("EGMPath"):
		{
			ss_value >> fname;
			Forces::setEGMfile(fname);
			break;
		}

		/// Path to EOP file (dut xp yp)
		case fnv1a("EOPPath"):
		{
			ss_value >> fname;
			Astrometry::setEOPfile(fname);
			break;
		}

		/// Path to TLS file (leap seconds) (cspice)
		case fnv1a("TLSPath"):
		{
			ss_value >> fname;
			Astrometry::setTLSfile(fname);
			break;
		}

		/// Path to ephemeris file (cspice example: ../de440.bsp)
		case fnv1a("EPHEMPath"):
		{
			ss_value >> fname;
			Astrometry::setEPHEMfile(fname.c_str());
			break;
		}

		/// Path to IGRF file (gnm hnm)
		case fnv1a("IGRFPath"):
		{
			ss_value >> fname;
			Torques::setIGRFfile(fname);
			break;
		}
		
		/// Path to output file of ephemeris
		case fnv1a("SAVEPath"):
		{
			ss_value >> fname;
			FILENAMES::ephemeris_filename = fname;
			break;
		}

		/// Path to output telemetry file
		case fnv1a("TELPath"):
		{
			ss_value >> fname;
			FILENAMES::telemetry_filename = fname;
			break;
		}

		// ---------------------- DEFAULT -------------------------- //

		default:
		{
			std::cerr << "\033[31mUnknown parameter: " << name << "\033[0m" << std::endl;
			return 1;
		}
		
		}

		ss_str.clear();
		ss_value.clear();

	}
	input.close();

	if (state_not_given) { std::cerr << "\033[31mState not given\033[0m" << std::endl; return 1; }
	if (time_not_given) { std::cerr << "\033[31mInitial time not given\033[0m" << std::endl; return 1; }
	if (interval_not_given) { std::cerr << "\033[31mInterval not given\033[0m" << std::endl; return 1; }
	if (step_not_given) { std::cerr << "\033[31mStep not given\033[0m" << std::endl; return 1; }
	return 0;
}
	

int Input::read_vtk_file(const std::string& filename, std::vector<Polygon>& polygons, const std::string& koeffs_filename)
{

	std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "\033[31m#02 Unable to open file " << filename << "\033[0m" << std::endl;
        return 1;
    }

    std::string line;
    // Пропускаем заголовок
    while (std::getline(file, line) && line.find("POINTS") == std::string::npos);

    // Читаем POINTS
    int num_nodes;
    std::vector<Node> nodes;

    std::sscanf(line.c_str(), "POINTS %d float", &num_nodes);
    nodes.resize(num_nodes);

    bool koeffs_given = true;
    std::string koeffs_line;
    std::ifstream koeffs_file(koeffs_filename);
    if (!koeffs_file.is_open())
    {
    	koeffs_given = false;
    	if (koeffs_filename != "")
    	{
    		std::cerr << "Unable to open koeffs vtk file "  << koeffs_filename << std::endl;
    	}
    }

    for (int i = 0; i < num_nodes; ++i)
    {
        std::getline(file, line);
        std::sscanf(line.c_str(), "%lf %lf %lf", 
                    &nodes[i].x, &nodes[i].y, &nodes[i].z);
    }

    // Читаем POLYGONS
    while (std::getline(file, line) && line.find("POLYGONS") == std::string::npos);
    int num_polygons, total_size;
    std::vector<int> node_ids;
    PositionVector side1, side2, side3, pos;
    double area;

    std::sscanf(line.c_str(), "POLYGONS %d %d", &num_polygons, &total_size);
    polygons.resize(num_polygons);
    for (int i = 0; i < num_polygons; ++i)
    {
        std::getline(file, line);
        std::stringstream ss(line);
        ss >> num_nodes;
        node_ids.resize(num_nodes);

        for (int j = 0; j < num_nodes; j++)
        {
        	ss >> node_ids[j];
        }

        if (num_nodes == 3)
        {
        	side1 = PositionVector({nodes[node_ids[1]].x - nodes[node_ids[0]].x, nodes[node_ids[1]].y - nodes[node_ids[0]].y, nodes[node_ids[1]].z - nodes[node_ids[0]].z}) / 1000.0;
        	side2 = PositionVector({nodes[node_ids[2]].x - nodes[node_ids[1]].x, nodes[node_ids[2]].y - nodes[node_ids[1]].y, nodes[node_ids[2]].z - nodes[node_ids[1]].z}) / 1000.0;
        	pos = PositionVector({nodes[node_ids[0]].x, nodes[node_ids[0]].y, nodes[node_ids[0]].z}) / 1000.0 + 2.0 / 3.0 * side1 + 1.0 / 3.0 * side2;
        	polygons[i].setPosition(pos);
        	area = 0.5 * side1.cross(side2).norm();
        	polygons[i].setArea(area);
        }
        else if (num_nodes == 4)
        {
        	// пока что просто возьмём среднее, не уверен, что это верно (если что, это середина отрезка, соединяющего середины диагоналей)
        	double pos_x = 0.0;
        	double pos_y = 0.0;
        	double pos_z = 0.0;
        	for (int k = 0; k < 4; k++)
        	{
        		pos_x += 0.25 * nodes[node_ids[k]].x / 1000.0;
        		pos_y += 0.25 * nodes[node_ids[k]].y / 1000.0;
        		pos_z += 0.25 * nodes[node_ids[k]].z / 1000.0;
        	}

        	polygons[i].setPosition(PositionVector({pos_x, pos_y, pos_z}));

        	side1 = PositionVector({nodes[node_ids[1]].x - nodes[node_ids[0]].x, nodes[node_ids[1]].y - nodes[node_ids[0]].y, nodes[node_ids[1]].z - nodes[node_ids[0]].z}) / 1000.0;
        	side2 = PositionVector({nodes[node_ids[2]].x - nodes[node_ids[1]].x, nodes[node_ids[2]].y - nodes[node_ids[1]].y, nodes[node_ids[2]].z - nodes[node_ids[1]].z}) / 1000.0;
        	area = 0.5 * side1.cross(side2).norm();

        	side1 = PositionVector({nodes[node_ids[3]].x - nodes[node_ids[2]].x, nodes[node_ids[3]].y - nodes[node_ids[2]].y, nodes[node_ids[3]].z - nodes[node_ids[2]].z}) / 1000.0;
        	side2 = PositionVector({nodes[node_ids[0]].x - nodes[node_ids[3]].x, nodes[node_ids[0]].y - nodes[node_ids[3]].y, nodes[node_ids[0]].z - nodes[node_ids[3]].z}) / 1000.0;
        	area += 0.5 * side1.cross(side2).norm();

        	polygons[i].setArea(area);
        }
        
        if ((koeffs_given) && (std::getline(koeffs_file, koeffs_line)))
        {
        	double alpha, mu;
        	std::sscanf(koeffs_line.c_str(), "%lf %lf", &alpha, &mu);
        	polygons[i].setReflectivityFactor(alpha);
        	polygons[i].setSpecularityFactor(mu);
        }
        else
        {
        	polygons[i].setReflectivityFactor(0.54);
        	polygons[i].setSpecularityFactor(1.0);
    	}
    }

    // Пропускаем CELL_TYPES
    while (std::getline(file, line) && line.find("CELL_DATA") == std::string::npos);

    // Читаем нормали
    double nx, ny, nz;
    while (std::getline(file, line) && line.find("VECTORS Normals") == std::string::npos);
    for (int i = 0; i < num_polygons; ++i) {
        std::getline(file, line);
        std::sscanf(line.c_str(), "%lf %lf %lf", 
                    &nx, &ny, &nz);
       	polygons[i].setNormal(PositionVector({nx, ny, nz}));
    }

    file.close();

    std::cout << "\033[32mDone reading vtk file\033[0m" << std::endl;

	return 0;
}

int Input::read_json_file(const std::string& filename, Satellite* sat, Time* time, double& interval, double& step, double& ost, bool& screen_check)
{
	std::ifstream data_file(filename, std::ifstream::binary);
	if (!data_file.is_open())
	{
		std::cerr << "\033[31m#01 No json file found with name: " << filename << "\033[0m" << std::endl;
		return 1;
	}
	std::cout << "\033[32mJson file is opened\033[0m" << std::endl;
	Json::Value data;
	try {
		data_file >> data;
	}
	catch (const std::exception& e)
	{
		std::cerr << "\033[31m#01 Error while reading JSON file:\033[0m" << std::endl;
		std::cerr << e.what() << std::endl;
		return 1;
	}

	/* ---------------------------------- SIMULATION ------------------------------------*/
	auto simulation = data["simulation"];

	if (simulation.isMember("interval") && !simulation["interval"].isNull())
	{
		try
		{
			interval = simulation["interval"].asDouble();
			if (interval < 0.0) throw;
			parameters_dict["Simulation"]["interval"] = true;
		}
		catch (...)
		{
			std::cerr << "\033[31m#1111_interval Invalid value of interval " << simulation["interval"] << "\033[0m" << std::endl;
			return 1;
		}
	}
	else
	{
		std::cerr << "\033[31m#1110_interval Interval not given\033[0m" << std::endl;
		return 1;
	}

	if (simulation.isMember("step") && !simulation["step"].isNull())
	{
		try
		{
			step = simulation["step"].asDouble();
			if (step < 0.0) throw;
			ost = step;
			parameters_dict["Simulation"]["step"] = true;
		}
		catch (...)
		{
			std::cerr << "\033[31m#1151_step Invalid value of step " << simulation["step"] << "\033[0m\n";
			return 1;
		}
	}
	else
	{
		std::cerr << "\033[31m#1150_step Step not given\033[0m" << std::endl;
		return 1;
	}

	if (simulation.isMember("start_time") && !simulation["start_time"].isNull())
	{
		try
		{
			time->setTime(Time(simulation["start_time"].asString()));
			parameters_dict["Simulation"]["start_time"] = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "\033[31m#1141_start_time " << e.what() << "\033[0m\n";
			return 1;
		}
	}
	else
	{
		std::cerr << "\033[31m#1140_start_time Start time not given\033[0m" << std::endl;
		return 1;
	}

	if (simulation.isMember("output_step") && !simulation["output_step"].isNull())
	{
		try
		{
			ost = simulation["output_step"].asDouble();
			parameters_dict["Simulation"]["output_step"] = true;
		}
		catch(...)
		{
			std::cerr << "\033[31m#1131_output_step Invalid value of output step " << simulation["output_step"] << "\033[0m\n";
			return 1;
		}
	}
	if (simulation.isMember("orbit_file") && !simulation["orbit_file"].isNull())
	{
		try
		{
			std::string orbit_filename = simulation["orbit_file"].asString();
			sat->setOrbitFilename(orbit_filename);
			parameters_dict["Simulation"]["orbit_file"] = true;
		}
		catch (...)
		{
			std::cerr << "\033[31m#1121_orbit_file Invalid value of orbit filename " << simulation["orbit_file"] << "\033[0m\n";
			return 1;
		}
	}
	if (simulation.isMember("screen_check") && !(simulation["screen_check"].isNull()))
	{
		try
		{
			std::string str_screen_check = simulation["screen_check"].asString();
			if (str_screen_check == "true")
			{
				screen_check = true;
			}
			else
			{
				screen_check = false;
			}
		}
		catch (...)
		{
			std::cerr << "\033[31m#1161 Unable to resolve screen check value " << simulation["screen_check"] << " as bool\033[0m\n";
			return 1;
		}
	}

	/* ---------------------------------- SPACECRAFT --------------------------------------*/
	auto spacecraft = data["spacecraft"];

	if (spacecraft.isMember("state_vector") && !spacecraft["state_vector"].isNull())
	{
		try 
		{
			auto st = get_doubles(spacecraft["state_vector"], "spacecraft.state_vector", 6);

			sat->setState(StateVector(st));
			parameters_dict["Spacecraft"]["state_vector"] = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "\033[31m#1251_state_vector " << e.what() << "\033[0m" << std::endl;
			return 1;
		}
	}
	else if (!(data["simulation"]["orbit_file"]))
	{
		std::cerr << "\033[31m#1250_state_vector State not given\033[0m" << std::endl;
		return 1;
	}

	if (spacecraft.isMember("quaternion") && !spacecraft["quaternion"].isNull())
	{
		try 
		{
			auto q = get_doubles(spacecraft["quaternion"], "spacecraft.quaternion", 4);
			Quaternion quat(q[0], q[1], q[2], q[3]);
			sat->setQuaternion(quat);
			parameters_dict["Spacecraft"]["quaternion"] = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "\033[31m#1241_quaternion " << e.what() << "\033[0m\n";
			return 1;
		}
		//boost::math::quaternion<double> quat {q[0].asDouble(), q[1].asDouble(), q[2].asDouble(), q[3].asDouble()};
	}
	else
	{
		std::cerr << "\033[31m#1240_quaternion Quaternion not given\033[0m" << std::endl;
		return 1;
	}

	if (spacecraft.isMember("inertia_tensor") && !spacecraft["inertia_tensor"].isNull())
	{
		try
		{
			auto it = get_doubles(spacecraft["inertia_tensor"], "spacecraft.inertia_tensor", 3);
			sat->setInertiaTensor(Matrix(3, 3, std::vector<std::vector<double>>{
				{it[0], 0.0, 0.0},
				{0.0, it[1], 0.0},
				{0.0, 0.0, it[2]}
				}));
			parameters_dict["Spacecraft"]["inertia_tensor"] = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "\033[31m#1221_inertia_tensor " << e.what() << "\033[0m\n";
			return 1;
		}
	}
	else
	{
		std::cerr << "\033[31m#1220_inertia_tensor Inertia tensor not given\033[0m\n";
		return 1;
	}

	if (spacecraft.isMember("angular_velocity") && !spacecraft["angular_velocity"].isNull())
	{
		try
		{
			auto av = get_doubles(spacecraft["angular_velocity"], "spacecraft.angular_velocity", 3);
			sat->setAngularVelocity(av);
			parameters_dict["Spacecraft"]["angular_velocity"] = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "\033[31m#1211_angular_velocity " << e.what() << "\033[0m\n";
			return 1;
		}
	}

	if (spacecraft.isMember("mass") && !spacecraft["mass"].isNull())
	{
		try
		{
			sat->setMass(spacecraft["mass"].asDouble());
			if (spacecraft["mass"].asDouble() < 0.0) throw;
			parameters_dict["Spacecraft"]["mass"] = true;
		}
		catch(...)
		{
			std::cerr << "\033[31m#1231_mass Invalid value of mass " << spacecraft["mass"] << "\033[0m\n";
			return 1;
		}
	}
	else
	{
		std::cerr << "\033[31m#1230_mass Mass not given\033[0m" << std::endl;
		return 1;
	}

	/* --------------------------------- GEOMETRY (SURFACE) -------------------------------*/
	auto geometry = data["geometry"];

	if (geometry.isMember("vtk_file") && !geometry["vtk_file"].isNull())
	{
		parameters_dict["Geometry"]["vtk_file"] = true;

		std::string coeffs_filename = "";
		std::string vtk_filename = "";
		std::vector<Polygon> polygons;
		try
		{
			vtk_filename = geometry["vtk_file"].asString();
		}
		catch (...)
		{
			std::cerr << "\033[31m#1341_vtk_file Invalid vtk_filename " << geometry["vtk_file"] << "\033[0m\n";
			return 1;
		}
		
		if (geometry.isMember("vtk_coeffs") && !geometry["vtk_coeffs"].isNull())
		{
			try
			{
				coeffs_filename = geometry["vtk_coeffs"].asString();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1351_vtk_coeffs Invalid coefficients filename " << geometry["vtk_coeffs"] << "\033[0m\n";
				return 1;
			}
		}
		if (read_vtk_file(vtk_filename, polygons, coeffs_filename))
		{
			std::cerr << "\033[33mSmth went wrong while reading vtk file. Polygons not included\033[0m" << std::endl;
			return 1;
		}
		else
		{
			sat->setPolygons(polygons);
		}
	}

	if (geometry.isMember("hdf5_file") && !geometry["hdf5_file"].isNull())
	{
		try {
			std::string hdf5_filename = geometry["hdf5_file"].asString();
			sat->setHdfFile(hdf5_filename);
			parameters_dict["Geometry"]["hdf5_file"] = true;
		}
		catch (...)
		{
			std::cerr << "\033[31m#1311_hdf5_file Invalid hdf5 filename " << geometry["hdf5_file"] << "\033[0m\n";
			return 1;
		}
	}

	if (geometry.isMember("polygons") && !geometry["polygons"].isNull())
	{
		int count;
		if (geometry["polygons"].isMember("count") && !geometry["polygons"]["count"].isNull())
		{
			try
			{
				count = geometry["polygons"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1321_polygons.count Invalid value of polygons count " << geometry["polygons"]["count"] << "\033[0m\n";
				return 1;
			}
			std::vector<Polygon> polygons(count);
			try
			{
				auto polys = get_parameters(geometry["polygons"]["parameters"], "geometry.polygons.parameters", count);
				for (int i = 0; i < count; i++)
				{
					try
					{
						auto position = get_doubles(polys[i]["position"], "geometry.polygons.parameters.position", 3);
						auto normal = get_doubles(polys[i]["normal"], "geometry.polygons.parameters.normal", 3);
						double area = polys[i]["area"].asDouble();
						double albedo = polys[i]["reflectivity"].asDouble();
						double specularity = polys[i]["specularity"].asDouble();
						polygons[i] = Polygon(
							position,
							normal,
							area,
							albedo,
							specularity
							);
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#1321_polygons[" + std::to_string(i) + "] " << e.what() << "\033[0m\n";
						return 1;
					}
				}
				sat->setPolygons(polygons);
				parameters_dict["Geometry"]["polygons"] = true;
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1321_polygons.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{ 
			std::cerr << "\033[31m#1320_polygons.count Count not given\033[0m\n";
			return 1;
		}
	}

	if (geometry.isMember("solar_panels") && !geometry["solar_panels"].isNull())
	{
		int count;
		if (geometry["solar_panels"].isMember("count") && !geometry["solar_panels"]["count"].isNull())
		{
		try
		{
				count = geometry["solar_panels"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1331_solar_panels.count Invalid value of solar_panels.count " << geometry["solar_panels"]["count"] << "\033[0m\n";
				return 1;
			}
			std::vector<Polygon> solar_panels(count);
			
			try
			{
				auto polys = get_parameters(geometry["solar_panels"]["parameters"], "geometry.solar_panels.parameters", count);
				for (int i = 0; i < count; i++)
				{
					try
					{
						auto position = get_doubles(polys[i]["position"], "geometry.solar_panels.parameters.position", 3);
						auto normal = get_doubles(polys[i]["normal"], "geometry.solar_panels.parameters.normal", 3);
						double area = polys[i]["area"].asDouble();
						double albedo = polys[i]["reflectivity"].asDouble();
						double specularity = polys[i]["specularity"].asDouble();
						int rai = polys[i]["rai"].asInt();
						solar_panels[i] = Polygon(
							position,
							normal,
							area,
							albedo,
							specularity,
							rai
							);
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#1331_solar_panel[" + std::to_string(i) + "] " << e.what() << "\033[0m\n";
						return 1;
					}
				}
				parameters_dict["Geometry"]["solar_panels"] = true;
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1331_solar_panels.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1330_solar_panels Count not given\033[0m\n";
			return 1;
		}
	}

	/* ---------------------------------- CONTROL SYSTEMS ---------------------------------*/
	auto control_systems = data["control_systems"];

	if (control_systems.isMember("gyrostats") && !control_systems["gyrostats"].isNull())
	{
		parameters_dict["Control_systems"]["gyrostats"] = true;
		int count;
		if (control_systems["gyrostats"].isMember("count") && !control_systems["gyrostats"]["count"].isNull())
		{
			try
			{
				count = control_systems["gyrostats"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1431_gyrostats.count Invalid value " << control_systems["gyrostats"]["count"] << " for gyrostats count\033[0m\n";
				return 1;
			}

			if (count % 3 != 0)
			{
				std::cerr << "\033[31m#1431_gyrostats There must be 3*N gyrostats. Check 'Gyrostats vs Reaction Wheels' chapter\033[0m" << std::endl;
				return 1;
			}

			try
			{
				auto gyrs = get_parameters(control_systems["gyrostats"]["parameters"], "control_systems.gyrostats.parameters", count);
				std::vector<AttitudeController> gyrostats(count);
				PositionVector coords, limits, tmp;
				Matrix inertia, ir;
				double angular_momentum = 0.0, angvel, mass;

				for (int i = 0; i < count; i++)
				{
					try
					{
						coords = get_doubles(gyrs[i]["location"], "gyrostats.parameters[" + std::to_string(i) + "].location", 3);
						limits = get_doubles(gyrs[i]["limits"], "gyrostats.parameters[" + std::to_string(i) + "].limits", 3);
						for (int k = 0; k < 3; k++)
						{
							ir[k][k] = gyrs[i]["inertia"][k].asDouble();
							limits[k] *= ir[k][k];
						}
						angvel = gyrs[i]["angular_velocity"].asDouble();
						mass = gyrs[i]["mass"].asDouble();
						inertia = inertia + ir + coords.skew() * coords.skew() * (-1.0 * mass);
						gyrostats[i] = AttitudeController(coords, angular_momentum, limits, mass);
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#1431_gyrostats["  + std::to_string(i) + "] " << e.what() << "\033[0m\n";
						return 1;
					}
				}

				sat->setInertiaTensor(sat->getInertiaTensor() + inertia);
				sat->setGyrostats(gyrostats);
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1431_gyrostats.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1430_gyrostats Count not given\033[0m\n";
			return 1;
		}
	}

	if (control_systems.isMember("reaction_wheels") && !control_systems["reaction_wheels"].isNull())
	{
		parameters_dict["Control_systems"]["reaction_wheels"] = true;
		int count;
		if (control_systems["reaction_wheels"].isMember("count") && !control_systems["reaction_wheels"]["count"].isNull())
		{
			try
			{
				count = control_systems["reaction_wheels"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1461_reaction_wheels.count Invalid value " << control_systems["reaction_wheels"]["count"] << " as count\033[0m\n";
				return 1;
			}
			if (count % 4 != 0)
			{
				std::cerr << "\033[31m#1461_reaction_wheels There must be exactly 4*N reaction wheels. Check 'Gyrostats vs Reaction Wheels' chapter\033[0m" << std::endl;
				return 1;
			}

			try
			{		
				auto all_rws = get_parameters(control_systems["reaction_wheels"]["parameters"], "control_systems.reaction_wheels.parameters", 1);

				for (int rwb = 0; rwb < count / 4; rwb++)
				{
					std::vector<ReactionWheel> reaction_wheels(4);
					PositionVector location, inertia_diag, limits, z_axis;
					PositionVector angvel({0.0, 0.0, 0.0, 0.0});
					Matrix inertia, ir, sat_inertia;
					std::string apex;
					double angular_momentum = 0.0, limit, mass, alpha, beta, tmp, dump_speed, acc_speed;
					
					try
					{
						auto rws = all_rws[rwb];

						location = get_doubles(rws["location"], "reaction_wheels.parameters.location", 3);
						inertia_diag = get_doubles(rws["inertia"], "reaction_wheels.parameters.inertia", 3);
						angvel = get_doubles(rws["initial_speed"], "reaction_wheels.parameters.initial_speed", 4);

						limit = rws["speed_limit"].asDouble();
						limit *= inertia_diag[2]; // angular momentum limit

						mass = rws["mass"].asDouble();

						auto angles = get_doubles(rws["angles"], "reaction_wheels.parameters.angles", 2);

						alpha = angles[0] * M_PI / 180.0;
						beta = angles[1] * M_PI / 180.0;

						apex = rws["pyramid_apex"].asString();

						if (rws["dump_speed"]) { dump_speed = rws["dump_speed"].asDouble(); }
						if (rws["acc_speed"]) { acc_speed = rws["acc_speed"].asDouble(); }

						z_axis[0] = std::sin(alpha);
						z_axis[1] = std::cos(alpha);
						z_axis[2] = std::cos(alpha);
						for (int i = 0; i < 4; i++)
						{
							if (i == 0)
							{
								z_axis[1] *= -1.0*std::cos(beta);
								z_axis[2] *= -1.0*std::sin(beta);
							}
							if (i == 1)
							{
								tmp = location[1];
								location[1] = -1.0 * location[2];
								location[2] = tmp;

								z_axis[1] *= std::sin(beta);
								z_axis[2] *= -1.0*cos(beta);
							}
							if (i == 2)
							{
								location[1] *= -1.0;
								location[2] *= -1.0;

								z_axis[1] *= std::cos(beta);
								z_axis[2] *= std::sin(beta);
							}
							if (i == 3)
							{
								tmp = location[1];
								location[1] = location[2];
								location[2] = -1.0 * tmp;

								z_axis[1] *= -1.0*std::sin(beta);
								z_axis[2] *= std::cos(beta);
							}

							reaction_wheels[i] = ReactionWheel(location, alpha, beta, z_axis,
								Matrix(3,3,{{inertia_diag[0], 0.0, 0.0},{0.0, inertia_diag[1], 0.0},{0.0, 0.0, inertia_diag[2]}}), angvel[i], limit, mass, apex, dump_speed, acc_speed);
							// converge inertia to satellite axes
							ir[0][0] = z_axis[1] * z_axis[1];
							ir[0][1] = z_axis[0] * z_axis[0];
							ir[0][2] = 0.0;

							ir[1][0] = z_axis[0]*z_axis[0]*z_axis[2]*z_axis[2];
							ir[1][1] = z_axis[1]*z_axis[1]*z_axis[2]*z_axis[2];
							ir[1][2] = (z_axis[0]*z_axis[0] - z_axis[1]*z_axis[1])*(z_axis[0]*z_axis[0] - z_axis[1]*z_axis[1]);

							ir[2][0] = z_axis[0] * z_axis[0];
							ir[2][1] = z_axis[1] * z_axis[1];
							ir[2][2] = z_axis[2] * z_axis[2];

							inertia_diag = mul(ir, inertia_diag);

							double parallel_axis_theorem = mass * location.norm() * location.norm();

							inertia[0][0] = inertia_diag[0] + parallel_axis_theorem;
							inertia[1][1] = inertia_diag[1] + parallel_axis_theorem;
							inertia[2][2] = inertia_diag[2] + parallel_axis_theorem;
							
							sat_inertia = sat_inertia + inertia + location.skew() * location.skew() * (-1.0 * mass);
						}
						sat->setInertiaTensor(sat->getInertiaTensor() + sat_inertia);
						sat->setReactionWheelsBlock(reaction_wheels);
						sat->setMass(sat->getMass() + (double)count * mass);
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#11_reaction_wheels[" + std::to_string(rwb) + "] " << e.what() << "\033[0m\n";
						return 1;
					}
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1461_reaction_wheels.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1460_reaction_wheels Count not given\033[0m\n";
			return 1;
		}
	}

	if (control_systems.isMember("magnetorquers") && !control_systems["magnetorquers"].isNull())
	{
		parameters_dict["Control_systems"]["magnetorquers"] = true;
		int count;
		if (control_systems["magnetorquers"].isMember("count") && !control_systems["magnetorquers"]["count"].isNull())
		{
			try
			{
				count = control_systems["magnetorquers"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1441_magnetorquers.count Invalid value " << control_systems["magnetorquers"]["count"] << " of magnetorquers count\033[0m\n";
				return 1;
			}

			try
			{
				auto magn = get_parameters(control_systems["magnetorquers"]["parameters"], "control_systems.magnetorquers.parameters", count);
				std::vector<AttitudeController> magnetorquers(count);
				double mass = 0.0, current = 0.0, total_mass = 0.0;
				PositionVector max_dipole;
				for (int i = 0; i < count; i++)
				{
					try
					{
						current = magn[i]["current"].asDouble();
						max_dipole = get_doubles(magn[i]["max_dipole"], "magnetorquers["+std::to_string(i)+"].parameters.max_dipole", 3);
						mass = magn[i]["mass"].asDouble();
						
						magnetorquers[i] = AttitudeController(
							PositionVector({0.0, 0.0, 0.0}),
							magn[i]["current"].asDouble(),
							max_dipole,
							mass
							);
						total_mass += mass;
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#11_magnetorquers[" + std::to_string(i) + "] " << e.what() << "\033[0m\n";
						return 1;
					}

				}
				sat->setMagnetorquers(magnetorquers);
				sat->setMass(sat->getMass() + total_mass);
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1441_magnetorquers.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1440_magnetorquers Count not given\033[0m\n";
			return 1;
		}
	}

	if (control_systems.isMember("pulse_engines") && !control_systems["pulse_engines"].isNull())
	{
		parameters_dict["Control_systems"]["pulse_engines"] = true;
		int count;
		if (control_systems["pulse_engines"].isMember("count") && !control_systems["pulse_engines"]["count"].isNull())
		{
			try
			{
				count = control_systems["pulse_engines"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1451_pulse_engines.count Invalid value " << control_systems["pulse_engines"]["count"] << " for count\033[0m\n";
				return 1;
			}

			try
			{
				auto thr = get_parameters(control_systems["pulse_engines"]["parameters"], "control_systems.pulse_engines.parameters", count);
				std::vector<AttitudeController> thrusters(count);
				double mass = 0.0, total_mass = 0.0;
				PositionVector position, limits;
				for (int i = 0; i < count; i++)
				{
					try
					{
						position = get_doubles(thr[i]["position"], "pulse_engines.parameters["+std::to_string(i)+"].position", 3);
						limits = get_doubles(thr[i]["limits"], "pulse_engines.parameters["+std::to_string(i)+"].limits", 3);
						mass = thr[i]["mass"].asDouble();
						thrusters[i] = AttitudeController(
							position,
							0.0,
							limits,
							mass
							);
						total_mass += mass;
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#1451_pulse_engines[" + std::to_string(i) + "] " << e.what() << "\033[0m\n";
						return 1;
					}
				}
				sat->setThrusters(thrusters);
				sat->setMass(sat->getMass() + total_mass);
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1451_pulse_engines.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1450_pulse_engines Count not given\033[0m\n";
			return 1;
		}
	}

	if (control_systems.isMember("correction_engines") && !control_systems["correction_engines"].isNull())
	{
		parameters_dict["Control_systems"]["correction_engines"] = true;
		int count;
		if (control_systems["correction_engines"].isMember("count") && !control_systems["correction_engines"]["count"].isNull())
		{
			try
			{
				count = control_systems["correction_engines"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1421_correction_engines.count Invalid value " << control_systems["correction_engines"]["count"] << " for count\033[0m\n";
				return 1;
			}

			try
			{
				auto thr = get_parameters(control_systems["correction_engines"]["parameters"], "control_systems.correction_engines.parameters", count);
				std::vector<AttitudeController> thrusters(count);
				double mass = 0.0, total_mass = 0.0;
				PositionVector position, limits;

				for (int i = 0; i < count; i++)
				{
					try
					{
						position = get_doubles(thr[i]["position"], "correction_engines["+std::to_string(i)+"].parameters.position", 3);
						limits = get_doubles(thr[i]["limits"], "correction_engines["+std::to_string(i)+"].parameters.limits", 3);
						mass = thr[i]["mass"].asDouble();

						thrusters[i] = AttitudeController(
							position,
							0.0,
							limits,
							mass
							);
						total_mass += mass;
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#1421_correction_engines[" + std::to_string(i) + "] " << e.what() << "\033[0m\n";
						return 1;
					}
				}
				sat->setCorrectionThrusters(thrusters);
				sat->setMass(sat->getMass() + total_mass);
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1421_correction_engines.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1420_correction_engines Coutn not given\033[0m\n";
			return 1;
		}
	}

	if (control_systems.isMember("control_order") && !control_systems["control_order"].isNull())
	{
		char first = '0', second = '0';
		if (control_systems["control_order"].isMember("first") && !control_systems["control_order"]["first"].isNull())
		{
			try
			{
				first = (char)control_systems["control_order"]["first"].asCString()[0];
				if ((first != '0') && (first != 'r') && (first != 'g') && (first != 'm')) throw std::runtime_error("");
			}
			catch (...)
			{
				std::cerr << "\033[31m#1411_control_order.first Invalid value " << control_systems["control_order"]["first"] << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1410_control_order.first Not given first control system\033[0m\n";
			return 1;
		}
		if (control_systems["control_order"].isMember("second") && !control_systems["control_order"]["second"].isNull())
		{
			try
			{
				second = (char)control_systems["control_order"]["second"].asCString()[0];
				if ((second != '0') && (second != 'r') && (second != 'g') && (second != 'm')) throw std::runtime_error("");
			}
			catch (...)
			{
				std::cerr << "\033[31m#1411_control_order.second Invalid value " << control_systems["control_order"]["second"] << "\033[0m\n";
				return 1;
			}
		}
		Control::setControlOrder(first, second);
		parameters_dict["Control_systems"]["control_order"] = true;
	}

	/* ---------------------------------- ORBITAL CONTROL ------------------------------------ */
	auto orbital_control = data["orbital_control"];

	if (orbital_control.isMember("corrections") && !orbital_control["corrections"].isNull())
	{
		parameters_dict["Orbital_control"]["corrections"] = true;
		int count;
		if (orbital_control["corrections"].isMember("count") && !orbital_control["corrections"]["count"].isNull())
		{
			try
			{
				count = orbital_control["corrections"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1511_orbital_control.corrections.count Invalid value " << orbital_control["corrections"]["count"] << " for count'\033[0m\n";
				return 1;
			}

			try
			{
				auto corrections = get_parameters(orbital_control["corrections"]["parameters"], "orbital_control.corrections.parameters", count);
				PositionVector pulse;
				Time start_time, stop_time;
				for (int i = 0; i < count; i++)
				{
					if (corrections[i].isMember("start_time") && !corrections[i]["start_time"].isNull())
					{
						try
						{
							start_time = Time(corrections[i]["start_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1511_orbital_control.corrections["+std::to_string(i)+"].start_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1510_orbital_control.corrections["+std::to_string(i)+"].start_time No start_time\033[0m\n";
						return 1;
					}
					if (corrections[i].isMember("stop_time") && !corrections[i]["stop_time"].isNull())
					{
						try
						{
							stop_time = Time(corrections[i]["stop_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1511_orbital_control.corrections["+std::to_string(i)+"].stop_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1510_orbital_control.corrections["+std::to_string(i)+"].stop_time No stop time\033[0m\n";
						return 1;
					}
					try
					{
						pulse = get_doubles(corrections[i]["pulse"], "orbital_control.corrections["+std::to_string(i)+"].parameters.pulse", 3);
						sat->setCorrection(start_time, stop_time, pulse);
					}
					catch (const std::exception& e)
					{
						std::cerr << "\033[31m#1511_orbital_control.corrections["+std::to_string(i)+"] " << e.what() << "\033[0m\n";
						return 1;
					}

				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1511_orbital_control.corrections.parameters " << e.what() << "\033[0m";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1510_orbital_control.corrections Count not given\033[0m\n";
			return 1;
		}
	}

	/* ---------------------------------------------- ATTITUDE MODES ------------------------- */
	auto attitude_modes = data["attitude_modes"];

	if (attitude_modes.isMember("stop_motion") && !attitude_modes["stop_motion"].isNull())
	{
		parameters_dict["Attitude_modes"]["stop_motion"] = true;
		int count;
		if (attitude_modes["stop_motion"].isMember("count") && !attitude_modes["stop_motion"]["count"].isNull())
		{
			try
			{
				count = attitude_modes["stop_motion"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1641_stop_motion.count Invalid value " << attitude_modes["stop_motion"]["count"] << " for count\033[0m\n";
				return 1;
			}
			
			try
			{
				auto sm = attitude_modes["stop_motion"]["parameters"];
				//boost::math::quaternion<double> quat;
				Time start_time, stop_time;
				for (int i = 0; i < count; i++)
				{
					if (sm[i].isMember("start_time") && !sm[i]["start_time"].isNull())
					{
						try
						{
							start_time = Time(sm[i]["start_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1641_stop_motion["+std::to_string(i)+"].start_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1640_stop_motion["+std::to_string(i)+"] No start_time\033[0m\n";
						return 1;
					}
					if (sm[i].isMember("stop_time") && !sm[i]["stop_time"].isNull())
					{
						try
						{
							stop_time = Time(sm[i]["stop_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1641_stop_motion["+std::to_string(i)+"].stop_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1640_stop_motion["+std::to_string(i)+"] No stop_time\033[0m\n";
						return 1;
					}

					sat->setStop(start_time, stop_time);

				}
				sat->disableCounterrotation();
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1641_stop_motion.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else 
		{
			std::cerr << "\033[31m#1640_stop_motion Count not given\033[0m\n";
			return 1;
		}
	}

	if (attitude_modes.isMember("slew_motion") && !attitude_modes["slew_motion"].isNull())
	{
		parameters_dict["Attitude_modes"]["slew_motion"] = true;
		int count;
		if (attitude_modes["slew_motion"].isMember("count") && !attitude_modes["slew_motion"]["count"].isNull())
		{
			try
			{
				count = attitude_modes["slew_motion"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1631_slew_motion.count Invalid value " << attitude_modes["slew_motion"]["count"] << " for count\033[0m\n";
				return 1;
			}

			try
			{
				auto sm = get_parameters(attitude_modes["slew_motion"]["parameters"], "slew_motion.parameters", count);
				Time start_time, stop_time;
				PositionVector q(4);
				for (int i = 0; i < count; i++)
				{
					if (sm[i].isMember("start_time") && !sm[i]["start_time"].isNull())
					{
						try
						{
							start_time = Time(sm[i]["start_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1631_slew_motion["+std::to_string(i)+"].start_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1630_slew_motion["+std::to_string(i)+"] No start_time\033[0m\n";
						return 1;
					}
					if (sm[i].isMember("stop_time") && !sm[i]["stop_time"].isNull())
					{
						try
						{
							stop_time = Time(sm[i]["stop_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1631_slew_motion["+std::to_string(i)+"].stop_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1630_slew_motion["+std::to_string(i)+"] No stop_time\033[0m\n";
						return 1;
					}
					if (sm[i].isMember("quaternion") && !sm[i]["quaternion"].isNull())
					{
						try
						{
							q = get_doubles(sm[i]["quaternion"], "slew_motion["+std::to_string(i)+"].quaternion", 4);
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1631_slew_motion["+std::to_string(i)+"].quaternion " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1630_slew_motion["+std::to_string(i)+"].quaternion Quaternion not given\033[0m\n";
						return 1;
					}
					sat->setSlew(start_time, stop_time, Quaternion(q[0], q[1], q[2], q[3]));
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1631_slew_motion.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1630_slew_motion.count Count not given\033[0m\n";
			return 1;
		}
	}

	if (attitude_modes.isMember("scan_motion") && !attitude_modes["scan_motion"].isNull())
	{
		parameters_dict["Attitude_modes"]["scan_motion"] = true;
		int count;
		if (attitude_modes["scan_motion"].isMember("count") && !attitude_modes["scan_motion"]["count"].isNull())
		{
			try
			{
				count = attitude_modes["scan_motion"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1621_scan_motion.count Invalid value " << attitude_modes["scan_motion"]["count"] << " for count\033[0m\n";
				return 1;
			}

			try
			{
				auto sm = get_parameters(attitude_modes["scan_motion"]["parameters"], "scan_motion.parameters", count);
				Time start_time, stop_time;
				PositionVector v(3);
				for (int i = 0; i < count; i++)
				{
					if (sm[i].isMember("start_time") && !sm[i]["start_time"].isNull())
					{
						try
						{
							start_time = Time(sm[i]["start_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1621_scan_motion["+std::to_string(i)+"].start_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1620_scan_motion["+std::to_string(i)+"] No start_time\033[0m\n";
						return 1;
					}
					if (sm[i].isMember("stop_time") && !sm[i]["stop_time"].isNull())
					{
						try
						{
							stop_time = Time(sm[i]["stop_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1621_scan_motion["+std::to_string(i)+"].stop_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1620_scan_motion["+std::to_string(i)+"] No stop_time\033[0m\n";
						return 1;
					}
					if (sm[i].isMember("velocity") && !sm[i]["velocity"].isNull())
					{
						try
						{
							v = get_doubles(sm[i]["velocity"], "scan_motion["+std::to_string(i)+"].velocity", 3);
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1621_scan_motion["+std::to_string(i)+"].velocity " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1620_scan_motion["+std::to_string(i)+"].velocity Velocity not given\033[0m\n";
						return 1;
					}
					sat->setScan(start_time, stop_time, v);
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1621_scan_motion.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1620_scan_motion.count Count not given\033[0m\n";
			return 1;
		}
	}

	if (attitude_modes.isMember("dump") && !attitude_modes["dump"].isNull())
	{
		parameters_dict["Attitude_modes"]["dump"] = true;
		int count;
		if (attitude_modes["dump"].isMember("count") && !attitude_modes["dump"]["count"].isNull())
		{
			try
			{
				count = attitude_modes["dump"]["count"].asInt();
			}
			catch (...)
			{
				std::cerr << "\033[31m#1611_dump.count Invalid value " << attitude_modes["dump"]["count"] << " for count\033[0m\n";
				return 1;
			}

			try
			{
				auto sm = get_parameters(attitude_modes["dump"]["parameters"], "dump.parameters", count);
				Time start_time, stop_time;
				for (int i = 0; i < count; i++)
				{
					if (sm[i].isMember("start_time") && !sm[i]["start_time"].isNull())
					{
						try
						{
							start_time = Time(sm[i]["start_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1611_dump["+std::to_string(i)+"].start_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1610_dump["+std::to_string(i)+"] No start_time\033[0m\n";
						return 1;
					}
					if (sm[i].isMember("stop_time") && !sm[i]["stop_time"].isNull())
					{
						try
						{
							stop_time = Time(sm[i]["stop_time"].asString());
						}
						catch (const std::exception& e)
						{
							std::cerr << "\033[31m#1611_dump["+std::to_string(i)+"].stop_time " << e.what() << "\033[0m\n";
							return 1;
						}
					}
					else
					{
						std::cerr << "\033[31m#1610_dump["+std::to_string(i)+"] No stop_time\033[0m\n";
						return 1;
					}
					sat->setDump(start_time, stop_time);
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "\033[31m#1611_dump.parameters " << e.what() << "\033[0m\n";
				return 1;
			}
		}
		else
		{
			std::cerr << "\033[31m#1610_dump.count Count not given\033[0m\n";
			return 1;
		}
	}

	if (sat->checkAttitudeModes())
	{
		return 1;
	}

	sat->mergeAttitudeModes();

	/* ------------------------------ FORCES & TORQUES ------------------------------------------------- */
	auto forces = data["forces"];

	if (forces.isMember("gravity_force") && !forces["gravity_force"].isNull())
	{
		if (forces["gravity_force"].asString() == "true")
		{
			Forces::account_for_earth_gravity = true;
			parameters_dict["Forces_and_torques"]["gravity_force"] = true;
		}
		else if (forces["gravity_force"].asString() == "false")
			Forces::account_for_earth_gravity = false;
		else
		{
			std::cerr << "\033[31m#1711_gravity_force Invalid value " << forces["gravity_force"] << " of flag\033[0m\n";
			return 1;
		}
	}

	if (forces.isMember("gravity_order") && !forces["gravity_order"].isNull())
	{
		int go;
		if (!forces["gravity_order"].isInt())
		{
			std::cerr << "\033[31m#1721_gravity_order invalid value " << forces["gravity_order"] << " of gravity order\033[0m\n";
			return 1;
		}
		go = forces["gravity_order"].asInt();
		if (go > 0)
		{
			Forces::setGravityOrder(go);
			parameters_dict["Forces_and_torques"]["gravity_order"] = true;
		}
		else
			Forces::setGravityOrder(0);
	}

	if (forces.isMember("outer_gravity") && !forces["outer_gravity"].isNull())
	{
		if (forces["outer_gravity"].asString() == "true")
		{
			Forces::account_for_outer_gravity = true;
			parameters_dict["Forces_and_torques"]["outer_gravity"] = true;
		}
		else if (forces["outer_gravity"].asString() == "false")
			Forces::account_for_outer_gravity = false;
		else
		{
			std::cerr << "\033[31#1751_outer_gravity Invalid value " << forces["outer_gravity"] << " of flag\033[0m\n";
			return 1;
		}
	}

	if (forces.isMember("solar_pressure_force") && !forces["solar_pressure_force"].isNull())
	{
		if (forces["solar_pressure_force"].asString() == "true")
		{
			Forces::account_for_solar_pressure = true;
			parameters_dict["Forces_and_torques"]["solar_pressure_force"] = true;
		}
		else if (forces["solar_pressure_force"].asString() == "false")
			Forces::account_for_solar_pressure = false;
		else
		{
			std::cerr << "\033[31#1761_solar_pressure_force Invalid value " << forces["solar_pressure_force"] << " of flag\033[0m\n";
			return 1;
		}
	}

	if (forces.isMember("gravity_torque") && !forces["gravity_torque"].isNull())
	{
		if (forces["gravity_torque"].asString() == "true")
		{
			Torques::account_for_earth_torque = true;
			parameters_dict["Forces_and_torques"]["gravity_torque"] = true;
		}
		else if (forces["gravity_torque"].asString() == "false")
			Torques::account_for_earth_torque = false;
		else
		{
			std::cerr << "\033[31m#1731_gravity_torque Invalid value " << forces["gravity_torque"] << " for flag\033[0m\n";
			return 1;
		}
	}

	if (forces.isMember("solar_pressure_torque") && !forces["solar_pressure_torque"].isNull())
	{
		if (forces["solar_pressure_torque"].asString() == "true")
		{
			Torques::account_for_solar_pressure = true;
			parameters_dict["Forces_and_torques"]["solar_pressure_torque"] = true;
		}
		else if (forces["solar_pressure_torque"].asString() == "false")
			Torques::account_for_solar_pressure = false;
		else
		{
			std::cerr << "\033[31m#1771_solar_pressure_torque Invalid value " << forces["solar_pressure_torque"] << " for flag\033[0m\n";
			return 1;
		}
	}

	if (forces.isMember("magnetic_torque") && !forces["magnetic_torque"].isNull())
	{
		if (forces["magnetic_torque"].asString() == "true")
		{
			Torques::account_for_magnetic_torque = true;
			parameters_dict["Forces_and_torques"]["magnetic_torque"] = true;
		}
		else if (forces["magnetic_torque"].asString() == "false")
			Torques::account_for_magnetic_torque = false;
		else
		{
			std::cerr << "\033[31m#1741_magnetic_torque Invalid value " << forces["magnetic_torque"] << " for flag\033[0m\n";
			return 1;
		}
	}

	/*-------------------------------- FILENAMES ------------------------------------------------------------*/
	auto filenames = data["filenames"];

	auto input_filenames = filenames["input"];
	auto output_filenames = filenames["output"];

	if (output_filenames.isMember("save_path") && !output_filenames["save_path"].isNull())
	{
		FILENAMES::ephemeris_filename = output_filenames["save_path"].asString();
		parameters_dict["Filenames"]["save_path"] = true;
	}

	if (output_filenames.isMember("telemetry_path") && !output_filenames["telemetry_path"].isNull())
	{
		FILENAMES::telemetry_filename = output_filenames["telemetry_path"].asString();
		parameters_dict["Filenames"]["telemetry_path"] = true;
	}

	if (output_filenames.isMember("output_info_path") && !output_filenames["output_info_path"].isNull())
	{
		FILENAMES::output_info_filename = output_filenames["output_info_path"].asString();
		parameters_dict["Filenames"]["output_info_path"] = true;
	}

	if (input_filenames.isMember("egm_path") && !input_filenames["egm_path"].isNull())
	{
		Forces::setEGMfile(input_filenames["egm_path"].asString());
		parameters_dict["Filenames"]["egm_path"] = true;
	}

	if (input_filenames.isMember("eop_path") && !input_filenames["eop_path"].isNull())
	{
		Astrometry::setEOPfile(input_filenames["eop_path"].asString());
		parameters_dict["Filenames"]["eop_path"] = true;
	}

	if (input_filenames.isMember("tls_path") && !input_filenames["tls_path"].isNull())
	{
		Astrometry::setTLSfile(input_filenames["tls_path"].asString());
		parameters_dict["Filenames"]["tls_path"] = true;
	}

	if (input_filenames.isMember("eph_path") && !input_filenames["eph_path"].isNull())
	{
		Astrometry::setEPHEMfile(input_filenames["eph_path"].asString());
		parameters_dict["Filenames"]["eph_path"] = true;
	}

	if (input_filenames.isMember("igrf_path") && !input_filenames["igrf_path"].isNull())
	{
		Torques::setIGRFfile(input_filenames["igrf_path"].asString());
		parameters_dict["Filenames"]["igrf_path"] = true;
	}

	if (input_filenames.isMember("gm_path") && !input_filenames["gm_path"].isNull())
	{
		Astrometry::setGMfile(input_filenames["gm_path"].asString());
		parameters_dict["Filenames"]["gm_path"] = true;
	}

	std::cout << "\033[32mDone reading json file\033[0m" << std::endl;

	if (simulation.isMember("input_statistics") && !simulation["input_statistics"].isNull())
	{
		try
		{
			if (simulation["input_statistics"].asString() == "true")
			{
				input_statistics();
				char ans;
				std::cout << "Do you wish to continue modeling with this set of parameters Y/N? ";
				std::cin >> ans;
				if ((ans != 'Y') && (ans != 'y')) return 1;
			}
		}
		catch(...)
		{

		}
	}
	return 0;
}


void Input::input_statistics()
{
	for (const auto& elem: parameters_dict)
	{
		std::cout << elem.first << ": " << std::endl;
		std::cout << std::left;
		for (const auto& param: elem.second)
		{
			if (param.second)
				std::cout << "     " << "\033[32m" << std::setw(26) << param.first << "OK" << "\033[0m" << std::endl;
			else
				std::cout << "     " << "\033[31m" << std::setw(26) << param.first << "NOT GIVEN" << "\033[0m" << std::endl;
		}
	}
}