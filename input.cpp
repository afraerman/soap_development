#include "stdafx.h"

inline constexpr std::uint32_t fnv1a(const char* str, std::uint32_t hash = 2166136261UL)
{
	return *str ? fnv1a(str + 1, (hash ^ *str) * 16777619ULL) : hash;
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
		std::cout << "No such input file or directory" << std::endl;
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
				std::cout << "Smth went wrong while reading vtk file. Polygons not included" << std::endl;
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
				std::cout << "There must be exeactly 4 reaction wheels in one block" << std::endl;
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
			std::cout << "Unknown parameter: " << name << std::endl;
			return 1;
		}
		
		}

		ss_str.clear();
		ss_value.clear();

	}
	input.close();

	if (state_not_given) { std::cout << "State not given" << std::endl; return 1; }
	if (time_not_given) { std::cout << "Initial time not given" << std::endl; return 1; }
	if (interval_not_given) { std::cout << "Interval not given" << std::endl; return 1; }
	if (step_not_given) { std::cout << "Step not given" << std::endl; return 1; }
	return 0;
}
	

int Input::read_vtk_file(const std::string& filename, std::vector<Polygon>& polygons, const std::string& koeffs_filename)
{

	std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file " << filename << std::endl;
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

    std::cout << "Done reading vtk file" << std::endl;

	return 0;
}

int Input::read_json_file(const std::string& filename, Satellite* sat, Time* time, double& interval, double& step, double& ost)
{
	std::ifstream data_file(filename, std::ifstream::binary);
	if (!data_file.is_open())
	{
		std::cout << "No json file found" << std::endl;
		return 1;
	}
	std::cout << "Json file is opened" << std::endl;
	Json::Value data;
	data_file >> data;

	/* ---------------------------------- SIMULATION ------------------------------------*/
	if (data["simulation"]["interval"])
		interval = data["simulation"]["interval"].asDouble();
	else
	{
		std::cout << "Interval not given" << std::endl;
		return 1;
	}

	if (data["simulation"]["step"])
	{
		step = data["simulation"]["step"].asDouble();
		ost = step;
	}
	else
	{
		std::cout << "Step not given" << std::endl;
		return 1;
	}

	if (data["simulation"]["start_time"])
		time->setTime(Time(data["simulation"]["start_time"].asString()));
	else
	{
		std::cout << "Start time not given" << std::endl;
		return 1;
	}

	if (data["simulation"]["output_step"])
		ost = data["simulation"]["output_step"].asDouble();

	if (data["simulation"]["orbit_file"])
	{
		std::string orbit_filename = data["simulation"]["orbit_file"].asString();
		sat->setOrbitFilename(orbit_filename);
	}

	/* ---------------------------------- SPACECRAFT --------------------------------------*/
	auto spacecraft = data["spacecraft"];

	if (spacecraft["state_vector"])
	{
		auto st = data["spacecraft"]["state_vector"];
		sat->setState(StateVector(std::vector<double>{st[0].asDouble(), st[1].asDouble(), st[2].asDouble(),
			st[3].asDouble(), st[4].asDouble(), st[5].asDouble()}));
	}
	else if (!(data["simulation"]["orbit_file"]))
	{
		std::cout << "State not given" << std::endl;
		return 1;
	}

	if (spacecraft["quaternion"])
	{
		auto q = spacecraft["quaternion"];
		//boost::math::quaternion<double> quat {q[0].asDouble(), q[1].asDouble(), q[2].asDouble(), q[3].asDouble()};
		Quaternion quat(q[0].asDouble(), q[1].asDouble(), q[2].asDouble(), q[3].asDouble());
		sat->setQuaternion(quat);
	}

	if (spacecraft["inertia_tensor"])
	{
		auto it = spacecraft["inertia_tensor"];
		sat->setInertiaTensor(Matrix(3, 3, std::vector<std::vector<double>>{
			{it[0].asDouble(), 0.0, 0.0},
			{0.0, it[1].asDouble(), 0.0},
			{0.0, 0.0, it[2].asDouble()}
		}));
	}

	if (spacecraft["angular_velocity"])
	{
		auto av = spacecraft["angular_velocity"];
		sat->setAngularVelocity(std::vector<double>{av[0].asDouble(), av[1].asDouble(), av[2].asDouble()});
	}

	if (spacecraft["mass"])
	{
		sat->setMass(spacecraft["mass"].asDouble());
	}
	else
	{
		std::cout << "Mass not given" << std::endl;
		return 1;
	}

	/* --------------------------------- GEOMETRY (SURFACE) -------------------------------*/
	auto geometry = data["geometry"];

	if (geometry["vtk_file"])
	{
		std::string koeffs_filename = "";
		std::vector<Polygon> polygons;
		std::string vtk_filename = geometry["vtk_file"].asString();
		if (geometry["vtk_koeffs"])
		{
			koeffs_filename = geometry["vtk_koeffs"].asString();
		}
		if (read_vtk_file(vtk_filename, polygons, koeffs_filename))
		{
			std::cout << "Smth went wrong while reading vtk file. Polygons not included" << std::endl;
		}
		else
		{
			sat->setPolygons(polygons);
		}
	}

	if (geometry["hdf5_file"])
	{
		std::string hdf5_filename = geometry["hdf5_file"].asString();
		sat->setHdfFile(hdf5_filename);
	}

	if (geometry["polygons"])
	{
		int count = geometry["polygons"]["count"].asInt();
		std::vector<Polygon> polygons(count);
		auto polys = geometry["polygons"]["parameters"];
		for (int i = 0; i < count; i++)
		{
			polygons[i] = Polygon(
				PositionVector({polys[i]["position"][0].asDouble(), polys[i]["position"][1].asDouble(), polys[i]["position"][2].asDouble()}),
				PositionVector({polys[i]["normal"][0].asDouble(), polys[i]["normal"][1].asDouble(), polys[i]["normal"][2].asDouble()}),
				polys[i]["area"].asDouble(),
				polys[i]["albedo"].asDouble(),
				polys[i]["specularity"].asDouble()
				);
		}
		sat->setPolygons(polygons);
	}

	if (geometry["solar_panels"])
	{
		int count = geometry["solar_panels"]["count"].asInt();
		std::vector<Polygon> solar_panels(count);
		auto polys = geometry["solar_panels"]["parameters"];
		for (int i = 0; i < count; i++)
		{
			solar_panels[i] = Polygon(
				PositionVector({polys[i]["position"][0].asDouble(), polys[i]["position"][1].asDouble(), polys[i]["position"][2].asDouble()}),
				PositionVector({polys[i]["normal"][0].asDouble(), polys[i]["normal"][1].asDouble(), polys[i]["normal"][2].asDouble()}),
				polys[i]["area"].asDouble(),
				polys[i]["albedo"].asDouble(),
				polys[i]["specularity"].asDouble(),
				polys[i]["rai"].asDouble()
				);
		}
	}

	/* ---------------------------------- CONTROL SYSTEMS ---------------------------------*/
	auto control_systems = data["control_systems"];

	if (control_systems["gyrostats"])
	{
		int count = control_systems["gyrostats"]["count"].asInt();
		if (count % 3 != 0)
		{
			std::cout << "There must be 3*N gyrostats. Check 'Gyrostats vs Reaction Wheels' chapter" << std::endl;
			return 1;
		}

		auto gyrs = control_systems["gyrostats"]["parameters"];
		std::vector<AttitudeController> gyrostats(count);
		PositionVector coords, limits, tmp;
		Matrix inertia, ir;
		double angular_momentum = 0.0, angvel, mass;

		for (int i = 0; i < count; i++)
		{
			for (int k = 0; k < 3; k++)
			{
				coords[k] = gyrs[i]["location"][k].asDouble();
				ir[k][k] = gyrs[i]["inertia"][k].asDouble();
				limits[k] = gyrs[i]["limits"][k].asDouble();
				limits[k] *= ir[k][k];
			}
			angvel = gyrs[i]["angular_velocity"].asDouble();
			mass = gyrs[i]["mass"].asDouble();
			inertia = inertia + ir + coords.skew() * coords.skew() * (-1.0 * mass);
			gyrostats[i] = AttitudeController(coords, angular_momentum, limits, mass);
		}

		sat->setInertiaTensor(sat->getInertiaTensor() + inertia);
		sat->setGyrostats(gyrostats);
	}

	if (control_systems["reaction_wheels"])
	{
		int count = control_systems["reaction_wheels"]["count"].asInt();
		if (count % 4 != 0)
		{
			std::cout << "There must be exactly 4*N reaction wheels. Check 'Gyrostats vs Reaction Wheels' chapter" << std::endl;
			return 1;
		}

		auto all_rws = control_systems["reaction_wheels"]["parameters"];

		for (int rwb = 0; rwb < count / 4; rwb++)
		{
			std::vector<ReactionWheel> reaction_wheels(4);
			PositionVector location, inertia_diag, limits, z_axis;
			PositionVector angvel({0.0, 0.0, 0.0, 0.0});
			Matrix inertia, ir, sat_inertia;
			std::string apex;
			double angular_momentum = 0.0, limit, mass, alpha, beta, tmp, dump_speed, acc_speed;
			
			auto rws = all_rws[rwb];

			for (int i = 0; i < 3; i++)
			{
				location[i] = rws["location"][i].asDouble();
				inertia_diag[i] = rws["inertia"][i].asDouble();
				angvel[i] = rws["initial_speed"][i].asDouble();
			}

			angvel[3] = rws["initial_speed"][3].asDouble();
			limit = rws["speed_limit"].asDouble();
			limit *= inertia_diag[2]; // angular momentum limit

			mass = rws["mass"].asDouble();

			alpha = rws["angles"][0].asDouble() * M_PI / 180.0;
			beta = rws["angles"][1].asDouble() * M_PI / 180.0;

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
	}

	if (control_systems["magnetorquers"])
	{
		int count = control_systems["magnetorquers"]["count"].asInt();
		auto magn = control_systems["magnetorquers"]["parameters"];
		std::vector<AttitudeController> magnetorquers(count);
		double mass = 0.0;
		for (int i = 0; i < count; i++)
		{
			magnetorquers[i] = AttitudeController(
				PositionVector(),
				magn[i]["current"].asDouble(),
				PositionVector({magn[i]["max_dipole"][0].asDouble(), magn[i]["max_dipole"][1].asDouble(), magn[i]["max_dipole"][2].asDouble()}),
				magn[i]["mass"].asDouble()
				);
			mass += magn[i]["mass"].asDouble();
		}
		sat->setMagnetorquers(magnetorquers);
		sat->setMass(sat->getMass() + mass);
	}

	if (control_systems["pulse_engines"])
	{
		int count = control_systems["pulse_engines"]["count"].asInt();
		auto thr = control_systems["pulse_engines"]["parameters"];
		std::vector<AttitudeController> thrusters(count);
		double mass = 0.0;

		for (int i = 0; i < count; i++)
		{
			thrusters[i] = AttitudeController(
				PositionVector({thr[i]["position"][0].asDouble(), thr[i]["position"][1].asDouble(), thr[i]["position"][2].asDouble()}),
				0.0,
				PositionVector({thr[i]["limits"][0].asDouble(), thr[i]["limits"][1].asDouble(), thr[i]["limits"][2].asDouble()}),
				thr[i]["mass"].asDouble()
				);
			mass += thr[i]["mass"].asDouble();
		}
		sat->setThrusters(thrusters);
		sat->setMass(sat->getMass() + mass);
	}

	if (control_systems["correction_engines"])
	{
		int count = control_systems["correction_engines"]["count"].asInt();
		auto thr = control_systems["correction_engines"]["parameters"];
		std::vector<AttitudeController> thrusters(count);
		double mass = 0.0;

		for (int i = 0; i < count; i++)
		{
			thrusters[i] = AttitudeController(
				PositionVector({thr[i]["position"][0].asDouble(), thr[i]["position"][1].asDouble(), thr[i]["position"][2].asDouble()}),
				0.0,
				PositionVector({thr[i]["limits"][0].asDouble(), thr[i]["limits"][1].asDouble(), thr[i]["limits"][2].asDouble()}),
				thr[i]["mass"].asDouble()
				);
			mass += thr[i]["mass"].asDouble();
		}
		sat->setCorrectionThrusters(thrusters);
		sat->setMass(sat->getMass() + mass);
	}

	if (control_systems["control_order"])
	{
		Control::setControlOrder((char)control_systems["control_order"]["first"].asCString()[0], (char)control_systems["control_order"]["second"].asCString()[0]);
	}

	/* ---------------------------------- ORBITAL CONTROL ------------------------------------ */
	auto orbital_control = data["orbital_control"];

	if (orbital_control["corrections"])
	{
		int count = orbital_control["corrections"]["count"].asInt();
		auto corrections = orbital_control["corrections"]["parameters"];
		PositionVector pulse;
		for (int i = 0; i < count; i++)
		{
			pulse = PositionVector({
				corrections[i]["pulse"][0].asDouble(), corrections[i]["pulse"][1].asDouble(), corrections[i]["pulse"][2].asDouble()
			});
			sat->setCorrection(Time(corrections[i]["start_time"].asString()), Time(corrections[i]["stop_time"].asString()), pulse);
		}
	}

	/* ---------------------------------------------- ATTITUDE MODES ------------------------- */
	auto attitude_modes = data["attitude_modes"];

	if (attitude_modes["stop_motion"])
	{
		int count = attitude_modes["stop_motion"]["count"].asInt();
		auto sm = attitude_modes["stop_motion"]["parameters"];
		//boost::math::quaternion<double> quat;
		for (int i = 0; i < count; i++)
		{
			sat->setStop(Time(sm[i]["start_time"].asString()), Time(sm[i]["stop_time"].asString()));
		}
		sat->disableCounterrotation();
	}

	if (attitude_modes["slew_motion"])
	{
		int count = attitude_modes["slew_motion"]["count"].asInt();
		auto sm = attitude_modes["slew_motion"]["parameters"];
		Quaternion quat;
		for (int i = 0; i < count; i++)
		{
			quat = Quaternion(sm[i]["quaternion"][0].asDouble(), sm[i]["quaternion"][1].asDouble(), sm[i]["quaternion"][2].asDouble(), sm[i]["quaternion"][3].asDouble());
			sat->setSlew(Time(sm[i]["start_time"].asString()), Time(sm[i]["stop_time"].asString()), quat);
		}
	}

	if (attitude_modes["scan_motion"])
	{
		int count = attitude_modes["scan_motion"]["count"].asInt();
		auto sc = attitude_modes["scan_motion"]["parameters"];
		for (int i = 0; i < count; i++)
		{
			PositionVector angvel = PositionVector({
				sc[i]["velocity"][0].asDouble(), sc[i]["velocity"][1].asDouble(), sc[i]["velocity"][2].asDouble()
			});
			sat->setScan(Time(sc[i]["start_time"].asString()), Time(sc[i]["stop_time"].asString()), angvel);
		}
	}

	if (attitude_modes["dump"])
	{
		int count = attitude_modes["dump"]["count"].asInt();
		auto dp = attitude_modes["dump"]["parameters"];
		for (int i = 0; i < count; i++)
		{
			sat->setDump(Time(dp[i]["start_time"].asString()), Time(dp[i]["stop_time"].asString()));			
		}
	}

	if (sat->checkAttitudeModes())
	{
		return 1;
	}

	sat->mergeAttitudeModes();

	/* ------------------------------ FORCES & TORQUES ------------------------------------------------- */
	auto forces = data["forces"];

	if (forces["gravity_force"])
	{
		if (forces["gravity_force"].asString() == "true")
			Forces::account_for_earth_gravity = true;
		else
			Forces::account_for_earth_gravity = false;
	}

	if (forces["gravity_order"])
	{
		if (forces["gravity_order"].asInt() != 0)
			Forces::setGravityOrder(forces["gravity_order"].asInt());
		else
			Forces::setGravityOrder(0);
	}

	if (forces["outer_gravity"])
	{
		if (forces["outer_gravity"].asString() == "true")
			Forces::account_for_outer_gravity = true;
		else
			Forces::account_for_outer_gravity = false;
	}

	if (forces["solar_pressure_force"])
	{
		if (forces["solar_pressure_force"].asString() == "true")
			Forces::account_for_solar_pressure = true;
		else
			Forces::account_for_solar_pressure = false;
	}

	if (forces["gravity_torque"])
	{
		if (forces["gravity_torque"].asString() == "true")
			Torques::account_for_earth_torque = true;
		else
			Torques::account_for_earth_torque = false;
	}

	if (forces["solar_pressure_torque"])
	{
		if (forces["solar_pressure_torque"].asString() == "true")
			Torques::account_for_solar_pressure = true;
		else
			Torques::account_for_solar_pressure = false;
	}

	if (forces["magnetic_torque"])
	{
		if (forces["magnetic_torque"].asString() == "true")
			Torques::account_for_magnetic_torque = true;
		else
			Torques::account_for_magnetic_torque = false;
	}

	/*-------------------------------- FILENAMES ------------------------------------------------------------*/
	auto filenames = data["filenames"];

	auto input_filenames = filenames["input"];
	auto output_filenames = filenames["output"];

	if (output_filenames["save_path"])
	{
		FILENAMES::ephemeris_filename = output_filenames["save_path"].asString();
	}

	if (output_filenames["telemetry_path"])
	{
		FILENAMES::telemetry_filename = output_filenames["telemetry_path"].asString();
	}

	if (output_filenames["output_info_path"])
	{
		FILENAMES::output_info_filename = output_filenames["output_info_path"].asString();
	}

	if (input_filenames["egm_path"])
	{
		Forces::setEGMfile(input_filenames["egm_path"].asString());
	}

	if (input_filenames["eop_path"])
	{
		Astrometry::setEOPfile(input_filenames["eop_path"].asString());
	}

	if (input_filenames["tls_path"])
	{
		Astrometry::setTLSfile(input_filenames["tls_path"].asString());
	}

	if (input_filenames["eph_path"])
	{
		Astrometry::setEPHEMfile(input_filenames["eph_path"].asString());
	}

	if (input_filenames["igrf_path"])
	{
		Torques::setIGRFfile(input_filenames["igrf_path"].asString());
	}

	if (input_filenames["gm_path"])
	{
		Astrometry::setGMfile(input_filenames["gm_path"].asString());
	}

	std::cout << "Done reading json file" << std::endl;

	return 0;
}
