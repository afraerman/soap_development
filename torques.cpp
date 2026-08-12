#include "stdafx.h"

PositionVector Torques::torques;
PositionVector Torques::center_of_pressure;
PositionVector Torques::uncompensated_torque({0.0, 0.0, 0.0});

std::string Torques::igrffilename = "../Files/2015_2020_2025_igrf.dat";
std::string Torques::thrustersfilename = "/media/alexey/Disk/Diploma/gracefo/thrusters_torques_2019_01_01.txt";
std::string Torques::magnfilename = "/media/alexey/Disk/Diploma/gracefo/magnets_2019_01_01.txt";
int Torques::magnetic_order = 13;
Matrix Torques::order = Matrix();
Matrix Torques::Ps_magnetic = Matrix();
Matrix Torques::ml_magnetic = Matrix();
std::vector<double> Torques::Gnm;
std::vector<double> Torques::Hnm;
std::vector<Time> Torques::thrusters_activation_times;
std::vector<PositionVector> Torques::thrusters_activation_torques;
std::vector<Time> Torques::magn_activation_times;
std::vector<PositionVector> Torques::magn_activation_torques;
std::vector<PositionVector> Torques::magnetic_field_data;
bool Torques::no_magnetic_coefficients = true;
bool Torques::no_magnetic_field_warning = false;

bool Torques::account_for_earth_torque = true;
bool Torques::account_for_solar_pressure = false;
bool Torques::account_for_magnetic_torque = false;

// ---------------------------------- PRIVATE -------------------------- //

void Torques::get_magnetic_coefficients(const Time& time)
{
	if ((time.getYear() < 2015) || (time.getYear() > 2025))
	{
		if (no_magnetic_field_warning) {return;}
		no_magnetic_field_warning = true;
		std::cout << "\033[33mNo model of magnetic field for this year (< 2015 or > 2025)\033[0m" << std::endl;
		return;
		Gnm = std::vector<double>(((magnetic_order + 2) * (magnetic_order + 1) / 2), 0.0);
		Hnm = std::vector<double>(((magnetic_order + 2) * (magnetic_order + 1) / 2), 0.0);
	}
	//std::cout << "I'm about to collect magnetic coefficients" << std::endl;
	no_magnetic_field_warning = false;

	if (igrffilename.length() == 0)
	{
		std::cerr << "\033[31m#051 No IGRF file given in the input file\033[0m" << std::endl;
		Gnm = std::vector<double>(((magnetic_order + 2) * (magnetic_order + 1) / 2), 0.0);
		Hnm = std::vector<double>(((magnetic_order + 2) * (magnetic_order + 1) / 2), 0.0);
		return;
	}

	order = Matrix(magnetic_order + 1, magnetic_order + 1);

	int i = 0;
	for (int n = 0; n < magnetic_order + 1; n++)
	{
		for (int m = 0; m <= n; m++)
		{
			order[n][m] = i;
			i++;
		}
	}

	Gnm = std::vector<double>((magnetic_order + 2) * (magnetic_order + 1) / 2);
	Hnm = std::vector<double>((magnetic_order + 2) * (magnetic_order + 1) / 2);

	Gnm[0] = 0.0; Hnm[0] = 0.0; // n = 0

	int i_g, i_h, nn, mm;
	double c_2015, c_2020, sv, c_base, c_dot, t_base, t;
	char coeff;

	i_g = 1; i_h = 2;
	std::ifstream igrf(igrffilename);
	if (igrf.is_open())
	{
		while (igrf >> coeff >> nn >> mm >> c_2015 >> c_2020 >> sv)
		{
			if (time.getYear() < 2020)
			{
				c_base = c_2015;
				t_base = 2015;
				c_dot = (c_2020 - c_2015) / 5.0;
			}
			else
			{
				c_base = c_2020;
				t_base = 2020;
				c_dot = sv;
			}

			t = (double)(time.getYear() - t_base) + double(time.getMonth() - 1) / 12.0 + (double)(time.getDay() - 1) / 365.0;

			if (coeff == 'g')
			{
				Gnm[i_g] = c_base + c_dot * t;
				i_g++;
				if (mm == 0) // there are no "h n 0" row in the file 
				{
					Hnm[i_h] = 0.0;
					i_h++;
				}
			}
			if (coeff == 'h')
			{
				Hnm[i_h] = c_base + c_dot * t;
				i_h++;
			}
			if ((i_g == magnetic_order) && (i_h == magnetic_order)) break;
		}
		igrf.close();
	}
	else {
		std::cerr << "\033[31m#052 Incorrect IGRF filename: " << igrffilename << "\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return;
	}
	return;
}

void Torques::get_thrusters_activation_times()
{
	if (thrustersfilename.length() == 0)
	{
		std::cerr << "\033[31m#061 Thrusters data file not given in the input file\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return;
	}

	std::ifstream thr(thrustersfilename);
	int h, m;
	double s, torx, tory, torz;
	char tmp;
	if (thr.is_open())
	{
		while (thr >> h >> tmp >> m >> tmp >> s >> torx >> tory >> torz)
		{
			thrusters_activation_times.push_back(Time(2019, 1, 1, h, m, s));
			thrusters_activation_torques.push_back(PositionVector({torx, tory, torz}));
		}
		thr.close();
	}
	else
	{
		std::cerr << "\033[31m#062 Incorrect thrusters data filename: " << thrustersfilename << "\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return;
	}
}

void Torques::get_magnets_activation_times()
{
	if (magnfilename.length() == 0)
	{
		std::cerr << "\033[31m#071 Magnets data file not given\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return;
	}

	std::ifstream mag(magnfilename);
	int h, m;
	double s, magx, magy, magz;
	char tmp;
	if (mag.is_open())
	{
		while (mag >> h >> tmp >> m >> tmp >> s >> magx >> magy >> magz)
		{
			magn_activation_times.push_back(Time(2019, 1, 1, h, m, s));
			magn_activation_torques.push_back(PositionVector({magx, magy, magz}));
		}
		mag.close();
	}
	else
	{
		std::cerr << "\033[31m#072 Incorrect magnetic toruqers data filename: " << magnfilename << "\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return;
	}
	

	std::ifstream mf("/media/alexey/Disk/Diploma/gracefo/magnetic_field_2019_01_01.txt");
	if (mf.is_open())
	{
		while (mf >> h >> tmp >> m >> tmp >> s >> magx >> magy >> magz)
		{
			magnetic_field_data.push_back(PositionVector({magx, magy, magz}) * 1000.0);
		}
		mf.close();
	}
	else
	{
		std::cerr << "\033[31mIncorrect magnetic field data filename\033[0m" << std::endl;
	}

}

double Torques::a_magnetic(int n, int m)
{
	if (n < m) return 0.0;
	return (double)(2.0 * n - 1) / std::sqrt((double)((n-m)*(n+m)));
}

double Torques::b_magnetic(int n, int m)
{
	if (n < m) return 0.0;
	return std::sqrt((double)((n + m - 1) * (n - m - 1)) / (double)((n - m) * (n + m)));
}

double Torques::f_magnetic(int n, int m)
{
	if (n < m) return 0.0;
	return std::sqrt((double)(n * n - m * m));
}

void Torques::allPs_magnetic(double cost, double sint, int pnm_order)
{
	Ps_magnetic = Matrix(pnm_order + 1, pnm_order + 1);
	Ps_magnetic[0][0] = 1.0;

	for (int n = 1; n < pnm_order + 1; n++)
	{
		for (int m = 0; m <= n; m++)
		{
			if (n == m)
			{
				if (n == 1)
				{
					Ps_magnetic[n][m] = sint;
				}
				else
				{
					Ps_magnetic[n][m] = sint * std::sqrt((double)(2*n - 1) / (double)(2*n)) * Ps_magnetic[n-1][m-1];
				}
			}
			else
			{
				if (n > 1) Ps_magnetic[n][m] = a_magnetic(n, m) * cost * Ps_magnetic[n - 1][m] - b_magnetic(n, m) * Ps_magnetic[n - 2][m];
				else Ps_magnetic[n][m] = a_magnetic(n, m) * cost * Ps_magnetic[n - 1][m];
			}
		}
	}
}

void Torques::allml_magnetic(double sinl, double cosl, int pnm_order)
{
	ml_magnetic = Matrix(2, pnm_order + 1);

	ml_magnetic[0][0] = 0.0; // sin(0)
	ml_magnetic[1][0] = 1.0; // cos(0)
	ml_magnetic[0][1] = sinl; // sin(lambda)
	ml_magnetic[1][1] = cosl; // cos(lambda)

	for (int i = 0; i < 2; i++)
	{
		for (int j = 2; j < pnm_order + 1; j++) ml_magnetic[i][j] = 2.0 * cosl * ml_magnetic[i][j - 1] - ml_magnetic[i][j - 2];
	}
}

PositionVector Torques::magnetic_field(const PositionVector& pos, const Time& time)
{
	if (no_magnetic_coefficients)
	{
		get_magnetic_coefficients(time);
		if (no_magnetic_field_warning)
		{
			return PositionVector({0.0, 0.0, 0.0});
		}
		no_magnetic_coefficients = false;
	}

	PositionVector gcrf_position = pos;
	PositionVector itrf_position;
	Matrix rc2t = Astrometry::getRc2t();

	// to ITRF
	itrf_position = mul(rc2t, gcrf_position);

	
	double r = itrf_position.norm();
	double cost = itrf_position[2] / r;
	double sint = sqrt(1.0 - cost * cost);
	double sinl = itrf_position[1] / r / sint;
	double cosl = itrf_position[0] / r / sint;

	allPs_magnetic(cost, sint, magnetic_order);
	allml_magnetic(sinl, cosl, magnetic_order);
	
	//double R = EARTH::RADIUS / 1000.0 / r;
	//double mu = EARTH::RADIUS / 1000.0;

	double R = 6371.2 / r;
	double mu = 6371.2;
	
	double gnm, hnm;
	double Pt;

	double vr = 0.0, vt = 0.0, vl = 0.0;

	for (int n = 1; n < magnetic_order+1; n++)
	{
		for (int m = 0; m <= n; m++)
		{
			if (n == m) Pt = (double)n * cost / sint * Ps_magnetic[n][m];
			else Pt = (double)n * cost / sint * Ps_magnetic[n][m] - f_magnetic(n,m) / sint * Ps_magnetic[n-1][m];
			
			int ind = (int)order[n][m];

			gnm = Gnm[ind];
			hnm = Hnm[ind];

			vr += (double)(n+1) * std::pow(R, n+1) * (gnm * ml_magnetic[1][m] + hnm * ml_magnetic[0][m]) * Ps_magnetic[n][m];
			vt += std::pow(R, n+1) * (gnm * ml_magnetic[1][m] + hnm * ml_magnetic[0][m]) * Pt;
			vl -= (double)(m) * std::pow(R, n+1) * (gnm * ml_magnetic[0][m] - hnm * ml_magnetic[1][m]) * Ps_magnetic[n][m];
		}
	}

	vr *= -1.0 * mu / r;
	vt *= mu;
	vl *= mu;

	
	PositionVector B = PositionVector();
	B[0] = (sint * cosl * vr + cost * cosl / r * vt - sinl * vl / r / sint);
	B[1] = (sint * sinl * vr + cost * sinl / r * vt + cosl * vl / r / sint);
	B[2] = (cost * vr - sint * vt / r);
	
	// back to GCRF
	Matrix rt2c = Astrometry::getRt2c();
	B = mul(rt2c, B);
	return B;
}

void Torques::earth_torque(const Satellite& sat, const Time& time)
{
	PositionVector r;
	double R = sat.getPosition().norm() * 1000.0;
	//double w = sat.getQuaternion().R_component_1();
	double w = sat.getQuaternion().get_scalar();
	//boost::math::quaternion<double> q = sat.getQuaternion();
	Quaternion q = sat.getQuaternion();
	//Matrix gtl = Matrix(q).transpose();

	Matrix I = sat.getInertiaTensor();

	if (sqrt(1.0 - w * w) < 1e-7)
	{
		r = sat.getPosition() * (1000.0 / R);
	}
	else
	{
		//r = mul(gtl, sat.getPosition());
		r = q.get_inverse() * sat.getPosition();
		r = r * (1000.0 / R);
	}

	//std::cout << "Earth torque: " << 3.0 * EARTH::GM / pow(R, 3) * (I[1][1] - I[2][2]) * r[1] * r[2] << '\t' << 3.0 * EARTH::GM / pow(R, 3) * (I[2][2] - I[0][0]) * r[2] * r[0] << '\t' <<
	//3.0 * EARTH::GM / pow(R, 3) * (I[0][0] - I[1][1]) * r[0] * r[1] << std::endl;

	torques[0] += 3.0 * EARTH::GM / pow(R, 3) * (I[1][1] - I[2][2]) * r[1] * r[2];
	torques[1] += 3.0 * EARTH::GM / pow(R, 3) * (I[2][2] - I[0][0]) * r[2] * r[0];
	torques[2] += 3.0 * EARTH::GM / pow(R, 3) * (I[0][0] - I[1][1]) * r[0] * r[1];

	// Moon torque
	/*
	SpiceDouble et = time.ET();
	double state[6];
	double lt;
	SpiceInt n;
	SpiceDouble moon_gm;
	spkezr_c("Moon", et, "J2000", "NONE", "earth", state, &lt);
	bodvrd_c("Moon", "GM", 1, &n, &moon_gm );
	
	PositionVector r_moon = sat.getPosition() - PositionVector({state[0], state[1], state[2]});
	double R_moon = r_moon.norm();
	r_moon = mul(gtl, r_moon) / R_moon;

	torques[0] += 3.0 * moon_gm / pow(R_moon, 3) * (I[1][1] - I[2][2]) * r_moon[1] * r_moon[2];
	torques[1] += 3.0 * moon_gm / pow(R_moon, 3) * (I[2][2] - I[0][0]) * r_moon[2] * r_moon[0];
	torques[2] += 3.0 * moon_gm / pow(R_moon, 3) * (I[0][0] - I[1][1]) * r_moon[0] * r_moon[1];
	*/
}

void Torques::solar_torque(Satellite& sat, const Time& time)
{
	if (sat.getPolygons().size() == 0)
	{
		std::cerr << "\033[33mNo polygons given, skipping solar pressure torque\033[0m" << std::endl;
		return;
	}
	double state[6];
	double lt, ef, cos_theta, alpha, mu, area;
	double rel_r = 0.0;
	PositionVector s, force, additional_torque;
	PositionVector total_force({0.0, 0.0, 0.0});

	center_of_pressure = PositionVector({0.0, 0.0, 0.0});
	double forces_summ = 0.0;

	SpiceDouble et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sat_pos = sat.getPosition();
	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	sat.setSunPosition(sun_pos);
	
	s = sat_pos - sun_pos;
	double r = s.norm();
	s = s * (1.0 / r);

	ef = Astrometry::eclipse_factor(sun_pos, sat_pos);
	
	//boost::math::quaternion<double> quat = sat.getQuaternion();
	Quaternion quat = sat.getQuaternion().get_inverse();
	//Matrix gtl = Matrix(quat).transpose();
	//s = mul(gtl, s);
	s = quat * s;

	sat.rotateSolarPanels(s);
	std::vector<Polygon> polygons = sat.getPolygons();
	std::vector<Polygon> solar_panels = sat.getSolarPanels();
	for (int i=0; i < solar_panels.size(); i++)
	{
		polygons.push_back(solar_panels[i]);
	}
	for (int i = 0; i < polygons.size(); i++)
	{
		s = sat_pos - sun_pos;
		//s = mul(gtl, s);
		s = quat * s;
		s = s + polygons[i].getPosition() / 1000.0;
		r = s.norm();
		s = s * (1.0 / r);
		// std::cout <<  s << std::endl;
		cos_theta = s.dot(polygons[i].getNormal());
		if (cos_theta >= 0) continue; // disgard invisible panels
		cos_theta *= -1.0;

		alpha = polygons[i].getReflectivityFactor();
		mu = polygons[i].getSpecularityFactor();
		area = polygons[i].getArea();
		
		/*
		force = s * ((1.0 - alpha)*(1.0 - alpha) * cos_theta) + polygons[i].getNormal() * (-2.0 * alpha * mu * alpha * mu * cos_theta * cos_theta)
			+ (s - polygons[i].getNormal() * (2.0 / 3.0)) * ((1.0 - mu)*(1.0 - mu) * alpha*alpha * cos_theta);
		
		
		force = force * area;
		*/
		
		force = s * ((1.0 - alpha) * cos_theta) + polygons[i].getNormal() * (-2.0 * alpha * mu * cos_theta * cos_theta)
			+ (s - 2.0 / 3.0 * polygons[i].getNormal()) * (1.0 - mu) * alpha * cos_theta;
		force = area * force;
		total_force += force;

		center_of_pressure += polygons[i].getPosition() * force.norm();
		forces_summ += force.norm();

		additional_torque = polygons[i].getPosition().cross(force) * (-1.0 * ef * SUN::FLUX / WORLD::SPEED_OF_LIGHT * pow(WORLD::AU / r, 2));
		//std::cout << "i = " << i << " solar torque " << additional_torque << std::endl;
		torques = torques + additional_torque;
	}
	center_of_pressure = center_of_pressure / forces_summ;
	// std::cout << "At time " << time << " center of pressure: " << center_of_pressure << std::endl;

	Forces::setSolarPressureCalculated(true);
	Forces::setSolarPressureForce(total_force * ef * SUN::FLUX / WORLD::SPEED_OF_LIGHT * pow(WORLD::AU / r, 2));
}

void Torques::srpTorque(Satellite& sat, const Time& time)
{
	SRPEngine engine(sat.getHdfFile());

	double state[6];
	double lt, ef;
	PositionVector s, additional_torque;
	SRPResult res;

	center_of_pressure = PositionVector({0.0, 0.0, 0.0});
	double forces_summ = 0.0;

	SpiceDouble et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sat_pos = sat.getPosition();
	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	sat.setSunPosition(sun_pos);
	
	s = sat_pos - sun_pos;
	double r = s.norm();
	s = s * (1.0 / r);

	ef = Astrometry::eclipse_factor(sun_pos, sat_pos);
	
	Quaternion quat = sat.getQuaternion().get_inverse();
	s = quat * s;

	engine.setSunDirection(s[0], s[1], s[2]);
	engine.setMaxReflections(2);

	if (is_rtx_device_available())
	{
    	res = engine.compute(SRPMethod::CentroidRTX);
	}
	else if (is_cuda_device_available())
	{
    	res = engine.compute(SRPMethod::CentroidGPU);
	}
	else
	{
    	res = engine.compute(SRPMethod::CentroidCPU);
	}

	additional_torque = PositionVector({res.total_moment[0], res.total_moment[1], res.total_moment[2]});

	torques += additional_torque * ef;

	Forces::setSolarPressureCalculated(true);
	Forces::setSolarPressureForce(PositionVector({res.total_force[0], res.total_force[1], res.total_force[2]}) * ef);
}

void Torques::magnetic_torque(const Satellite& sat)
{
	double R = sat.getPosition().norm();

	PositionVector er = sat.getPosition() * (1.0 / R);

	R *= 1000.0; // km -> m
	PositionVector k_E(std::vector<double>{ 0.0, 0.0, 1.0 });

	// Magnetic field vector in GCRF
	PositionVector magnetic_field;
	magnetic_field = (k_E - er * 3.0 * k_E.dot(er)) * (WORLD::mu_0 * EARTH::mu_E / pow(R, 3));

	// GCRF -> local
	//Matrix gtl = Matrix(sat.getQuaternion()).transpose();
	//magnetic_field = mul(gtl, magnetic_field);
	magnetic_field = sat.getQuaternion().get_inverse() * magnetic_field;

	torques = torques + magnetic_field.cross(sat.getMagneticMomentum());
}

// ---------------------------------- PUBLIC ---------------------- //

void Torques::setIGRFfile(const std::string& filename)
{
	igrffilename = filename;
}

PositionVector Torques::allTorques(Satellite& sat, const Time& time)
{
	Forces::setSolarPressureCalculated(false);
	
	torques[0] = 0.0; torques[1] = 0.0; torques[2] = 0.0;
	//std::cout << "===============================" << std::endl;
	if (account_for_earth_torque) earth_torque(sat, time);
	//std::cout << torques << std::endl;
	if (account_for_solar_pressure)
	{
		if (sat.getHdfFile() != "") srpTorque(sat, time);
		else solar_torque(sat, time);
	}
	//std::cout << torques << std::endl;
	if (account_for_magnetic_torque) magnetic_torque(sat);
	//std::cout << "===============================" << std::endl;

	// это если есть файл
	PositionVector pulse_torque = sat.getQuaternion().get_inverse() * (sat.getPulseAcceleration(time).cross(sat.getPulseEngineLocation()) * sat.getMass());

	torques = torques + pulse_torque;

	return torques;
}

double Torques::testMagnetic(const Time& time)
{
	// check magnetic coefficients
	/*
	get_magnetic_coefficients(time);
	std::cout << " " << '\t';
	for (int i=0; i<14; i++)
	{
		std::cout << i << '\t';
	}
	std::cout << '\n';
	for (int n = 0; n < 14; n++)
	{
		std::cout << n << '\t';
		for (int m=0; m<=n; m++)
		{
			int idx = order[n][m];
			std::cout << Gnm[idx] << '\t';
		}
		std::cout << '\n';
	}
	return 0.0;
	*/

	// check Pnms for theta = 90+10.489, lambda = -46.080
	/*
	double theta = (90+10.489) * M_PI / 180.0;
	double cost = std::cos(theta);
	double sint = std::sin(theta);

	allPs_magnetic(cost, sint, magnetic_order);

	std::cout << " " << '\t';
	for (int i=0; i<14; i++)
	{
		std::cout << i << '\t';
	}
	std::cout << '\n';
	for (int n = 0; n < 14; n++)
	{
		std::cout << n << '\t';
		for (int m=0; m<=n; m++)
		{
			std::cout << Ps_magnetic[n][m] << '\t';
		}
		std::cout << '\n';
	}
	*/

	// check mls for theta = 115.55, lambda = 289.31
	/*
	double lambda = 289.31 * M_PI / 180.0;
	double cosl = std::cos(lambda);
	double sinl = std::sin(lambda);

	allml_magnetic(sinl, cosl, magnetic_order);

	for (int i=0; i<14; i++)
	{
		std::cout << "sin(" << i << "l) = " << ml_magnetic[0][i] << std::endl;
		std::cout << "cos(" << i << "l) = " << ml_magnetic[1][i] << std::endl;
	}
	return 0.0;
	*/

	// check Pt for theta = 115.55, lambda = 289.31
	
	double theta = (90.0+10.489) * M_PI / 180.0;
	double cost = std::cos(theta);
	double sint = std::sin(theta);

	allPs_magnetic(cost, sint, magnetic_order);
	
	double Pt;
	std::cout << '\t';
	for (int i=0; i<14; i++)
	{
		std::cout << i << '\t';
	}
	std::cout << '\n';
	for (int n = 1; n < magnetic_order+1; n++)
	{
		std::cout << n << '\t';
		for (int m = 0; m <= n; m++)
		{
			Pt = (double)n * cost / sint * Ps_magnetic[n][m] - f_magnetic(n,m) / sint * Ps_magnetic[n-1][m];
			std::cout << Pt << '\t';
		}
		std::cout << '\n';
	}
	/**/
	
	// magnetic map
	/*
	Astrometry::EOP(time);
	Astrometry::rotationMatrices(time);
	double cost, sint, cosl, sinl, r;
	PositionVector map;
	r = EARTH::RADIUS / 1000.0;
	PositionVector position;
	std::ofstream output("../../output/magnetic_map.txt");
	for (double theta = 0.05; theta < M_PI; theta += 5.0 * M_PI / 180.0)
	{
		for (double lambda = 0.05; lambda < 2.0 * M_PI; lambda += 5.0 * M_PI / 180.0)
		{
			cost = std::cos(theta);
			sint = std::sin(theta);
			cosl = std::cos(lambda);
			sinl = std::sin(lambda);
			position = PositionVector(std::vector<double>{r*sint*cosl, r*sint*sinl, r*cost});
			position = mul(Astrometry::getRt2c(), position);
			map = mul(Astrometry::getRc2t(), magnetic_field(position, time));
			output << theta << '\t' << lambda << '\t' << map[0] << '\t' << map[1] << '\t' << map[2] << std::endl;
		}
	}
	output.close();
	*/

	// B for certain point
	/*
	Astrometry::EOP(time);
	Astrometry::rotationMatrices(time);
	double theta, lambda, sint, cost, sinl, cosl, r;

	theta = (90+10.489) * M_PI / 180.0;
	lambda = (360.0-46.080) * M_PI / 180.0;

	cost = std::cos(theta);
	sint = std::sin(theta);
	cosl = std::cos(lambda);
	sinl = std::sin(lambda);
	r = EARTH::RADIUS / 1000.0 + 500.0;
	PositionVector position({r*sint*cosl, r*sint*sinl, r*cost});
	position = mul(Astrometry::getRt2c(), position);
	PositionVector magn = mul(Astrometry::getRc2t(), magnetic_field(position, time));
	std::cout << magn.norm() << std::endl;
	*/

	return 0.0;	
}

PositionVector Torques::getMagneticField(const PositionVector& pos, const Time& time)
{
	return magnetic_field(pos, time);
}

PositionVector Torques::getUncompensatedTorque()
{
	return uncompensated_torque;
}

void Torques::setUncompensatedTorque(const PositionVector& t)
{
	uncompensated_torque = t;
}

PositionVector Torques::getCenterOfPressure()
{
	return center_of_pressure;
}