#include<stdafx.h>

std::string Forces::egmfilename = FILENAMES::files_directory + "/EGM2008.dat";
int Forces::gravity_order = 30;
Matrix Forces::order;
Matrix Forces::Ps = Matrix();
Matrix Forces::ml = Matrix();
std::vector<double> Forces::Cnm;
std::vector<double> Forces::Snm;
PositionVector Forces::forces(std::vector<double>{0.0, 0.0, 0.0});
bool Forces::no_coefficients = true;
std::vector<const SpiceChar*> Forces::bodies{ "10", "301", "1", "2", "4", "5", "6", "7", "8", "9"};

PositionVector Forces::solar_pressure_force;
bool Forces::solar_pressure_calculated = false;

bool Forces::account_for_earth_gravity = true;
bool Forces::account_for_outer_gravity = false;
bool Forces::account_for_solar_pressure = false;
bool Forces::account_for_solar_pressure_gmat = false;
bool Forces::account_for_relativity = false;

void Forces::setGravityOrder(const int go)
{
	gravity_order = go;
}

void Forces::setEGMfile(const std::string& filename)
{
	egmfilename = filename;
}

void Forces::setSolarPressureForce(const PositionVector& force)
{
	solar_pressure_force = force;
}

void Forces::setSolarPressureCalculated(const bool flag)
{
	solar_pressure_calculated = flag;
}

int Forces::getGravityCoefficients()
{
	std::cout << "\033[32mI'm about to collect gravity coefficients\033[0m" << std::endl;

	if (egmfilename.length() == 0)
	{
		std::cerr << "\033[31m#042 No egmfilename given in the input file\033[0m" << std::endl;
		return 1;
	}

	order = Matrix(gravity_order + 1, gravity_order + 1);

	int i = 0;
	for (int n = 0; n < gravity_order + 1; n++)
	{
		for (int m = 0; m <= n; m++)
		{
			order[n][m] = i; // ��������� ������ ��������
			i++;
		}
	}

	Cnm = std::vector<double>((gravity_order + 2)*(gravity_order + 1) / 2);
	Snm = std::vector<double>((gravity_order + 2)*(gravity_order + 1) / 2);

	Cnm[0] = 1.0; Cnm[1] = 0.0; Cnm[2] = 0.0; // n = 0; n = 1;
	Snm[0] = 0.0; Snm[1] = 0.0; Snm[2] = 0.0; // n = 0; n = 1;

	i = 3;
	int nn, mm;
	double cc, ss, errc, errs;
	std::ifstream egm(egmfilename);
	if (!(egm.is_open()))
	{
		std::cerr << "\033[31m#041 No such EGM file " << egmfilename << "\033[0m" << std::endl;
		return 1;
	}
	
	while (egm >> nn >> mm >> cc >> ss >> errc >> errs)
	{
		Cnm[i] = cc;
		Snm[i] = ss;
		i++;
		if ((nn == gravity_order) && (mm == gravity_order)) break;
	}
	egm.close();
	return 0;
}

// coefficient for recurrent Pnm
double Forces::a(int n, int m)
{
	if (n < m) return 0.0;
	return sqrt((double)(4 * n * n - 1) / (double)(n * n - m * m));
}

// coefficient for recurrent Pnm
double Forces::b(int n, int m)
{
	if (n < m) return 0.0;
	return sqrt((double)((2 * n + 1)*(n + m - 1)*(n - m - 1)) / (double)((n - m)*(n + m)*(2 * n - 3)));
}

// coefficient for recurrent P(1)nm
double Forces::f(int n, int m)
{
	if (n < m) return 0.0;
	return sqrt((double)((n - m)*(n + m)) * ((double)(2 * n + 1) / (double)(2 * n - 1)));
}

// create all Pnm for given cost = cos(theta)
void Forces::allPs(double cost, int pnm_order)
{
	Ps = Matrix(pnm_order + 1, pnm_order + 1);
	Ps[0][0] = 1.0;

	for (int n = 1; n < pnm_order + 1; n++)
	{
		for (int m = 0; m <= n; m++)
		{
			if (n == m) Ps[n][m] = 1.0;
			else
			{
				if (n > 1) Ps[n][m] = a(n, m) * cost * Ps[n - 1][m] - b(n, m) * Ps[n - 2][m];
				else Ps[n][m] = a(n, m) * cost * Ps[n - 1][m];
			}
		}
	}
}

// create all cos(mlambda) and sin(mlambda) for given lambda and M 
void Forces::allml(double sinl, double cosl, int pnm_order)
{
	ml = Matrix(2, pnm_order + 1);

	ml[0][0] = 0.0; // sin(0)
	ml[1][0] = 1.0; // cos(0)
	ml[0][1] = sinl; // sin(lambda)
	ml[1][1] = cosl; // cos(lambda)

	for (int i = 0; i < 2; i++)
	{
		for (int j = 2; j < pnm_order + 1; j++) ml[i][j] = 2.0 * cosl * ml[i][j - 1] - ml[i][j - 2];
	}
}

void Forces::earthGravityForce(const PositionVector& pos, const Time& time)
{
	Astrometry::rotationMatrices(time);

	if (no_coefficients)
	{
		int responce = getGravityCoefficients();
		if (responce)
		{
			throw std::runtime_error("");
		}
		no_coefficients = false;
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
	r *= 1000.0;

	allPs(cost, gravity_order);
	allml(sinl, cosl, gravity_order);

	double R = EARTH::RADIUS / r;
	double mu = EARTH::GM / r;

	double xr1, xr2, xt1, xt2, xl1, xl2;
	double cnm, snm;
	double Pt;
	std::vector<double> omega_r(gravity_order + 1);
	std::vector<double> omega_t(gravity_order + 1);
	std::vector<double> omega_l(gravity_order + 1);

	double vr = 0.0, vt = 0.0, vl = 0.0;

	for (int m = 0; m < gravity_order + 1; m++)
	{
		xr1 = 0.0; xr2 = 0.0; xt1 = 0.0; xt2 = 0.0; xl1 = 0.0; xl2 = 0.0;

		for (int n = std::max(2, m); n < gravity_order + 1; n++)
		{
			if ((n == m) or (n - 1 < m)) Pt = (double)m * cost / sint * Ps[n][m];
			else Pt = ((double)n * cost * Ps[n][m] - f(n, m) * Ps[n - 1][m]) / sint;

			int ind = (int)order[n][m];

			cnm = Cnm[ind] * pow(R, n);
			snm = Snm[ind] * pow(R, n);

			xl1 += cnm * Ps[n][m]; // * 1e-280;
			xl2 += snm * Ps[n][m]; // * 1e-280;

			xr1 -= (double)(n + 1) / r * cnm * Ps[n][m]; // * 1e-280;
			xr2 -= (double)(n + 1) / r * snm * Ps[n][m]; // * 1e-280;

			xt1 += cnm * Pt; // * 1e-280;
			xt2 += snm * Pt; // * 1e-280;
		}

		omega_r[m] = mu * (ml[1][m] * xr1 + ml[0][m] * xr2);
		omega_t[m] = mu * (ml[1][m] * xt1 + ml[0][m] * xt2);
		omega_l[m] = mu * (double)m * (ml[1][m] * xl2 - ml[0][m] * xl1);
	}

	// Modified forward column method

	double u;
	for (int m = gravity_order; m > 0; m--)
	{
		u = sqrt((double)(2 * m + 1) / (double)(2 * m)) * sint;
		if (m == 1) u = sqrt(3.0) * sint;

		vr = (vr + omega_r[m]) * u;
		vt = (vt + omega_t[m]) * u;
		vl = (vl + omega_l[m]) * u;
	}

	vr += omega_r[0];
	vt += omega_t[0];
	vl += omega_l[0];


	//vr *= 1e280;
	//vt *= 1e280;
	//vl *= 1e280;

	// for the central field
	vr -= mu / r;

	// components of gravitational acceleration
	PositionVector g = PositionVector();
	g[0] = (sint * cosl * vr + cost * cosl / r * vt - sinl * vl / r / sint) / 1000.0;
	g[1] = (sint * sinl * vr + cost * sinl / r * vt + cosl * vl / r / sint) / 1000.0;
	g[2] = (cost * vr - sint * vt / r) / 1000.0;
	
	// back to GCRF
	Matrix rt2c = Astrometry::getRt2c();
	g = mul(rt2c, g);

	forces = forces + g;
}

void Forces::centralForce(const Satellite& sat, const Time& time)
{
	PositionVector position = sat.getPosition();
	PositionVector itrf_position = mul(Astrometry::getRc2t(), position);
	double r = itrf_position.norm() * 1000.0;
	itrf_position = itrf_position * (-1.0 * EARTH::GM / r / r / r);
	position = mul(Astrometry::getRt2c(), itrf_position);
	
	forces = forces + position;
}


void Forces::outerBodiesGravityForce(const PositionVector& pos, const Time& time)
{
	double state[6];
	double lt;
	double* rel_vec = new double[3];
	double rel_r = 0.0;
	double* force = new double[3];
	force[0] = 0.0; force[1] = 0.0; force[2] = 0.0;

	SpiceDouble et = time.ET();

	SpiceInt n;
	SpiceDouble gm;

	int number_of_bodies = (int)bodies.size();

	for (int body = 0; body < number_of_bodies; body++)
	{
		spkezr_c(bodies[body], et, "J2000", "NONE", "earth", state, &lt);
		bodvrd_c(bodies[body], "GM", 1, &n, &gm );
		PositionVector body_position(std::vector<double>{state[0], state[1], state[2]});

		rel_r = 0.0;
		for (int i = 0; i < 3; i++)
		{
			rel_vec[i] = body_position[i] - pos[i];
			rel_r += rel_vec[i] * rel_vec[i];
		}

		rel_r = sqrt(rel_r);
		double R = body_position.norm();

		for (int i = 0; i < 3; i++)
		{
			force[i] += gm * (rel_vec[i] / pow(rel_r, 3) - state[i] / pow(R, 3));
		}
	}

	forces[0] += force[0];
	forces[1] += force[1];
	forces[2] += force[2];

	delete[] rel_vec;
	delete[] force;

}

void Forces::solarPressureForce(Satellite& sat, const Time& time)
{
	if (solar_pressure_calculated)
	{
		forces += solar_pressure_force * 0.001 / sat.getMass(); // m/s^2 -> km/s^2
		return;
	}

	if (sat.getPolygons().size() == 0)
	{
		std::cerr << "\033[31m#22 No polygons given, unable to calculate solar pressure force\033[0m" << std::endl;
		throw std::runtime_error("");
	}

	double* state = new double[3];
	double lt, cos_theta, alpha, mu;
	PositionVector force, gcrf_normal;
	SpiceDouble et = time.ET();
	// std::cout << "ET: " << et << std::endl;
	spkpos_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	
	// std::cout << "Sat position \n" << sat.getPosition() << std::endl;
	sat.setSunPosition(sun_pos);
	
	// Sun to satellite unit vector
	PositionVector rv = sat.getPosition() - sun_pos;
	double r = rv.norm();
	rv = rv * (1.0 / r);

	//Matrix gtl = Matrix(sat.getQuaternion()).transpose();
	//sat.rotateSolarPanels(mul(gtl, rv));
	sat.rotateSolarPanels(sat.getQuaternion().get_inverse() * rv);
	//Matrix ltg = gtl.transpose();

	double ef = Astrometry::eclipse_factor(sun_pos, sat.getPosition());
	
	std::vector<Polygon> polygons = sat.getPolygons();
	std::vector<Polygon> solar_panels = sat.getSolarPanels();
	int number_of_rejected_polygons = 0;
	for (int i = 0; i < solar_panels.size(); i++)
	{
		polygons.push_back(solar_panels[i]);
	}
	for (int i = 0; i < polygons.size(); i++)
	{
		//gcrf_normal = mul(ltg, polygons[i].getNormal());
		gcrf_normal = sat.getQuaternion() * polygons[i].getNormal();

		cos_theta = rv.dot(gcrf_normal);

		if (cos_theta >= 0)
		{
			number_of_rejected_polygons++;
			continue;
		}

		cos_theta *= -1.0;
		
		alpha = polygons[i].getReflectivityFactor();
		mu = polygons[i].getSpecularityFactor();

		//force = rv * (1.0 - alpha)*(1.0 - alpha) + gcrf_normal * (-2.0 * alpha * mu * alpha * mu * cos_theta)
		//	+ (rv - gcrf_normal * (2.0 / 3.0)) * ((1.0 - mu)*(1.0 - mu) * alpha*alpha);
		force = rv * (1.0 - alpha) + gcrf_normal * (-2.0 * alpha * mu * cos_theta)
			+ (rv - gcrf_normal * (2.0 / 3.0)) * ((1.0 - mu) * alpha);
		force = force * (cos_theta * polygons[i].getArea() * ef * SUN::FLUX * pow(WORLD::AU / r, 2) / WORLD::SPEED_OF_LIGHT / sat.getMass()); // m/s^2
		force = force * (0.001); // m/s^2 -> km/s^2


		forces = forces + force;
	}
	delete[] state;
	state = NULL;
}

void Forces::srpForce(Satellite& sat, const Time& time)
{
	if (solar_pressure_calculated)
	{
		forces += solar_pressure_force * 0.001 / sat.getMass(); // m/s^2 -> km/s^2
		return;
	}

	SRPEngine engine(sat.getHdfFile());

	double state[6];
	double lt, ef;
	PositionVector s;
	SRPResult res;

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

	forces += PositionVector({res.total_force[0], res.total_force[1], res.total_force[2]}) * ef;

	Forces::setSolarPressureCalculated(true);
	Forces::setSolarPressureForce(PositionVector({res.total_force[0], res.total_force[1], res.total_force[2]}) * ef);
}

void Forces::solarPressureGmat(Satellite& sat, const Time& time)
{
	double* state = new double[3];
	double lt, cos_theta, alpha, mu;
	PositionVector force, gcrf_normal;
	SpiceDouble et = time.ET();
	// std::cout << "ET: " << et << std::endl;
	spkpos_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	
	// std::cout << "Sun position \n" << sun_pos << std::endl;
	// std::cout << "Sat position \n" << sat.getPosition() << std::endl;
	
	// Sun to satellite unit vector
	PositionVector rv = sat.getPosition() - sun_pos;
	double r = rv.norm();
	rv = rv * (1.0 / r);

	double ef = Astrometry::eclipse_factor(sun_pos, sat.getPosition());

	double Cr = 1.0; // reflectivity coeff
	double A = 1.36; // area, m^2

	force = ef * SUN::FLUX * pow(WORLD::AU / r, 2) / WORLD::SPEED_OF_LIGHT / sat.getMass() * Cr * A * rv / 1000.0;

	forces = forces + force;

}

PositionVector Forces::allForces(Satellite& sat, const Time& time)
{
	forces[0] = 0.0; forces[1] = 0.0; forces[2] = 0.0;
	
	if (account_for_earth_gravity)
	{
		if (gravity_order == 0)
		{
			centralForce(sat, time);
		}
		else
		{
			earthGravityForce(sat.getPosition(), time);
		}
	}
	if (account_for_outer_gravity) outerBodiesGravityForce(sat.getPosition(), time);
	if (account_for_solar_pressure)
	{
		if (sat.getHdfFile() != "") srpForce(sat, time);
		else solarPressureForce(sat, time);
	}
	if (account_for_solar_pressure_gmat) solarPressureGmat(sat, time);
	
	forces = forces + sat.getPulseAcceleration(time) / 1000.0;

	return forces;
}

void Forces::checkCoeffs()
{
	getGravityCoefficients();
	for (int i=0; i < (gravity_order + 2)*(gravity_order + 1)/2; i++)
	{
		std::cout << Cnm[i] << std::endl;
	}
}

void Forces::checkFilenames()
{
	std::cout << "EGM filename : " << egmfilename << std::endl;
}

void Forces::checkPnm()
{
	double phi = 56.6 * M_PI / 180.0;
	double cost = std::sin(phi);

	allPs(cost, 10);

	std::cout << "\033[31m";
	std::cout << std::setprecision(17);
	for (int n = 0; n < 6; n++)
	{
		for (int m = 0; m <= n; m++)
		{
			std::cout << Ps[n][m] << "\t|";
		}
		std::cout << std::endl;
	}
	std::cout << "\033[0m";
}

void Forces::check2000nan()
{
	double phi = 0.0 * M_PI / 180.0;
	double r = 6400.0;
	double lambda = 0.;

	PositionVector pos({r * std::cos(phi), 0.0, r * std::sin(phi)});
	Time time(2012, 1, 1, 0, 0, 0.0);

	earthGravityForce(pos, time);
	std::cout << forces << std::endl;

}