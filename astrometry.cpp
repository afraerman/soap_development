#include "stdafx.h"

std::string Astrometry::eopfilename = "../Files/eop_new.txt";
std::string Astrometry::tlsfilename = "../Files/naif0012.tls";
std::string Astrometry::ephfilename = "../Files/de440.bsp";
std::string Astrometry::gmfilename = "../Files/gm_de440.tpc";
int Astrometry::order = 7;
int Astrometry::_mjd = 0;
Matrix Astrometry::duts = Matrix(order, 2);
Matrix Astrometry::xps = Matrix(order, 2);
Matrix Astrometry::yps = Matrix(order, 2);

Matrix Astrometry::rc2t;
Matrix Astrometry::rc2t1;
Matrix Astrometry::rt2c;

bool Astrometry::no_ephemeris = true;

void Astrometry::setEOPfile(const std::string& filename)
{
	eopfilename = filename;
}
void Astrometry::setTLSfile(const std::string& filename)
{
	tlsfilename = filename;
}
void Astrometry::setEPHEMfile(const std::string& filename)
{
	ephfilename = filename;
}
void Astrometry::setGMfile(const std::string& filename)
{
	gmfilename = filename;
}

void Astrometry::get_ephemeris() 
{
	// Leap-seconds
	ConstSpiceChar* tls = tlsfilename.c_str();
	furnsh_c(tls);

	// Ephemeris
	ConstSpiceChar* eph = ephfilename.c_str();
	furnsh_c(eph);

	// Gravitation parameters (GM)
	ConstSpiceChar* gm = gmfilename.c_str();
	furnsh_c(gm);
}

double Astrometry::eclipse_factor(const PositionVector& sun_pos, const PositionVector& sat_pos)
{
	double p;
	PositionVector rv = sun_pos - sat_pos;

	double R_sun_app = asin(SUN::RADIUS / rv.norm());
	double R_earth_app = asin((EARTH::RADIUS / 1000.0) / sat_pos.norm());

	double d = acos(-1.0 * sat_pos.dot(rv) / sat_pos.norm() / rv.norm());

	if (d >= R_sun_app + R_earth_app) p = 0.0;
	else if (d <= R_earth_app - R_sun_app) p = 100.0;
	else if ((fabs(R_sun_app - R_earth_app) < d) && (d < R_sun_app + R_earth_app))
	{
		double c1 = (d*d + R_sun_app * R_sun_app - R_earth_app * R_earth_app) / (2.0 * d);
		double c2 = sqrt(R_sun_app * R_sun_app - c1 * c1);
		double A = pow(R_sun_app, 2) * acos(c1 / R_sun_app) + pow(R_earth_app, 2) * acos((d - c1) / R_earth_app) - d * c2;

		p = 100.0 * A / (M_PI * pow(R_sun_app, 2));
	}
	else p = 100.0 * R_earth_app * R_earth_app / R_sun_app / R_sun_app;

	return 1.0 - p / 100.0;
}

void Astrometry::EOP(const Time& time)
{
	//std::cout << "I'm about to collect EOPs" << std::endl;

	int year_r, month_r, day_r, mjd_r, dat;
	double x_r, y_r, dut_r;
	double lod, dpsi, deps, dx, dy, djm0, djm;

	std::ifstream eop(eopfilename);
	if (eop.is_open())
	{
		iauCal2jd(time.getYear(), time.getMonth(), time.getDay(), &djm0, &djm);
		
		for (int i = 0; i < order; i++)
		{
			eop >> year_r >> month_r >> day_r >> mjd_r >> x_r >> y_r >> dut_r >> lod >> dpsi >> deps >> dx >> dy >> dat;
			duts[i][0] = (double)i * 86400.0;
			xps[i][0] = (double)i * 86400.0;
			yps[i][0] = (double)i * 86400.0;

			duts[i][1] = dut_r;
			xps[i][1] = x_r;
			yps[i][1] = y_r;
		}

		int stop_sign = 0;
		bool found = false;
		while (eop >> year_r >> month_r >> day_r >> mjd_r >> x_r >> y_r >> dut_r >> lod >> dpsi >> deps >> dx >> dy >> dat)
		{
			for (int i = 0; i < order - 1; i++)
			{
				duts[i][1] = duts[i+1][1];
				xps[i][1] = xps[i+1][1];
				yps[i][1] = yps[i+1][1];
			}
			
			duts[order-1][1] = dut_r;
			xps[order-1][1] = x_r;
			yps[order-1][1] = y_r;

			if (mjd_r == (int)djm - order/2)
			{
				found = true;
			}

			if (found)
			{
				//duts[i][0] = (double)i * 86400.0;
				//xps[i][0] = (double)i * 86400.0;
				//yps[i][0] = (double)i * 86400.0;

				//duts[i][1] = dut_r;
				//xps[i][1] = x_r;
				//yps[i][1] = y_r;

				//if (i == 3) _mjd = mjd_r;

				stop_sign++;
			}

			if (stop_sign == order) break;
		}
		eop.close();
	}
}

double Astrometry::lagrange_interpol(const Matrix& m, double x)
{
	double result = 0.0;
	for (int i = 0; i < order; i++)
	{
		double polinomial = 1.0;
		for (int j = 0; j < order; j++)
		{
			if (j != i)
			{
				polinomial *= (x - m[j][0]) / (m[i][0] - m[j][0]);
			}
		}
		result += m[i][1] * polinomial;
	}
	return result;
}

void Astrometry::rotationMatrices(const Time& time)
{
	int j;
	double utc1, utc2, ut11, ut12, tai1, tai2, tt1, tt2;
	double dut, xp, yp;
	double rc2t_double[3][3], rc2t1_double[3][3];

	double x = (double)(order/2)*86400.0 + time.toSeconds(); // x is in the middle of the interpolation range
	
	/**/
	dut = lagrange_interpol(duts, x);
	xp = lagrange_interpol(xps, x);
	yp = lagrange_interpol(yps, x);
	

	//dut = duts[order/2][1];
	//xp = xps[order/2][1];
	//yp = yps[order/2][1];

	xp = xp * M_PI / 648000.0;
	yp = yp * M_PI / 648000.0;

	/*UTC into internal format*/
	j = iauDtf2d("UTC", time.getYear(), time.getMonth(), time.getDay(), time.getHours(), time.getMinutes(), time.getSeconds(), &utc1, &utc2);

	/*UTC -> UT1*/
	j = iauUtcut1(utc1, utc2, dut, &ut11, &ut12);

	/*UTC -> TAI*/
	j = iauUtctai(utc1, utc2, &tai1, &tai2);

	/*TAI -> TT*/
	j = iauTaitt(tai1, tai2, &tt1, &tt2);

	iauC2t00b(tt1, tt2, ut11, ut12, xp, yp, rc2t_double);
	/*
	double sp = iauSp00(tt1, tt2);
	double rpom[3][3];
	iauPom00(xp, yp, sp, rpom);

	double era = iauEra00(ut11, ut12);
	double r[3][3];

	double pnm[3][3];
	iauPnm06a(tt1, tt2, pnm);

	double rerapnm[3][3];
	iauCr(pnm, r);
	iauRz(era, r);
	iauRxr(rpom, r, rc2t_double);
	*/

	rc2t = Matrix();
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			rc2t[i][j] = rc2t_double[i][j];
		}
	}
	rt2c = rc2t.transpose();

	// ---- finding a derivative ---------- //

	double sp = iauSp00(tt1, tt2);
	double rpom[3][3];
	iauPom00(xp, yp, sp, rpom);

	// ��������������� �������
	double m[3][3], thnm[3][3], dthnm[3][3];

	// ���������� Rpm
	iauTr(rpom, rpom);
	iauRxr(rpom, rc2t_double, thnm);

	// ������ �����������
	m[0][0] = 0.0; m[0][1] = EARTH::mean_earth_rotation_rate; m[0][2] = 0.0;
	m[1][0] = -1.0 * EARTH::mean_earth_rotation_rate; m[1][1] = 0.0; m[1][2] = 0.0;
	m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 0.0;

	iauRxr(m, thnm, dthnm);

	// ���������� Rpm
	iauTr(rpom, rpom);
	iauRxr(rpom, dthnm, rc2t1_double);

	rc2t1 = Matrix();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rc2t1[i][j] = rc2t1_double[i][j];
		}
	}
}

Matrix Astrometry::getRc2t()
{
	return rc2t;
}

Matrix Astrometry::getRc2t1()
{
	return rc2t1;
}

Matrix Astrometry::getRt2c()
{
	return rt2c;
}

PositionVector Astrometry::get_rotation_axis(const Time& t)
{
	double xp = xps[order/2][1];
	double yp = yps[order/2][1];

	xp = xp * M_PI / 648000.0;
	yp = -1.0 * yp * M_PI / 648000.0;

	return PositionVector({std::sin(xp) * std::cos(yp), std::sin(xp) * std::sin(yp), std::cos(xp)});
}

void Astrometry::check_filenames()
{
	std::cout << "EOP filename: " << eopfilename << std::endl;
	std::cout << "TLS filename: " << tlsfilename << std::endl;
	std::cout << "EPHEM filename: " << ephfilename << std::endl;
	std::cout << "GM filename: " << gmfilename << std::endl;
}