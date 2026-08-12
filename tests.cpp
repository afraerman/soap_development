#include "stdafx.h"

bool Test::isClose(PositionVector& v1, PositionVector& v2)
{
	double k = 0.0;
	for (int i = 0; i < v1.getValues().size(); i++) k += pow((v1[i] - v2[i]), 2);
	return sqrt(k) < 1e-7;
}
bool Test::isClose(Matrix& m1, Matrix& m2)
{
	Matrix m3;
	m3 = m2 * (-1.0);
	m3 = m3 + m1;
	double k = 0;
	for (int i = 0; i < m1.getRows(); i++)
	{
		for (int j = 0; j < m1.getColumns(); j++)
		{
			k += m3[i][j] * m3[i][j];
		}
	}
	return sqrt(k) < 1e-5;
}

void Test::createPositoinVector()
{
	std::vector<double> a(3);
	a[0] = 1.0; a[1] = 2.0; a[2] = 3.0;
	PositionVector b = PositionVector(a);
	std::cout << b << std::endl;
}

void Test::createMatrix()
{
	std::vector<std::vector<double>> b(3, std::vector<double>(3));
	b[0][0] = 1.0; b[0][1] = 2.0; b[0][2] = 3.0;
	b[1][0] = 4.0; b[1][1] = 5.0; b[1][2] = 6.0;
	b[2][0] = 7.0; b[2][1] = 8.0; b[2][2] = 9.0;
	Matrix m = Matrix(3, 3, b);
	for (int i = 0; i < m.getRows(); i++)
	{
		for (int j = 0; j < m.getColumns(); j++)
		{
			std::cout << m[i][j] << '\t';
		}
		std::cout << std::endl;
	}
}

void Test::createMatrix2()
{
	Matrix c;
	c = Matrix(3, 3);
	c[0][0] = 1.0;
}

void Test::addMatrices()
{
	std::vector<std::vector<double>> b(3, std::vector<double>(3));
	b[0][0] = 1.0; b[0][1] = 2.0; b[0][2] = 3.0;
	b[1][0] = 4.0; b[1][1] = 5.0; b[1][2] = 6.0;
	b[2][0] = 7.0; b[2][1] = 8.0; b[2][2] = 9.0;
	Matrix m = Matrix(3, 3, b);
	Matrix m2 = Matrix(3, 3, b);

	std::vector<std::vector<double>> ans(3, std::vector<double>(3));
	ans[0][0] = 2.0; ans[0][1] = 4.0; ans[0][2] = 6.0;
	ans[1][0] = 8.0; ans[1][1] = 10.0; ans[1][2] = 12.0;
	ans[2][0] = 14.0; ans[2][1] = 16.0; ans[2][2] = 18.0;
	Matrix answer(3, 3, ans);
	m = m + m2;

	if (isClose(m, answer))
	{
		std::cout << "Adding matrices -- OK" << std::endl;
	}
	else
	{
		std::cout << "Instead of" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << answer[i][j] << '\t';
			}
			std::cout << std::endl;
		}
		std::cout << "got" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << m[i][j] << '\t';
			}
			std::cout << std::endl;
		}
	}
}

void Test::mulToMatrix()
{
	std::vector<std::vector<double>> b(3, std::vector<double>(3));
	b[0][0] = 1.0; b[0][1] = 2.0; b[0][2] = 3.0;
	b[1][0] = 4.0; b[1][1] = 5.0; b[1][2] = 6.0;
	b[2][0] = 7.0; b[2][1] = 8.0; b[2][2] = 9.0;
	Matrix m = Matrix(3, 3, b);

	std::vector<std::vector<double>> ans(3, std::vector<double>(3));
	ans[0][0] = 2.0; ans[0][1] = 4.0; ans[0][2] = 6.0;
	ans[1][0] = 8.0; ans[1][1] = 10.0; ans[1][2] = 12.0;
	ans[2][0] = 14.0; ans[2][1] = 16.0; ans[2][2] = 18.0;
	Matrix answer(3, 3, ans);

	m = m * 2.0;

	if (isClose(m, answer))
	{
		std::cout << "Multiplication of Matrix and double -- OK" << std::endl;
	}
	else
	{
		std::cout << "Instead of" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << answer[i][j] << '\t';
			}
			std::cout << std::endl;
		}
		std::cout << "got" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << m[i][j] << '\t';
			}
			std::cout << std::endl;
		}
	}
}

void Test::dotTest()
{
	std::vector<double> p1(3);
	p1[0] = 1.0; p1[1] = 2.0; p1[2] = 3.0;
	PositionVector v1(p1);

	std::vector<double> p2(3);
	p2[0] = 4.0; p2[1] = 5.0; p2[2] = 6.0;
	PositionVector v2(p2);

	double dot_prod = v1.dot(v2);
	if (dot_prod == 32.0)
	{
		std::cout << "Dot product OK" << std::endl;
	}
	else
	{
		std::cout << "Instead of 32.0 got " << dot_prod << std::endl;
	}

}

void Test::crossTest()
{
	std::vector<double> p1(3);
	p1[0] = 1.0; p1[1] = 2.0; p1[2] = 3.0;
	PositionVector v1(p1);

	std::vector<double> p2(3);
	p2[0] = 4.0; p2[1] = 5.0; p2[2] = 6.0;
	PositionVector v2(p2);

	PositionVector cross_prod = v1.cross(v2);
	std::vector<double> ans(3);
	ans[0] = -3.0; ans[1] = 6.0; ans[2] = -3.0;
	PositionVector answer(ans);
	
	if (isClose(answer, cross_prod))
	{
		std::cout << "Cross product OK" << std::endl;
	}
	else
	{
		std::cout << "Instead of " << answer <<  " got " << cross_prod << std::endl;
	}
}

void Test::outerTest()
{
	std::vector<double> p1(3);
	p1[0] = 1.0; p1[1] = 2.0; p1[2] = 3.0;
	PositionVector v1(p1);

	std::vector<double> p2(3);
	p2[0] = 4.0; p2[1] = 5.0; p2[2] = 6.0;
	PositionVector v2(p2);

	Matrix m = v1.outer(v2);

	std::vector<std::vector<double>> ans(3, std::vector<double>(3));
	ans[0][0] = 4.0; ans[0][1] = 5.0; ans[0][2] = 6.0;
	ans[1][0] = 8.0; ans[1][1] = 10.0; ans[1][2] = 12.0;
	ans[2][0] = 12.0; ans[2][1] = 15.0; ans[2][2] = 18.0;
	Matrix answer(3,3,ans);
	
	if (isClose(answer, m))
	{
		std::cout << "Outer product OK" << std::endl;
	}
	else
	{
		std::cout << "Instead of" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << answer[i][j] << '\t';
			}
			std::cout << std::endl;
		}
		std::cout << "got" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << m[i][j] << '\t';
			}
			std::cout << std::endl;
		}
	}
}

void Test::skewTest()
{
	std::vector<double> p1(3);
	p1[0] = 1.0; p1[1] = 2.0; p1[2] = 3.0;
	PositionVector v1(p1);

	std::vector<double> p2(3);
	p2[0] = 4.0; p2[1] = 5.0; p2[2] = 6.0;
	PositionVector v2(p2);

	Matrix m = v1.skew();
	v1 = mul(m, v2);

	std::vector<double> ans(3);
	ans[0] = -3.0; ans[1] = 6.0; ans[2] = -3.0;
	PositionVector answer(ans);

	if (isClose(answer, v1))
	{
		std::cout << "Skew OK" << std::endl;
	}
	else
	{
		std::cout << "From vector {1.0 2.0 3.0} got" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << m[i][j] << '\t';
			}
			std::cout << std::endl;
		}
	}

}

void Test::LUTest()
{
	std::vector<std::vector<double>> a(3, std::vector<double>(3));
	a[0][0] = 1.0; a[0][1] = 1.0; a[0][2] = 1.0;
	a[1][0] = 4.0; a[1][1] = 3.0; a[1][2] = -1.0;
	a[2][0] = 3.0; a[2][1] = 5.0; a[2][2] = 3.0;

	Matrix m = Matrix(3, 3, a);
	m.LUdecompose();

	a[1][1] = -1.0; a[1][2] = -5.0;
	a[2][1] = -2.0; a[2][2] = -10.0;

	Matrix answer = Matrix(3, 3, a);

	if (isClose(m, answer)) std::cout << "LU decomposed -- OK" << std::endl;
	else
	{
		std::cout << "My LU" << std::endl;
		for (int i = 0; i < m.getRows(); i++)
		{
			for (int j = 0; j < m.getColumns(); j++)
			{
				std::cout << m[i][j] << '\t';
			}
			std::cout << std::endl;
		}
		std::cout << "Correct LU" << std::endl;
		for (int i = 0; i < answer.getRows(); i++)
		{
			for (int j = 0; j < answer.getColumns(); j++)
			{
				std::cout << answer[i][j] << '\t';
			}
			std::cout << std::endl;
		}
	}
}

void Test::solveTest()
{
	std::vector<double> a(3);
	a[0] = 3.0; a[1] = 4.0; a[2] = 5.0;
	
	PositionVector answer = PositionVector(a);

	a[0] = 18.0; a[1] = 50.0; a[2] = 25.0;

	PositionVector f = PositionVector(a);

	std::vector<std::vector<double>> b(3, std::vector<double>(3));
	b[0][0] = 1.0; b[0][1] = 0.0; b[0][2] = 3.0;
	b[1][0] = 4.0; b[1][1] = 7.0; b[1][2] = 2.0;
	b[2][0] = 0.0; b[2][1] = 0.0; b[2][2] = 5.0;

	Matrix m = Matrix(3, 3, b);

	solve(m, f);
	if (isClose(f, answer))
	{
		std::cout << "Solve -- OK" << std::endl;
	}
	else
	{
		std::cout << f << std::endl;
		std::cout << answer << std::endl;
	}
}

void Test::matrixChangeTest()
{
	Matrix a(3, 3);
	a[0] = std::vector<double>(3, 4);

	Matrix answer(3, 3);
	answer[0][0] = 4.0; answer[0][1] = 4.0; answer[0][2] = 4.0;

	if (isClose(a, answer))
	{
		std::cout << "Matrix change of a row -- OK" << std::endl;
	}
	else
	{
		std::cout << "Instead of first row 4.0, rest - 0.0 got" << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << a[i][j] << '\t';
			}
			std::cout << std::endl;
		}
	}
}

void Test::matrixInputTest()
{
	Matrix a;
	std::cin >> a[0][0] >> a[0][1] >> a[0][2];

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::cout << a[i][j] << '\t';
		}
		std::cout << std::endl;
	}
}

void Test::mulTest()
{
	Matrix a;
	a[0][0] = 0.1;
	a[1][1] = 10.0;
	a[2][2] = 10.0;

	PositionVector b({ 0.0, 0.0, 0.01 });

	std::cout << mul(a, b) << std::endl;
}

void Test::quat2mat2quat()
{
	Quaternion q1(-0.801707, -0.116075, 0.322711, -0.48954);
	Matrix m = q1.to_matrix();
	Quaternion q2(m);

	double diff = q1.get_w() * q2.get_w() + q1.get_x() * q2.get_x() + q1.get_y() * q2.get_y() + q1.get_z() * q2.get_z();
	if (std::fabs(diff - 1) < 1e-4 || fabs(diff + 1) < 1e-4)
	{
		std::cout << "OK" << std::endl;
	}
	else
	{
		std::cout << diff << std::endl;
		std::cout << "FAILED\n True answer " << q1 << "\nFalse answer " << q2 << std::endl;
	}
}

void Test::allMatricesTests()
{
	addMatrices();
	mulToMatrix();
	LUTest();
	matrixChangeTest();
	quat2mat2quat();
}

void Test::quaternionTest()
{
	PositionVector pos({1.0, 2.0, 3.0});
	Quaternion q(0.707107, 0.707107, 0.0, 0.0);
	Quaternion my_q(0.707107, 0.707107, 0.0, 0.0);

	std::cout << q.get_inverse() << std::endl;
	std::cout << my_q.get_inverse() << std::endl << std::endl;

	std::cout << mul(q.to_matrix(), pos) << std::endl;
	std::cout << my_q * pos << std::endl << std::endl;
	std::cout << mul(my_q.to_matrix(), pos) << std::endl;

	std::cout << q.to_matrix() << std::endl;
	std::cout << my_q.to_matrix() << std::endl << std::endl;
}

void Test::swapPositionVectorsTest()
{
	PositionVector v1({1.0, 2.0, 3.0});
	PositionVector v2 = v1;

	v1 = v1 * 3;
	std::cout << v1 << std::endl;
	std::cout << v2 << std::endl;
	v1 = v2;
	std::cout << v1 << std::endl;

}

void Test::quatVectorMulTEst()
{
	Quaternion q(0.707107, 0.707107, 0.0, 0.0);
	PositionVector v1({1.0, 2.0, 3.0});
	std::cout << q * v1<< std::endl;
	std::cout << mul(q.to_matrix(), v1) << std::endl;
}

void Test::controlDefactorTest()
{
	Matrix I(3, 3, {{15560.0, 0.0, 0.0}, {0.0, 21510.0, 0.0}, {0.0, 0.0, 22750.0}});
	Quaternion initial(0.19670857936400293,	-0.37412024472939981,	-0.84748619945631387,	-0.3211028204344783);

	Quaternion target(0.3015036,	0.3631667,	0.2085039,	0.8565813);

	Time init_time(2015, 2, 12, 15, 44, 17.0);
	Time target_time(2015, 2, 13, 0, 51, 16.4);

	double gap = target_time - init_time;
	std::cout << gap << std::endl;

	Control::setTargetTime(gap);
	Control::defactorTarget(I, initial, target);
}

void Test::quatDivisionTest()
{
	Quaternion q1(0.4384506,	0.4671011,	0.7512849,	0.1585832);
	Quaternion q2(0.1063209,	0.3250136, 0.9339571,	0.1038563);

	std::cout << q1.get_inverse() * q2 << std::endl;
}

void Test::eopTest()
{
	Time time(2000, 1, 1, 9, 0, 0.0); // 2000-01-01T09:00:00.0
	Astrometry::EOP(time);

}

void Test::timeSubtractTest()
{
	Time t1 = Time(2019, 1, 1, 0, 0, 0.0);
	Time t2 = Time(2019, 1, 1, 0, 0, 2.0);
	std::cout << t2 - t1 << std::endl;

	t1 = Time(2019, 1, 1, 0, 0, 0.0);
	t2 = Time(2019, 1, 1, 0, 2, 0.0);
	std::cout << t2 - t1 << std::endl;


	t1 = Time(2019, 1, 1, 0, 0, 0.0);
	t2 = Time(2019, 1, 1, 2, 0, 0.0);
	std::cout << t2 - t1 << std::endl;

	t1 = Time(2019, 1, 1, 0, 0, 0.0);
	t2 = Time(2019, 1, 2, 0, 0, 0.0);
	std::cout << t2 - t1 << std::endl;

	t1 = Time(2019, 1, 1, 0, 0, 0.0);
	t2 = Time(2019, 2, 1, 0, 0, 0.0);
	std::cout << t2 - t1 << std::endl;

	t1 = Time(2019, 1, 1, 0, 0, 0.0);
	t2 = Time(2020, 1, 1, 0, 0, 0.0);
	std::cout << t2 - t1 << std::endl;
}

void Test::magneticFieldMap()
{
	Time time(2019, 1, 1, 0, 0, 0.0);
	Torques::testMagnetic(time);
}

void Test::gravityMap()
{
	Time time(2019, 1, 1, 0, 0, 0.0);
	// Forces::gravityMap(time);
}

void Test::satelliteChangesTest()
{
	Satellite sat = Satellite();
	Satellite sat2 = sat;
	
	std::vector<double> new_position{ 10.0,20.0,30.0 };
	PositionVector pos(new_position);
	sat2.setPosition(pos);
	
	std::cout << sat.getPosition() << '\t' << sat2.getPosition() << std::endl;
}

void Test::satelliteGyrostatsTest()
{
	std::string filename = "../input_parameters.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double os;

	if (Input::read_input_file(filename, sat, time, interval, step, os))
	{
		return;
	}

	PositionVector mom = PositionVector({ 0.10, 0.20, 0.30 });

	sat->setControlMomentum(mom, 'g');
	std::cout << sat->getGyrostatsMomentum() << std::endl;

	std::cout << mom << std::endl;
	sat->setControlMomentum(mom, 'g');
	std::cout << sat->getGyrostatsMomentum() << std::endl;
	
}
void Test::overlapModesTest()
{
	std::string filename = "../overlapingModes.json";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double os;
	bool screen_check = true;
	std::vector<std::vector<Time>> scans, stops;

	if (Input::read_json_file(filename, sat, time, interval, step, os, screen_check))
	{
		scans = sat->getScanTimes();
		stops = sat->getStopTimes();
		for (auto scan: scans)
		{
			std::cout << scan[0] << '\t';
		}
		std::cout << std::endl;
		for (auto stop: stops)
		{
			std::cout << stop[0] << '\t';
		}
		std::cout << std::endl;
		return;
	}
}

void Test::orbitIntegratorTest1()
{
	Satellite satellite = Satellite();
	PositionVector pos(std::vector<double>{3988.3094462457951, -5292.6653976602517, 0.0});
	PositionVector vel(std::vector<double>{-0.7065073653665, -0.5323914865994, 7.7165125877266});
	StateVector st(pos, vel);
	satellite.setState(st);

	Time time(2012, 12, 17, 20, 00, 00.000);

	Satellite *sat;
	Time* t;
	sat = &satellite;
	t = &time;

	Forces::setGravityOrder(100);
	Forces::account_for_earth_gravity = true;
	Forces::account_for_solar_pressure = false;
	Forces::account_for_outer_gravity = true;


	const std::string savefilename = "/media/alexey/Disk1/asc/integrator_test/Alexey_100.txt";

	OrbitIntegrator orbit(sat, t, 86400.0, 10.0, 10.0, false);
	orbit.integrate(savefilename);
	
	std::cout << time << '\t' << satellite.getPosition() << std::endl;
}

void Test::orbitIntegratorForcelessTest2()
{
	Satellite satellite;
	PositionVector pos(std::vector<double>{3988.3094462457951, -5292.6653976602517, 0.0});
	PositionVector vel(std::vector<double>{-0.7065073653665, -0.5323914865994, 7.7165125877266});
	StateVector st(pos, vel);
	satellite.setState(st);
	
	Time time(2012, 12, 17, 20, 00, 00.000);

	Satellite *sat;
	Time* t;
	sat = &satellite;
	t = &time;

	const std::string savefile = "output.txt";

	OrbitIntegrator orbit(sat, t, 50000.0, 0.6, false);
	orbit.integrate(savefile);

	std::cout << *t << '\t' << satellite.getPosition() << std::endl;
}

void Test::attitudeIntegratorTest()
{
	PositionVector pos(std::vector<double>{7001.8571730459826,	4453.3634968502747, - 0.0035871888914568643});
	PositionVector vel(std::vector<double>{0.0, 0.0, 0.0});
	StateVector st(pos, vel);
	Matrix inertia(3, 3, std::vector<std::vector<double>>{ {0.1, 0.0, 0.0}, { 0.0, 10.0, 0.0 }, { 0.0, 0.0, 10.0 }});


	Satellite satellite(st, inertia);
	satellite.setAngularVelocity(std::vector<double>{0.0, 0.0, 0.0});
	//satellite.setQuaternion(boost::math::quaternion<double>{1.0, 0.0, 0.0, 0.0});
	satellite.setQuaternion(Quaternion());
	

	Time time;

	Satellite *sat;
	Time* t;
	sat = &satellite;
	t = &time;

	const std::string savefile = "rotation_output_011010.txt";

	AttitudeIntegrator attitude(sat, t, 12000.0, 0.6, false);
	attitude.integrate(savefile);

	std::cout << *t << '\t' << satellite.getPosition() << std::endl;
}

void Test::autostepTest()
{
	std::string input_filename = "/media/alexey/Disk/asc/solar_pressure/simulations/mm_json.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double output_step;
	bool screen_check = true;
	
	if (Input::read_json_file(input_filename, sat, time, interval, step, output_step, screen_check))
	{
		std::cout << "Smth went wrong while reading file " << input_filename << std::endl;
		return;
	}

	const std::string savefile = FILENAMES::ephemeris_filename;

	FullMotionIntegrator fullmotion(sat, time, interval, step, output_step, false);
	fullmotion.AttitudeIntegrator::integrate();

}

void Test::counterrotationTest()
{
	std::string filename = "../diploma_sat.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double output_step;

	if (Input::read_input_file(filename, sat, time, interval, step, output_step))
	{
		return;
	}
	
	// sat->setNewPolygons();
	const std::string savefile = FILENAMES::ephemeris_filename;
	//Torques::get_thrusters_activation_times();
	//Torques::get_magnets_activation_times();

	FullMotionIntegrator fullmotion(sat, time, interval, step, output_step, false);
	fullmotion.AttitudeIntegrator::integrate(savefile);
	std::cout << *time << '\t' << satellite.getPosition() << std::endl;
}

void Test::vtkSolarPressure()
{
	std::string input_filename = "/media/alexey/Disk/asc/solar_pressure/simulations/mm_json.txt";
	// std::string vtk_filename = "/media/alexey/Disk/asc/solar_pressure/КЭМ/vtk/ТЭ6_polygons.vtk";

	/*
	std::vector<Polygon> polygons;
	if (Input::read_vtk_file(vtk_filename, polygons))
	{
		return;
	}
	*/

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double output_step;
	bool screen_check = true;
	
	if (Input::read_json_file(input_filename, sat, time, interval, step, output_step, screen_check))
	{
		std::cout << "Smth went wrong while reading file " << input_filename << std::endl;
		return;
	}

	// sat->setPolygons(polygons);

	FullMotionIntegrator fullmotion(sat, time, interval, step, output_step, false);
	fullmotion.AttitudeIntegrator::integrate();

}

void Test::getSolarCoordinatesTest()
{
	Time time(2031, 2, 1, 0, 0, 0.0);
	double state[6];
	double lt;

	std::string tlsfilename = "../../Files//naif0012.tls";
	std::string ephfilename = "../../Files/de440.bsp";

	ConstSpiceChar* tls = tlsfilename.c_str();
	furnsh_c(tls);

	// Ephemeris
	ConstSpiceChar* eph = ephfilename.c_str();
	furnsh_c(eph);

	SpiceDouble et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	std::cout << "At time " << time << " sun_pos: " << sun_pos << std::endl;
	PositionVector sat_pos({-280713.13351, 1319580.39604, 601721.49012});
	PositionVector r = sat_pos - sun_pos;
	r = r / r.norm();
	std::cout << r << std::endl;

	PositionVector x  = PositionVector({-1.0 * r[1] / sqrt(1.0 - r[2]*r[2]), r[0] / sqrt(1.0 - r[2]*r[2]), 0.0});
	PositionVector z  = PositionVector({r[0]*r[2] / sqrt(1.0 - r[2]*r[2]), r[1]*r[2] / sqrt(1.0 - r[2]*r[2]), -1.0*sqrt(1.0 - r[2]*r[2])});

	x = x / x.norm();
	z = z / z.norm();

	Matrix m(3,3,
		{
			{x[0], x[1], x[2]},
			{r[0], r[1], r[2]},
			{z[0], z[1], z[2]}
		});
	m = m.transpose();
	
	//boost::math::quaternion<double> q = m.to_quat();
	Quaternion q(m);

	//std::cout << q.R_component_1() << '\t' << q.R_component_2() << '\t' << q.R_component_3() << '\t' << q.R_component_4() << std::endl;
	std::cout << q << std::endl;

}

void Test::fullMotionIntegratorTest()
{
	PositionVector pos(std::vector<double>{8000.0, 0.0, 0.0});
	PositionVector vel(std::vector<double>{0.0, 8.048, 0.0});
	StateVector st(pos, vel);
	Matrix inertia(3, 3, std::vector<std::vector<double>>{ {0.1, 0.0, 0.0}, { 0.0, 10.0, 0.0 }, { 0.0, 0.0, 10.0 }});


	Satellite satellite(st, inertia);
	satellite.setAngularVelocity(std::vector<double>{0.0, 0.0, 0.0});
	//satellite.setQuaternion(boost::math::quaternion<double>{1.0, 0.0, 0.0, 0.0});
	satellite.setQuaternion(Quaternion());

	Time time;

	Satellite *sat;
	Time* t;
	sat = &satellite;
	t = &time;

	const std::string savefile = "new_full_motion_output_011010.txt";

	FullMotionIntegrator fullmotion(sat, t, 50000.0, 0.6, false);
	fullmotion.OrbitIntegrator::integrate(savefile);

	std::cout << *t << '\t' << satellite.getPosition() << std::endl;
}

void Test::rotationMatricesTest()
{
	Time time(2032, 12, 17, 20, 00, 00.000);
	Astrometry::EOP(time);
	Astrometry::rotationMatrices(time);
	/*
	Matrix rt2c = Astrometry::getRt2c();

	double theta, lambda, sint, cost, sinl, cosl, r;
	PositionVector answer, xitrf;
	theta = (90) * M_PI / 180.0;
	lambda = (0) * M_PI / 180.0;

	cost = std::cos(theta);
	sint = std::sin(theta);
	cosl = std::cos(lambda);
	sinl = std::sin(lambda);
	r = EARTH::RADIUS / 1000.0 + 500.0;
	xitrf = PositionVector({r*sint*cosl, r*sint*sinl, r*cost});
	answer = PositionVector({-1208.172131506423, 6771.055932632426, 2.378616073934796});

	std::cout << "MY:" << '\t' << mul(rt2c, xitrf) << std::endl;
	std::cout << "Real: " << '\t' << answer << std::endl;

	theta = (90+10.489) * M_PI / 180.0;
	lambda = (360-46.080) * M_PI / 180.0;

	cost = std::cos(theta);
	sint = std::sin(theta);
	cosl = std::cos(lambda);
	sinl = std::sin(lambda);
	r = EARTH::RADIUS / 1000.0 + 500.0;
	xitrf = PositionVector({r*sint*cosl, r*sint*sinl, r*cost});
	answer = PositionVector({3969.506301935791, 5474.144855657393, -1259.199070906477});

	std::cout << "MY:" << '\t' << mul(rt2c, xitrf) << std::endl;
	std::cout << "Real: " << '\t' << answer << std::endl;
	*/
	Matrix rc2t, rc2t1;
	rc2t = Astrometry::getRc2t();
	rc2t1 = Astrometry::getRc2t1();

	PositionVector p({3988.3094462457951, -5292.6653976602517, 0.0});
	PositionVector v({-0.7065073653665, -0.5323914865994, 7.7165125877266});

	PositionVector ritrf, vitrf;

	ritrf = mul(rc2t, p);
	vitrf = mul(rc2t, v) + mul(rc2t1, p);

	std::cout << std::setprecision(17);
	std::cout << ritrf << std::endl;
	std::cout << vitrf << std::endl;
}

void Test::inputFileTest()
{
	std::string filename = "../input_parameters.txt";
	std::string line;
	std::ifstream input(filename);

	getline(input, line);
	while (line != "META_STOP")
	{
		std::cout << line << std::endl;
		getline(input, line);
	}
}

void Test::inputParametersTest()
{
	std::string filename = "../test_input.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double output_step;

	if (Input::read_input_file(filename, sat, time, interval, step, output_step))
	{
		return;
	}
	
	std::cout << output_step << std::endl;
	Astrometry::check_filenames();
	Astrometry::get_ephemeris();

}

void Test::fullLaunchTest()
{
	std::string filename = "../input_parameters.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double os;

	if (Input::read_input_file(filename, sat, time, interval, step, os))
	{
		return;
	}

	const std::string savefile = "output.txt";

	FullMotionIntegrator fullmotion(sat, time, interval, step, false);
	fullmotion.OrbitIntegrator::integrate(savefile);

	std::cout << *time << '\t' << satellite.getPosition() << std::endl;
}

void Test::solarPresureTest()
{
	std::string filename = "../Tests/solar_pressure/solar_pressure_input.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double os;

	if (Input::read_input_file(filename, sat, time, interval, step, os))
	{
		return;
	}

	PositionVector f;
	
	PositionVector position = satellite.getPosition();
	double r = position.norm() * 1000.0;
	std::cout << std::setprecision(27);
	f = position * (-1.0 * EARTH::GM / r / r / r);
	std::cout << "Central force" << '\n' << f << std::endl;

	Astrometry::get_ephemeris();
	f = Forces::allForces(satellite, t);
	std::cout << "Solar pressure" <<'\n' << f << std::endl;
	
	kclear_c();
}

void Test::multipleAssignmentTest()
{
	PositionVector v1, v2, v3 = PositionVector({ 0.0, 0.0, 0.0 });

	std::cout << v1 << std::endl;
	std::cout << v2 << std::endl;
	std::cout << v3 << std::endl;

	v1[0] = 1.0;
	v2[1] = 2.0;

	std::cout << v1 << std::endl;
	std::cout << v2 << std::endl;
	std::cout << v3 << std::endl;
}

void Test::inputTest()
{
	std::string a = "hello";
	Astrometry::setTLSfile(a);
	Astrometry::check_filenames();
}

void Test::inputJsonTest()
{
	Satellite satellite;
	Satellite* sat = &satellite;

	Time t;
	Time* time = &t;
	double interval;
	double step;
	double os;
	bool screen_check = true;

	if (Input::read_json_file("/media/alexey/Disk/Diploma/json_test.txt", sat, time, interval, step, os, screen_check))
	{
		return;
	}

	std::cout << "All parameters read successsfully" << std::endl;

	std::cout << sat->getReactionWheelsBlock().size() << std::endl;

}

void Test::gravityCoeffsTest()
{
	Forces::checkCoeffs();
}

void Test::pnmTest()
{
	Forces::checkPnm();
}

void Test::reactionWheelsTest()
{
	std::string filename = "../Test input.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double os;

	if (Input::read_input_file(filename, sat, time, interval, step, os))
	{
		return;
	}
	
	PositionVector momentum({2.0, 3.0, 4.0});
	PositionVector init_mom({0.0, 0.0, 0.0, 0.0});
	for (int i=0; i < 4; i++)
	{
		init_mom[i] = sat->getReactionWheelsBlock()[0][i].getStoredMomentum();
	}
	PositionVector angles = sat->getReactionWheelsBlock()[0][0].getAngles();
	// manual redistribution
	PositionVector redis = Control::redistributeCompensationMomentum(momentum, angles);
	std::cout << "Manual redistribution\n" << redis << std::endl;

	redis = Control::combineReactionWheelsBlockMomentum(redis, angles);
	std::cout << redis << std::endl;

	// Automatic redis
	redis = sat->setReactionWheelsMomentum(momentum);
	std::cout << "Automatic redis\nLeftovers\n" << redis << std::endl;
	std::cout << sat->getReactionWheelsBlockMomentum() << std::endl;
	redis = redis + sat->discharge('r');
	std::cout << sat->getReactionWheelsBlockMomentum() << std::endl;
	std::cout << redis << std::endl;
}

void Test::thrustersTest()
{
	Satellite sat;
	sat.setMass(6600.0);
	std::vector<AttitudeController> thrusters(6);

	thrusters[0] = AttitudeController(PositionVector({1.5, 1.5, 0.0}), 0.0, PositionVector({0.0, 220.0, 0.0}), 1000.0);
	thrusters[1] = AttitudeController(PositionVector({-1.5, -1.5, 0.0}), 0.0, PositionVector({0.0, 220.0, 0.0}), 1000.0);
	thrusters[2] = AttitudeController(PositionVector({1.5, 0.0, 1.5}), 0.0, PositionVector({0.0, 0.0, 220.0}), 1000.0);
	thrusters[3] = AttitudeController(PositionVector({-1.5, 0.0, -1.5}), 0.0, PositionVector({0.0, 0.0, 220.0}), 1000.0);
	thrusters[4] = AttitudeController(PositionVector({0.0, 1.5, 1.5}), 0.0, PositionVector({0.0, 0.0, 220.0}), 1000.0);
	thrusters[5] = AttitudeController(PositionVector({0.0, -1.5, -1.5}), 0.0, PositionVector({0.0, 0.0, 220.0}), 1000.0);

	sat.setThrusters(thrusters);
	std::cout << sat.getVelocity() << std::endl;
	sat.setThrustersMomentum(PositionVector({1000.0, 2000.0, 3000.0}), 1.0);
	std::cout << sat.getThrustersMomentum() << std::endl;
	for (int i=0; i < 6; i++)
	{
		std::cout << sat.getThrusters()[i].getMass() << std::endl;
	}
	std::cout << sat.getVelocity() << std::endl;
}

void Test::solarPanelsTest()
{
	std::string filename = "../Test input.txt";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double os;

	if (Input::read_input_file(filename, sat, time, interval, step, os))
	{
		return;
	}

	std::vector<Polygon> solar_panels = satellite.getSolarPanels();
	for (int i=0; i<solar_panels.size(); i++)
	{
		std::cout << solar_panels[i].getNormal() << std::endl;
	}

	PositionVector s({1.0, 0.0, 0.0});
	satellite.rotateSolarPanels(s);
	solar_panels = satellite.getSolarPanels();
	for (int i=0; i<solar_panels.size(); i++)
	{
		std::cout << solar_panels[i].getNormal() << std::endl;
	}

	s = -1.0 * s;
	satellite.rotateSolarPanels(s);
	solar_panels = satellite.getSolarPanels();
	for (int i=0; i<solar_panels.size(); i++)
	{
		std::cout << solar_panels[i].getNormal() << std::endl;
	}

	s = PositionVector({0.0, 0.0, 1.0});
	satellite.rotateSolarPanels(s);
	solar_panels = satellite.getSolarPanels();
	for (int i=0; i<solar_panels.size(); i++)
	{
		std::cout << solar_panels[i].getNormal() << std::endl;
	}

	s = PositionVector({0.707, -0.707, 0.0});
	satellite.rotateSolarPanels(s);
	solar_panels = satellite.getSolarPanels();
	for (int i=0; i<solar_panels.size(); i++)
	{
		std::cout << solar_panels[i].getNormal() << std::endl;
	}

	s = PositionVector({0.5777350, -0.5777350, 0.5777350});
	satellite.rotateSolarPanels(s);
	solar_panels = satellite.getSolarPanels();
	for (int i=0; i<solar_panels.size(); i++)
	{
		std::cout << solar_panels[i].getNormal() << std::endl;
	}
}

void Test::fingMmZeroSolarPressureAttitude()
{
	std::string input_filename = "/media/alexey/Disk/asc/solar_pressure/simulations/mm_json.txt";
	std::string vtk_filename = "/media/alexey/Disk/asc/solar_pressure/КЭМ/vtk/ТЭ6_polygons.vtk";

	std::vector<Polygon> polygons;
	if (Input::read_vtk_file(vtk_filename, polygons))
	{
		return;
	}

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double output_step;
	bool screen_check = true;
	
	if (Input::read_json_file(input_filename, sat, time, interval, step, output_step, screen_check))
	{
		std::cout << "Smth went wrong while reading file " << input_filename << std::endl;
		return;
	}

	sat->setPolygons(polygons);
	const std::string savefile = FILENAMES::ephemeris_filename;
	
	Astrometry::get_ephemeris();
	Astrometry::rotationMatrices(t);
	
	Quaternion sat_quat = satellite.getQuaternion();
	std::vector<PositionVector> normals = satellite.getShield6Normals();

	PositionVector axis = sat_quat * PositionVector({-1.0, 0.0, 0.0});
	double angle = 5.0 * M_PI / 180.0;

	Quaternion rot_quat {cos(angle / 2.0), axis[0]*sin(angle / 2.0), axis[1]*sin(angle / 2.0), axis[2]*sin(angle / 2.0)}; // rotate 5 degrees
	PositionVector tor, sun_pos;
	PositionVector r;

	Torques::allTorques(satellite, t);
	r = satellite.getPosition() - satellite.getSunPosition();
	r = r / r.norm();

	PositionVector bottom = sat_quat * normals[1];

	std::cout << "y: " << r << " bot normal: " << bottom / bottom.norm() << std::endl;

	for (int i = 0; i < 0; i++)
	{
		tor = Torques::allTorques(satellite, t);
		r = satellite.getPosition() - satellite.getSunPosition();
		r = r / r.norm();


		std::cout << "(r, top) = " << r.dot(sat_quat * normals[0]) << '\t' << "(r, bot) = " << r.dot(sat_quat * normals[1]) << " ";
		std::cout << sat_quat << ": " << tor << std::endl;
		sat_quat = rot_quat * sat_quat;
		satellite.setQuaternion(sat_quat);
	}

}

void Test::checkNan()
{
	Time t = Time(2012, 1, 1, 0, 0, 0.0);

	Astrometry::get_ephemeris();
	Astrometry::EOP(t);
	Astrometry::rotationMatrices(t);

	Forces::setGravityOrder(1500);
	Forces::check2000nan();
}

void Test::srplibTest()
{
	SRPEngine engine("/media/alexey/Disk1/asc/solar_pressure/srp_test_prjct/data_hdf");        // scans folder for .h5 files


    engine.setSunDirection(1.0, 0.0, 0.0);      // sun along +X (body frame)
    engine.setMaxReflections(2);                // 2 specular bounces
    engine.setGridStep(0.05);                   // 5 cm cell (PixelGrid* only)

    engine.dataset().articulate("SolarPanel_L", JointConfig::rotY(M_PI / 4));

    SRPResult r = engine.compute(SRPMethod::CentroidCPU);

    std::printf("F = [%.3e, %.3e, %.3e] N\n",
                r.total_force[0], r.total_force[1], r.total_force[2]);
    std::printf("M = [%.3e, %.3e, %.3e] N*m\n",
                r.total_moment[0], r.total_moment[1], r.total_moment[2]);

    // For coloured 3-D visualisation of shadows + reflections:
    engine.computeViz(SRPMethod::CentroidCPU);
    engine.visualizeLastResult();
}