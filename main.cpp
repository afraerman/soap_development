#include "stdafx.h"

void quickstart();
void multiple_satellites();
void solarCoordinates();
void read_old_format();
void euler_angles();
void solarFile();

int main()
{
	//Test::srplibTest();
	//solarCoordinates();
	quickstart();
	//euler_angles();
	//multiple_satellites();
	return 0;
}


void quickstart()
{
	std::string input_filename;
	//std::cout << "Enter input parameters filename: ";
	std::cin >> input_filename;
	//input_filename = "/media/alexey/Disk1/asc/solar_pressure/simulations/mm.json";

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double output_step;
	
	if (Input::read_json_file(input_filename, sat, time, interval, step, output_step))
	{
		std::cout << "Smth went wrong while reading file " << input_filename << std::endl;
		return;
	}

	FullMotionIntegrator fullmotion(sat, time, interval, step, output_step, false);
	fullmotion.AttitudeIntegrator::integrate();

	return;
}

void multiple_satellites()
{
	std::string input_filename;
	std::vector<std::string> filenames;

	std::cout << "Enter filename: ";
	std::cin >> input_filename;

	if (Input::read_multiple_satellites_filenames(input_filename, filenames))
	{
		std::cout << "No such filename " << input_filename << std::endl;
		return;
	}

	Satellite satellite;
	Satellite* sat = &satellite;
	Time t;
	Time* time = &t;
	double interval;
	double step;
	double output_step;
	

	for (auto filename : filenames)
	{
		
		if (Input::read_json_file(filename, sat, time, interval, step, output_step))
		{
			std::cout << "Smth went wrong while reading file " << input_filename << std::endl;
			return;
		}

		FullMotionIntegrator fullmotion(sat, time, interval, step, output_step, false);
		fullmotion.AttitudeIntegrator::integrate();
		satellite.set_to_default();
		
	}


}

void solarCoordinates()
{
	Time time(2031, 9, 01, 0, 0, 0.0);
	double state[6];
	double lt;

	std::string tlsfilename = "../Files//naif0012.tls";
	std::string ephfilename = "../Files/de440.bsp";

	ConstSpiceChar* tls = tlsfilename.c_str();
	furnsh_c(tls);

	// Ephemeris
	ConstSpiceChar* eph = ephfilename.c_str();
	furnsh_c(eph);

	SpiceDouble et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	std::cout << "At time " << time << " geocentric sun_pos:\n" << sun_pos  << " km" << std::endl;
	std::cout << "At time " << time << " normalized geocentric sun_pos:\n" << sun_pos / sun_pos.norm() << std::endl;

	/*
	PositionVector sat_pos({292875.512303868017625, -1555016.957377417711541, -554100.582155641284771});
	
	PositionVector v0({1.0, 0.0, 0.0});
	PositionVector v1 = sun_pos - sat_pos;

	Quaternion q(v0, v1);
	std::cout << "Quaternion positioning Ox to the Sun: " << q << std::endl;

	time = Time(2030, 02, 01, 0, 0, 0.0);
	et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	sun_pos = PositionVector(std::vector<double>{state[0], state[1], state[2]});
	std::cout << "At time " << time << " geocentric sun_pos:\n" << sun_pos  << " km" << std::endl;
	std::cout << "At time " << time << " normalized geocentric sun_pos:\n" << sun_pos / sun_pos.norm() << std::endl;
	*/
}

void solarFile()
{
	Time time(2030, 01, 01, 0, 0, 0.0);
	double state[6];
	double lt;

	std::string tlsfilename = "../Files//naif0012.tls";
	std::string ephfilename = "../Files/de440.bsp";

	ConstSpiceChar* tls = tlsfilename.c_str();
	furnsh_c(tls);

	// Ephemeris
	ConstSpiceChar* eph = ephfilename.c_str();
	furnsh_c(eph);

	SpiceDouble et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	double time_step = 1800.0;
	std::ofstream solar("/media/alexey/Disk1/asc/arka/simulation/solar_position.txt");
	if (!solar.is_open())
	{
		std::cout << "Can't create a solar file with such path: " << "/media/alexey/Disk1/asc/arka/simulation/solar_position.txt" << std::endl;
		return;
	}

	for (int i = 0; i < 24720; i++)
	{
		solar << time << '\t' << state[0] << '\t' << state[1] << '\t' << state[2] << std::endl;
		time += time_step;
		et = time.ET();
		spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);
	}

	solar.close();
	return;
}

void read_old_format()
{
	std::string filename = "/media/alexey/Disk1/GitHub/Diploma/linux_prjct/mysat_input.txt";

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
	
	//sat->setNewPolygons();
	const std::string savefile = FILENAMES::ephemeris_filename;
	//Torques::get_thrusters_activation_times();
	//Torques::get_magnets_activation_times();

	FullMotionIntegrator fullmotion(sat, time, interval, step, output_step, false);
	fullmotion.AttitudeIntegrator::integrate(savefile);
	std::cout << *time << '\t' << satellite.getPosition() << std::endl;
}

void euler_angles()
{
	double psi, theta, phi;

	psi = 70.4053;
	theta = -91.3949;
	phi = -126.846;

	Matrix m = Matrix({psi, theta, phi});
	psi *= -1.0; theta *= -1.0; phi *= -1.0;
	Matrix m2 = Matrix({psi, theta, phi});

	std::cout << m << std::endl;
	std::cout << m2 << std::endl;
}