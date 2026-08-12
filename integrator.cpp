#include "stdafx.h"

Integrator::Integrator()
{
	*satellite = Satellite();
	*time = Time();
	interval = 12000.0;
	step = -1.0;
	autostep = true;
	tolerance = 1e-5;
	from_the_start = true;
	elapsed_time = 0.0;
	output_step = 0.0;
	enable_screen_check = true;
}

Integrator::Integrator(Satellite* sat, Time* t, double i, double st, double ost, bool astep, double tol, bool screen_check)
{
	satellite = sat;
	time = t;
	interval = i;
	step = st;
	autostep = astep;
	tolerance = tol;
	from_the_start = true;
	elapsed_time = 0.0;
	output_step = ost;
	enable_screen_check = screen_check;

	if (step <= 0) autoStepOn();
}

void Integrator::autoStepOn()
{
	autostep = true;
}

void Integrator::setStep(double st)
{
	step = st;
	autostep = false;
}

void Integrator::setOutputStep(double ost)
{
	output_step = ost;
}

void Integrator::setInterval(double interv)
{
	interval = interv;
}

double Integrator::getInterval()
{
	return interval;
}

Satellite* Integrator::getSatellite()
{
	return satellite;
}

Time* Integrator::getTime()
{
	return time;
}

void Integrator::make_ephemeris_header(std::ofstream& os)
{
	os << "header:" << std::endl;
	os << "\tglobal_attibutes:"  << std::endl;
	os << "\t\tacknowledgement: this ephemeris file was created using SOAP software of ASC LPI RSSI" << std::endl;

	os << "\t\tsoftware:" << std::endl;
	os << "\t\t\tname: SOAP (Spacecraft Orbit and Attitude Prediction)" << std::endl;
	os << "\t\t\tversion: " << std::endl;
	os << "\t\t\tcommit: " << std::endl;
	os << "\t\t\tinstitution: Лаборатория баллистико-навигационного обеспечения космических проектов, Астрокосмический центр ФИАН" << std::endl;
	os << "\t\t\tauthors:" << std::endl;
	os << "\t\t\t\t- Фраерман А.В. — разаботчик" << std::endl;
	os << "\t\t\t\t- Запевалин П.Р. — научный руководитель" << std::endl;
	os << "\t\t\treferences:" << std::endl;
	os << "\t\t\t\t- <статья, где описан комплекс или его модели>" << std::endl;
	os << "\t\t\t\t- <работа, где SOAP использован для расчётов>" << std::endl;
	os << "\t\t\tcontact: fraerman@asc.rssi.ru" << std::endl;
	os << "\t\t\tlicense: <лицензия>" << std::endl;

	os << "\tvariables:" << std::endl;
	os << "\t\t- timestamp:" << std::endl;
	os << "\t\t\tcomment: 1st column" << std::endl;
	os << "\t\t\tdescription: epoch\n";
	os << "\t\t\tunits: YYYY-MM-DDTHH:MM:SS.S" << std::endl;
	
	os << "\t\t- coord_x:" << std::endl;
	os << "\t\t\tcomment: 2nd column\n";
	os << "\t\t\tdescription: X coordinate in geocentric inertial frame\n";
	os << "\t\t\tunits: km\n";
	
	os << "\t\t- coord_y:" << std::endl;
	os << "\t\t\tcomment: 3d column\n";
	os << "\t\t\tdescription: Y coordinate in geocentric inertial frame\n";
	os << "\t\t\tunits: km\n";

	os << "\t\t- coord_z:" << std::endl;
	os << "\t\t\tcomment: 4th column\n";
	os << "\t\t\tdescription: Z coordinate in geocentric inertial frame\n";
	os << "\t\t\tunits: km\n";

	os << "\t\t- vel_x:" << std::endl;
	os << "\t\t\tcomment: 5th column\n";
	os << "\t\t\tdescription: X velocity component in geocentric inertial frame\n";
	os << "\t\t\tunits: km / s\n";

	os << "\t\t- vel_y:" << std::endl;
	os << "\t\t\tcomment: 6th column\n";
	os << "\t\t\tdescription: Y velocity component in geocentric inertial frame\n";
	os << "\t\t\tunits: km / s\n";

	os << "\t\t- vel_z:" << std::endl;
	os << "\t\t\tcomment: 7th column\n";
	os << "\t\t\tdescription: Z velocity component in geocentric inertial frame\n";
	os << "\t\t\tunits: km / s\n";

	os << "\t\t- quat_w:" << std::endl;
	os << "\t\t\tcomment: 8th column\n";
	os << "\t\t\tdescription: scalar component of rotation quaternion\n";

	os << "\t\t- quat_x:" << std::endl;
	os << "\t\t\tcomment: 9th column\n";
	os << "\t\t\tdescription: i-component of rotation quaternion\n";

	os << "\t\t- quat_y:" << std::endl;
	os << "\t\t\tcomment: 10th column\n";
	os << "\t\t\tdescription: j-component of rotation quaternion\n";

	os << "\t\t- quat_z:" << std::endl;
	os << "\t\t\tcomment: 11th column\n";
	os << "\t\t\tdescription: k-component of rotation quaternion\n";

	os << "#End of header\n";
}

void Integrator::make_telemetry_header(std::ofstream& os)
{
	os << "header:\n";
	os << "\tglobal_attibutes:\n";
	os << "\t\tacknowledgement: this telemetry file was created using SOAP software of ASC LPI RSSI\n";

	os << "\tvariables:\n";
	os << "\t\t- timestamp:" << std::endl;
	os << "\t\t\tcomment: 1st column" << std::endl;
	os << "\t\t\tdescription: epoch\n";
	os << "\t\t\tunits: YYYY-MM-DDTHH:MM:SS.S" << std::endl;

	os << "\t\t- torque_x:\n";
	os << "\t\t\tcomment: 2nd column\n";
	os << "\t\t\tdescription: x-component of external torque in local reference frame\n";
	os << "\t\t\tunits: N * m\n";

	os << "\t\t- torque_y:\n";
	os << "\t\t\tcomment: 3rd column\n";
	os << "\t\t\tdescription: y-component of external torque in local reference frame\n";
	os << "\t\t\tunits: N * m\n";

	os << "\t\t- torque_z:\n";
	os << "\t\t\tcomment: 4th column\n";
	os << "\t\t\tdescription: z-component of external torque in local reference frame\n";
	os << "\t\t\tunits: N * m\n";

	int column = 5;

	if ((Control::getControlOrder()[0] == 'r') || (Control::getControlOrder()[1] == 'r'))
	{
		int rws = (int)satellite->getReactionWheelsBlock().size() * 4;
		for (int i = 0; i < rws; i++)
		{
			os << "\t\t- rw_" << i + 1 << ":\n";
			os << "\t\t\tcomment: " << column << "th column\n";
			os << "\t\t\tdescription: angular momentum of " << i << "th reaction wheel\n";
			os << "\t\t\tunits: N * m * s\n";
			column++;
		}

		os << "\t\t- rw_x:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: x-component of total angular momentum of reaction wheels in local reference frame\n";
		os << "\t\t\tunits: N * m * s\n";
		column++;

		os << "\t\t- rw_y:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: y-component of total angular momentum of reaction wheels in local reference frame\n";
		os << "\t\t\tunits: N * m * s\n";
		column++;

		os << "\t\t- rw_z:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: z-component of total angular momentum of reaction wheels in local reference frame\n";
		os << "\t\t\tunits: N * m * s\n";
		column++;
	}

	if ((Control::getControlOrder()[0] == 'g') || (Control::getControlOrder()[1] == 'g'))
	{
		os << "\t\t- gyr_x:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: x-component of total gyrostats angular momentum in local reference frame\n";
		os << "\t\t\tunits: N * m * s\n";
		column++;

		os << "\t\t- gyr_y:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: y-component of total gyrostats angular momentum in local reference frame\n";
		os << "\t\t\tunits: N * m * s\n";
		column++;

		os << "\t\t- gyr_z:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: z-component of total gyrostats angular momentum in local reference frame\n";
		os << "\t\t\tunits: N * m * s\n";
		column++;
	}

	if ((Control::getControlOrder()[0] == 'm') || (Control::getControlOrder()[1] == 'm'))
	{
		os << "\t\t- magn_x:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: x-component of magnetorquers total magnetic momentum in local reference frame\n";
		os << "\t\t\tunits: A * m^2\n";
		column++;

		os << "\t\t- magn_y:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: y-component of magnetorquers total magnetic momentum in local reference frame\n";
		os << "\t\t\tunits: A * m^2\n";
		column++;

		os << "\t\t- magn_z:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: z-component of magnetorquers total magnetic momentum in local reference frame\n";
		os << "\t\t\tunits: A * m^2\n";
		column++;
	}

	auto thrusters = satellite->getThrusters();

	for (int i = 0; i < (int)thrusters.size(); i++)
	{
		os << "\t\t- thr_:" << i+1 << "\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: mass of fuel used by " << i+1 << "th thruster\n";
		os << "\t\t\tunits: kg\n";
		column++;
	}
	/*
	os << "\t\t- thr_y:\n";
	os << "\t\t\tcomment: " << column << "th column\n";
	os << "\t\t\tdescription: y-component of thrusters total momentum in local reference frame\n";
	os << "\t\t\tunits: N * m\n";
	column++;

	os << "\t\t- thr_z:\n";
	os << "\t\t\tcomment: " << column << "th column\n";
	os << "\t\t\tdescription: z-component of thrusters total momentum in local reference frame\n";
	os << "\t\t\tunits: N * m\n";
	column++;
	*/
	os << "\t\t- ang_x:\n";
	os << "\t\t\tcomment: " << column << "th column\n";
	os << "\t\t\tdescription: x-component of satellite angular momentum in local reference frame\n";
	os << "\t\t\tunits: N * m * s\n";
	column++;

	os << "\t\t- ang_y:\n";
	os << "\t\t\tcomment: " << column << "th column\n";
	os << "\t\t\tdescription: y-component of satellite angular momentum in local reference frame\n";
	os << "\t\t\tunits: N * m * s\n";
	column++;

	os << "\t\t- ang_z:\n";
	os << "\t\t\tcomment: " << column << "th column\n";
	os << "\t\t\tdescription: z-component of satellite angular momentum in local reference frame\n";
	os << "\t\t\tunits: N * m * s\n";
	column++;

	if (Torques::account_for_solar_pressure)
	{
		os << "\t\t- center_of_pressure:\n";
		os << "\t\t\tcomment: " << column << "th column\n";
		os << "\t\t\tdescription: center of solar pressure\n";
		os << "\t\t\tunits: meters\n";
		column++;
	}

	os << "#End of header\n";
}

void Integrator::integrate(std::string savefilename)
{
	if (output_step <= 0) output_step = interval;
	double output_time = output_step;
	int exponent = 0;
	Satellite var_sat1, var_sat2;
	Time var_time1, var_time2;

	std::ofstream telemetry(FILENAMES::telemetry_filename);
	if (!telemetry.is_open())
	{
		std::cerr << "\033[31m#013 Can't create a telemetry file with such path: " << FILENAMES::telemetry_filename << "\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return;
	}
	make_telemetry_header(telemetry);
	
	if (savefilename == "")
		savefilename = FILENAMES::ephemeris_filename;
	std::ofstream output(savefilename);
	if (!output.is_open())
	{
		std::cerr << "\033[31m#012 Can't create an output file with such path: " << savefilename << "\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return;
	}
	make_ephemeris_header(output);
	output << std::setprecision(17);
	telemetry << std::setprecision(17);
	output << *time << '\t' << satellite->getPosition() << '\t' << satellite->getVelocity() << '\t' << satellite->outputQuaternion() << std::endl;

	double h = (step > 0.0) ? step : output_step; // REVISE: interval / 100.0;
	// double t = 0.0;
	if (autostep)
	{
		while (elapsed_time < interval)
		{
			var_sat1 = *satellite;
			var_time1 = *time;

			// screen check
			if (enable_screen_check)
			{
				if ((int)elapsed_time / (100 * (int)h) != (int)(elapsed_time - h) / (100 * (int)h))
					std::cout << *time << std::endl;
			}

			// final step to match interval
			if (elapsed_time + h > interval) h = interval - elapsed_time;

			integrationMethod(var_sat1, var_time1, h, from_the_start);
			var_sat1.update();
			if (integration_error)
			{
				std::cout << "integration terminated due to the above error" << std::endl;
				output.close();
				telemetry.close();
				return;
			}

			var_sat2 = *satellite;
			var_time2 = *time;
			from_the_start = true;

			h /= 2.0;
			exponent++;

			for (int i = 0; i < 2; i++)
			{
				integrationMethod(var_sat2, var_time2, h, from_the_start);
				var_sat2.update();
				if (integration_error)
				{
					std::cout << "integration terminated due to the above error" << std::endl;
					output.close();
					telemetry.close();
					return;
				}
				from_the_start = false;
				var_time2 += h;
			}

			// Поиск нужного h
			while (comparison(var_sat1, var_sat2) > tolerance)
			{
				std::cout << "h = " << h*2 << " is too big, decreasing" << std::endl;
				// Значит нужно уменьшать шаг. Попробуем сделать одно движение с h и два с h/2

				// шаг с h
				var_sat1 = *satellite;
				var_time1 = *time;
				from_the_start = true;
				integrationMethod(var_sat1, var_time1, h, from_the_start);
				var_sat1.update();
				if (integration_error)
				{
					std::cout << "integration terminated due to the above error" << std::endl;
					output.close();
					telemetry.close();
					return;
				}

				var_sat2 = *satellite;
				var_time2 = *time;
				from_the_start = true;

				// 2 шага с h/2
				h /= 2.0;
				exponent++;
				
				for (int i = 0; i < 2; i++)
				{
					integrationMethod(var_sat2, var_time2, h, from_the_start);
					var_sat2.update();
					if (integration_error)
					{
						std::cout << "integration terminated due to the above error" << std::endl;
						output.close();
						telemetry.close();
						return;
					}
					from_the_start = false;
					var_time2 += h;
				}
			}

			// Выбор размера шага: раз решения для последнего h и предпоследнего h неразличимы, то берём больший из них, то есть
			h *= 2.0;
			
			// Ровно настолько сейчас "улетел" var_sat2, поэтому
						
			// Transition to next step
			elapsed_time += h;
			*satellite = var_sat2;
			*time += h;

			Astrometry::rotationMatrices(*time);

			// Чтобы шаг имел шанс возрасти, давайте сделаем вот такой финт:
			h = std::min(h*2, output_time - elapsed_time);

			// std::cout <<  "Current step: " << h << std::endl;

			// Это теперь наш шаг, который мы хотим шагнуть, он точно является долей output_step, поэтому опасности "перешагнуть" точку вывода нет
			exponent = 0.0;

			// OUTPUT
			if ((elapsed_time == output_time) || (elapsed_time == interval))
			{
				output << *time << '\t' << satellite->getPosition() << '\t' << satellite->getVelocity() << '\t' << satellite->outputQuaternion() << std::endl;
				output_time += output_step;
			}

			telemetry << *time << '\t' << Torques::allTorques(*satellite, *time) << '\t';
			// если есть блок маховиков
			if ((Control::getControlOrder()[0] == 'r') || (Control::getControlOrder()[1] == 'r'))
			{
				telemetry << satellite->getReactionWheelsBlockMomentum(-1) << '\t' << satellite->getReactionWheelsBlockMomentum3d() << '\t';
			}
			// если есть независимые маховики
			else if ((Control::getControlOrder()[0] == 'g') || (Control::getControlOrder()[1] == 'g'))
			{
				telemetry <<  satellite->getGyrostatsMomentum() << '\t';
			}
			// если есть КМИО
			if ((Control::getControlOrder()[0] == 'm') || (Control::getControlOrder()[1] == 'm'))
			{
				telemetry << satellite->getMagneticMomentum() << '\t';
			}

			telemetry << satellite->getThrustersMomentum() << '\t' << satellite->getAngularMomentum() << std::endl;
		}
	}
	else
	{
		h = step;
		while (elapsed_time < interval)
		{
			// screen check
			if (enable_screen_check)
			{	
				if ((int)elapsed_time / 100 != (int)(elapsed_time - h) / 100)
					std::cout << *time << std::endl;
			}
			
			if (elapsed_time + h > interval) h = interval - elapsed_time;
			integrationMethod(*satellite, *time, h, from_the_start);
			satellite->update();
			elapsed_time -= satellite->getSetbackTime();
			elapsed_time = std::floor(elapsed_time / step) * step;
			output_time -= std::floor(satellite->getSetbackTime() / output_step) * output_step;
			if (integration_error)
			{
				std::cout << "integration terminated due to the above error" << std::endl;
				output.close();
				telemetry.close();
				return;
			}
			from_the_start = false;
			// t += h;
			elapsed_time += h;
			*time += h;

			Astrometry::rotationMatrices(*time);
			

			// OUTPUT
			if (elapsed_time == output_time)
			{
				output << *time << '\t' << satellite->getPosition() << '\t' << satellite->getVelocity() << '\t' << satellite->outputQuaternion() << std::endl;
				output_time += output_step;
			}
			
			if (satellite->make_telemetry)
			{
				telemetry << *time << '\t' << Torques::allTorques(*satellite, *time) << '\t';
				// если есть блок маховиков
				if ((Control::getControlOrder()[0] == 'r') || (Control::getControlOrder()[1] == 'r'))
				{
					telemetry << satellite->getReactionWheelsBlockMomentum(-1) << '\t' << satellite->getReactionWheelsBlockMomentum3d() << '\t';
				}
				// если есть независимые маховики
				else if ((Control::getControlOrder()[0] == 'g') || (Control::getControlOrder()[1] == 'g'))
				{
					telemetry <<  satellite->getGyrostatsMomentum() << '\t';
				}
				// если есть КМИО
				if ((Control::getControlOrder()[0] == 'm') || (Control::getControlOrder()[1] == 'm'))
				{
					telemetry << satellite->getMagneticMomentum() << '\t';
				}

				telemetry << satellite->getThrustersMomentum() << '\t' << satellite->getAngularMomentum();

				if (Torques::account_for_solar_pressure)
				{
					telemetry << '\t' << Torques::getCenterOfPressure();
				}
				telemetry << std::endl;				

				satellite->make_telemetry = false;
			}
					
			//telemetry << *time << '\t' << satellite->getAngularMomentum() << '\t' << mul(Matrix(satellite->getQuaternion()).transpose(), Torques::getMagneticField(satellite->getPosition(), *time)) << std::endl;
		}
	}
	output.close();
	telemetry.close();

	std::ofstream outputinfo(FILENAMES::output_info_filename);
	if (!outputinfo.is_open())
	{
		std::cerr << "\033[31m#014 Can't create an outputinfo file with filename " << FILENAMES::output_info_filename << "\033[0m\n";
		std::exit(EXIT_FAILURE);
		return;
	}
	std::setprecision(17);
	//outputinfo << "START_TIME" << '\t' << "MODE" << '\t' << "FLAG" << '\t' << "DURATION" << '\t' << "MOMENTUM" << '\t' << "FUEL" << std::endl;
	//outputinfo << std::format("{:<21}    {:<4}    {:<4}    {:<17}    {:<53}    {:<17}\n", "START_TIME", "MODE", "FLAG", "DURATION", "MOMENTUM", "FUEL");
	outputinfo << std::left << std::setw(25) << std::setfill(' ') << "START_TIME";
	outputinfo << std::left << std::setw(8) << std::setfill(' ') << "MODE";
	outputinfo << std::left << std::setw(8) << std::setfill(' ') << "FLAG";
	outputinfo << std::left << std::setw(23) << std::setfill(' ') << "DURATION";
	outputinfo << std::left << std::setw(23) << std::setfill(' ') << "ACTUAL_DURATION";
	outputinfo << std::left << std::setw(59) << std::setfill(' ') << "MOMENTUM";
	outputinfo << std::left << std::setw(17) << std::setfill(' ') << "FUEL";
	outputinfo <<  std::endl;
	auto oi = satellite->getOutputInfo();
	for (auto elem: oi)
	{
		//outputinfo << elem.start_time << '\t' << elem.mode << '\t' << (int)elem.isObserved << '\t' << elem.duration << '\t' << elem.momentum << '\t' << elem.fuel << std::endl;
		//outputinfo << std::format("{:<21}    {:<4}    {:<4}    {:<53}    {:<17}\n", elem.start_time, elem.mode, (int)elem.isObserved, elem.duration, elem.momentum, elem.fuel);
		outputinfo << std::left << std::setw(25) << std::setfill(' ') << elem.start_time.toString();
		outputinfo << std::left << std::setw(8) << std::setfill(' ') << elem.mode;
		outputinfo << std::left << std::setw(8) << std::setfill(' ') << elem.isObserved;
		outputinfo << std::left << std::setw(23) << std::setfill(' ') << elem.duration;
		outputinfo << std::left << std::setw(23) << std::setfill(' ') << elem.sd_time;
		outputinfo << std::left << std::setw(59) << std::setfill(' ') << elem.momentum.toString();
		outputinfo << std::left << std::setw(17) << std::setfill(' ') << elem.fuel;
		outputinfo << std::endl;
	}
	outputinfo.close();
}

Integrator::~Integrator()
{

}
