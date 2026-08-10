#include "stdafx.h"

Satellite::Satellite()
{
	pv = StateVector();
	mass = 10.0;
	inertia_tensor = Matrix(3, 3, std::vector<std::vector<double>>{ {10.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 10.0}});
	angular_velocity = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	//quaternion = boost::math::quaternion<double>{ 1.0, 0.0, 0.0, 0.0 };
	quaternion = Quaternion();
	magnetic_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	counterrotate = false;
	angular_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	target = "none";
	need_for_update = false;
}

Satellite::Satellite(StateVector& p, const Matrix& inertia)
{
	pv = p;
	inertia_tensor = inertia;

	mass = 10.0;
	angular_velocity = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	// quaternion = boost::math::quaternion<double>{ 1.0, 0.0, 0.0, 0.0 };
	quaternion = Quaternion();
	magnetic_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	counterrotate = false;
	angular_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	target = "none";
	need_for_update = false;
}

//Satellite::Satellite(StateVector& p, const Matrix& inertia, double m, const std::vector<Polygon>& pols, const boost::math::quaternion<double>& quat)
Satellite::Satellite(StateVector& p, const Matrix& inertia, double m, const std::vector<Polygon>& pols, const Quaternion& quat)
{
	pv = p;
	mass = m;
	polygons = pols;
	inertia_tensor = inertia;
	quaternion = quat;
	angular_velocity = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	magnetic_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	counterrotate = false;
	angular_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	target = "none";
	need_for_update = false;
}

StateVector Satellite::getState() const
{
	return pv;
}
PositionVector Satellite::getPosition() const
{
	return pv.getPosition();
}
PositionVector Satellite::getVelocity() const
{
	return pv.getVelocity();
}
double Satellite::getMass() const
{
	return mass;
}
Matrix Satellite::getInertiaTensor() const
{
	return inertia_tensor;
}
std::vector<Polygon> Satellite::getPolygons() const
{
	return polygons;
}
std::vector<Polygon> Satellite::getSolarPanels() const
{
	return solar_panels;
}
std::vector<AttitudeController> Satellite::getGyrostats() const
{
	return gyrostats;
}
std::vector<AttitudeController> Satellite::getMagnetorquers() const
{
	return magnetorquers;
}
std::vector<AttitudeController> Satellite::getThrusters() const
{
	return thrusters;
}
std::vector<AttitudeController> Satellite::getCorrectionThrusters() const
{
	return correction_thrusters;
}
std::vector<std::vector<ReactionWheel>> Satellite::getReactionWheelsBlock() const
{
	return reaction_wheels;
}
PositionVector Satellite::getPulseEngineLocation() const
{
	return pulse_engine_location;
}
//boost::math::quaternion<double> Satellite::getQuaternion() const
Quaternion Satellite::getQuaternion() const
{
	return quaternion;
}
PositionVector Satellite::outputQuaternion() const
{
	double w, x, y, z;
	/*
	w = quaternion.R_component_1();
	x = quaternion.R_component_2();
	y = quaternion.R_component_3();
	z = quaternion.R_component_4();
	*/
	w = quaternion.get_w();
	x = quaternion.get_x();
	y = quaternion.get_y();
	z = quaternion.get_z();
	return PositionVector({w, x, y, z});
}
PositionVector Satellite::getGyrostatsMomentum() const
{
	PositionVector gyr_mom;
	gyr_mom[0] = 0.0; gyr_mom[1] = 0.0; gyr_mom[2] = 0.0;

	for (int i = 0; i < gyrostats.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (gyrostats[i].getLimit()[j])
			{
				gyr_mom[j] += gyrostats[i].getStoredMomentum();
			}
		}
	}
	return gyr_mom;
}
PositionVector Satellite::getAngularVelocity() const
{
	return angular_velocity;
}
PositionVector Satellite::getAngularMomentum() const
{
	return angular_momentum;
}
PositionVector Satellite::getMagneticMomentum() const
{
	PositionVector magn_mom;
	magn_mom[0] = 0.0; magn_mom[1] = 0.0; magn_mom[2] = 0.0;

	for (int i=0; i < magnetorquers.size(); i++)
	{
		for (int j=0; j < 3; j++)
		{
			if (magnetorquers[i].getLimit()[j])
			{
				magn_mom[j] += magnetorquers[i].getStoredMomentum();
			}
		}
	}
	return magnetic_momentum + magn_mom;
}
PositionVector Satellite::getReactionWheelsBlockMomentum(int i) const
{
	PositionVector reaction_wheels_momentum({0.0, 0.0, 0.0, 0.0});
	if (reaction_wheels.size() == 0)
	{
		return reaction_wheels_momentum;
	}
	
	if (i == -1)
	{
		PositionVector all_reaction_wheels(std::vector<double>(4*reaction_wheels.size()));
		for (int k = 0; k < reaction_wheels.size(); k++)
		{
			for (int j = 0; j < 4; j++)
			{
				all_reaction_wheels[4 * k + j] = reaction_wheels[k][j].getStoredMomentum();
			}
		}
		return all_reaction_wheels;
	}

	for (int j = 0; j < 4; j++)
	{
		reaction_wheels_momentum[j] = reaction_wheels[i][j].getStoredMomentum();
	}
	return reaction_wheels_momentum;
}
PositionVector Satellite::getReactionWheelsBlockMomentum3d() const
{
	PositionVector mom({0.0, 0.0, 0.0});
	for (int i = 0; i < reaction_wheels.size(); i++)
	{
		PositionVector block_mom = getReactionWheelsBlockMomentum(i);
		if (block_mom[0] == 0.0 && block_mom[1] == 0.0 && block_mom[2] == 0.0 and block_mom[3] == 0.0)
		{
			continue;
		}
		mom = mom + Control::combineReactionWheelsBlockMomentum(block_mom, reaction_wheels[i][0].getAngles(), reaction_wheels[i][0].getApex());
	}
	return mom;
}
PositionVector Satellite::getThrustersMomentum() const
{
	/*
	PositionVector tor({0.0, 0.0, 0.0});
	for (int i = 0; i < thrusters.size(); i++)
	{
		tor = tor + thrusters[i].getLocation().cross(thrusters[i].getLimit()) * thrusters[i].getStoredMomentum();
		
	}
	return tor;
	*/
	// return thrusters_momentum;
	// давайте вернём вектор распределения масс по осям. При этом направление не будет иметь значения
	/*
	PositionVector mass_distribution({0.0, 0.0, 0.0});

	for (auto thruster: thrusters)
	{
		for (int i = 0; i < 3; i++)
		{
			if (thruster.getLimit()[i])
			{
				mass_distribution[i] += thruster.getStoredMomentum();
				break;
			}
		}
	}
	return mass_distribution;
	*/

	// давайте вернём вектор расхода толпива каждым двигателем
	PositionVector mass_distribution(std::vector<double>((int)thrusters.size(), 0.));
	for (int i = 0; i < (int)thrusters.size(); i++)
	{
		mass_distribution[i] = thrusters[i].getStoredMomentum();
	}
	return mass_distribution;
}
bool Satellite::getCounterrotation() const
{
	return counterrotate;
}
std::vector<std::vector<Time>> Satellite::getStopTimes() const
{
	return stop_periods;
}
std::vector<std::vector<Time>> Satellite::getScanTimes() const
{
	return scan_periods;
}
std::vector<std::vector<Time>> Satellite::getDumpTimes() const
{
	return dump_periods;
}
std::vector<std::vector<Time>> Satellite::getSlewTimes() const
{
	return slew_periods;
}
std::vector<Time> Satellite::getPulseTimes() const
{
	return pulse_periods;
}
std::vector<std::vector<Time>> Satellite::getCorrectionTimes() const
{
	return correction_periods;
}
PositionVector Satellite::getTargetMomentum(const Time& t) const
{
	for (int i = 0; i < scan_periods.size(); i++)
	{
		if (t < scan_periods[i][0]) return -1.0 * mul(inertia_tensor, scan_velocities[i]); // mul(this->getInertiaTensor(), scan_velocities[i]);
	}
	return PositionVector({ 0.0, 0.0, 0.0 });
}
//boost::math::quaternion<double> Satellite::getTargetQuaternion(const Time& t) const
Quaternion Satellite::getTargetQuaternion(const Time& t) const
{
	for (int i = 0; i < slew_periods.size(); i++)
	{
		if (t < slew_periods[i][0]) return slew_attitudes[i];
	}
	//return boost::math::quaternion<double>{0.0, 0.0, 0.0, 0.0};
	return Quaternion(0.0, 0.0, 0.0, 0.0);
}
PositionVector Satellite::getPulseAcceleration(const Time& t) const
{
	for (int i = 0; i < pulse_periods.size(); i++)
	{
		if (t == pulse_periods[i]) return pulse_forces[i] / mass;
	}
	return PositionVector({0.0, 0.0, 0.0});
}
Time Satellite::getTargetTime(const Time& t)
{
	Time stop_time(1, 1, 1, 0, 0, 0.0);
	Time scan_time(1, 1, 1, 0, 0, 0.0);
	
	target = "none";

	for (int i = 0; i < stop_periods.size(); i++)
	{
		if (t < stop_periods[i][0])
		{
			stop_time = stop_periods[i][0];
			break;
		}
	}

	for (int i = 0; i < scan_periods.size(); i++)
	{
		if (t < scan_periods[i][0])
		{
			scan_time = scan_periods[i][0];
			break;
		}
	}

	if (stop_time.getYear() != 1)
	{
		if ((scan_time.getYear() != 1) && (scan_time < stop_time))
		{
			target = "scan";
			return scan_time;
		}
		target = "stop";
		return stop_time;
	}
	if (scan_time.getYear() != 1) { target = "scan"; }
	return scan_time;
}
std::string Satellite::getTarget() const
{
	return target;
}
std::string Satellite::getOrbitFilename() const
{
	return orbit_filename;
}
PositionVector Satellite::getSunPosition() const
{
	return sun_position;
}
double Satellite::getDumpDuration() const
{
	// для каждого маховика нужно найти его накопленный момент и взять скорость разгрузки. Из всех полученных времён взять максимальное.
	double dump_time = 0.0;
	double dump_speed, speed;
	for (int rwb = 0; rwb < reaction_wheels.size(); rwb++)
	{
		for (int i = 0; i < 4; i++)
		{
			dump_speed = reaction_wheels[rwb][i].getDumpSpeed();
			speed = reaction_wheels[rwb][i].getStoredMomentum();
			dump_time = std::max(dump_time, speed / dump_speed);
		}
	}
	return dump_time;
}

std::vector<PositionVector> Satellite::getShield6Normals() const
{
	PositionVector top = PositionVector({0.0, 0.0, 0.0});
	PositionVector bottom = PositionVector({0.0, 0.0, 0.0});
	double n_top = 0.0;
	double n_bot = 0.0;

	for (int i = 0; i < polygons.size(); i++)
	{
		if (polygons[i].getPosition()[2] >= 0)
		{
			top += polygons[i].getNormal();
			n_top++;
		}
		else
		{
			bottom += polygons[i].getNormal();
			n_bot++;
		}
	}

	return std::vector<PositionVector>{top, bottom};
}

double Satellite::getSetbackTime() const
{
	return setback_time;
}

std::vector<Satellite::OutputInfo> Satellite::getOutputInfo() const
{
	return all_modes;
}

Satellite::OutputInfo Satellite::getNextTarget() const
{
	if (target_index < (int)all_modes.size())
		return all_modes[target_index];

	OutputInfo oi;
	oi.mode = "none";
	return oi;
}

std::string Satellite::getHdfFile() const
{
	return hdf5_filename;
}

void Satellite::deleteFailedTarget()
{
	Time start_time = all_modes[target_index-1].start_time;
	std::string mode = all_modes[target_index-1].mode;
	if (mode == "scan")
	{
		for (int i = 0; i < (int)scan_periods.size(); i++)
		{
			if (scan_periods[i][0] == start_time)
			{
				scan_periods.erase(scan_periods.begin() + i);
				scan_velocities.erase(scan_velocities.begin() + i);
			}
		}
	}
	else if (mode == "slew")
	{
		for (int i = 0; i < (int)slew_periods.size(); i++)
		{
			if (slew_periods[i][0] == start_time)
			{
				slew_periods.erase(slew_periods.begin() + i);
				slew_attitudes.erase(slew_attitudes.begin() + i);
			}
		}
	}
	else if (mode == "dump")
	{
		for (int i = 0; i < (int)dump_periods.size(); i++)
		{
			if (dump_periods[i][0] == start_time)
			{
				dump_periods.erase(dump_periods.begin() + i);
			}
		}
	}
	else if (mode == "stop")
	{
		for (int i = 0; i < (int)stop_periods.size(); i++)
		{
			if (stop_periods[i][0] == start_time)
			{
				stop_periods.erase(stop_periods.begin() + i);
			}
		}
	}

	std::cerr << "\033[32mMode was deleted, " << (int)all_modes.size() - target_index << " modes left\033[0m" << std::endl;
}

void Satellite::targetFailed()
{
	all_modes[target_index-1].isObserved = false;
	deleteFailedTarget();
}

int Satellite::checkAttitudeModes()
{
	this->sortTransitionPeriods();

	int scans = (int)scan_periods.size();
	int stops = (int)stop_periods.size();
	int dumps = (int)dump_periods.size();
	int slews = (int)slew_periods.size();

	// self-overlaps
	for (int scan = 0; scan < scans - 1; scan++)
	{
		if (scan_periods[scan][1] >= scan_periods[scan + 1][0])
		{
			std::cerr << "\033[31m#24 Scan modes self overlap\033[0m" << std::endl;
			return 1;
		}
	}
	for (int stop = 0; stop < stops - 1; stop++)
	{
		if (stop_periods[stop][1] > stop_periods[stop + 1][0]) 
		{
			std::cerr << "\033[31m#24 Stop modes self overlap\033[0m" << std::endl;
			return 1;
		}
	}
	for (int dump = 0; dump < dumps - 1; dump++)
	{
		if (dump_periods[dump][1] > dump_periods[dump + 1][0]) 
		{
			std::cerr << "\033[31m#24 Dump modes self overlap\033[0m" << std::endl;
			return 1;
		}
	}
	for (int slew = 0; slew < slews - 1; slew++)
	{
		if (slew_periods[slew][1] > slew_periods[slew + 1][0]) 
		{
			std::cerr << "\033[31m#24 Slew modes self overlap\033[0m" << std::endl;
			return 1;
		}
	}

	// scan-stop cross-overlaps
	int i = 0;
	int j = 0;
	while ((i < scans) && (j < stops))
	{
		if ((scan_periods[i][0] >= stop_periods[j][0]) && (scan_periods[i][0] <= stop_periods[j][1])) // scan while stop
		{
			std::cerr << "\033[31m#24 Scan modes overlap with stop modes\033[0m" << std::endl;
			return 1;
		}
		if ((stop_periods[j][0] >= scan_periods[i][0]) && (stop_periods[j][0] <= scan_periods[i][1])) // stop while scan
		{
			std::cerr << "\033[31m#24 Stop modes overlap with scan modes\033[0m" << std::endl;
			return 1;
		}

		if (stop_periods[j][1] == scan_periods[i][0]) // scan right after stop
			return 1;
		if (scan_periods[i][1] == stop_periods[j][0]) // stop right after scan
			return 1;

		// these two don't overlap, go next
		if (scan_periods[i][1] <= stop_periods[j][0]) // scan before stop
			i++;
		else // stop before scan
			j++;
	}

	// dump-stop cross-overlaps
	i = 0;
	j = 0;
	while ((i < dumps) && (j < stops))
	{
		if ((dump_periods[i][0] > stop_periods[j][0]) && (dump_periods[i][0] < stop_periods[j][1])) // dump while stop
		{
			std::cerr << "\033[31m#24 Dump modes overlap with stop modes\033[0m" << std::endl;
			return 1;
		}
		if ((stop_periods[j][0] > dump_periods[i][0]) && (stop_periods[j][0] < dump_periods[i][1])) // stop while dump
		{
			std::cerr << "\033[31m#24 Stop modes overlap with dump modes\033[0m" << std::endl;
			return 1;
		}

		// these two don't overlap, go next
		if (dump_periods[i][1] <= stop_periods[j][0]) // dump before stop
			i++;
		else // stop before dump
			j++;
	}

	// slew-stop cross-overlaps
	i = 0;
	j = 0;
	while ((i < slews) && (j < stops))
	{
		if ((slew_periods[i][0] > stop_periods[j][0]) && (slew_periods[i][0] < stop_periods[j][1])) // slew while stop
		{
			std::cerr << "\033[31m#24 Slew modes overlap with stop modes\033[0m" << std::endl;
			return 1;
		}
		if ((stop_periods[j][0] > slew_periods[i][0]) && (stop_periods[j][0] < slew_periods[i][1])) // stop while slew
		{
			std::cerr << "\033[31m#24 Stop modes overlap with slew modes\033[0m" << std::endl;
			return 1;
		}

		// these two don't overlap, go next
		if (slew_periods[i][1] <= stop_periods[j][0]) // slew before stop
			i++;
		else // stop before slew
			j++;
	}

	// slew-dump cross-overlaps
	i = 0;
	j = 0;
	while ((i < slews) && (j < dumps))
	{
		if ((slew_periods[i][0] > dump_periods[j][0]) && (slew_periods[i][0] < dump_periods[j][1])) // slew while dump
		{
			std::cerr << "\033[31m#24 Slew modes overlap with dump modes\033[0m" << std::endl;
			return 1;
		}
		if ((dump_periods[j][0] > slew_periods[i][0]) && (dump_periods[j][0] < slew_periods[i][1])) // dump while slew
		{
			std::cerr << "\033[31m#24 Dump modes overlap with slew modes\033[0m" << std::endl;
			return 1;
		}

		// these two don't overlap, go next
		if (slew_periods[i][1] <= dump_periods[j][0]) // slew before dump
			i++;
		else // dump before slew
			j++;
	}

	return 0;

}

void Satellite::mergeAttitudeModes()
{
	// merge scan and stop
	int scans = (int)scan_periods.size();
	int stops = (int)stop_periods.size();
	int dumps = (int)dump_periods.size();
	int slews = (int)slew_periods.size();

	int i = 0;
	int j = 0;

	std::vector<OutputInfo> scanstops;

	while ((i < scans) && (j < stops))
	{
		if (scan_periods[i][1] <= stop_periods[j][0]) // scan before stop
		{
			OutputInfo oi;
			oi.start_time = scan_periods[i][0];
			oi.target_momentum = scan_velocities[i];
			oi.mode = "scan";
			oi.duration = scan_periods[i][1] - scan_periods[i][0];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});

			scanstops.push_back(oi);
			i++;
		}
		else
		{
			OutputInfo oi;
			oi.start_time = stop_periods[j][0];
			oi.mode = "stop";
			oi.duration = stop_periods[j][1] - stop_periods[j][0];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});

			scanstops.push_back(oi);
			j++;
		}
	}
	if (j == stops)
	{
		while (i < scans)
		{
			OutputInfo oi;
			oi.start_time = scan_periods[i][0];
			oi.mode = "scan";
			oi.target_momentum = scan_velocities[i];
			oi.duration = scan_periods[i][1] - scan_periods[i][0];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});
			scanstops.push_back(oi);
			i++;
		}
	}
	else if (i == scans)
	{
		while (j < stops)
		{
			OutputInfo oi;
			oi.start_time = stop_periods[j][0];
			oi.mode = "stop";
			oi.duration = stop_periods[j][1] - stop_periods[j][0];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});

			scanstops.push_back(oi);
			j++;
		}
	}

	i = 0;
	j = 0;
	std::vector<OutputInfo> dumpslews;

	while ((i < dumps) && (j < slews))
	{
		if (dump_periods[i][1] <= slew_periods[j][0]) // dump before slew
		{
			OutputInfo oi;
			oi.start_time = dump_periods[i][0];
			oi.mode = "dump";
			oi.duration = dump_periods[i][1] - dump_periods[i][0];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});

			dumpslews.push_back(oi);
			i++;
		}
		else
		{
			OutputInfo oi;
			oi.start_time = slew_periods[j][0];
			oi.mode = "slew";
			oi.target_quat = slew_attitudes[j];
			oi.duration = slew_periods[j][1] - slew_periods[j][0];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});

			dumpslews.push_back(oi);
			j++;
		}		
	}
	if (j == slews)
	{
		while (i < dumps)
		{
			OutputInfo oi;
			oi.start_time = dump_periods[i][0];
			oi.mode = "dump";
			oi.duration = dump_periods[i][1] - dump_periods[i][0];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});
			dumpslews.push_back(oi);
			i++;
		}
	}
	else if (i == dumps)
	{
		while (j < slews)
		{
			OutputInfo oi;
			oi.start_time = slew_periods[j][0];
			oi.mode = "slew";
			oi.duration = slew_periods[j][1] - slew_periods[j][0];
			oi.target_quat = slew_attitudes[j];
			oi.isObserved = true;
			oi.fuel = 0.0;
			oi.momentum = PositionVector({0.0, 0.0, 0.0, 0.0});

			dumpslews.push_back(oi);
			j++;
		}
	}

	int ss = (int)scanstops.size();
	int ds = (int)dumpslews.size();

	i = 0;
	j = 0;

	while ((i < ss) && (j < ds))
	{
		if (dumpslews[j].start_time <= scanstops[i].start_time)
		{
			all_modes.push_back(dumpslews[j]);
			j++;
		}
		else
		{
			all_modes.push_back(scanstops[i]);
			i++;
		}
	}
	if (i == ss)
	{
		while (j < ds)
		{
			all_modes.push_back(dumpslews[j]);
			j++;
		}
	}
	else
	{
		while (i < ss)
		{
			all_modes.push_back(scanstops[i]);
			i++;
		}
	}
}

void Satellite::update()
{
	if (need_for_update)
	{
		pv = upd_pv;
		angular_momentum = upd_angular_momentum;
		quaternion = upd_quaternion;
	}
	need_for_update = false;
}

void Satellite::setState(const StateVector& st)
{
	pv = st;
}
void Satellite::setUpdateState(const StateVector& st)
{
	upd_pv = st;
}
void Satellite::setMass(double m)
{
	mass = m;
}
void Satellite::setPosition(PositionVector& p)
{
	PositionVector v = pv.getVelocity();
	pv = StateVector(p, v);
}
void Satellite::setPosition(const PositionVector& p)
{
	pv.setPosition(p);
}
void Satellite::setAngularVelocity(const std::vector<double>& av)
{
	angular_velocity = PositionVector(av);
	angular_momentum = mul(inertia_tensor, av);
}
void Satellite::setAngularMomentum(const PositionVector& ang_mom)
{
	angular_momentum = ang_mom;
}
void Satellite::setUpdateAngularMomentum(const PositionVector& ang_mom)
{
	upd_angular_momentum = ang_mom;
}
void Satellite::setInertiaTensor(const Matrix& inertia)
{
	inertia_tensor = inertia;
}
void Satellite::setPolygons(const std::vector<Polygon>& pols)
{
	polygons = pols;
}
void Satellite::setSolarPanels(const std::vector<Polygon>& sp)
{
	solar_panels = sp;
}
void Satellite::setGyrostats(const std::vector<AttitudeController>& gyrs)
{
	gyrostats = gyrs;
}
void Satellite::setMagnetorquers(const std::vector<AttitudeController>& magns)
{
	magnetorquers = magns;
}
void Satellite::setReactionWheelsBlock(const std::vector<ReactionWheel>& rwb)
{
	reaction_wheels.push_back(rwb);
}
void Satellite::setThrusters(const std::vector<AttitudeController>& thr)
{
	thrusters = thr;
}
void Satellite::setCorrectionThrusters(const std::vector<AttitudeController>& thr)
{
	correction_thrusters = thr;
}
void Satellite::setPulseEngineLocation(const PositionVector& location)
{
	pulse_engine_location = location;
}
//void Satellite::setQuaternion(const boost::math::quaternion<double>& quat)
void Satellite::setQuaternion(const Quaternion& quat)
{
	quaternion = quat;
}
void Satellite::setUpdateQuaternion(const Quaternion& q)
{
	upd_quaternion = q;
}
void Satellite::setSunPosition(const PositionVector& sun_pos)
{
	sun_position = sun_pos;
}
void Satellite::setSetbackTime(double sbt)
{
	setback_time = sbt;
}
void Satellite::setTargetDuration(const Time& t, const double step)
{
	all_modes[target_index-1].sd_time = t - all_modes[target_index-1].start_time + step;
}
void Satellite::setTargetDuration(const double dur)
{
	all_modes[target_index-1].sd_time = dur;
}
void Satellite::setTargetFuel(double f)
{
	all_modes[target_index-1].fuel += f;
}
void Satellite::setTargetMomentum(const PositionVector& mom)
{
	all_modes[target_index-1].momentum = mom;
}
void Satellite::setHdfFile(const std::string& fname)
{
	hdf5_filename = fname;
}

PositionVector Satellite::setControlMomentum(const PositionVector& momentum_to_compensate, char controller)
{
	std::vector<AttitudeController> *control_elements;
	if (controller == 'g')
	{
		control_elements = &gyrostats;
	}
	else if (controller == 'm')
	{
		control_elements = &magnetorquers;
	}
	else
	{
		std::cerr << "\033[31m#11_control_order No such control element " << controller  << "\033[0m" << std::endl;
		std::exit(EXIT_FAILURE);
		return momentum_to_compensate;
	}
	// squeeze as much momentum as I can
	
	double delta, stored_momentum;
	PositionVector limits;
	PositionVector momentum = momentum_to_compensate;

	for (int i = 0; i < (*control_elements).size(); i++)
	{
		limits = (*control_elements)[i].getLimit();
		stored_momentum = (*control_elements)[i].getStoredMomentum();

		for (int j = 0; j < 3; j++)
		{
			if ((limits[j]) && (momentum[j]))
			{
				delta = stored_momentum; // what was
				stored_momentum = (momentum[j] < 0.0) ? std::max(-1.0 * limits[j], stored_momentum + momentum[j]) : std::min(limits[j], stored_momentum + momentum[j]);
				delta = stored_momentum - delta; // actual delta
				momentum[j] -= delta;
				if (std::fabs(stored_momentum) == limits[j]) (*control_elements)[i].requestDischarge();
			}
		}
		(*control_elements)[i].setStoredMomentum(stored_momentum);
	}
	return momentum;
}
PositionVector Satellite::setGyrostatsMomentum(const PositionVector& momentum_to_compensate)
{
	// squeeze as much momentum as I can
	
	double delta, stored_momentum;
	PositionVector limits;
	PositionVector momentum = momentum_to_compensate;

	for (int i = 0; i < gyrostats.size(); i++)
	{
		limits = gyrostats[i].getLimit();
		stored_momentum = gyrostats[i].getStoredMomentum();

		for (int j = 0; j < 3; j++)
		{
			if ((limits[j]) && (momentum[j]))
			{
				delta = stored_momentum; // what was
				stored_momentum = (momentum[j] < 0.0) ? std::max(-1.0 * limits[j], stored_momentum + momentum[j]) : std::min(limits[j], stored_momentum + momentum[j]);
				delta = stored_momentum - delta; // actual delta
				momentum[j] -= delta;
				if (std::fabs(stored_momentum) == limits[j]) gyrostats[i].requestDischarge();
			}
		}
		gyrostats[i].setStoredMomentum(stored_momentum);
	}
	return momentum;
}
PositionVector Satellite::discharge(char system)
{
	PositionVector moment_to_compensate = PositionVector(std::vector<double>{ 0.0, 0.0, 0.0 });
	if (system == 'g') // "gyrostats"
	{
		for (int i = 0; i < gyrostats.size(); i++)
		{
			if (gyrostats[i].requireDischarge())
			{
				for (int j = 0; j < 3; j++)
				{
					if (gyrostats[i].getLimit()[j])
					{
						moment_to_compensate[j] += gyrostats[i].getStoredMomentum();
						gyrostats[i].discharge();
					}
				}
			}
		}
	}
	else if (system == 'm') // magnetorquers
	{
		for (int i = 0; i < magnetorquers.size(); i++)
		{
			magnetorquers[i].setStoredMomentum(0.0);
		}
	}
	else if (system == 'r') // reaction wheels 4block
	{
		for (int rwb = 0; rwb < reaction_wheels.size(); rwb++)
		{
			PositionVector new_momenums({0.0, 0.0, 0.0, 0.0});
			PositionVector angles = reaction_wheels[rwb][0].getAngles();
			for (int i=0; i<4; i++)
			{
				if (reaction_wheels[rwb][i].requireDischarge())
				{
					new_momenums[i] = reaction_wheels[rwb][i].getStoredMomentum();
					reaction_wheels[rwb][i].discharge();
				}
			}
			moment_to_compensate = moment_to_compensate + Control::combineReactionWheelsBlockMomentum(new_momenums, angles, reaction_wheels[rwb][0].getApex());
		}
	}
	else if (system == 't') // thrusters
	{
		for (int i = 0; i < thrusters.size(); i++)
		{
			thrusters[i].setStoredMomentum(0.0);
		}
	}
	return moment_to_compensate;
}
PositionVector Satellite::discharge_all(char system)
{
	PositionVector moment_to_compensate = PositionVector(std::vector<double>{ 0.0, 0.0, 0.0 });
	bool someone_needs_a_discharge = false;
	if (system == 'g') // "gyrostats"
	{
		for (int i = 0; i < gyrostats.size(); i++)
		{
			if (gyrostats[i].requireDischarge()) 
			{
				someone_needs_a_discharge = true;
				break;
			}
		}
		if (someone_needs_a_discharge)
		{
			for (int i = 0; i < gyrostats.size(); i++)
			{
				for (int j = 0; j < 3; j++)
				{
					if (gyrostats[i].getLimit()[j])
					{
						moment_to_compensate[j] += gyrostats[i].getStoredMomentum();
						gyrostats[i].discharge();
					}
				}
			}
		}
	}
	else if (system == 'm') // magnetorquers
	{
		for (int i = 0; i < magnetorquers.size(); i++)
		{
			magnetorquers[i].setStoredMomentum(0.0);
		}
	}
	else if (system == 'r') // reaction wheels 4block
	{
		for (int rwb = 0; rwb < reaction_wheels.size(); rwb++)
		{
			PositionVector new_momentums({0.0, 0.0, 0.0, 0.0});
			PositionVector angles = reaction_wheels[rwb][0].getAngles();
			for (int i = 0; i < 4; i++)
			{
				if (reaction_wheels[rwb][i].requireDischarge())
				{
					someone_needs_a_discharge = true;
					break;
				}
			}
			if (someone_needs_a_discharge)
			{
				for (int i=0; i<4; i++)
				{
					new_momentums[i] = reaction_wheels[rwb][i].getStoredMomentum();
					reaction_wheels[rwb][i].discharge();
				}
				moment_to_compensate = moment_to_compensate + Control::combineReactionWheelsBlockMomentum(new_momentums, angles, reaction_wheels[rwb][0].getApex());
			}
		}
	}
	else if (system == 't') // thrusters
	{
		for (int i = 0; i < thrusters.size(); i++)
		{
			thrusters[i].setStoredMomentum(0.0);
		}
	}
	return moment_to_compensate;
}
PositionVector Satellite::forced_dump(char system, const double step)
{
	PositionVector moment_to_compensate({0.0, 0.0, 0.0});
	double delta, stored_momentum, dump_speed;
	if (system == 'r')
	{
		for (int rwb = 0; rwb < reaction_wheels.size(); rwb++)
		{
			PositionVector new_momentums({0.0, 0.0, 0.0, 0.0});
			PositionVector angles = reaction_wheels[rwb][0].getAngles();
			for (int i = 0; i < 4; i++)
			{
				stored_momentum = reaction_wheels[rwb][i].getStoredMomentum();
				dump_speed = reaction_wheels[rwb][i].getDumpSpeed();
				if (stored_momentum > 0)
				{
					delta = std::min(stored_momentum, dump_speed * step);
				}
				else
				{
					delta = std::max(stored_momentum, -1.0 * dump_speed * step);
				}

				new_momentums[i] = delta;
				reaction_wheels[rwb][i].setStoredMomentum(stored_momentum - delta);
			}
			moment_to_compensate = moment_to_compensate + Control::combineReactionWheelsBlockMomentum(new_momentums,
				reaction_wheels[rwb][0].getAngles(), reaction_wheels[rwb][0].getApex());
		}
	}
	return moment_to_compensate;
}
PositionVector Satellite::setMagneticMomentum(const PositionVector& momentum_to_compensate, const PositionVector& magnetic_field)
{
	// squeeze as much momentum as I can
	
	double delta, stored_momentum;
	PositionVector limits;
	PositionVector B = magnetic_field* 1e-9;
	PositionVector magn_momentum = momentum_to_compensate - momentum_to_compensate.dot(B) / std::pow(B.norm(), 2) * B; // perpendicular part
	magn_momentum = magn_momentum.cross(B) / std::pow(B.norm(), 2); // needed magnetic momentum of magnetorquers

	for (int i = 0; i < magnetorquers.size(); i++)
	{
		limits = magnetorquers[i].getLimit();
		for (int j = 0; j < 3; j++)
		{
			if ((limits[j]) && (magn_momentum[j]))
			{
				stored_momentum = (magn_momentum[j] < 0.0) ? std::max(-1.0 * limits[j], magn_momentum[j]) : std::min(limits[j], magn_momentum[j]);
				magn_momentum[j] -= stored_momentum;
			}
		}
		magnetorquers[i].setStoredMomentum(stored_momentum);
	}
	return momentum_to_compensate - B.cross(this->getMagneticMomentum());
}
void Satellite::setMagneticMomentumFromFile(const PositionVector& magn)
{
	magnetorquers[0].setStoredMomentum(magn[0]);
	magnetorquers[1].setStoredMomentum(magn[1]);
	magnetorquers[2].setStoredMomentum(magn[2]);
}
PositionVector Satellite::setReactionWheelsMomentum(const PositionVector& momentum_to_compensate)
{
	if (reaction_wheels.empty())
	{
		return momentum_to_compensate;
	}

	PositionVector new_momentums;
	PositionVector local_angles, initial_momentums;
	PositionVector momentum = momentum_to_compensate;

	double delta, limit, stored_momentum;

	PositionVector all_angles(std::vector<double>(4*reaction_wheels.size()));
	for (int rwb = 0; rwb < reaction_wheels.size(); rwb++)
	{
		local_angles = reaction_wheels[rwb][0].getAngles();
		for (int i = 0; i< 4; i++)
		{ all_angles[4 * rwb + i] = local_angles[i]; }
	}

	// моменты ВСЕХ маховиков, которые должны получиться
	new_momentums = Control::redistributeCompensationMomentum(momentum, all_angles, reaction_wheels[0][0].getApex());
	
	for (int rwb = 0; rwb < reaction_wheels.size(); rwb++)
	{
		// моменты 4-х маховиков текущего блока
		initial_momentums = getReactionWheelsBlockMomentum(rwb);
				
		// new_momentums = new_momentums - initial_momentums; // that much need to add

		// squeeze as much as possible
		for (int i = 0; i < 4; i++)
		{
			new_momentums[4 * rwb + i] = new_momentums[4 * rwb + i] - initial_momentums[i]; // that much need to add
			stored_momentum = initial_momentums[i];
			delta = stored_momentum; // what was
			limit = reaction_wheels[rwb][i].getLimit();
			stored_momentum = (new_momentums[4 * rwb + i] < 0.0) ? std::max(-1.0 * limit, stored_momentum + new_momentums[4 * rwb + i]) : std::min(limit, stored_momentum + new_momentums[4 * rwb + i]);
			delta = stored_momentum - delta; // actual delta
			new_momentums[4 * rwb + i] -= delta;
			if (std::fabs(stored_momentum) == limit)
			{
				reaction_wheels[rwb][i].requestDischarge();
			}
			reaction_wheels[rwb][i].setStoredMomentum(stored_momentum);
			/*
			if (rwb == 0)
			{
				if (std::fabs(stored_momentum) == limit)
				{
					new_momentums[i] += delta;
				}
				else
				{
					reaction_wheels[0][i].setAngularMomentum(stored_momentum);
				}
			}
			else
			{
				if (std::fabs(stored_momentum) == limit)
				{
					reaction_wheels[1][i].requestDischarge();
				}
				reaction_wheels[1][i].setAngularMomentum(stored_momentum);
			}
			*/
		}
		// momentum = Control::combineReactionWheelsBlockMomentum(new_momentums, angles); // that much left to compensate
	}
	return Control::combineReactionWheelsBlockMomentum(new_momentums, all_angles, reaction_wheels[0][0].getApex()); // that much left to compensate
}
void Satellite::setThrustersMomentum(const PositionVector& thr_mom, double step)
{
	/*
	// используется тот нетривиальный факт, что хотя бы одна из компонент position равна 0
	// рис. 4.2.3. расположения ДС на ММ говорит, что это нифига не так)))
	thrusters_momentum = thr_mom;
	PositionVector dm;
	PositionVector pulse({0.0, 0.0, 0.0});
	double mass_change = 0.0;
	double new_mass = mass;
	for (int i = 0; i < thrusters.size(); i++)
	{
		dm = thrusters[i].getLocation().cross(thrusters[i].getLimit());
		for (int j = 0; j < 3; j++)
		{
			if (dm[j] != 0.0)
			{
				if (dm[j] * thr_mom[j] > 0.0)
				{
					mass_change = thr_mom[j] / dm[j] * step;
					thrusters[i].setStoredMomentum(thr_mom[j]);
					thrusters[i].setMass(thrusters[i].getMass() - mass_change);
					pulse = pulse + thrusters[i].getLimit() * mass_change;
					new_mass -= mass_change;
					break;
				}
			}
		}
	}
	this->setState(StateVector(this->getPosition(), (mass * this->getVelocity() + pulse / this->getMass()) / new_mass));
	this->setMass(new_mass);
	*/

	std::vector<int> suitable_thrusters {};
	int number_of_thrusters = 0;
	double mass_change = 0.0;
	PositionVector pulse({0.0, 0.0, 0.0});
	double new_mass = mass;

	for (int i = 0; i < 3; i++)
	{
		suitable_thrusters = std::vector<int> {};
		number_of_thrusters = 0;
		
		// find all suitable thrusters for a given thr_mom component
		for (int j = 0; j < (int)thrusters.size(); j++)
		{
			if (thrusters[j].getLocation().cross(thrusters[j].getLimit())[i] * thr_mom[i] < 0)
			{
				suitable_thrusters.push_back(j);
				number_of_thrusters++;
			}
		}

		// redistribute thr_mom[i] on all suitable thrusters equally
		for (int thruster_id: suitable_thrusters)
		{
			mass_change = std::fabs(thr_mom[i] / number_of_thrusters * step / thrusters[thruster_id].getLocation().cross(thrusters[thruster_id].getLimit())[i]);
			thrusters[thruster_id].setStoredMomentum(thrusters[thruster_id].getStoredMomentum() + mass_change);
			thrusters[thruster_id].setMass(thrusters[thruster_id].getMass() - mass_change);
			pulse = pulse - thrusters[thruster_id].getLimit() * mass_change; // '-' as pulse should be opposite to limits vector.
			new_mass -= mass_change;
		}
	}

	PositionVector v = upd_pv.getVelocity();
	PositionVector p = upd_pv.getPosition();
	v = (mass * v + pulse / mass) / new_mass;
	upd_pv = StateVector(p, v);
	mass = new_mass;
	need_for_update = true;
}
void Satellite::requestUpdate()
{
	need_for_update = true;
}
void Satellite::enableCounterrotation()
{
	counterrotate = true;
}
void Satellite::disableCounterrotation()
{
	counterrotate = false;
}
//void Satellite::setStopMotion(const Time& start, const Time& stop, const boost::math::quaternion<double>& quat)
void Satellite::setStop(const Time& start, const Time& stop)
{
	stop_periods.push_back(std::vector<Time>{start, stop});
}
void Satellite::setScan(const Time& start, const Time& stop, const PositionVector& ang_vel)
{
	scan_periods.push_back(std::vector<Time>{start, stop});
	scan_velocities.push_back(ang_vel);
}
void Satellite::setDump(const Time& start, const Time& stop)
{
	dump_periods.push_back(std::vector<Time>{start, stop});
}
void Satellite::setSlew(const Time& start, const Time& stop, const Quaternion& quat)
{
	slew_periods.push_back(std::vector<Time>{start, stop});
	slew_attitudes.push_back(quat);
}
void Satellite::setPulse(const Time& t, const PositionVector& force)
{
	pulse_periods.push_back(t);
	pulse_forces.push_back(force);
}
void Satellite::setCorrection(const Time& start, const Time& stop, const PositionVector& pulse)
{
	correction_periods.push_back(std::vector<Time>{start, stop});
	correction_pulses.push_back(pulse);
}
void Satellite::sortTransitionPeriods()
{
	std::vector<Time> tmp;
	PositionVector tmp_vel;
	for (int i = 0; i < (int)scan_periods.size() - 1; i++)
	{
		for (int j = 0; j < (int)scan_periods.size() - 1 - i; j++)
		{
			if (scan_periods[j][0] > scan_periods[j+1][0])
			{
				tmp = scan_periods[j];
				scan_periods[j] = scan_periods[j+1];
				scan_periods[j+1] = tmp;

				tmp_vel = scan_velocities[j];
				scan_velocities[j] = scan_velocities[j+1];
				scan_velocities[j+1] = tmp_vel;
			}
		}
	}
	
	Quaternion tmp_quat;
	for (int i = 0; i < (int)slew_periods.size() - 1; i++)
	{
		for (int j = 0; j < (int)slew_periods.size() - 1 - i; j++)
		{
			if (slew_periods[j][0] > slew_periods[j+1][0])
			{
				tmp = slew_periods[j];
				slew_periods[j] = slew_periods[j+1];
				slew_periods[j+1] = tmp;

				tmp_quat = slew_attitudes[j];
				slew_attitudes[j] = slew_attitudes[j+1];
				slew_attitudes[j+1] = tmp_quat;
			}
		}
	}

	for (int i = 0; i < (int)dump_periods.size() - 1; i++)
	{
		for (int j = 0; j < (int)dump_periods.size() - 1; j++)
		{
			if (dump_periods[j][0] > dump_periods[j+1][0])
			{
				tmp = dump_periods[j];
				dump_periods[j] = dump_periods[j+1];
				dump_periods[j+1] = tmp;
			}
		}
	}

	for (int i = 0; i < (int)stop_periods.size() - 1; i++)
	{
		for (int j = 0; j < (int)stop_periods.size() - 1; j++)
		{
			if (stop_periods[j][0] > stop_periods[j+1][0])
			{
				tmp = stop_periods[j];
				stop_periods[j] = stop_periods[j+1];
				stop_periods[j+1] = tmp;
			}
		}
	}
}
void Satellite::rotateSolarPanels(const PositionVector& s)
{
	for (int i=0; i < solar_panels.size(); i++)
	{
		int rai = solar_panels[i].getRotationAxisIndex();
		if (rai == -1)
		{
			std::cerr << "\033[33m#9 Solar panel #" << i << " can not be rotated\033[0m" << std::endl;
			continue;
		}
		PositionVector n = solar_panels[i].getNormal();
		double phi, c1, c2;
		if (rai == 0)
		{
			 double phi = std::atan2(-1.0 * n.cross(s)[0], n.dot(s));
			 for (int k = 0; k < 2; k++)
			 {
				c1 = n[1] * std::cos(phi)  + n[2] * std::sin(phi);
				c2 = -1.0 * n[1] * std::sin(phi) + n[2] * std::cos(phi);
				PositionVector new_n({0.0, c1, c2});
				if (s.dot(new_n) < 0.0)
				{
					solar_panels[i].setNormal(new_n);
					break;
				}
				else
				{
					phi += M_PI;
				}
			 }
		}
		else if (rai == 1)
		{
			phi = std::atan2(-1.0 * n.cross(s)[1], n.dot(s));
			for (int k=0; k<2; k++)
			{
				c1 = n[0] * std::cos(phi) - n[2] * std::sin(phi);
				c2 = n[0]  * std::sin(phi) + n[2] * std::cos(phi);
				PositionVector new_n({c1, 0.0, c2});
				if (s.dot(new_n) < 0.0)
				{
					solar_panels[i].setNormal(new_n);
					break;
				}
				else
				{
					phi += M_PI;
				}
			}
		}
		else if (rai == 2)
		{
			phi = std::atan2(n.cross(s)[2], n.dot(s));
			for (int k=0; k<2; k++)
			{
				c1 = n[0] * std::cos(phi) + n[1] * std::sin(phi);
				c2 = -1.0 * n[0] * std::sin(phi) + n[1] * std::cos(phi);
				PositionVector new_n({c1, c2, 0.0});
				if (s.dot(new_n) < 0.0)
				{
					solar_panels[i].setNormal(new_n);
					break;
				}
				else
				{
					phi += M_PI;
				}
			}
		}
		else
		{
			std::cerr << "\033[31m#11_rai Incorrect value of rai = " << rai << " on solar panel #" << i << "\033[0m" << std::endl;
			std::exit(EXIT_FAILURE);
			continue;
		}
	}
}
void Satellite::setNewPolygons()
{
	/* Это для ориентации 0.7404253	-0.6646763	0.0997737	0.0045806 
	Polygon s1 = Polygon(PositionVector({0.0, 4.89, 0.0}), PositionVector({0.0, 0.0, 1.0}), 14.86, 0.09, 1.0, 1);
	Polygon s2 = Polygon(PositionVector({0.0, -4.89, 0.0}), PositionVector({0.0, 0.0, 1.0}), 12.88, 0.09, 1.0, 1);
	
	Polygon p3 = Polygon(PositionVector({-1.025, 0.0, 0.0}), PositionVector({-1.0, 0.0, 0.0}), 6.4, 0.8656, 0.134);
	Polygon p4 = Polygon(PositionVector({0.0, 0.0, -1.265}), PositionVector({0.0, 0.0, -1.0}), 5.2, 0.8656, 0.134);
	Polygon p5 = Polygon(PositionVector({0.0, 1.265, 0.0}), PositionVector({0.0, 1.0, 0.0}), 2.6, 0.8656, 0.134);
	Polygon p6 = Polygon(PositionVector({0.0, 0.0, -2.931}), PositionVector({-1.0, 0.0, 0.0}), 21.7, 0.8656, 0.134);
	Polygon p8 = Polygon(PositionVector({1.025, 1.864, -1.0}), PositionVector({-1.0, 0.0, 0.0}), 0.614, 0.8656, 0.134);

	this->setPolygons({p3, p4, p5, p6, p8});
	this->setSolarPanels({s1, s2});
	*/

	/* Для ориентации 0.1260455	-0.9330310	0.2089708	0.2643803 	*/
	Polygon s1 = Polygon(PositionVector({0.0, -4.89, 0.0}), PositionVector({-1.0, 0.0, 0.0}), 14.86, 0.09, 1.0, 1);
	Polygon s2 = Polygon(PositionVector({0.0, 4.89, 0.0}), PositionVector({-1.0, 0.0, 0.0}), 14.86, 0.09, 1.0, 1);

	Polygon p3 = Polygon(PositionVector({-1.025, 0.0, 0.0}), PositionVector({-1.0, 0.0, 0.0}), 6.4, 0.8656, 0.134);
	Polygon p4 = Polygon(PositionVector({0.0, 0.0, 1.265}), PositionVector({0.0, 0.0, 1.0}), 5.2, 0.8656, 0.134);
	Polygon p5 = Polygon(PositionVector({0.0, 0.0, 2.931}), PositionVector({-1.0, 0.0, 0.0}), 21.7, 0.8656, 0.134);
	
	this->setPolygons({p3, p4, p5});
	this->setSolarPanels({s1, s2});

}
void Satellite::setOrbitFilename(const std::string& filename)
{
	orbit_filename = filename;
}
void Satellite::set_to_default()
{
	pv = StateVector();
	mass = 10.0;
	inertia_tensor = Matrix(3, 3, std::vector<std::vector<double>>{ {10.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 10.0}});
	angular_velocity = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	//quaternion = boost::math::quaternion<double>{ 1.0, 0.0, 0.0, 0.0 };
	quaternion = Quaternion();
	magnetic_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	counterrotate = false;
	angular_momentum = PositionVector(std::vector<double>{0.0, 0.0, 0.0});
	target = "none";
	need_for_update = false;

	scan_periods.clear();
	scan_velocities.clear();

	stop_periods.clear();
	
	slew_periods.clear();
	slew_attitudes.clear();

	dump_periods.clear();
}
Satellite::~Satellite() {}