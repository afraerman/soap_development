#include "stdafx.h"

void AttitudeIntegrator::compute_jacobian(const Matrix& I, const PositionVector& rho, double step)
{
	jacobian = (I * sqrt(1.0 - phi.dot(phi)) + I * phi.outer(phi) * (-1.0 / sqrt(1.0 - phi.dot(phi))) + phi.skew() * I + mul(I, phi).skew() * (-1.0)) * (2.0 / step)
		+ rho.outer(phi) * (-1.0 / sqrt(1.0 - phi.dot(phi))) + rho.skew() * (-1.0);
}

void AttitudeIntegrator::compute_momentum(const Matrix& I, const PositionVector& rho, double step, char operation)
{
	if (operation == '-')
	{
		momentum = ((mul(I, phi) + rho * (step / 2.0)) * sqrt(1.0 - phi.dot(phi)) + phi.cross(mul(I, phi) + rho * (step / 2.0)) * (-1.0)) * (2.0 / step);
	}
	else if (operation == '+')
	{
		momentum = ((mul(I, phi) + rho * (step / 2.0)) * sqrt(1.0 - phi.dot(phi)) + phi.cross(mul(I, phi) + rho * (step / 2.0))) * (2.0 / step);
	}
	else
	{
		std::cerr << "\033[31mUnknown operation: " << operation << "\033[0m" << '\n';
		std::exit(EXIT_FAILURE);
	}
}

bool AttitudeIntegrator::scan_time(const Satellite& sat, const Time& time) const
{
	std::vector<std::vector<Time>> scan_times = sat.getScanTimes();
	for (int i = 0; i < scan_times.size(); i++)
	{
		if ((scan_times[i][0] <= time) && (time < scan_times[i][1])) return true;
	}
	return false;
}

bool AttitudeIntegrator::stop_time(const Satellite& sat, const Time& time) const
{
	std::vector<std::vector<Time>> stop_times = sat.getStopTimes();
	for (int i = 0; i < stop_times.size(); i++)
	{
		if ((stop_times[i][0] <= time) && (time < stop_times[i][1])) return true;
	}
	return false;
}

bool AttitudeIntegrator::dump_time(const Satellite& sat, const Time& time) const
{
	std::vector<std::vector<Time>> dump_times = sat.getDumpTimes();
	double dump_duration = sat.getDumpDuration();

	for (int i = 0; i < dump_times.size(); i++)
	{
		if ((dump_times[i][0] <= time) && (time < dump_times[i][1])) return true;
	}
	return false;
}

bool AttitudeIntegrator::slew_time(const Satellite& sat, const Time& time) const
{
	std::vector<std::vector<Time>> slew_times = sat.getSlewTimes();
	for (int i = 0; i < (int)slew_times.size(); i++)
	{
		if ((slew_times[i][0] <= time) && (time < slew_times[i][1])) return true;
	}
	return false;
}

int AttitudeIntegrator::acquire_target(Satellite& sat, Time& time, const double step, const PositionVector& torque)
{
	Control::makeCheckpoint(time, sat);
	sat.setSetbackTime(0.0);
	auto target = sat.getNextTarget();
	sat.target_index++;

	// sat.setTargetDuration(time);
	end_of_current_target = (int)std::round(target.duration/step);
	elapsed_time = (int)std::round(-1.0 * (target.start_time - time));

	mode = target.mode;
	target_time = target.start_time;
	if (mode == "scan")
		//target_momentum = sat.getTargetMomentum(time);
		target_momentum = target.target_momentum;
	else if (mode == "slew")
		//target_attitude = sat.getTargetQuaternion(time);
		target_attitude = target.target_quat;
	std::cout << "\033[32mCurrent time " << time << '\t' << "Target time "  << target_time << '\t' << "Target " << mode << "\033[0m" << std::endl;
	if (mode != "none") // there is a real target
	{
		t = 0.0;
		//gap = target_time - time;
		gap = target.duration;
		Control::setInitialMomentum(sat.getAngularMomentum());
		std::cout << "Gap = " << gap << std::endl;
		Control::setGap(gap);
		Control::setTargetTime(gap);
		
		// Control initial parameters determination
		if (mode == "scan")
		{
			// Don't need for MM, so probably doesn't work right now
			Control::setTargetMomentum(target_momentum);
			Control::setRotationMomentum(target_momentum);
			std::cout << "Target acquired: " << target_momentum << std::endl;
			real_scan_phi = target_momentum;
		}
		else if (DEVELOPER::attitude_testing_mode)
		{
			std::cout << "Developer mode" << std::endl;
		}
		else if (mode == "slew")
		{
			// Stupid 3-axis rotation
			//Control::defactorTarget(sat.getInertiaTensor(), sat.getQuaternion(), target_attitude);
			
			// "Smart" one-axis rotation
			//if (std::isnan(torque[0])) torque = future_torque.get();
			int res = Control::setRotationFromQuat(sat, sat.getQuaternion(), target_attitude, gap, step, initial_momentum, torque);
			if (res) // rotation not possible (to little time)
			{
				//skip_until = sat.getEndOfTarget(time);
				//integration_error = true;
				std::cout << "\033[34mAt time " << time << " back to checkpoint\033[0m" << std::endl;
				Control::getCheckpoint(time, sat);
				sat.setSetbackTime(sat.getSetbackTime() + step);
				sat.targetFailed();
				target_acquired = false;
				time = time + (-1)*step;
				sat.setUpdateAngularMomentum(sat.getAngularMomentum());
				sat.setUpdateQuaternion(sat.getQuaternion());
				elapsed_time = 0;
				end_of_current_target = 0;
				return 1;
			}
			std::cout << "\033[32mTarget acquired: ";
			std::cout << target_attitude << "\033[0m" << std::endl;
		}
	}
	target_acquired = true;
	return 0;
}

void AttitudeIntegrator::integrationMethod(Satellite& sat, Time& time, double step, bool from_the_start)
{
	//auto future_torque = std::async(std::launch::async, &Torques::allTorques, std::ref(sat), std::ref(time));

	// All torques influencing the satellite
	//PositionVector torque = PositionVector({ std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN() });
	PositionVector torque = Torques::allTorques(sat, time);
	
	// Momentum to compensate with control systems available
	PositionVector momentum_to_compensate = PositionVector({ 0.0, 0.0, 0.0 });
	
	// Momentum to generate using magnetorquers
	PositionVector magnetorquers_momentum = PositionVector({ 0.0, 0.0, 0.0 });

	// Momentum to generate using gyrostats
	PositionVector gyrostats_momentum = PositionVector({ 0.0, 0.0, 0.0 });

	// Momentum to generate using thrusters
	PositionVector thrusters_momentum = PositionVector({ 0.0, 0.0, 0.0 });
	PositionVector excessive_momentum = PositionVector({ 0.0, 0.0, 0.0 });

	//PositionVector magnetic_field = mul(Matrix(sat.getQuaternion()).transpose(), Torques::getMagneticField(sat.getPosition(), time));
	PositionVector magnetic_field = sat.getQuaternion().get_inverse() * Torques::getMagneticField(sat.getPosition(), time);

	target_momentum = sat.getTargetMomentum(time);
	
	PositionVector phi2;

	Matrix compensation_matrix;
	if (from_the_start)
	{
		initial_momentum = mul(sat.getInertiaTensor(), sat.getAngularVelocity());
		sat.setAngularMomentum(initial_momentum);
		phi = sat.getAngularVelocity() * (step / 2.0);
		end_of_current_target = 0;
		elapsed_time = 0;
	}
	else
	{
		initial_momentum = momentum;
	}

	sat.discharge('m');
	sat.discharge('t');

	sat.make_telemetry = true;
	
	//std::cout << "attitude integrator: ";

	//torque = future_torque.get();

	if (dump_time(sat, time))
	{
		if ((not target_acquired) && (from_the_start))
		{
			int res = acquire_target(sat, time, step, torque);
			if (res) return;
		}

		target_acquired = false;
		// preserve angular momentum
		momentum_to_compensate = sat.forced_dump('r', step);
		sat.setTargetMomentum(sat.getReactionWheelsBlockMomentum(-1));
		if (momentum_to_compensate.norm())
		{
			sat.setTargetDuration(time, step);
		}
		//torque = future_torque.get();
		momentum_to_compensate += initial_momentum - torque * step;
		thrusters_momentum = momentum_to_compensate;
	}
	// SCANNING ATTITUDE MODE (preserving angular momentum (angular velocity) (incremential quaternion))
	else if (scan_time(sat, time)) 
	{
		if ((not target_acquired) && (from_the_start))
		{
			int res = acquire_target(sat, time, step, torque);
			if (res) return;
		}

		/*
		// Подруливание магнеторкерами
		// Попытка #1
		//boost::math::quaternion<double> corr_quat = (1.0 / sat.getQuaternion()) * real_scan_quat;
		//phi = PositionVector({corr_quat.R_component_2(), corr_quat.R_component_3(), corr_quat.R_component_4()});
		// ПРОВАЛ

		// Попытка #2
		// phi = real_scan_phi;
		// ПРОВАЛ

		// Попытка #3
		// real_scan_phi - момент
		//

		// compute_momentum(sat.getInertiaTensor(), PositionVector({ 0.0, 0.0, 0.0 }), step, '+');
		// momentum_to_compensate = initial_momentum - torque * step - momentum;
		momentum_to_compensate = initial_momentum - momentum - torque * step;
		// momentum_to_compensate = -1.0 * torque * step - Torques::getUncompensatedTorque() * step;

		compensation_matrix = phi.skew();
		compensation_matrix[0][0] = sqrt(1.0 - phi.dot(phi));
		compensation_matrix[1][1] = sqrt(1.0 - phi.dot(phi));
		compensation_matrix[2][2] = sqrt(1.0 - phi.dot(phi));
		
		// Control::control_while_scan(sat, time);
		for (int i=0; i<2; i++)
		{
			// MAGNETORQUER
			if (control_order[i] == 'm')
			{
				gyrostats_momentum = sat.getGyrostatsMomentum() + sat.getReactionWheelsBlockMomentum3d();
				magnetorquers_momentum = (momentum_to_compensate - mul(compensation_matrix, gyrostats_momentum)) / step;
				// magnetorquers_momentum = momentum_to_compensate / step;
				excessive_momentum = sat.setMagneticMomentum(magnetorquers_momentum, magnetic_field);
				momentum_to_compensate = excessive_momentum * step;
			}
			
			// GYROSTAT (there will never be both 4block and gyrostats)
			else if (control_order[i] == 'g')
			{
				// std::cout << momentum_to_compensate << '\t';
				gyrostats_momentum = momentum_to_compensate; //- magnetic_field.cross(sat.getMagneticMomentum()) * step;
				solve(compensation_matrix, gyrostats_momentum);

				gyrostats_momentum = gyrostats_momentum - sat.getGyrostatsMomentum();
				
				excessive_momentum = sat.setGyrostatsMomentum(gyrostats_momentum);
				excessive_momentum = excessive_momentum + sat.discharge('g');

				momentum_to_compensate = mul(compensation_matrix, excessive_momentum);
				// std::cout << excessive_momentum << std::endl;
			}
			
			// REACTION WHEELS 4BLOCK (there will never be both 4block and gyrostats)
			else if (control_order[i] == 'r')
			{
				gyrostats_momentum = momentum_to_compensate - sat.getMagneticMomentum() * step;
				solve(compensation_matrix, gyrostats_momentum);

				excessive_momentum = sat.setReactionWheelsMomentum(gyrostats_momentum);
				excessive_momentum = excessive_momentum + sat.discharge('r');

				momentum_to_compensate = mul(compensation_matrix, excessive_momentum);
			}
		}
		*/
		//torque = future_torque.get();
		momentum_to_compensate = initial_momentum - torque * step + Control::getRotationMomentum();
		momentum_to_compensate = sat.setReactionWheelsMomentum(momentum_to_compensate);
		momentum_to_compensate = momentum_to_compensate + sat.discharge('r');

		// THRUSTERS
		if (momentum_to_compensate.norm())
		{
			/* 
			int h, h1, m, m1, s, s1;
			h = time.getHours();
			m = time.getMinutes();
			s = (int)time.getSeconds();
			for (int i = 0; i < Torques::thrusters_activation_times.size(); i++)
			{
				h1 = Torques::thrusters_activation_times[i].getHours();
				m1 = Torques::thrusters_activation_times[i].getMinutes();
				s1 = (int)Torques::thrusters_activation_times[i].getSeconds();
				if ((h == h1) && (m == m1) && (s == s1))
				{
					std::cout << "At time " << time << " thrusters ON!" << std::endl;
					thrusters_momentum = -1.0 * Torques::thrusters_activation_torques[i];
					break;
				}
				if (h1 > h)
				{
					break;
				}
			}
			*/
			//std::cout << momentum_to_compensate << std::endl;
			thrusters_momentum = momentum_to_compensate / step;
			// Torques::setUncompensatedTorque(momentum_to_compensate / step);
		}

		/*
		correction[0] = 0.0; correction[1] = 0.0; correction[2] = 0.0;
		target_acquired = false;

		int h, h1, m, m1, s, s1;
		h = time.getHours();
		m = time.getMinutes();
		s = (int)time.getSeconds();
		for (int i = 0; i < Torques::thrusters_activation_times.size(); i++)
		{
			h1 = Torques::thrusters_activation_times[i].getHours();
			m1 = Torques::thrusters_activation_times[i].getMinutes();
			s1 = (int)Torques::thrusters_activation_times[i].getSeconds();
			if ((h == h1) && (m == m1) && (s == s1))
			{
				std::cout << "At time " << time << " thrusters ON!" << std::endl;
				thrusters_momentum = Torques::thrusters_activation_torques[i];
				break;
			}
			if (h1 > h)
			{
				break;
			}
		}

		for (int i = 0; i < Torques::magn_activation_times.size(); i++)
		{
			h1 = Torques::magn_activation_times[i].getHours();
			m1 = Torques::magn_activation_times[i].getMinutes();
			s1 = (int)Torques::magn_activation_times[i].getSeconds();
			if ((h == h1) && (m == m1) && (s == s1))
			{
				// std::cout << "At time " << time << " magnets ON!" << std::endl;
				sat.setMagneticMomentumFromFile(Torques::magn_activation_torques[i]);
				magnetic_field = Torques::magnetic_field_data[i];
				break;
			}
			if (h1 > h)
			{
				break;
			}
		}
		*/

		if (!scan_time(sat, time + step))
		{
			sat.setTargetMomentum(sat.getReactionWheelsBlockMomentum(-1));
		}
	
	}
	
	// CONSTANT ATTITUDE MODE (preserving attitude (quaternion))
	else if (stop_time(sat, time))
	{
		if ((not target_acquired) && (from_the_start))
		{
			int res = acquire_target(sat, time, step, torque);
			if (res) return;
		}

		target_acquired = false;
		//torque = future_torque.get();
		momentum_to_compensate = initial_momentum - torque * step;
		//Control::while_stop()
		for (int i = 0; i < 2; i++)
		{
			// MAGNETROQUERS
			if (control_order[i] == 'm')
			{
				// gyrostats_momentum = sat.getGyrostatsMomentum() + sat.getReactionWheelsBlockMomentum3d();
				// magnetorquers_momentum = (momentum_to_compensate - gyrostats_momentum) / step;
				magnetorquers_momentum = momentum_to_compensate / step;
				excessive_momentum = sat.setMagneticMomentum(magnetorquers_momentum, magnetic_field);
				momentum_to_compensate = excessive_momentum * step;
			}
			// GYROSTATS
			else if (control_order[i] == 'g')
			{
				//gyrostats_momentum = momentum_to_compensate - sat.getMagneticMomentum() * step - sat.getGyrostatsMomentum();
				gyrostats_momentum = momentum_to_compensate - sat.getGyrostatsMomentum();
				excessive_momentum = sat.setControlMomentum(gyrostats_momentum, 'g');
				excessive_momentum = excessive_momentum + sat.discharge_all('g');
				momentum_to_compensate = excessive_momentum;
			}
			// REACTION WHEELS 4BLOCK
			else if (control_order[i] == 'r')
			{
				// gyrostats_momentum = momentum_to_compensate - sat.getMagneticMomentum() * step;
				gyrostats_momentum = momentum_to_compensate;
				excessive_momentum = sat.setReactionWheelsMomentum(gyrostats_momentum);
				excessive_momentum = excessive_momentum + sat.discharge_all('r');
				momentum_to_compensate = excessive_momentum;
			}
		}

		if (momentum_to_compensate.norm())
		{
			// std::cout << "Thrusters momentum: " << momentum_to_compensate << std::endl;
			//thrusters_momentum = momentum_to_compensate / step;


			std::cout << "\033[34mAt time " << time << " back to checkpoint\033[0m" << std::endl;
			Control::getCheckpoint(time, sat);
			sat.setSetbackTime(sat.getSetbackTime() + step);
			sat.targetFailed();
			target_acquired = false;
			time = time + (-1)*step;
			sat.setUpdateAngularMomentum(sat.getAngularMomentum());
			sat.setUpdateQuaternion(sat.getQuaternion());
			elapsed_time = 0;
			return;
		}
		else
		{
			sat.setTargetDuration(time, step);
		}

		if (!stop_time(sat, time+step))
		{
			sat.setTargetMomentum(sat.getReactionWheelsBlockMomentum(-1));
		}
		
		correction[0] = 0.0; correction[1] = 0.0; correction[2] = 0.0;
	}

	// ONE-ANGLE SLEW MOTION
	else if (slew_time(sat, time))
	{
		if ((not target_acquired) && (from_the_start))
		{
			int res = acquire_target(sat, time, step, torque);
			if (res) return;
		}

		target_acquired = false;
		//torque = future_torque.get();
		PositionVector torquelike_correction = -1.0 * torque + Control::beforeTarget(t, step, 'm');
		//PositionVector inertial_correction = initial_momentum - torque * step - Control::beforeTarget(t, step, 'g');

		
		PositionVector inertial_correction = initial_momentum - torque * step - Control::performSlew(sat, t, step, 'r');

		//PositionVector inertial_correction = initial_momentum - torque * step + Control::getRotationMomentum();

		// Control_order
		for (int i = 0; i < 2; i++)
		{
			if (control_order[i] == 'm')
			{
				//correction = sat.setMagneticMomentum(torquelike_correction, magnetic_field);
			}
			else if (control_order[i] == 'r')
			{
				//std::cout << time << '\t';
				correction = sat.setReactionWheelsMomentum(inertial_correction);
				correction = correction + sat.discharge('r');
			}

		}

		// THRUSTERS
		if (correction.norm())
		{
			std::cout << "\033[34mAt time " << time << " back to checkpoint\033[0m" << std::endl;
			Control::getCheckpoint(time, sat);
			
			sat.targetFailed();
			target_acquired = false;
			time = time + (-1)*step;
			sat.setUpdateAngularMomentum(sat.getAngularMomentum());
			sat.setUpdateQuaternion(sat.getQuaternion());
			return;
		}
		// thrusters_momentum = torquelike_correction;
		t += step;

		if (!slew_time(sat, time + step))
		{
			sat.setTargetMomentum(sat.getReactionWheelsBlockMomentum(-1));
		}
	}

	// FREE ATTITUDE MODE aka get next target
	if (elapsed_time + 1 >= end_of_current_target)
	{
		// get next target
		if (not target_acquired)
		{
			int res = acquire_target(sat, time, step, torque);
			if (res) return;
		}
		
		// perform rotation to match target attitude and momentum
		if (DEVELOPER::attitude_testing_mode)
		{
			Control::setTargetMomentum(PositionVector({0.0, 0.0, 0.0}));
			// PositionVector torquelike_correction = -1.0 * torque + Control::testingControl(sat.getInertiaTensor(), t, step);
			// thrusters_momentum = torquelike_correction;
			//std::cout << t << '\t' << thrusters_momentum << std::endl;
			//if (std::isnan(torque[0])) torque = future_torque.get();
			PositionVector inertial_correction = -1.0 * torque * step - Control::testingControl(sat.getInertiaTensor(), t, step);
			//std::cout << t << '\t' << inertial_correction << std::endl;

			inertial_correction = sat.setReactionWheelsMomentum(inertial_correction);
			if (inertial_correction.norm())
			{
				thrusters_momentum = inertial_correction / step;
			}

			t += step;
		}
		else if (mode == "scan") // not in use for MM, ignore
		{
			//if (std::isnan(torque[0])) torque = future_torque.get();
			PositionVector torquelike_correction = -1.0 * torque + Control::beforeTarget(t, step, 'm');
			//PositionVector inertial_correction = initial_momentum - torque * step - Control::beforeTarget(t, step, 'g');

			PositionVector inertial_correction = initial_momentum - torque * step + Control::getRotationMomentum();

			// Control_order
			for (int i = 0; i < 2; i++)
			{
				if (control_order[i] == 'm')
				{
					//correction = sat.setMagneticMomentum(torquelike_correction, magnetic_field);
				}
				else if (control_order[i] == 'r')
				{
					//std::cout << time << '\t';
					correction = sat.setReactionWheelsMomentum(inertial_correction);
					correction = correction + sat.discharge('r');
				}
				
				else if (control_order[i] == 'k')
				{	
					// find new value of phi "using thrusters"

					phi2 = phi;

					gyrostats_momentum = sat.getGyrostatsMomentum() + sat.getReactionWheelsBlockMomentum3d();
					//if (std::isnan(torque[0])) torque = future_torque.get();

					// Iterations to find new value of phi
					for (int iteration = 0; iteration < ntrial; iteration++)
					{
						if (1.0 - phi.dot(phi) <= 1e-5)
						{
							std::cerr << "\033[31m#31 Rotation to a more than 180 degrees per time step on iteration " << iteration << '\n';
							std::cerr << phi << "\033[0m" << std::endl;
							integration_error = true;
							return;
						}

						compute_momentum(sat.getInertiaTensor(), gyrostats_momentum, step, '+');
						momentum = momentum + torque * step + torquelike_correction * step - initial_momentum + magnetic_field.cross(sat.getMagneticMomentum()) * 1e-9 * step;

						if (momentum.norm() <= tolf) break;

						compute_jacobian(sat.getInertiaTensor(), gyrostats_momentum, step);
						momentum = momentum * (-1.0);
						solve(jacobian, momentum); // the solution is stored in "momentum" variable

						if (momentum.norm() <= tolx) break;

						phi = phi + momentum;
					}

					// new, desired value of phi is found, now let's find RW correction to achieve the same value of phi

					gyrostats_momentum = PositionVector({0.0, 0.0, 0.0});
					compute_momentum(sat.getInertiaTensor(), gyrostats_momentum, step, '+');
					inertial_correction = initial_momentum - momentum - torque * step;
					// inertial_correction = -1.0 * inertial_correction;

					/*
					compensation_matrix = phi.skew();
					compensation_matrix[0][0] = sqrt(1.0 - phi.dot(phi));
					compensation_matrix[1][1] = sqrt(1.0 - phi.dot(phi));
					compensation_matrix[2][2] = sqrt(1.0 - phi.dot(phi));
					solve(compensation_matrix, inertial_correction);
					*/
					correction = sat.setReactionWheelsMomentum(inertial_correction);
					correction = correction + sat.discharge('r');

					phi = phi2;				
				}

			}

			// THRUSTERS
			if (correction.norm())
			{
				//std::cout << "At time " << time << " thrusters ON" << std::endl;
				//thrusters_momentum = correction;

				sat.targetFailed();
				std::cout << "\033[34mAt time " << time << " back to checkpoint\033[0m" << std::endl;
				Control::getCheckpoint(time, sat);
				sat.setSetbackTime(sat.getSetbackTime() + step);
				std::cout << time << std::endl;
				std::cout << sat.getState() << '\t' << sat.getQuaternion() << std::endl;
				target_acquired = false;
				time = time + (-1)*step;
				sat.setUpdateAngularMomentum(sat.getAngularMomentum());
				sat.setUpdateQuaternion(sat.getQuaternion());
				return;
			}
			// thrusters_momentum = torquelike_correction;
			t += step;
		}
		//thrusters_momentum = thrusters_momentum - (std::sqrt(1.0 - phi.dot(phi)) * sat.getGyrostatsMomentum() + phi.cross(sat.getGyrostatsMomentum()));
		// std::cout << "This line is executed only once" << std::endl;
	}

	elapsed_time++;
	
	gyrostats_momentum = sat.getGyrostatsMomentum() + sat.getReactionWheelsBlockMomentum3d();

	//if (std::isnan(torque[0])) torque = future_torque.get();

	// Iterations to find new value of phi
	for (int iteration = 0; iteration < ntrial; iteration++)
	{
		if (1.0 - phi.dot(phi) <= 1e-5)
		{
			std::cerr << "\033[31m#31 Rotation to a more than 180 degrees per time step on iteration " << iteration << '\n';
			std::cerr << phi << "\033[0m" << std::endl;
			integration_error = true;
			return;
		}

		compute_momentum(sat.getInertiaTensor(), gyrostats_momentum, step, '+');
		momentum = momentum + torque * step + thrusters_momentum * step - initial_momentum + magnetic_field.cross(sat.getMagneticMomentum()) * 1e-9 * step;

		if (momentum.norm() <= tolf) break;

		compute_jacobian(sat.getInertiaTensor(), gyrostats_momentum, step);
		momentum = momentum * (-1.0);
		solve(jacobian, momentum); // the solution is stored in "momentum" variable

		if (momentum.norm() <= tolx) break;

		phi = phi + momentum;
	}

	compute_momentum(sat.getInertiaTensor(), gyrostats_momentum, step, '-');
	sat.setUpdateAngularMomentum(momentum - gyrostats_momentum);
	

	sat.setThrustersMomentum(thrusters_momentum, step);
	sat.setTargetFuel(sat.getThrustersMomentum().sum());

	if (phi.dot(phi) > 1.0 - 1e-5)
	{
		std::cerr << "\033[31m#31 Too much rotation: phi = " << phi << "\033[0m" << std::endl;
		integration_error = true;
		return;
	}
	//sat.setQuaternion(sat.getQuaternion() * boost::math::quaternion<double>(sqrt(1.0 - phi.dot(phi)), phi[0], phi[1], phi[2]));
	sat.setUpdateQuaternion(sat.getQuaternion() * Quaternion(phi));

	if (stop_time(sat, time + step)) sat.make_telemetry = true;
	
}

double AttitudeIntegrator::comparison(const Satellite& sat1, const Satellite& sat2)
{

	//boost::math::quaternion<double> quat = sat1.getQuaternion() - sat2.getQuaternion();
	Quaternion quat = sat1.getQuaternion() - sat2.getQuaternion();
	/*
	double w = quat.R_component_1();
	double x = quat.R_component_2();
	double y = quat.R_component_3();
	double z = quat.R_component_4();
	*/
	double w = quat.get_w();
	double x = quat.get_x();
	double y = quat.get_y();
	double z = quat.get_z();
	return sqrt(w * w + x * x + y * y + z * z);

	/*
	boost::math::quaternion<double> q1 = sat1.getQuaternion();
	boost::math::quaternion<double> q2 = sat2.getQuaternion();

	double w1, w2, x1, x2, y1, y2, z1, z2;
	w1 = q1.R_component_1();
	w2 = q2.R_component_1();
	
	x1 = q1.R_component_2();
	x2 = q2.R_component_2();

	y1 = q1.R_component_3();
	y2 = q2.R_component_3();

	z1 = q1.R_component_4();
	z2 = q2.R_component_4();

	return sqrt(fabs(1.0 - (w1*w2 + x1*x2 + y1*y2 + z1*z2)));
	*/
}

AttitudeIntegrator::AttitudeIntegrator(): Integrator()
{
	ntrial = 4;
	tolx = 1e-15;
	tolf = 1e-15;
	correction[0] = 0.0; correction[1] = 0.0; correction[2] = 0.0;
	gap = 0.0;
	control_order = Control::getControlOrder();
}

AttitudeIntegrator::AttitudeIntegrator(Satellite* sat, Time* time, double interval, double step, double output_step, double autostep, double tolerance,
	int _ntrial, double _tolx, double _tolf):
	Integrator(sat, time, interval, step, output_step, autostep, tolerance)
{
	ntrial = _ntrial;
	tolx = _tolx;
	tolf = _tolf;
	correction[0] = 0.0; correction[1] = 0.0; correction[2] = 0.0;
	gap = 0.0;
	if (Astrometry::no_ephemeris)
	{
		Astrometry::get_ephemeris();
		Astrometry::no_ephemeris = false;
	}
	control_order = Control::getControlOrder();
}

AttitudeIntegrator::~AttitudeIntegrator()
{
	if (!Astrometry::no_ephemeris)
		kclear_c();
	Astrometry::no_ephemeris = true;
}
