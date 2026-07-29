#include "stdafx.h"

void OrbitIntegrator::generateSubsteps(double T)
{
	static double st[] = {
		.212340538239152 , .590533135559265, .911412040487296,

		.0985350857988264,.3045357266463639,.5620251897526139,
		.8019865821263918,.9601901429485313,

		.0562625605369221,.1802406917368924,.3526247171131696,
		.5471536263305554,.7342101772154105,.8853209468390958,
		.9775206135612875,

		.0362578128832095,.1180789787899987,.2371769848149604,
		.3818827653047060,.5380295989189891,.6903324200723622,
		.8238833438370047,.9256126102908040,.9855875903511235,

		.0252736203975203,.0830416134474051,.1691751003771814,
		.2777967151090321,.4015027202328608,.5318623869104160,
		.6599918420853348,.7771593929561621,.8753807748555569,
		.9479645488728194,.9899817195383196,

		.0186103650109879,.0614755408992690,.1263051786933106,
		.2098429717265625,.3078989982803983,.4155560359786595,
		.5274156139958823,.6378686027177612,.7413764592942375,
		.8327489886084423,.9074047753009974,.9616018612603216,
		.9926353489739107 };

	switch (nor)
	{
	case 7:
		substeps = std::vector<double>(3);
		for (int i = 0; i < 3; i++)
		{
			substeps[i] = st[i] * T;
		}
		break;
	case 11:
		substeps = std::vector<double>(5);
		for (int i = 0; i < 5; i++)
		{
			substeps[i] = st[i + 3] * T;
		}
		break;
	case 15:
		substeps = std::vector<double>(7);
		for (int i = 0; i < 7; i++)
		{
			substeps[i] = st[i + 8] * T;
		}
		break;
	case 19:
		substeps = std::vector<double>(9);
		for (int i = 0; i < 9; i++)
		{
			substeps[i] = st[i + 15] * T;
		}
		break;
	case 23:
		substeps = std::vector<double>(11);
		for (int i = 0; i < 11; i++)
		{
			substeps[i] = st[i + 24] * T;
		}
		break;
	case 27:
		substeps = std::vector<double>(13);
		for (int i = 0; i < 13; i++)
		{
			substeps[i] = st[i + 35] * T;
		}
		break;
	}

}

void OrbitIntegrator::generateCMatrix()
{
	int l = (int)substeps.size();
	c = Matrix(l, l);

	c[0][0] = 1.0;
	for (int i = 1; i < l; i++)
	{
		c[0][i] = -1.0 * substeps[i - 1] * c[0][i - 1];
	}
	for (int i = 1; i < l; i++)
	{
		c[i][i] = 1.0;
		for (int j = i + 1; j < l; j++)
		{
			c[i][j] = c[i - 1][j - 1] - substeps[i] * c[i][j - 1];
		}
	}
}

void OrbitIntegrator::initializeCoefficients(Time& t, bool from_the_start)
{
	if (from_the_start)
	{
		initial_alpha = Matrix((int)substeps.size(), number_of_equations);
		alpha = initial_alpha;
		alpha1.clear();
		alpha2.clear();
		number_of_iterations = 4;
	}
	else if (alpha2.size() == 0) number_of_iterations = 4;
	else number_of_iterations = 2;

	if (!(end_time == t))
	{
		alpha = initial_alpha;		
	}
	else
	{
		initial_alpha = alpha;
	}
}

void OrbitIntegrator::updateAlpha(Satellite& sat, const Time& time, int i)
{
	PositionVector forces = Forces::allForces(sat, time + substeps[i]);
	std::vector<double> new_alpha_i(number_of_equations, 0);
	for (int k = 0; k < number_of_equations; k++)
	{
		new_alpha_i[k] = (forces[k] - initial_forces[k]) / substeps[i];
	}
	for (int k = 0; k < number_of_equations; k++)
	{
		for (int j = 0; j < i; j++)
		{
			new_alpha_i[k] = (new_alpha_i[k] - alpha[j][k]) / (substeps[i] - substeps[j]);
		}
	}
	alpha[i] = new_alpha_i;
}

void OrbitIntegrator::updateCoefficients(Satellite& sat, const Time& time, int i)
{
	updateAlpha(sat, time, i);
	a_arr = c * alpha;
	a_arr = a_arr.transpose();
}

void OrbitIntegrator::interpolateCoefficients()
{
	if (alpha1.size() == 0)
	{
		alpha1 = alpha[0];
		std::cout << "first integration" << std::endl;
	}
	else if (alpha2.size() == 0)
	{
		alpha2 = alpha[0];
		std::cout << "second integration" << std::endl;

		for (int k = 0; k < alpha.getColumns(); k++)
			alpha[0][k] = 2.0 * alpha2[k] - alpha1[k];
	}
	else
	{
		alpha1 = alpha2;
		alpha2 = alpha[0];

		for (int k = 0; k < alpha.getColumns(); k++)
			alpha[0][k] = 2.0 * alpha2[k] - alpha1[k];
	}
}

int OrbitIntegrator::getNextState(Satellite& sat, Time& t)
{

	std::string str;
	double x, y, z, vx, vy, vz;
	StateVector state;
	Time current_time;

	std::ifstream orbit_file(sat.getOrbitFilename());
	if (!orbit_file.is_open())
	{
		std::cout << "No such orbit file" << std::endl;
		return 1;
	}

	getline(orbit_file, str);
	while (str != "META_STOP")
	{
		getline(orbit_file, str);
	}
	while (orbit_file >> str >> x >> y >> z >> vx >> vy >> vz)
	{
		current_time = Time(str);
		if (current_time > t)
		{
			break;
		}
	}
	state = StateVector(PositionVector({x, y, z}), PositionVector({vx, vy, vz}));
	sat.setUpdateState(state);
	sat.requestUpdate();

	return 0;
}

void OrbitIntegrator::integrationMethod(Satellite& sat, Time& t, double t_final, bool from_the_start)
{
	if (sat.getOrbitFilename().length() > 1)
	{
		if (getNextState(sat, t))
		{
			integration_error = true;
		}
		return;
	}
	generateSubsteps(t_final);
	generateCMatrix();
	
	PositionVector initial_position = sat.getPosition();
	initial_forces = Forces::allForces(sat, t);
	initializeCoefficients(t, from_the_start);

	a_arr = c * alpha;
	a_arr = a_arr.transpose();
	
	//PositionVector new_position;
	PositionVector _mult;
	PositionVector powered_substeps;
	PositionVector velocity = sat.getVelocity();
	StateVector st, initial_state;

	initial_state = sat.getState();
	
	int l = (int)substeps.size();

	for (int iteration = 0; iteration < number_of_iterations; iteration++)
	{
		// for each substep
		for (int i = 0; i < l; i++)
		{
			_mult = velocity * substeps[i];
			current_position = initial_position + _mult;

			_mult = initial_forces * (pow(substeps[i], 2) / 2.0);
			current_position = current_position + _mult;
			for (int j = 3; j < 3 + l; j++)
			{
				powered_substeps[j - 3] = pow(substeps[i], j) / double(j*(j - 1));
			}
			_mult = mul(a_arr, powered_substeps);
			current_position = current_position + _mult;

			st = StateVector(current_position, velocity);
			sat.setState(st);

			updateCoefficients(sat, t, i);
		}
	}

	// final values
	_mult = velocity * t_final;
	current_position = initial_position + _mult;

	_mult = initial_forces * (pow(t_final, 2) / 2.0);
	current_position = current_position + _mult;
	for (int j = 3; j < 3 + l; j++)
	{
		powered_substeps[j - 3] = pow(t_final, j) / double(j*(j - 1));
	}
	_mult = mul(a_arr, powered_substeps);
	current_position = current_position + _mult;

	_mult = initial_forces * t_final;
	velocity = velocity + _mult;
	for (int j = 2; j < 2 + l; j++)
	{
		powered_substeps[j - 2] = pow(t_final, j) / double(j);
	}
	_mult = mul(a_arr, powered_substeps);
	velocity = velocity + _mult;

	// update
	st = StateVector(current_position, velocity);
	//Astrometry::rotationMatrices(t);
	sat.setUpdateState(st);
	sat.requestUpdate();
	sat.setState(initial_state);
	
	end_time = t;
	interpolateCoefficients();
	
	
	// EMELIANOV //
	/*
	PositionVector position = sat.getPosition();
	PositionVector velocity = sat.getVelocity();

	double x[7];
	double v[7];

	x[0] = 0.0; v[0] = 0.0;
	x[1] = position[0]; x[4] = velocity[0];
	x[2] = position[1]; x[5] = velocity[1];
	x[3] = position[2]; x[6] = velocity[2];

	int nf = 0, ns = 0;
	int *nnf = &nf, *nns =&ns;
	rada27e(t, x, v, 0.0, t_final, 0.2, -4, 6, 2, &nf, &ns, 1, 7, 0);

	position = PositionVector({x[1], x[2], x[3]});
	velocity = PositionVector({x[4], x[5], x[6]});

	StateVector st(position, velocity);

	sat.setState(st);
	*/
}

double OrbitIntegrator::comparison(const Satellite& sat1, const Satellite& sat2)
{
	PositionVector pos = sat1.getPosition() - sat2.getPosition();
	PositionVector vel = sat1.getState().getVelocity() - sat2.getState().getVelocity();
	double n1 = pos.norm() / sat1.getPosition().norm();
	double n2 = vel.norm() / sat1.getState().getVelocity().norm();
	return sqrt(n1*n1 + n2*n2);
}

OrbitIntegrator::OrbitIntegrator() : Integrator()
{
	number_of_equations = 3;
	number_of_iterations = 2;
	nor = 7;
}

OrbitIntegrator::OrbitIntegrator(Satellite* satellite, Time* time, double interval, double step, double ost, bool autostep, double tolerance, int noe, int noi, int _nor):
	Integrator(satellite, time, interval, step, ost, autostep, tolerance)
{
	number_of_equations = noe;
	number_of_iterations = noi;
	nor = _nor;
	Astrometry::EOP(*time);
	Astrometry::rotationMatrices(*time);
	if (Astrometry::no_ephemeris)
	{
		Astrometry::get_ephemeris();
		Astrometry::no_ephemeris = false;
	}
}

OrbitIntegrator::~OrbitIntegrator()
{
	if (!Astrometry::no_ephemeris)
		kclear_c();
};