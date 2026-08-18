#pragma once
#define _USE_MATH_DEFINES

#include<iostream>
#include<ostream>
#include<fstream>
#include<sstream>
#include<format>
#include<iomanip>
#include<cmath>
#include<map>
#include<limits>
#include<algorithm>

#if defined(_WIN32)
	#include<json.h>
#else
	#include<jsoncpp/json/json.h>
#endif

#include<stdint.h>
#include<./libs/sofa/sofa.h>
#include<./libs/sofa/sofam.h>
#include<vector>
#include<string>
#include<regex>
#include<future>
#include<functional>
#include<filenames.h>
#include<./libs/cspice/include/SpiceUsr.h>
#include "SRPLibrary.h"

#include<date_time.h>
#include<matrix.h>
#include<position_vector.h>
#include<quaternion.h>
#include<state_vector.h>
#include<celestial_bodies.h>
#include<astrometry.h>

#include<polygon.h>
#include<attitude_controller.h>
#include<reaction_wheel.h>
#include<satellite.h>

#include<forces.h>
#include<torques.h>
#include<control.h>
#include<integrator.h>
#include<orbit_integrator.h>
#include<attitude_integrator.h>
#include<full_motion_integrator.h>
#include<input.h>
#include<tests.h>

#include<stdlib.h>