#include "stdafx.h"

double WORLD::AU = 149597870.691;
double WORLD::SPEED_OF_LIGHT = 299792458.0;
double WORLD::mu_0 = 1.25663706212*1e-6;

double EARTH::RADIUS = 6378136.3;
double EARTH::GM = 398600441500000.0;
double EARTH::mean_earth_rotation_rate = 7.2921158553*1e-5;
double EARTH::mu_E = M_2_PI * 1e22;
const char* EARTH::name = "earth";

double SUN::GM = 132712440017.99;
double SUN::RADIUS = 695990;
double SUN::FLUX = 1367.0;
const char* SUN::name = "sun";