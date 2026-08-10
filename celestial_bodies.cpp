#include "stdafx.h"

constexpr double WORLD::AU = 149597870.700;
constexpr double WORLD::SPEED_OF_LIGHT = 299792458.0;
constexpr double WORLD::mu_0 = 1.25663706212*1e-6;

constexpr double EARTH::RADIUS = 6378136.3;
constexpr double EARTH::GM = 398600441500000.0;
constexpr double EARTH::mean_earth_rotation_rate = 7.2921158553*1e-5;
constexpr double EARTH::mu_E = M_2_PI * 1e22;
constexpr char* EARTH::name = "earth";

constexpr double SUN::GM = 132712440017.99;
constexpr double SUN::RADIUS = 695990;
constexpr double SUN::FLUX = 1361.0;
constexpr char* SUN::name = "sun";