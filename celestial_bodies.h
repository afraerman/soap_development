#pragma once

// World constants
class WORLD
{
public:
	static constexpr double AU = 149597870.700; // km
	static constexpr double SPEED_OF_LIGHT = 299792458.0; // m / sec
	static constexpr double mu_0 = 1.25663706212*1e-6; // N / A^2
};

// Earth constants
class EARTH
{
public:
	static constexpr const char* name = "earth";
	static constexpr double RADIUS = 6378136.3; // m
	static constexpr double GM = 398600441500000.0; // m^3 /sec^2
	static constexpr double mean_earth_rotation_rate = 7.2921158553*1e-5; // rad / sec
	static constexpr double mu_E = M_2_PI * 1e22; // A * m^2
};

// Sun constants
class SUN
{
public:
	static constexpr const char* name = "sun";
	static constexpr double GM = 132712440017.99; // km^3 / sec^2
	static constexpr double RADIUS = 695990; // km
	static constexpr double FLUX = 1361.0; // Watt / m^2
};