#pragma once

// World constants
class WORLD
{
public:
	static double AU; // km
	static double SPEED_OF_LIGHT; // m / sec
	static double mu_0; // N / A^2
};

// Earth constants
class EARTH
{
public:
	static const char* name;
	static double RADIUS ; // m
	static double GM; // m^3 /sec^2
	static double mean_earth_rotation_rate; // rad / sec
	static double mu_E; // A * m^2
};

// Sun constants
class SUN
{
public:
	static const char* name;
	static double GM; // km^3 / sec^2
	static double RADIUS; // km
	static double FLUX; // Watt / m^2
};