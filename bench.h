#pragma once

struct Bench
{
	std::string attitude = "";
	std::map<std::string, bool> write = {
		{ "init", false },
		{ "rotate", false },
		{ "copmute", false },
		{ "postprocess", false },
		{ "right_side", false },
		{ "non_sun", false }
	};
	bool rotate = false;
	double factor = 1.0;
	bool make_verdict = false;
};

inline Bench bench;