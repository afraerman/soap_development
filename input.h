#pragma once

class Input
{
private:
	struct Node {
    double x, y, z;
	};

	struct Element {
    std::vector<int> node_ids;
	};

	static inline std::map<std::string, std::map<std::string, bool>> parameters_dict = {
		{"Simulation", {
			{"start_time", false},
			{"interval", false},
			{"step", false},
			{"output_step", false},
			{"orbit_file", false}
			}
		},

		{"Spacecraft", {
			{"state_vector", false},
			{"mass", false},
			{"quaternion", false},
			{"inertia_tensor", false},
			{"angular_velocity", false}
			}
		},

		{"Geometry", {
			{"vkt_file", false},
			{"hdf5_file", false},
			{"polygons", false},
			{"solar_panels", false}
			}
		},

		{"Control_systems", {
			{"gyrostats", false},
			{"reaction_wheels", false},
			{"magnetorquers", false},
			{"pulse_engines", false},
			{"correction_engines", false},
			{"control_order", false}
			}
		},

		{"Orbital_control", {
			{"corrections", false}
			}
		},

		{"Attitude_modes", {
			{"stop_motion", false},
			{"slew_motion", false},
			{"scan_motion", false},
			{"dump", false}
			}
		},

		{"Forces_and_torques", {
			{"gravity_force", false},
			{"gravity_order", false},
			{"outer_gravity", false},
			{"solar_pressure_force", false},
			{"gravity_torque", false},
			{"solar_pressure_torque", false},
			{"magnetic_torque", false}
			}
		},

		{"Filenames", {
			{"egm_path", false},
			{"eop_path", false},
			{"tls_path", false},
			{"eph_path", false},
			{"igrf_path", false},
			{"gm_path", false},
			{"save_path", false},
			{"telemetry_path", false},
			{"output_info_path", false}
			}
		}
	};

	static void input_statistics();
	static void require_array_of_doubles(const Json::Value& node, const std::string& path, size_t expected_size);
	static std::vector<double> get_doubles(const Json::Value& node, const std::string& path, size_t expected_size);
	static std::vector<Json::Value> get_parameters(const Json::Value& node, const std::string& path, size_t expected_size);

public:
	/// Legacy currently-not-in-use function. Function in use: read_json_file
	static int read_input_file(const std::string& filename, Satellite* sat, Time* time, double& interval, double& step, double& ost);
	
	/// Reads VKT file of geometry
	/// @param filename vkt-filename
	/// @param polygons vector in which polygons will be stored
	/// @param coeffs_filename  optional filename of reflectivity and specularity coefficients
	/// @return 1 if anything goes wrong while reading file. Error 02 -- error while opening vkt file, 1_? -- incorrect structure of VTK-file
	static int read_vtk_file(const std::string& filename, std::vector<Polygon>& polygons, const std::string& coeffs_filename = "");

	/// Reads an input JSON file
	/// @param filename  input json filename
	/// @param sat satellite
	/// @param time timestamp
	/// @param interval interval of integration
	/// @param step step of integration
	/// @param ost step of output data
	/// @param screen_check Flag for enabeling screen check while integrating
	/// @return 1 if anything goes wrong. Error 01 -- error while opening json, 1_#name -- incorrect field in json file
	static int read_json_file(const std::string& filename, Satellite* sat, Time* time, double& interval, double& step, double& ost, bool& screen_check);
	static int read_multiple_satellites_filenames(const std::string& filename, std::vector<std::string>& filenames);
};