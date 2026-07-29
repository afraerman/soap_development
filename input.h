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

public:
	static int read_input_file(const std::string& filename, Satellite* sat, Time* time, double& interval, double& step, double& ost);
	static int read_vtk_file(const std::string& filename, std::vector<Polygon>& polygons, const std::string& koeffs_filename = "");
	static int read_json_file(const std::string& filename, Satellite* sat, Time* time, double& intergval, double& step, double& ost);
	static int read_multiple_satellites_filenames(const std::string& filename, std::vector<std::string>& filenames);
};