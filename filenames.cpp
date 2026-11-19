#include "stdafx.h"

std::string FILENAMES::files_directory = std::string(SOAP_DATA_DIR);
std::string FILENAMES::ephemeris_filename = std::string(SOAP_SOURCE_DIR) + "/output.txt";
std::string FILENAMES::telemetry_filename = std::string(SOAP_SOURCE_DIR) + "/telemetry.txt";
std::string FILENAMES::output_info_filename = std::string(SOAP_SOURCE_DIR) + "/output_info.txt";
std::string FILENAMES::verdict_filename     = std::string(SOAP_SOURCE_DIR) + "/verdict.csv";

void FILENAMES::reset_filenames()
{
	ephemeris_filename = std::string(SOAP_SOURCE_DIR) + "/output.txt";
	telemetry_filename = std::string(SOAP_SOURCE_DIR) + "/telemetry.txt";
	output_info_filename = std::string(SOAP_SOURCE_DIR) + "/output_info.txt";
	verdict_filename   = std::string(SOAP_SOURCE_DIR) + "/verdict.csv";
}

bool DEVELOPER::attitude_testing_mode = false;