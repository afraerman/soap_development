class FILENAMES
{
public:
    static std::string files_directory;
    static std::string ephemeris_filename;
    static std::string telemetry_filename;
    static std::string output_info_filename;

    static void reset_filenames();
};

class DEVELOPER
{
public:
    static bool attitude_testing_mode;
};