#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include<sofa.h>
#include<sofam.h>
#include "filenames.h"
#include<SpiceUsr.h>
#include "SRPLibrary.h"
#include "../date_time.h"
#include "../matrix.h"
#include "../position_vector.h"
#include "../quaternion.h"
#include "../state_vector.h"
#include "../astrometry.h"
#include "../polygon.h"
#include "../attitude_controller.h"
#include "../reaction_wheel.h"
#include "../satellite.h"


#include "../srp_manager.h"
using Catch::Approx;



TEST_CASE("SRP Manager: Multiple results are equals", "[srp_manager]") {
    // arrange
    Satellite sat;
    sat.setPosition(PositionVector({0.0, 6900.0, 0.0}));
    sat.setHdfFile(std::string(SOAP_SOURCE_DIR)+"/tests/");

    Time time(2026, 9, 15, 17, 50, 0.0);

    FILENAMES::files_directory = std::string(SOAP_DATA_DIR);
	Astrometry::setEOPfile(FILENAMES::files_directory + "/eop.txt");
	Astrometry::setTLSfile(FILENAMES::files_directory + "/naif0012.tls");
	Astrometry::setEPHEMfile(FILENAMES::files_directory + "/de440.bsp");
	Astrometry::setGMfile(FILENAMES::files_directory + "/gm_de440.tpc");
	
    Astrometry::EOP(time);
	Astrometry::rotationMatrices(time);
	if (Astrometry::no_ephemeris)
	{
		Astrometry::get_ephemeris();
		Astrometry::no_ephemeris = false;
	}

    SRPManager::initSRPEngine(sat.getHdfFile());
	SRPManager::warmupSRP();

    SRPManager::launchJob(sat, time);

    // act
    auto res = SRPManager::getResult();

    // assert
    for (int i = 0; i < 5; i++)
    {
    	auto res2 = SRPManager::getResult();
    	for (int idx = 0; idx < 3; idx++)
    	{
    		REQUIRE(res.total_force[idx] == Approx(res2.total_force[idx]));
    		REQUIRE(res.total_moment[idx] == Approx(res2.total_moment[idx]));
    	}
    }
    SRPManager::shutdown();
}