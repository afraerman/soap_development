#include "stdafx.h"

std::unique_ptr<SRPEngine> SRPManager::srp_engine;
bool                       SRPManager::srp_engine_ready{false};
bool                       SRPManager::initialized_{false};
int                        SRPManager::srp_calculated{0};
int                        SRPManager::steps_to_calculate{1};
std::thread                SRPManager::worker_;
std::mutex                 SRPManager::mutex_;
std::condition_variable    SRPManager::cv_job_;
std::condition_variable    SRPManager::cv_done_;
std::queue<SRPManager::Job> SRPManager::jobs_;
std::optional<SRPResult>   SRPManager::result_;
std::atomic<bool>          SRPManager::stop_{false};
std::atomic<bool>          SRPManager::busy_{false};
double                     SRPManager::phi0_distance_scaling{1.0};

void SRPManager::initSRPEngine(const std::string& filename)
{
	std::lock_guard lock(mutex_);
	if (initialized_)
		return;
	
	try
	{
		std::stringstream ss_str(filename);
		char delimiter='/';
		std::string token, folder="";
		std::vector<std::string> tokens;

		while (getline(ss_str, token, delimiter))
		{
			tokens.push_back(token);
		}

		if (!tokens.empty())
		{
			tokens.pop_back();
		}

		for (auto& t: tokens)

		{
			folder = folder + "/" + t;
		}

		srp_engine = std::make_unique<SRPEngine>(folder);
		srp_engine->dataset().load(filename);

		g_srp_phi0 = 4.56e-6;

		stop_ = false;
		busy_ = false;
		result_.reset();
		while (!jobs_.empty()) jobs_.pop();

		worker_ = std::thread(&SRPManager::workerLoop);
		initialized_ = true;
		srp_calculated = 0;
	}
	catch (const std::exception &e)
	{
		std::cerr << "\033[31m#1312_hdf5_file Can't open file: " << e.what() << std::endl;
		throw std::runtime_error("");
	}
	catch (...)
	{
		std::cerr << "\033[31m#1312_hdf5_file Can't open file: " << filename << "\033[0m" << std::endl;
		throw std::runtime_error("");
	}
}

void SRPManager::warmupSRP()
{
	if (!initialized_)
		throw std::runtime_error("SRPManager::warmup() before init()");
	if (srp_engine_ready)
		return;

	launchAsync(PositionVector({1.0, 0.0 ,0.0}), 2);
	(void)getResult();

    srp_engine_ready = true;
}

void SRPManager::shutdown()
{
	{
		std::lock_guard lock(mutex_);
		if (!initialized_)
			return;
		stop_ = true;
	}
	cv_job_.notify_one();
	if (worker_.joinable())
		worker_.join();

	std::lock_guard lock(mutex_);
	srp_engine.reset();
	initialized_ = false;
	srp_engine_ready = false;
}

void SRPManager::launchJob(Satellite& sat, const Time& time)
{
	if (srp_calculated)
	{
		srp_calculated--;
		//forces += solar_pressure_force * 0.001 / sat.getMass(); // m/s^2 -> km/s^2
		return;
	}

	srp_calculated = steps_to_calculate - 1;

	// call to this function means that there is hdf5_file -> init happened (as well as warm-up)
	// SRPEngine engine(sat.getHdfFile());

	double state[6];
	double lt, ef;
	PositionVector s;
	SRPResult res;
	int max_reflections = 2;

	SpiceDouble et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sat_pos = sat.getPosition();
	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	sat.setSunPosition(sun_pos);
	
	s = sun_pos - sat_pos;
	double r = s.norm();
	s = s * (1.0 / r);

	setPhi0DistanceScaling(SUN::FLUX / WORLD::SPEED_OF_LIGHT * pow(WORLD::AU / r, 2));

	Astrometry::eclipseFactor(sun_pos, sat_pos);
	
	Quaternion quat = sat.getQuaternion().get_inverse();
	s = quat * s;

	if (sat.getMaxReflections() != 0)
		max_reflections = sat.getMaxReflections();

	launchAsync(s, max_reflections);
}

void SRPManager::launchAsync(const PositionVector& sun_direction, int max_reflections)
{
	if (!initialized_)
		throw std::runtime_error("SRPManager::launchAsync() before init()");

	{
		std::lock_guard lock(mutex_);
		while (!jobs_.empty())
			jobs_.pop();
		jobs_.push(Job(sun_direction[0], sun_direction[1], sun_direction[2], max_reflections));
		result_.reset();
		busy_ = true;
	}
	cv_job_.notify_one();
}

SRPResult SRPManager::getResult()
{
	if (!initialized_)
		throw std::runtime_error("SRPManager::getResult() -- no result (shutdown?)");
	
	std::unique_lock lock(mutex_);
    cv_done_.wait(lock, [] {
        return result_.has_value() || stop_;
    });

	SRPResult result = *result_;
	busy_ = false;
	return result;
}

bool SRPManager::isBusy()
{
	return busy_.load();
}

void SRPManager::cancelPending()
{
	std::lock_guard lock(mutex_);
	while (!jobs_.empty())
		jobs_.pop();
	result_.reset();
	busy_ = false;
}

void SRPManager::workerLoop()
{
	while (true) {
		Job job;
		{
			std::unique_lock lock(mutex_);
			cv_job_.wait(lock, [] {
				return stop_ || !jobs_.empty();
			});

			if (stop_ && jobs_.empty())
				return;

			job = jobs_.front();
			jobs_.pop();
		}

		SRPResult result;
		try
		{
			srp_engine->setSunDirection(job.sx, job.sy, job.sz);
			srp_engine->setMaxReflections(job.max_reflections);

			if (is_rtx_device_available())
				result = srp_engine->compute(SRPMethod::CentroidRTX);
			else if (is_cuda_device_available())
				result = srp_engine->compute(SRPMethod::CentroidGPU);
			else
				result = srp_engine->compute(SRPMethod::CentroidCPU);
		}
		catch (const std::exception& e)
		{
			std::cerr << "\033[31mSRPManager worker exception: " << e.what() << "\033[0m\n";
		}

		{
			std::lock_guard lock(mutex_);
			result_ = result;
			busy_ = false;
		}
		cv_done_.notify_all();
	}
}



void SRPManager::setSrpCalculated(const int state)
{
	srp_calculated = state;
}

void SRPManager::setHowOftenCalculate(const int dur)
{
	steps_to_calculate = dur;
}

int SRPManager::getSrpCalculated()
{
	return srp_calculated;
}

double SRPManager::getPhi0DistanceScaling()
{
	return phi0_distance_scaling;
}

void SRPManager::setPhi0DistanceScaling(double sc)
{
	phi0_distance_scaling = sc;
}