#include "stdafx.h"

std::unique_ptr<SRPEngine> SRPManager::srp_engine;
bool                       SRPManager::srp_engine_ready{false};
bool                       SRPManager::initialized_{false};
bool                       SRPManager::srp_calculated{false};
std::thread                SRPManager::worker_;
std::mutex                 SRPManager::mutex_;
std::condition_variable    SRPManager::cv_job_;
std::condition_variable    SRPManager::cv_done_;
std::queue<SRPManager::Job> SRPManager::jobs_;
std::optional<SRPResult>   SRPManager::result_;
std::atomic<bool>          SRPManager::stop_{false};
std::atomic<bool>          SRPManager::busy_{false};

void SRPManager::initSRPEngine(const std::string& filename)
{
	std::lock_guard lock(mutex_);
	if (initialized_)
		return;
	
	try
	{
		srp_engine = std::make_unique<SRPEngine>(filename);
		stop_ = false;
		busy_ = false;
		result_.reset();
		while (!jobs_.empty()) jobs_.pop();

		worker_ = std::thread(&SRPManager::workerLoop);
		initialized_ = true;
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

void SRPManager::launchJob(Satellite& sat, const Time& time, int max_reflections)
{
	if (srp_calculated)
	{
		//forces += solar_pressure_force * 0.001 / sat.getMass(); // m/s^2 -> km/s^2
		return;
	}

	// call to this function meand that there is hdf5_file -> init happened (as well as warm-up)
	// SRPEngine engine(sat.getHdfFile());

	double state[6];
	double lt, ef;
	PositionVector s;
	SRPResult res;

	SpiceDouble et = time.ET();
	spkezr_c("sun", et, "J2000", "NONE", "earth", state, &lt);

	PositionVector sat_pos = sat.getPosition();
	PositionVector sun_pos(std::vector<double>{state[0], state[1], state[2]});
	sat.setSunPosition(sun_pos);
	
	s = sun_pos - sat_pos;
	double r = s.norm();
	s = s * (1.0 / r);

	Astrometry::eclipseFactor(sun_pos, sat_pos);
	
	Quaternion quat = sat.getQuaternion().get_inverse();
	s = quat * s;

	launchAsync(s, max_reflections);

	setSrpCalculated(true);
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


bool SRPManager::getSrpCalculated()
{
	return srp_calculated;
}

void SRPManager::setSrpCalculated(const bool state)
{
	srp_calculated = state;
}