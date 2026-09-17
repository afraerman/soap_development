#pragma once

#include<memory>
#include<mutex>
#include<condition_variable>
#include<thread>
#include<atomic>
#include<optional>
#include<queue>
#include<functional>

class SRPManager
{
private:
	SRPManager() = delete;

	struct Job {
		double sx, sy, sz;
		int max_reflections;
	};

	static void workerLoop();

	static std::unique_ptr<SRPEngine> srp_engine;
	static bool                       srp_engine_ready;
	static bool                       initialized_;
	static bool                       srp_calculated;
	static std::thread                worker_;
	static std::mutex                 mutex_;
	static std::condition_variable    cv_job_;
	static std::condition_variable    cv_done_;
	static std::queue<Job>            jobs_;
	static std::optional<SRPResult>   result_;
	static std::atomic<bool>          stop_;
	static std::atomic<bool>          busy_;

public:
	static void initSRPEngine(const std::string& filename);
	static void warmupSRP();
	static void shutdown();

	static void launchJob(Satellite& sat, const Time& time, int max_reflections = 2);
	static void launchAsync(const PositionVector& sun_direction, int max_reflections = 2);
	static bool isBusy();
	static void cancelPending();

	static SRPResult getResult();

	static bool getSrpCalculated();
	static void setSrpCalculated(const bool state);
};