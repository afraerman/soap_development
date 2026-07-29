#pragma once
class Satellite
{
private:
	StateVector pv;
	double mass;
	Matrix inertia_tensor;
	std::vector<Polygon> polygons;
	std::vector<Polygon> solar_panels;
	std::string hdf5_filename = "";
	std::vector<AttitudeController> gyrostats;
	std::vector<AttitudeController> new_gyrostats;
	std::vector<AttitudeController> magnetorquers;
	std::vector<AttitudeController> thrusters;
	std::vector<AttitudeController> correction_thrusters; // двигатели коррекции
	std::vector<std::vector<ReactionWheel>> reaction_wheels;
	PositionVector pulse_engine_location;
	//boost::math::quaternion<double> quaternion;
	Quaternion quaternion;
	PositionVector angular_velocity;
	PositionVector angular_momentum;
	PositionVector magnetic_momentum;
	PositionVector thrusters_momentum;
	PositionVector sun_position;

	// preserving attitude
	bool counterrotate;
	bool need_for_update;
	//std::vector<boost::math::quaternion<double>> stop_attitudes;
	//std::vector<Quaternion> stop_attitudes;
	std::vector<std::vector<Time>> stop_periods;

	// scan mode
	std::vector<std::vector<Time>> scan_periods;
	std::vector<PositionVector> scan_velocities;

	// forces dump (discharge)
	std::vector<std::vector<Time>> dump_periods;

	// Slew
	std::vector<std::vector<Time>> slew_periods;
	std::vector<Quaternion> slew_attitudes;

	// Pulse engines
	std::vector<Time> pulse_periods;
	std::vector<PositionVector> pulse_forces;

	// Correction engines
	std::vector<std::vector<Time>> correction_periods;
	std::vector<PositionVector> correction_pulses;

	std::string target;

	std::string orbit_filename;

	// Update values
	StateVector upd_pv;
	PositionVector upd_angular_momentum;
	Quaternion upd_quaternion;

	/// @brief Sort periods of attitude modes by start time from the earliest to lattest.
	void sortTransitionPeriods();

	struct OutputInfo {
		// known values
		Time start_time;
		PositionVector target_momentum; // for scan
		Quaternion target_quat; // for stop
		double duration;
		std::string mode;
		
		// unknown values
		double fuel;
		PositionVector momentum;
		bool isObserved;
		double sd_time; // slew/dump time
	};

	static inline std::vector<OutputInfo> all_modes{};

	double setback_time;


public:
	static inline int target_index{0};
	Satellite();
	Satellite(StateVector&, const Matrix&);
	//Satellite(StateVector&, const Matrix&, double, const std::vector<Polygon>&, const boost::math::quaternion<double>&);
	Satellite(StateVector&, const Matrix&, double, const std::vector<Polygon>&, const Quaternion&);

	StateVector getState() const;
	PositionVector getPosition() const;
	PositionVector getVelocity() const;
	double getMass() const;
	Matrix getInertiaTensor() const;
	std::vector<Polygon> getPolygons() const;
	std::vector<Polygon> getSolarPanels() const;
	std::vector<AttitudeController> getGyrostats() const;
	std::vector<AttitudeController> getMagnetorquers() const;
	std::vector<AttitudeController> getThrusters() const;
	std::vector<AttitudeController> getCorrectionThrusters() const;
	std::vector<std::vector<ReactionWheel>> getReactionWheelsBlock() const;
	PositionVector getPulseEngineLocation() const;
	//boost::math::quaternion<double> getQuaternion() const;
	Quaternion getQuaternion() const;
	PositionVector outputQuaternion() const;
	PositionVector getGyrostatsMomentum() const;
	PositionVector getAngularVelocity() const;
	PositionVector getAngularMomentum() const;
	PositionVector getMagneticMomentum() const;
	PositionVector getReactionWheelsBlockMomentum(int i=0) const;
	PositionVector getReactionWheelsBlockMomentum3d() const;
	PositionVector getThrustersMomentum() const;
	bool getCounterrotation() const;
	std::vector<std::vector<Time>> getStopTimes() const;
	std::vector<std::vector<Time>> getScanTimes() const;
	std::vector<std::vector<Time>> getDumpTimes() const;
	std::vector<std::vector<Time>> getSlewTimes() const;
	std::vector<std::vector<Time>> getCorrectionTimes() const;
	std::vector<Time> getPulseTimes() const;
	PositionVector getTargetMomentum(const Time& t) const;
	//boost::math::quaternion<double> getTargetQuaternion(const Time& t) const;
	Quaternion getTargetQuaternion(const Time& t) const;
	PositionVector getPulseAcceleration(const Time& t) const;
	Time getTargetTime(const Time& t);
	std::string getTarget() const;
	std::string getOrbitFilename() const;
	PositionVector getSunPosition() const;
	double getSetbackTime() const;

	/// @brief Minimum theoretical time to dump all reaction wheels
	double getDumpDuration() const;

	// Function for MM shield6
	std::vector<PositionVector> getShield6Normals() const;

	std::vector<OutputInfo> getOutputInfo() const;
	
	std::string getHdfFile() const;

	OutputInfo getNextTarget() const;
	void targetFailed();
	void deleteFailedTarget();

	
	void update();
	int checkAttitudeModes();

	void setState(const StateVector&);
	void setUpdateState(const StateVector&);
	void setMass(double);
	void setPosition(PositionVector&);
	void setPosition(const PositionVector&);

	void setInertiaTensor(const Matrix&);
	void setAngularVelocity(const std::vector<double>&);
	void setUpdateAngularVelocity(const PositionVector&);
	void setAngularMomentum(const PositionVector&);
	void setUpdateAngularMomentum(const PositionVector&);
	void setPolygons(const std::vector<Polygon>&);
	void setSolarPanels(const std::vector<Polygon>&);
	void setGyrostats(const std::vector<AttitudeController>&);
	void setMagnetorquers(const std::vector<AttitudeController>&);
	void setThrusters(const std::vector<AttitudeController>&);
	void setCorrectionThrusters(const std::vector<AttitudeController>&);
	void setReactionWheelsBlock(const std::vector<ReactionWheel>&);
	void setPulseEngineLocation(const PositionVector& location);
	//void setQuaternion(const boost::math::quaternion<double>&);
	void setQuaternion(const Quaternion&);
	void setUpdateQuaternion(const Quaternion&);
	void setSunPosition(const PositionVector&);
	void setSetbackTime(double);
	
	void setTargetDuration(const Time&, const double);
	void setTargetDuration(const double);
	void setTargetFuel(double);
	void setTargetMomentum(const PositionVector&);

	void setHdfFile(const std::string&);

	
	// NOT IN USE
	PositionVector setControlMomentum(const PositionVector&, char);

	/// @brief Increase gyrostats kinetic momentum by a given value
	/// @param mom Kinetic momentum to add
	/// @return Remaining kinetic momentum (gyrostats have limits)
	PositionVector setGyrostatsMomentum(const PositionVector& mom);

	/// @brief Set magnetoruqers magnetic momentums to compensate as much torque as possible
	/// @param tor Torque to compensate
	/// @param magnetic_field Vector of magnetic induction [nT]
	/// @return Remainig torque
	PositionVector setMagneticMomentum(const PositionVector& tor, const PositionVector& magnetic_field);
	void setMagneticMomentumFromFile(const PositionVector &magn);
	
	/// @brief Increases reaction wheels kinetic momentums to compensate given kinetic momentum
	/// @param mom Kinetic momentum to compensate
	/// @return Remaining kinetic momentum
	PositionVector setReactionWheelsMomentum(const PositionVector& mom);

	/// @brief Fully discharge given control system
	/// @param system r - for reaction wheels block, g - for gyrostats, m - magnetorquers
	/// @return Kinetic momentum stored in gyrostats or magnetic momentum stored in magnetorquers
	PositionVector discharge(char system);

	/// @brief Experimental functions: fully discharge given control system (all components!)
	/// @param system r - for reaction wheels block, g - for gyrostats, m - magnetorquers
	/// @return Kinetic momentum stored in gyrostats or magnetic momentum stored in magnetorquers
	PositionVector discharge_all(char system);

	/// @brief Forcedly discharges all elements of System, accounting for maximum dump speed
	/// @param system r - for reaction wheels block
	/// @param step - integration step
	PositionVector forced_dump(char system, const double step);

	/// @brief Set thrusters torque. No limitations
	/// @param tor Torque
	void setThrustersMomentum(const PositionVector& tor, double step);

	void requestUpdate();

	// NOT IN USE
	void enableCounterrotation();
	// IN USE FOR NO REASON (in readfile)t
	void disableCounterrotation();

	//void setStopMotion(const Time& start, const Time& stop, const boost::math::quaternion<double>& quat);
	void setStop(const Time& start, const Time& stop);
	void setScan(const Time& start, const Time& stop, const PositionVector& ang_vel);
	void setDump(const Time& start, const Time& stop);
	void setSlew(const Time& start, const Time& stop, const Quaternion& quat);
	void setPulse(const Time& t, const PositionVector& force);
	void setCorrection(const Time& start, const Time& stop, const PositionVector& pls);

	/// @brief Rotate solar panels to face the Sun as much as possible
	/// @param s GCRF Sun position
	void rotateSolarPanels(const PositionVector& s);

	/// For Spectr-R change polygons according to specific attitude.
	void setNewPolygons();

	void setOrbitFilename(const std::string& filename);

	void set_to_default();

	void mergeAttitudeModes();

	bool make_telemetry = false;
	
	~Satellite();
};