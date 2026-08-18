#pragma once

class Control
{
private:
    static PositionVector u(double t);
    static PositionVector u2(double t);
    static PositionVector initial_momentum;
    static PositionVector target_momentum;
    static PositionVector rotation_momentum;
    static double target_time;
    static double gap;
    static std::vector<char> control_order;
    
    static std::vector<PositionVector> rotation_momentums;
    static std::vector<double> target_times;

    static PositionVector rotation_axis;

    static void check_angles(const Matrix& r, double& psi, double& theta, double& phi);
    static Matrix getTransformMatrixFromApex(const PositionVector& angles, const std::string& apex);


    // SLEW parameters
    static PositionVector slew_speedup_velocity; // 4-мерный вектор приращения моментов маховиков (> 0)
    static PositionVector slew_dump_velocity; // 4-мерный вектор приращения моментов маховиков (> 0)
    static PositionVector slew_plateau_velocity; // 4-мерный вектор моментов маховиков на "плато"
    static double slew_speedup_time;
    static double slew_plateau_time;
    static double slew_dump_time;

    /// @brief sets dutations of speedup and speeddown (dump) for slew
    /// @param sat -- satellite
    /// @param time -- time in seconds -- duration of the slew
    /// @param angle -- 0 <= angle <= pi
    /// @param axis -- rotation axis
    /// @param step -- integration step
    static void distributeTimeForSlew(const Satellite& sat, const double time, const double angle, const PositionVector& axis, const double step);


    // CHECKPOINT
    static Satellite checkpoint_sat;
    static Time checkpoint_time;

public:
    // Setters
    static void setInitialMomentum(const PositionVector& im);
    static void setTargetMomentum(const PositionVector& tm);
    static void setRotationMomentum(const PositionVector& rm);
    static void setRotationAxis(const PositionVector& ra);
    static void setTargetTime(const double tt);
    static void setGap(const double g);
    static void setControlOrder(char first, char second);
    //static void defactorTarget(const Matrix&, const boost::math::quaternion<double>&, const boost::math::quaternion<double>&);
    

    
    // Getters
    static std::vector<char> getControlOrder();
    //static PositionVector getAngvelFromQuaternion(const boost::math::quaternion<double>&, const boost::math::quaternion<double>&);
    static PositionVector getAngvelFromQuaternion(const Quaternion&, const Quaternion&);
    static double getTargetTime();
    static double getGap();
    static PositionVector getRotationAxis();
    static PositionVector getRotationMomentum();
    //static PositionVector getSlewSpeedupVelocity();
    //static PositionVector getSlewDumpVelocity();
    //static double getSlewSpeedupTime();
    //static double getSlewPlateauTime();
    //static double getSlewDumpTime();

    
    /// @brief find mininmum possible slew time
    /// @param sat -- satellite
    /// @param angle -- 0 <= angle <= pi
    /// @param axis -- rotation axis
    /// @param step -- integration step
    static double getMinSlewTime(const Satellite& sat, double angle, const PositionVector& axis, double step, const PositionVector& initial_momentum, const PositionVector& torque);
    
    // Reaction wheels momentums redistributions (3d <-> 4d)
    static PositionVector redistributeCompensationMomentum(const PositionVector& target_mom, const PositionVector& angles, const std::string& apex="X");
    static PositionVector combineReactionWheelsBlockMomentum(const PositionVector& momentums, const PositionVector& sin_cos, const std::string& apex="X");

    // Callable preparatory functions

    static void defactorTarget(const Matrix&, const Quaternion&, const Quaternion&);
    static int setRotationFromQuat(Satellite&, const Quaternion&, const Quaternion&, const double gap, const double step, const PositionVector& initial_momentum, const PositionVector& torque);
    static void makeCheckpoint(const Time& t, const Satellite& sat);


    // Is it even in use??
    static void cancelDefactorisation();
    
    // Callable action functions

    static PositionVector beforeTarget(double current_time, double step, char system);

    /// @breif incremental rotation of reaction wheels to perform slew
    /// @param current time -- (double) time in seconds from the start of the slew
    /// @param step -- (double) integration step
    /// @param system -- (char) used system (currently not in use, always reaction wheels)
    static PositionVector performSlew(const Satellite& sat, double current_time, double step, char system);

    static void getCheckpoint(Time& t, Satellite& sat);


    static PositionVector testingControl(const Matrix& I, double current_time, double step);
};