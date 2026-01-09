#ifndef SINGLELEGMPC_SIMULVISUALIZER_HPP
#define SINGLELEGMPC_SIMULVISUALIZER_HPP

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "unistd.h"
#include "Eigen/Dense"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include <cstdlib>
#include <filesystem>

class SimulVisualizer
{
public:
    SimulVisualizer();
    ~SimulVisualizer();
    void initRobotPose();
    void updateVisual();
    void updateCurrent();
    void openSimulation();
private:
    void openRaisimServer();
    std::string getProcessPID(const std::string& command);

private:
    int mMap;
    raisim::World* mWorld;
    raisim::RaisimServer* mServer;
    raisim::ArticulatedSystem* mRobot;
    raisim::HeightMap* mRaisimHeightMap;
    raisim::Visuals* mSlopeBox[4][25];
    raisim::Visuals* mSlopeBaseBox;
    raisim::Visuals* mSlopeContactFoot[4];
    raisim::Visuals* mSlopeShoulderPosition[4];
    raisim::Visuals* mSlopeOrientation[3];
    raisim::Visuals* mSlopeDesiredOrientation[3];
    SharedMemory* sharedMemory;
    _HWD_* HWData;

    double mRawEulerAngle[3];
    double tempV[3];
    Vec3<double> mShoulderPosition[4];

    raisim::Visuals* mStumbleDesired;

    int mMulti;
    enum eMap
    {
        MAP_DEFAULT,
        MAP_STAIR,
        MAP_HILL,
        MAP_SLOPE,
        MAP_ROLL_SLOPE,
        MAP_UNEVEN_TERRAIN,
        MAP_UNEVEN_TERRAIN_SLOPE
    };
};

#endif //SINGLELEGMPC_SIMULVISUALIZER_HPP
