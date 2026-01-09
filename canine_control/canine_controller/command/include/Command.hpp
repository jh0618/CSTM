#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "EnumClasses.hpp"
#include "SharedMemory.hpp"
#include "Filter.hpp"
#include "JoystickInfo.hpp"
#include "CommandMapping.hpp"
#include "RobotMath.hpp"

class Command {
public:
    Command();
    void commandFunction();

private:
    SharedMemory* sharedMemory;
    _HWD_* HWData;

    void mappingCommand();

    void setJoyInput();
    void FSMStandUpFunction();
    void FSMSitDownFunction();
    void FSMConstStandFunction();
    void FSMTrotSlowFunction();
    void FSMTrotFastFunction();
    void FSMTrotOverlapFunction();
    void FSMTrotStopFunction();
private:

    const double mCmdVelThreshold;
    const double mTrotStopThreshold;

    Eigen::Vector3d mJoyLinVel;
    Eigen::Vector3d mJoyAngVel;
    bool mHoldCommand;
    int mLastJoyCommand;

    double mStumbleRecoveryTime;
    bool mbStumbleRecovery;
    Eigen::Vector3d mGlobalStumbleStableBasePosition;
    Vec3<double> mHip2ShoulderPosition[4];
    Vec3<double> mHipPosition[4];
    Vec3<double> mShoulderPosition[4];
    Vec3<double> desiredVelocity;
    double mStumbleRelaxTime;
};


#endif //RAISIM_COMMAND_H
