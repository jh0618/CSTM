//
// Created by jaehoon on 23. 5. 21.
//

#ifndef CAMEL_RAISIM_PROJECTS_CUBICSWINGLEGLOCAL_HPP
#define CAMEL_RAISIM_PROJECTS_CUBICSWINGLEGLOCAL_HPP

#include <cmath>
#include <TrajectoryGenerator.hpp>
#include "EigenTypes.hpp"
#include "EnumClasses.hpp"
#include "SharedMemory.hpp"
#include "CubicReplanning.hpp"
#include <RobotMath.hpp>

class CubicSwingLegLocal{
public:
    CubicSwingLegLocal();
    void SetParameters();
    void SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg);
    void GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg);
    void GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg);
    void GetAccelerationTrajectory(Vec3<double> &desiredAcceleration, const int &leg);
    void ReplanningXY(Vec3<double>& desiredPosition, const int& leg);
private:
    SharedMemory* sharedMemory;
    CubicReplanning mCubicFootTrajectoryGenerator[12];
    bool mbIsNewTrajectory[4];
    double mReferenceTime[4];
    double mTimeDuration;

    double mLiftUpDuration;
    double mLiftUpHeight[4];
    double mTargetHeight[4];

    Vec3<double> mInitPos[4];
    Vec3<double> normal[4];
    Vec3<double> mInitPosZ[4];
};

#endif //CAMEL_RAISIM_PROJECTS_CUBICSWINGLEGLOCAL_HPP
