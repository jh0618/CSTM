//
// Created by jaehoon on 23. 9. 15.
//

#ifndef CAMEL_CANINE_SWINGLEGLINEAR_HPP
#define CAMEL_CANINE_SWINGLEGLINEAR_HPP

#include <cmath>
#include <TrajectoryGenerator.hpp>
#include "EigenTypes.hpp"
#include "EnumClasses.hpp"
#include "SharedMemory.hpp"
#include "SwingLegReplanning.hpp"

class SwingLegLinear{
public:
    SwingLegLinear();
    void SetParameters();
    void SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg);
    void GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg);
    void GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg);
    void GetAccelerationTrajectory(Vec3<double> &desiredAcceleration, const int &leg);
    void ReplanningXY(Vec3<double>& desiredPosition, const int& leg);

private:
    void setLiftUpPDgains(const int& leg);
    void setLiftDownPDgains(const int& leg);

private:
    SharedMemory* sharedMemory;
    SwingLegReplanning mSwingLegFootTrajectoryGenerator[12];
    bool mbIsCubic1Trajectory[4];
    bool mbIsCubic2Trajectory[4];
    bool mbIsLineTrajectory[4];
    bool mbIsUpdateContactTrajectory[4];
    double mReferenceTime[4];
    double mTimeDuration;

    double mLiftUpDuration;
    double mTransitionDuration;
    double mTransitionRatio;
    double mLiftUpHeight[4];
    double mTransitionHeight[4];
    double mInitX[4];
    double mInitY[4];
    double mDesiredX[4];
    double mDesiredY[4];
    double mTargetHeight[4];
    Vec3<double> mDeisredPosition[4][3];
};

#endif //CAMEL_CANINE_SWINGLEGLINEAR_HPP
