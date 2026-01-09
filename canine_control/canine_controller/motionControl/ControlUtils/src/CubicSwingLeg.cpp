//
// Created by jaehoon on 23. 4. 14.
//

#include <ControlUtils/CubicSwingLeg.hpp>


CubicSwingLeg::CubicSwingLeg()
    : mbIsNewTrajectory{ false, false, false, false }
    , mLiftUpHeight{ -0.22, -0.22, -0.22, -0.22 }
    , mTargetHeight{ -0.34, -0.34, -0.34, -0.34 }
{
    sharedMemory = SharedMemory::getInstance();
}

void CubicSwingLeg::SetParameters()
{
    mTimeDuration = sharedMemory->swingPeriod;
    mLiftUpDuration = mTimeDuration / 2;
}

void CubicSwingLeg::SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    mReferenceTime[leg] = sharedMemory->localTime;
    mCubicFootTrajectoryGenerator[leg*3].updateCubicTrajectory(initPos[0],desiredPos[0], sharedMemory->bodyBase2FootVelocity[leg][0], -sharedMemory->bodyBaseDesiredVelocity[0], sharedMemory->localTime,mTimeDuration);
    mCubicFootTrajectoryGenerator[leg*3+1].updateCubicTrajectory(initPos[1],desiredPos[1], sharedMemory->bodyBase2FootVelocity[leg][1], -sharedMemory->bodyBaseDesiredVelocity[1], sharedMemory->localTime,mTimeDuration);
    mCubicFootTrajectoryGenerator[leg*3+2].updateCubicTrajectory(initPos[2],mLiftUpHeight[leg], sharedMemory->bodyBase2FootVelocity[leg][2], 0.0,sharedMemory->localTime,mLiftUpDuration);
    mbIsNewTrajectory[leg] = true;

    for (int i = 0; i < 3; i++)
    {
        sharedMemory->swingPgain[leg][i] = 0.0;
        sharedMemory->swingDgain[leg][i] = 1.0;
    }
}

void CubicSwingLeg::GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg)
{
    desiredPosition[0] = mCubicFootTrajectoryGenerator[leg * 3].getCubicPositionTrajectory(sharedMemory->localTime);
    desiredPosition[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getCubicPositionTrajectory(sharedMemory->localTime);
    desiredPosition[2] = mCubicFootTrajectoryGenerator[leg * 3 + 2].getCubicPositionTrajectory(sharedMemory->localTime);

    if ((sharedMemory->localTime > mLiftUpDuration + mReferenceTime[leg]) && mbIsNewTrajectory[leg])
    {
        mbIsNewTrajectory[leg] = false;
        mCubicFootTrajectoryGenerator[leg * 3 + 2].updateCubicTrajectory(desiredPosition[2], mTargetHeight[leg], 0.0, 0.0, sharedMemory->localTime, mTimeDuration - mLiftUpDuration);
        for (int i = 0; i < 3; i++)
        {
            sharedMemory->swingPgain[leg][i] = 0.0;
            sharedMemory->swingDgain[leg][i] = 1.0;
        }
    }
}

void CubicSwingLeg::GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg)
{
    desiredVelocity[0] = mCubicFootTrajectoryGenerator[leg * 3].getCubicVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getCubicVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[2] = mCubicFootTrajectoryGenerator[leg * 3 + 2].getCubicVelocityTrajectory(sharedMemory->localTime);
}

void CubicSwingLeg::GetAccelerationTrajectory(Vec3<double>& desiredAcceleration, const int& leg)
{
    desiredAcceleration[0] = mCubicFootTrajectoryGenerator[leg * 3].getCubicAccelerationTrajectory(sharedMemory->localTime);
    desiredAcceleration[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getCubicAccelerationTrajectory(sharedMemory->localTime);
    desiredAcceleration[2] = mCubicFootTrajectoryGenerator[leg * 3 + 2].getCubicAccelerationTrajectory(sharedMemory->localTime);
}

void CubicSwingLeg::ReplanningXY(Vec3<double>& desiredPosition, const int& leg)
{
    // mCubicFootTrajectoryGenerator[leg * 3].Replanning(desiredPosition[0], sharedMemory->localTime);
    // mCubicFootTrajectoryGenerator[leg * 3 + 1].Replanning(desiredPosition[1], sharedMemory->localTime);

    mCubicFootTrajectoryGenerator[leg * 3].ReplanningGoalVel(desiredPosition[0], -sharedMemory->bodyBaseDesiredVelocity[0], sharedMemory->localTime);
    mCubicFootTrajectoryGenerator[leg * 3 + 1].ReplanningGoalVel(desiredPosition[1],-sharedMemory->bodyBaseDesiredVelocity[1], sharedMemory->localTime);
}