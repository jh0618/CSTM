//
// Created by jaehoon on 23. 9. 15.
//

#include <ControlUtils/SwingLeg.hpp>


SwingLeg::SwingLeg()
    : mbIsLineTrajectory{false, false, false, false}
    , mbIsCubic1Trajectory{false, false, false, false}
    , mbIsCubic2Trajectory{false, false, false, false}
    , mbIsUpdateContactTrajectory{false, false, false, false}
    , mLiftUpHeight{-0.25, -0.25, -0.25, -0.25}
    , mTransitionHeight{-0.286, -0.286, -0.286, -0.286}
    , mTargetHeight{-0.34, -0.34, -0.34, -0.34} //TODO: should be checked
{
    sharedMemory = SharedMemory::getInstance();
    mTransitionRatio = 2/5;
}

void SwingLeg::SetParameters()
{
    mTimeDuration = sharedMemory->swingPeriod;
    mLiftUpDuration = mTimeDuration/2;
    mTransitionRatio = 2/5;
    mTransitionDuration = mLiftUpDuration*2/5;
}

void SwingLeg::setLiftUpPDgains(const int& leg)
{
    sharedMemory->swingPgain[leg][0] = 30.0;
    sharedMemory->swingPgain[leg][1] = 30.0;
    sharedMemory->swingPgain[leg][2] = 30.0;

    sharedMemory->swingDgain[leg][0] = 3.0;
    sharedMemory->swingDgain[leg][1] = 3.0;
    sharedMemory->swingDgain[leg][2] = 3.0;
}

void SwingLeg::setLiftDownPDgains(const int& leg)
{
    sharedMemory->swingPgain[leg][0] = 10.0;
    sharedMemory->swingPgain[leg][1] = 10.0;
    sharedMemory->swingPgain[leg][2] = 10.0;

    sharedMemory->swingDgain[leg][0] = 3.0;
    sharedMemory->swingDgain[leg][1] = 3.0;
    sharedMemory->swingDgain[leg][2] = 3.0;
}

void SwingLeg::SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    mTransitionHeight[leg] = (mLiftUpHeight[leg] - mTargetHeight[leg]) * (1-mTransitionRatio) + mTargetHeight[leg]; // -0.29

    mReferenceTime[leg] = sharedMemory->localTime;
    double XYFinishRatio = 1-mTransitionRatio/2;
//    double XYFinishRatio = 1;
    mSwingLegFootTrajectoryGenerator[leg*3].updateCubicTrajectory(initPos[0],desiredPos[0],0.0,0.0,sharedMemory->localTime,mTimeDuration*XYFinishRatio);
    mSwingLegFootTrajectoryGenerator[leg*3+1].updateCubicTrajectory(initPos[1],desiredPos[1],0.0,0.0,sharedMemory->localTime,mTimeDuration*XYFinishRatio);
    mSwingLegFootTrajectoryGenerator[leg*3+2].updateCubicTrajectory(initPos[2],mLiftUpHeight[leg],0.0,0.0,sharedMemory->localTime,mLiftUpDuration);

    mbIsCubic1Trajectory[leg] = true;
    mbIsCubic2Trajectory[leg] = false;
    mbIsLineTrajectory[leg] = false;

    mbIsUpdateContactTrajectory[leg] = false;

    for (int i = 0; i < 3; i++)
    {
        sharedMemory->swingPgain[leg][i] = 0.0;
        sharedMemory->swingDgain[leg][i] = 1.0;
    }
}

void SwingLeg::GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg)
{
    for (int i = 0; i < 3; i++)
    {
        sharedMemory->swingPgain[leg][i] = 0.0;
        sharedMemory->swingDgain[leg][i] = 1.0;
    }

    desiredPosition[0] = mSwingLegFootTrajectoryGenerator[leg*3].getCubicPositionTrajectory(sharedMemory->localTime);
    desiredPosition[1] = mSwingLegFootTrajectoryGenerator[leg*3+1].getCubicPositionTrajectory(sharedMemory->localTime);

    if(mbIsCubic1Trajectory[leg])
    {
        desiredPosition[2] = mSwingLegFootTrajectoryGenerator[leg*3+2].getCubicPositionTrajectory(sharedMemory->localTime);
    }

    if((sharedMemory->localTime > mLiftUpDuration + mReferenceTime[leg]) && mbIsCubic1Trajectory[leg])
    {
        mbIsCubic1Trajectory[leg] = false;
        double desiredVelocity = mSwingLegFootTrajectoryGenerator[leg*3+2].getCubicVelocityTrajectory(sharedMemory->localTime);
        double goalVelocity = (mTargetHeight[leg] - mTransitionHeight[leg]) / mTransitionDuration;
        mSwingLegFootTrajectoryGenerator[leg*3+2].updateCubicTrajectory(desiredPosition[2],mTransitionHeight[leg], desiredVelocity, goalVelocity,sharedMemory->localTime, mLiftUpDuration - mTransitionDuration);
        mbIsCubic2Trajectory[leg] = true;
    }

    if(mbIsCubic2Trajectory[leg])
    {
        desiredPosition[2] = mSwingLegFootTrajectoryGenerator[leg*3+2].getCubicPositionTrajectory(sharedMemory->localTime);
    }

    if((sharedMemory->localTime > mTimeDuration - mTransitionDuration + mReferenceTime[leg]) && mbIsCubic2Trajectory[leg])
    {
        mbIsCubic2Trajectory[leg] = false;
        mSwingLegFootTrajectoryGenerator[leg*3+2].updateLineTrajectory(desiredPosition[2],mTargetHeight[leg], sharedMemory->localTime, mTransitionDuration);
        mbIsLineTrajectory[leg] = true;
    }

    if(mbIsLineTrajectory[leg])
    {
        desiredPosition[2] = mSwingLegFootTrajectoryGenerator[leg*3+2].getLinePositionTrajectory(sharedMemory->localTime);
    }
}

void SwingLeg::GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg)
{
    desiredVelocity[0] = mSwingLegFootTrajectoryGenerator[leg*3].getCubicVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[1] = mSwingLegFootTrajectoryGenerator[leg*3+1].getCubicVelocityTrajectory(sharedMemory->localTime);

    if(mbIsCubic1Trajectory[leg] || mbIsCubic2Trajectory[leg])
    {
        desiredVelocity[2] = mSwingLegFootTrajectoryGenerator[leg*3+2].getCubicVelocityTrajectory(sharedMemory->localTime);
    }

    if(mbIsLineTrajectory[leg])
    {
        desiredVelocity[2] = mSwingLegFootTrajectoryGenerator[leg*3+2].getLineVelocityTrajectory(sharedMemory->localTime);
    }
}

void SwingLeg::GetAccelerationTrajectory(Vec3<double> &desiredAcceleration, const int &leg)
{
    desiredAcceleration[0] = mSwingLegFootTrajectoryGenerator[leg*3].getCubicAccelerationTrajectory(sharedMemory->localTime);
    desiredAcceleration[1] = mSwingLegFootTrajectoryGenerator[leg*3+1].getCubicAccelerationTrajectory(sharedMemory->localTime);
    if(mbIsCubic1Trajectory[leg] || mbIsCubic2Trajectory[leg])
    {
        desiredAcceleration[2] = mSwingLegFootTrajectoryGenerator[leg*3+2].getCubicAccelerationTrajectory(sharedMemory->localTime);
    }

    if(mbIsLineTrajectory[leg])
    {
        desiredAcceleration[2] = 0.0;
    }
}

void SwingLeg::ReplanningXY(Vec3<double>& desiredPosition, const int& leg)
{
    mSwingLegFootTrajectoryGenerator[leg*3].Replanning(desiredPosition[0], sharedMemory->localTime);
    mSwingLegFootTrajectoryGenerator[leg*3+1].Replanning(desiredPosition[1], sharedMemory->localTime);
}
