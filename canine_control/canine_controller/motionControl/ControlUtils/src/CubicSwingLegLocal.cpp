//
// Created by jaehoon on 23. 5. 21.
//

#include <ControlUtils/CubicSwingLegLocal.hpp>

//#define Y

CubicSwingLegLocal::CubicSwingLegLocal()
    : mbIsNewTrajectory{ false, false, false, false }
    , mLiftUpHeight{ -0.25, -0.25, -0.25, -0.25 }
    , mTargetHeight{ -0.34, -0.34, -0.34, -0.34 }
{
    sharedMemory = SharedMemory::getInstance();
}

void CubicSwingLegLocal::SetParameters()
{
    mTimeDuration = sharedMemory->swingPeriod;
    mLiftUpDuration = mTimeDuration / 2;
}

void CubicSwingLegLocal::SetFootTrajectory(const Vec3<double>& initPos, const Vec3<double>& desiredPos, const int& leg)
{
    mReferenceTime[leg] = sharedMemory->localTime;
#ifdef Y
    Mat3<double> rot = RobotMath::GetSlopeRotationMat(sharedMemory->estimatedGlobalSlopeRPY).transpose();
    normal[leg].setZero();
    normal[leg][2] = 1;
    normal[leg] = rot * normal[leg];
    mInitPosZ[leg] = initPos.dot(normal[leg]) * normal[leg];
    mInitPos[leg] = initPos - mInitPosZ[leg];
    // x, y  on slope plane
    mCubicFootTrajectoryGenerator[leg * 3].updateTrajectory(mInitPos[leg][0], desiredPos[0], sharedMemory->localTime, mTimeDuration * 3 / 4);
    mCubicFootTrajectoryGenerator[leg * 3 + 1].updateTrajectory(mInitPos[leg][1], desiredPos[1], sharedMemory->localTime, mTimeDuration * 3 / 4);
    if (initPos[2] > -0.15) // short stand leg
    {
        mCubicFootTrajectoryGenerator[leg * 3 + 2].updateTrajectory(0, 0.02, sharedMemory->localTime, mLiftUpDuration);
    }
    else // normal
    {
        mCubicFootTrajectoryGenerator[leg * 3 + 2].updateTrajectory(0, 0.13, sharedMemory->localTime, mLiftUpDuration);
    }
#else
    mCubicFootTrajectoryGenerator[leg * 3].updateTrajectory(initPos[0], desiredPos[0], sharedMemory->localTime, mTimeDuration);
    mCubicFootTrajectoryGenerator[leg * 3 + 1].updateTrajectory(initPos[1], desiredPos[1], sharedMemory->localTime, mTimeDuration);
    if (initPos[2] <= -0.15)
    {
        mCubicFootTrajectoryGenerator[leg * 3 + 2].updateTrajectory(initPos[2], initPos[2] + 0.13, sharedMemory->localTime, mLiftUpDuration);
    }
    else
    {
        mCubicFootTrajectoryGenerator[leg * 3 + 2].updateTrajectory(initPos[2], mLiftUpHeight[2], sharedMemory->localTime, mLiftUpDuration);
    }
#endif
    mbIsNewTrajectory[leg] = true;

    for (int i = 0; i < 3; i++)
    {
        sharedMemory->swingPgain[leg][i] = 0.0;
        sharedMemory->swingDgain[leg][i] = 1.0;
//        sharedMemory->swingPgain[leg][i] = 15.0;
//        if(i == 2){
//            sharedMemory->swingPgain[leg][i] = 5;
//        }
//        sharedMemory->swingDgain[leg][i] = 2.0;
    }
}

void CubicSwingLegLocal::GetPositionTrajectory(Vec3<double>& desiredPosition, const int& leg)
{
#ifdef Y
    Vec3<double> xyPosition;
    xyPosition[0] = mCubicFootTrajectoryGenerator[leg * 3].getPositionTrajectory(sharedMemory->localTime);
    xyPosition[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getPositionTrajectory(sharedMemory->localTime);
    xyPosition[2] = 0;
    desiredPosition = xyPosition + mInitPosZ[leg] + normal[leg] * mCubicFootTrajectoryGenerator[leg * 3 + 2].getPositionTrajectory(sharedMemory->localTime);
    if ((sharedMemory->localTime > mLiftUpDuration + mReferenceTime[leg]) && mbIsNewTrajectory[leg])
    {
        mbIsNewTrajectory[leg] = false;
        double currentDesiredPosition = mCubicFootTrajectoryGenerator[leg * 3 + 2].getPositionTrajectory(sharedMemory->localTime);
        mCubicFootTrajectoryGenerator[leg * 3 + 2].updateTrajectory(currentDesiredPosition, 0.0, sharedMemory->localTime, mTimeDuration - mLiftUpDuration);
        for (int i = 0; i < 3; i++)
        {
            sharedMemory->swingPgain[leg][i] = 15.0;
            if(i == 2){
                sharedMemory->swingPgain[leg][i] = 3;
            }
            sharedMemory->swingDgain[leg][i] = 2.0;
        }
    }
#else
    desiredPosition[0] = mCubicFootTrajectoryGenerator[leg * 3].getPositionTrajectory(sharedMemory->localTime);
    desiredPosition[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getPositionTrajectory(sharedMemory->localTime);
    desiredPosition[2] = mCubicFootTrajectoryGenerator[leg * 3 + 2].getPositionTrajectory(sharedMemory->localTime);
    if ((sharedMemory->localTime > mLiftUpDuration + mReferenceTime[leg]) && mbIsNewTrajectory[leg])
    {
        mbIsNewTrajectory[leg] = false;
        mCubicFootTrajectoryGenerator[leg * 3 + 2].updateTrajectory(desiredPosition[2], mTargetHeight[leg], sharedMemory->localTime, mTimeDuration - mLiftUpDuration);
        for (int i = 0; i < 3; i++)
        {
            sharedMemory->swingPgain[leg][i] = 0.0;
            sharedMemory->swingDgain[leg][i] = 1.0;
//            sharedMemory->swingPgain[leg][i] = 15.0;
//            if(i == 2){
//                sharedMemory->swingPgain[leg][i] = 3;
//            }
//            sharedMemory->swingDgain[leg][i] = 2.0;
        }
    }
#endif
}

void CubicSwingLegLocal::GetVelocityTrajectory(Vec3<double>& desiredVelocity, const int& leg)
{
#ifdef Y
    Vec3<double> velocity;
    velocity[0] = mCubicFootTrajectoryGenerator[leg * 3].getVelocityTrajectory(sharedMemory->localTime);
    velocity[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getVelocityTrajectory(sharedMemory->localTime);
    velocity[2] = 0;
    desiredVelocity = velocity + normal[leg] * mCubicFootTrajectoryGenerator[leg * 3 + 2].getVelocityTrajectory(sharedMemory->localTime);
#else
    desiredVelocity[0] = mCubicFootTrajectoryGenerator[leg * 3].getVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getVelocityTrajectory(sharedMemory->localTime);
    desiredVelocity[2] = mCubicFootTrajectoryGenerator[leg * 3 + 2].getVelocityTrajectory(sharedMemory->localTime);
#endif
}

void CubicSwingLegLocal::GetAccelerationTrajectory(Vec3<double>& desiredAcceleration, const int& leg)
{
#ifdef Y
    Vec3<double> acceleration;
    acceleration[0] = mCubicFootTrajectoryGenerator[leg * 3].getAccelerationTrajectory(sharedMemory->localTime);
    acceleration[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getAccelerationTrajectory(sharedMemory->localTime);
    acceleration[2] = 0;
    desiredAcceleration = acceleration + normal[leg] * mCubicFootTrajectoryGenerator[leg * 3 + 2].getAccelerationTrajectory(sharedMemory->localTime);
#else
    desiredAcceleration[0] = mCubicFootTrajectoryGenerator[leg * 3].getAccelerationTrajectory(sharedMemory->localTime);
    desiredAcceleration[1] = mCubicFootTrajectoryGenerator[leg * 3 + 1].getAccelerationTrajectory(sharedMemory->localTime);
    desiredAcceleration[2] = mCubicFootTrajectoryGenerator[leg * 3 + 2].getAccelerationTrajectory(sharedMemory->localTime);
#endif
}

void CubicSwingLegLocal::ReplanningXY(Vec3<double>& desiredPosition, const int& leg)
{
    mCubicFootTrajectoryGenerator[leg * 3].Replanning(desiredPosition[0], sharedMemory->localTime);
    mCubicFootTrajectoryGenerator[leg * 3 + 1].Replanning(desiredPosition[1], sharedMemory->localTime);
}