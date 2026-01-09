//
// Created by jaehoon on 23. 9. 15.
//

#include <ControlUtils/SwingLegReplanning.hpp>

SwingLegReplanning::SwingLegReplanning()
{
    updateMatrixA(0);
    sharedMemory = SharedMemory::getInstance();
}

void SwingLegReplanning::updateCubicTrajectory(double currentPosition,double goalPosition,double currentVelocity, double goalVelocity, double currentTime,double timeDuration)
{
    updateMatrixA(0);
    mCubicFunctionValue << currentPosition, goalPosition, currentVelocity*mTimeDuration, goalVelocity*mTimeDuration;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCubicCoefficient();
}

void SwingLegReplanning::updateLineTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration)
{
    mLineCoefficient[0] = (goalPosition - currentPosition) / timeDuration;
    mLineCoefficient[1] = currentPosition;

    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

void SwingLegReplanning::calculateCubicCoefficient()
{
    mCubicCoefficient = mCubicMatrixA * mCubicFunctionValue;
}

double SwingLegReplanning::getCubicPositionTrajectory(double currentTime)
{
    if(currentTime>(mReferenceTime + mTimeDuration))
    {
        currentTime = mReferenceTime + mTimeDuration;
    }
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return mCubicCoefficient(0,0) * pow(normalizedTime, 3.0) + mCubicCoefficient(1,0) * pow(normalizedTime, 2.0) + mCubicCoefficient(2,0) * normalizedTime + mCubicCoefficient(3, 0);
}

double SwingLegReplanning::getCubicVelocityTrajectory(double currentTime)
{
    if(currentTime>(mReferenceTime + mTimeDuration))
    {
        currentTime = mReferenceTime + mTimeDuration;
    }
    if(currentTime<mReferenceTime)
    {
        currentTime = mReferenceTime;
    }
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (3.0 *mCubicCoefficient(0,0) * pow(normalizedTime, 2.0) + 2.0 * mCubicCoefficient(1,0) * normalizedTime + mCubicCoefficient(2,0)) / mTimeDuration;
}

double SwingLegReplanning::getCubicAccelerationTrajectory(double currentTime)
{
    if(currentTime>(mReferenceTime + mTimeDuration))
    {
        return 0.0;
    }
    else if(currentTime<mReferenceTime)
    {
        currentTime = mReferenceTime;
    }
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (6.0 *mCubicCoefficient(0,0) * normalizedTime + 2.0 * mCubicCoefficient(1,0)) / pow(mTimeDuration, 2.0);
}

double SwingLegReplanning::getLinePositionTrajectory(double currentTime)
{
    return mLineCoefficient[0]*(currentTime - mReferenceTime) + mLineCoefficient[1];
}

double SwingLegReplanning::getLineVelocityTrajectory(double currentTime)
{
    return mLineCoefficient[0];
}

void SwingLegReplanning::updateMatrixA(double currentTime)
{
    mCubicMatrixA << pow(currentTime, 3), pow(currentTime, 2), currentTime, 1,
        1, 1, 1, 1,
        3*pow(currentTime, 2), 2*currentTime, 1, 0,
        3, 2, 1, 0;
    mCubicMatrixA = mCubicMatrixA.inverse();
}

void SwingLegReplanning::LineReplanning(double originalPosition, double goalPosition)
{
    double currentPosition = getLinePositionTrajectory(sharedMemory->localTime);

//    mLineCoefficient[0] = (goalPosition - currentPosition) / mTimeDuration;
//    mLineCoefficient[1] = currentPosition;

    mLineCoefficient[0] = (goalPosition - originalPosition) / mTimeDuration;
    mLineCoefficient[1] = originalPosition;
}

void SwingLegReplanning::CubicReplanning(double originalPosition, double goalPosition)
{
    updateMatrixA(0);
    mCubicFunctionValue << originalPosition, goalPosition, 0.0, 0.0;
    calculateCubicCoefficient();
}

void SwingLegReplanning::Replanning(double goalPosition, double currentTime)
{
    double currentPosition = getCubicPositionTrajectory(currentTime);
    double currentVelocity = getCubicVelocityTrajectory(currentTime);
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    updateMatrixA(normalizedTime);
    mCubicFunctionValue << currentPosition, goalPosition, currentVelocity*mTimeDuration, 0;
    calculateCubicCoefficient();
}

void SwingLegReplanning::ReplanningGoalVel(double goalPosition, double goalVelocity, double currentTime)
{
    double currentPosition = getCubicPositionTrajectory(currentTime);
    double currentVelocity = getCubicVelocityTrajectory(currentTime);
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    updateMatrixA(normalizedTime);
    mCubicFunctionValue << currentPosition, goalPosition, currentVelocity*mTimeDuration, goalVelocity*mTimeDuration;
    calculateCubicCoefficient();
}
