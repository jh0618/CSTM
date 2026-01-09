//
// Created by ys on 24. 7. 2.
//


#include "LegTrajectoryGenerator/LegTrajectoryGen.hpp"

HomeMotion::HomeMotion()
{
    InitializeVariable();
}

void HomeMotion::InitializeVariable()
{
    sharedMemory = SharedMemory::getInstance();
    HomePhase = 0;
    mRefTime = 0;
    for(int idx = 0; idx < 4; idx++){
        mJointPos[idx].setZero();
        mJointRefPos[idx].setZero();
        mJointRefVel[idx].setZero();
    }
}

void HomeMotion::SetHomeTrajectory()
{
    getState();
    switchHomePhase();
    setJointTrajectory();
}

void HomeMotion::getState()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int mt = 0; mt < 3; mt++)
        {
            mJointPos[leg][mt] = sharedMemory->motorPosition[leg * 3 + mt];
        }
    }
}

void HomeMotion::switchHomePhase()
{
    switch (HomePhase)
    {
    case HOME_NO_ACT:
        break;
    case HOME_STAND_UP_PHASE1:
        for (int leg = 0; leg < 4; leg++)
        {
            double homeHip = 90;
            double homeKnee = -157;
            double timeDuration = 0.9;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[leg * 3].updateTrajectory(mJointPos[leg][0], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[leg * 3 + 1].updateTrajectory(mJointPos[leg][1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[leg * 3 + 2].updateTrajectory(mJointPos[leg][2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        HomePhase = HOME_STAND_UP_PHASE2;
        break;
    case HOME_STAND_UP_PHASE2:
        if (sharedMemory->localTime > mRefTime + 0.1)
        {
            HomePhase = HOME_STAND_UP_PHASE3;
        }
        break;
    case HOME_STAND_UP_PHASE3:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 45;
            double homeKnee = -85.0;
            double timeDuration = 1.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mJointRefPos[idx][0], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mJointRefPos[idx][1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mJointRefPos[idx][2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        HomePhase = HOME_NO_ACT;
        break;
    case HOME_STAND_DOWN_PHASE1:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 90;
            double homeKnee = -150;
            double timeDuration = 2.0;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mJointPos[idx][0], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mJointPos[idx][1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mJointPos[idx][2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        HomePhase = HOME_STAND_DOWN_PHASE2;
        break;
    case HOME_STAND_DOWN_PHASE2:
        if (sharedMemory->localTime > mRefTime + 0.5)
        {
            HomePhase = HOME_STAND_DOWN_PHASE3;
        }
        break;
    case HOME_STAND_DOWN_PHASE3:
        for (int idx = 0; idx < 4; idx++)
        {
            double homeHip = 126;
            double homeKnee = -159;
            double timeDuration = 1.5;
            mRefTime = sharedMemory->localTime + timeDuration;
            mCubicTrajectoryGen[idx * 3].updateTrajectory(mJointRefPos[idx][0], 0.0, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 1].updateTrajectory(mJointRefPos[idx][1], homeHip * D2R, sharedMemory->localTime, timeDuration);
            mCubicTrajectoryGen[idx * 3 + 2].updateTrajectory(mJointRefPos[idx][2], homeKnee * D2R, sharedMemory->localTime, timeDuration);
        }
        HomePhase = HOME_NO_ACT;
        break;
    default:
        break;
    }
}

void HomeMotion::setJointTrajectory()
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int mt = 0; mt < 3; mt++)
        {
            mJointRefPos[leg][mt] = mCubicTrajectoryGen[leg * 3 + mt].getPositionTrajectory(sharedMemory->localTime);
            mJointRefVel[leg][mt] = mCubicTrajectoryGen[leg * 3 + mt].getVelocityTrajectory(sharedMemory->localTime);
            sharedMemory->motorDesiredPosition[leg * 3 + mt] = mJointRefPos[leg][mt];
            sharedMemory->motorDesiredVelocity[leg * 3 + mt] = mJointRefVel[leg][mt];
        }
    }
    if(sharedMemory->FSMState == FSM_SIT_DOWN && HomePhase == HOME_NO_ACT && mRefTime < sharedMemory->localTime)
    {
        sharedMemory->bIsEndHome = true;
    }
}

void HomeMotion::initHomeUpTrajectory()
{
    HomePhase = HOME_STAND_UP_PHASE1;
}

void HomeMotion::initHomeDownTrajectory()
{
    HomePhase = HOME_STAND_DOWN_PHASE1;
}