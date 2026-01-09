//
// Created by hwayoung on 24. 3. 12.
//

#include "BodyTrajectoryGenerator/BodyTrajectoryGen.hpp"

CubicTrajectoryGenerator baseLinearTrajectory[3];
CubicTrajectoryGenerator localZTrajectory;
CanineFilter::MAF MAFbaseAngVel_ref(200);


BodyTrajectoryGen::BodyTrajectoryGen()
{
    sharedMemory = SharedMemory::getInstance();
    mbStumbleRecovery = false;
    mGlobalStumbleStableBasePosition.setZero();
    mNonStumbleCount = 0;
}

void BodyTrajectoryGen::InitUpTrajectory()
{
    double timeDuration = STAND_UP_TIME;
    Eigen::Matrix3d Rot = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion);
    Eigen::Vector3d Up = { -0.095, 0, 0 };
    Up = Rot * Up;
//    Eigen::Vector3d desiredBasePosition = sharedMemory->globalBasePosition + Rot*(sharedMemory->bodyBase2FootPosition[FL_IDX] + sharedMemory->bodyBase2FootPosition[FR_IDX] + sharedMemory->bodyBase2FootPosition[HL_IDX] + sharedMemory->bodyBase2FootPosition[HR_IDX]);
    baseLinearTrajectory[0].updateTrajectory(sharedMemory->globalBasePosition[0], sharedMemory->globalBasePosition[0] + Up[0], sharedMemory->localTime, timeDuration);
    baseLinearTrajectory[1].updateTrajectory(sharedMemory->globalBasePosition[1], sharedMemory->globalBasePosition[1] + Up[1], sharedMemory->localTime, timeDuration);
    baseLinearTrajectory[2].updateTrajectory(sharedMemory->globalBasePosition[2], sharedMemory->globalBasePosition[2] + Up[2] + 0.28, sharedMemory->localTime, timeDuration);
    localZTrajectory.updateTrajectory(sharedMemory->bodyBasePosition[2], 0.33, sharedMemory->localTime, timeDuration);
    sharedMemory->globalBaseDesiredEulerAngle[0] = 0.0;
    sharedMemory->globalBaseDesiredEulerAngle[1] = 0.0;
    sharedMemory->globalBaseDesiredEulerAngle[2] = sharedMemory->globalBaseEulerAngle[2];
    sharedMemory->globalReferencePosition.setZero();
}

void BodyTrajectoryGen::InitDownTrajectory()
{
    double timeDuration = STAND_DOWN_TIME;
    Eigen::Matrix3d Rot = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion);
    Eigen::Vector3d Down = { +0.08, 0, 0 };
    Down = Rot * Down;
    baseLinearTrajectory[0].updateTrajectory(sharedMemory->globalBaseDesiredPosition[0], sharedMemory->globalBasePosition[0] + Down[0], sharedMemory->localTime, timeDuration);
    baseLinearTrajectory[1].updateTrajectory(sharedMemory->globalBaseDesiredPosition[1], sharedMemory->globalBasePosition[1] + Down[1], sharedMemory->localTime, timeDuration);
    baseLinearTrajectory[2].updateTrajectory(sharedMemory->globalBaseDesiredPosition[2], sharedMemory->globalBasePosition[2] + Down[2] - 0.28, sharedMemory->localTime, timeDuration);
    sharedMemory->globalBaseDesiredEulerAngle[0] = 0.0;
    sharedMemory->globalBaseDesiredEulerAngle[1] = 0.0;
    sharedMemory->globalBaseDesiredEulerAngle[2] = sharedMemory->globalBaseEulerAngle[2];
    sharedMemory->globalReferencePosition.setZero();
}

void BodyTrajectoryGen::UpdateDesiredRotation()
{
    sharedMemory->bodyBaseDesiredAngularVelocity[0] = sharedMemory->commandAngularVelocity[0];
    sharedMemory->bodyBaseDesiredAngularVelocity[1] = sharedMemory->commandAngularVelocity[1];
    sharedMemory->bodyBaseDesiredAngularVelocity[2] = MAFbaseAngVel_ref.GetFilteredVar(sharedMemory->commandAngularVelocity[2]);
    sharedMemory->globalBaseDesiredAngularVelocity = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion) * sharedMemory->bodyBaseDesiredAngularVelocity;
    mRotWorld2Body = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseDesiredQuaternion);

    double pc = cos(sharedMemory->globalBaseEulerAngle[1]);
    double pt = tan(sharedMemory->globalBaseEulerAngle[1]);
    double yc = cos(sharedMemory->globalBaseEulerAngle[2]);
    double ys = sin(sharedMemory->globalBaseEulerAngle[2]);
    Mat3<double> rot;
    rot << yc / pc, ys / pc, 0,
        -ys, yc, 0,
        yc * pt, ys * pt, 1;
    mGlobalBaseDesiredEulerRate = rot * sharedMemory->globalBaseDesiredAngularVelocity;


    if (sharedMemory->FSMState == FSM_STAND)
    {
        sharedMemory->globalBaseDesiredEulerAngle = sharedMemory->globalBaseDesiredEulerAngle + mGlobalBaseDesiredEulerRate * LOW_CONTROL_dT;
    }
    else
    {
        sharedMemory->globalBaseDesiredEulerAngle[0] = sharedMemory->estimatedGlobalSlopeRPY[0] + mGlobalBaseDesiredEulerRate[0] * LOW_CONTROL_dT;
        sharedMemory->globalBaseDesiredEulerAngle[1] = sharedMemory->estimatedGlobalSlopeRPY[1] + mGlobalBaseDesiredEulerRate[1] * LOW_CONTROL_dT;
        sharedMemory->globalBaseDesiredEulerAngle[2] = sharedMemory->globalBaseDesiredEulerAngle[2] + mGlobalBaseDesiredEulerRate[2] * LOW_CONTROL_dT;
    }

    double cy = cos(sharedMemory->globalBaseDesiredEulerAngle[2] * 0.5);
    double sy = sin(sharedMemory->globalBaseDesiredEulerAngle[2] * 0.5);
    double cp = cos(sharedMemory->globalBaseDesiredEulerAngle[1] * 0.5);
    double sp = sin(sharedMemory->globalBaseDesiredEulerAngle[1] * 0.5);
    double cr = cos(sharedMemory->globalBaseDesiredEulerAngle[0] * 0.5);
    double sr = sin(sharedMemory->globalBaseDesiredEulerAngle[0] * 0.5);

    sharedMemory->globalBaseDesiredQuaternion[0] = cr * cp * cy + sr * sp * sy;
    sharedMemory->globalBaseDesiredQuaternion[1] = sr * cp * cy - cr * sp * sy;
    sharedMemory->globalBaseDesiredQuaternion[2] = cr * sp * cy + sr * cp * sy;
    sharedMemory->globalBaseDesiredQuaternion[3] = cr * cp * sy - sr * sp * cy;
}

void BodyTrajectoryGen::UpdateDesiredLinear()
{
    if (sharedMemory->FSMState == FSM_STAND_UP || sharedMemory->FSMState == FSM_SIT_DOWN)
    {
        sharedMemory->globalBaseDesiredPosition[0] = baseLinearTrajectory[0].getPositionTrajectory(sharedMemory->localTime + LOW_CONTROL_dT);
        sharedMemory->globalBaseDesiredPosition[1] = baseLinearTrajectory[1].getPositionTrajectory(sharedMemory->localTime + LOW_CONTROL_dT);
        sharedMemory->globalBaseDesiredPosition[2] = baseLinearTrajectory[2].getPositionTrajectory(sharedMemory->localTime + LOW_CONTROL_dT);
    }
    else
    {
        Eigen::Vector3d des;
        if (sharedMemory->bodyBaseVelocity.norm() <= 0.2)
        {
            des = sharedMemory->bodyBaseDesiredVelocity;
        }
        else
        {
            des[0] = (sharedMemory->bodyBaseVelocity[0] + sharedMemory->bodyBaseDesiredVelocity[0]) / 2;
            des[1] = (sharedMemory->bodyBaseVelocity[1] + sharedMemory->bodyBaseDesiredVelocity[1]) / 2;
            des[2] = sharedMemory->bodyBaseDesiredVelocity[2];
        }
        des = mRotWorld2Body * des;
        sharedMemory->globalBaseDesiredPosition = sharedMemory->globalBaseDesiredPosition + des * LOW_CONTROL_dT;
    }
    for (int idx = 0; idx < 4; idx++)
    {
        if (sharedMemory->isFirstStand[idx])
        {
            if(sharedMemory->bodyBaseVelocity[0]<0.05) /// need to outside test
            {
                localZTrajectory.updateTrajectory(sharedMemory->bodyBasePosition[2], 0.33, sharedMemory->localTime, sharedMemory->standPeriod);
            }
            else
            {
                localZTrajectory.updateTrajectory(sharedMemory->bodyBasePosition[2], 0.34, sharedMemory->localTime, sharedMemory->standPeriod);
            }
            sharedMemory->isFirstStand[idx] = false;
        }
    }

    Eigen::Vector3d localDesiredVelocity = Eigen::Vector3d::Zero();
    localDesiredVelocity = sharedMemory->commandLinearVelocity;
    if (sharedMemory->FSMState == FSM_STAND)
    {
        localDesiredVelocity[2] = (0.33 - sharedMemory->bodyBasePosition[2]) * 10;
    }
    if(sharedMemory->FSMState == FSM_READY){
        localDesiredVelocity[2] = 0;
    }
    else
    {
        localDesiredVelocity[2] = localZTrajectory.getVelocityTrajectory(sharedMemory->localTime + LOW_CONTROL_dT);
        if (isnan(localDesiredVelocity[2]))
        {
            localDesiredVelocity[2] = (0.33 - sharedMemory->bodyBasePosition[2]) * 10;
        }
    }
    Eigen::Matrix3d rotBody2Ground = RobotMath::GetSlopeRotationMat(sharedMemory->estimatedLocalSlopeRPY);

    localDesiredVelocity = rotBody2Ground * localDesiredVelocity;
    sharedMemory->bodyBaseDesiredVelocity = localDesiredVelocity;
    sharedMemory->globalBaseDesiredVelocity = mRotWorld2Body * localDesiredVelocity;
}

void BodyTrajectoryGen::GenerateBaseTrajectory()
{
    UpdateDesiredRotation();
    UpdateDesiredLinear();
}