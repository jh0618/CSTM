//
// Created by hs on 22. 10. 14.
//

#include "StateEstimator.hpp"

CanineFilter::Vec3LPF globalVelLPF(ESTIMATOR_dT, 15);
CanineFilter::Vec3LPF globalPosLPF(ESTIMATOR_dT, 100);
CanineFilter::Vec3MAF globalVelMAF(10, 10, 10);
CanineFilter::Vec3MAF globalPosMAF(1, 1, 1);
CanineFilter::Vec3LPF localVelLPF(ESTIMATOR_dT, 15);
CanineFilter::Vec3LPF localPosLPF(ESTIMATOR_dT, 100);
CanineFilter::Vec3MAF localVelMAF(10, 10, 10);
CanineFilter::Vec3MAF localPosMAF(1, 1, 1);
CanineFilter::Vec3MAF globalSlopeMAF(300, 300, 300);
CanineFilter::Vec3MAF localSlopeMAF(100, 100, 100);

StateEstimator::StateEstimator(RigidBodyDynamics::Model* model)
    : mDt(ESTIMATOR_dT)
    , mModel(model)
    , mbIsFirstRun(true)
    , mAlphaStep(25)
    , contactStateEstimator(model, ESTIMATOR_dT)
{
    for (int i = 0; i < 4; i++)
    {
        mbPrevGateTableState[i] = true;
        mbContactState[i] = true;
        mbIsFirstCon[i] = true;
        mDelay[i] = 20;
        mDelayStep[i] = 0;
    }
    /// leg kinematics
    mGlobalBasePosition << 0.0, 0.0, 0.0942;
    mGlobalBaseVelocity << 0.0, 0.0, 0.0;
    mGroundPosition << 0.0, 0.0, 0.0942;
    mGroundVelocity << 0.0, 0.0, 0.0;
    sharedMemory = SharedMemory::getInstance();
    HWData = HWD::getInstance();
}

void StateEstimator::StateEstimatorFunction()
{
    if (sharedMemory->motorStatus)
    {
        /// get the Canine's current states.
        getRobotState();

        /// 1. calculate Canine's body foot positions.
        getRobotBodyFootPosition();

        /// 2. calculate ground contact state for each foot.
        getContactState();

        /// 4. calculate Canine's global foot positions.
        getRobotGlobalFootPosition();

        /// 3. calculate position and linear velocity.
        getRobotLinearState();

        /// 5. calculate Slope's inclination.
        getEstimatedGroundSlope();

        /// update the states.
        updateRobotState();
    }
}

void StateEstimator::getContactState()
{
    /// todo. 나중에 함수 없애고 정리할 거임
    contactStateEstimator.GetContactState(
        mGlobalBaseQuaternion.data(),
        mBodyBaseAngularVelocity.data(),
        mGlobalBaseVelocity.data(),
        mMotorPosition,
        mMotorVelocity,
        mMotorDesiredTorque,
        mbGateTableState,
        mSwingPeriod,
        mStandPeriod,
        mbContactState,
        mTimeStep);
}

void StateEstimator::getEstimatedGroundSlope()
{
    static Eigen::Vector3d LastContactFootPosition[4];
    Eigen::Vector3d ContactFootPos[4];
    for (int idx = 0; idx < 4; idx++)
    {
        if (mbContactState[idx])
        {
            ContactFootPos[idx] = mBodyBase2FootPosition[idx];
            LastContactFootPosition[idx] = mBodyBase2FootPosition[idx];
        }
        else
        {
            ContactFootPos[idx] = LastContactFootPosition[idx];
        }
    }
    Eigen::Vector3d normal = fitPlaneToPoints(ContactFootPos);

    Eigen::Matrix3d rotationMatrix = alignRectangleAxes(normal, ContactFootPos);
    Eigen::Vector3d localSlopeRPY = RobotMath::rot2RPY(rotationMatrix);
    localSlopeRPY[2] = 0;
    Eigen::Vector3d MAFlocalSlopeRPY = localSlopeMAF.GetFilteredVar(localSlopeRPY);
    mEstimatedLocalSlopeRPY = MAFlocalSlopeRPY;
    mEstimatedLocalSlopeQuat = RobotMath::rpy2QUAT(MAFlocalSlopeRPY);

    Mat3<double> Rot = RobotMath::GetBaseRotationMat(mGlobalBaseQuaternion);
    Eigen::Vector3d rpy = RobotMath::rot2RPY(Rot * rotationMatrix);
    rpy[2] = 0;
    Eigen::Vector3d MAFrpy = globalSlopeMAF.GetFilteredVar(rpy);
    mEstimatedGlobalSlopeRPY = MAFrpy;
    mEstimatedGlobalSlopeQuat = RobotMath::rpy2QUAT(MAFrpy);
}

void StateEstimator::getRobotBodyFootPosition()
{
    RobotMath::TransMatBody2Foot(&mTransMat[FL_IDX], FL_IDX,
        mMotorPosition[FL_IDX * 3],
        mMotorPosition[FL_IDX * 3 + 1],
        mMotorPosition[FL_IDX * 3 + 2]);
    RobotMath::TransMatBody2Foot(&mTransMat[FR_IDX], FR_IDX,
        mMotorPosition[FR_IDX * 3],
        mMotorPosition[FR_IDX * 3 + 1],
        mMotorPosition[FR_IDX * 3 + 2]);
    RobotMath::TransMatBody2Foot(&mTransMat[HL_IDX], HL_IDX,
        mMotorPosition[HL_IDX * 3],
        mMotorPosition[HL_IDX * 3 + 1],
        mMotorPosition[HL_IDX * 3 + 2]);
    RobotMath::TransMatBody2Foot(&mTransMat[HR_IDX], HR_IDX,
        mMotorPosition[HR_IDX * 3],
        mMotorPosition[HR_IDX * 3 + 1],
        mMotorPosition[HR_IDX * 3 + 2]);

    Mat3<double> Rot = RobotMath::GetBaseRotationMat(mGlobalBaseQuaternion);
    for (int leg = 0; leg < 4; leg++)
    {
        mBodyBase2FootPosition[leg] = mTransMat[leg].block(0, 3, 3, 1);
    }
}

void StateEstimator::getRobotGlobalFootPosition()
{
    Mat3<double> Rot = RobotMath::GetBaseRotationMat(mGlobalBaseQuaternion);
    for (int leg = 0; leg < 4; leg++)
    {
        mGlobalFootPosition[leg] = mGlobalBasePosition + Rot * mBodyBase2FootPosition[leg];
    }
}

void StateEstimator::getRobotLinearState()
{
    Mat3<double> jacobian;
    Vec3<double> globalBaseVel[4];
    Vec3<double> motorPositions[4];
    Vec3<double> jointVelocities[4];
    Vec3<double> angularVel;

    for (int idx = 0; idx < 4; idx++)
    {
        motorPositions[idx][0] = mMotorPosition[idx * 3];
        motorPositions[idx][1] = mMotorPosition[idx * 3 + 1];
        motorPositions[idx][2] = mMotorPosition[idx * 3 + 2];

        jointVelocities[idx][0] = mMotorVelocity[idx * 3];
        jointVelocities[idx][1] = mMotorVelocity[idx * 3 + 1];
        jointVelocities[idx][2] = mMotorVelocity[idx * 3 + 2];
    }

    Eigen::Matrix3d rotWorld2Body = RobotMath::GetBaseRotationMat(mGlobalBaseQuaternion);
    Eigen::Matrix3d rotBody2Ground = RobotMath::GetSlopeRotationMat(mEstimatedLocalSlopeRPY);

    angularVel[0] = mBodyBaseAngularVelocity[0];
    angularVel[1] = mBodyBaseAngularVelocity[1];
    angularVel[2] = mBodyBaseAngularVelocity[2];

    if (mbIsFirstRun)
    {
        for (int idx = 0; idx < 4; idx++)
        {
            mGlobalContactFootPos[idx] = mGlobalBasePosition + rotWorld2Body * mBodyBase2FootPosition[idx];
            mGroundContactFootPos[idx].setZero();
            mbPrevContactState[idx] = mbContactState[idx];
        }
        mbIsFirstRun = false;
    }

    for (int idx = 0; idx < 4; idx++)
    {
        if (!mbPrevContactState[idx] && mbContactState[idx])
        {
            mGlobalContactFootPos[idx] = mGlobalBasePosition + rotWorld2Body * mBodyBase2FootPosition[idx];
            mGroundContactFootPos[idx] = rotBody2Ground.transpose() * mBodyBase2FootPosition[idx];
            mGroundContactFootPos[idx][2] = 0;
            mDelayStep[idx] = 0;
        }
        mbPrevContactState[idx] = mbContactState[idx];
    }

    int contactLegNum = 0;
    Vec3<double> globalBasePosition;
    Vec3<double> globalBaseVelocity;
    Eigen::Vector3d groundBasePosition;
    Eigen::Vector3d groundBaseVelocity;
    globalBasePosition.setZero();
    globalBaseVelocity.setZero();
    groundBasePosition.setZero();
    groundBaseVelocity.setZero();

    for (int leg = 0; leg < 4; leg++)
    {
        if (sharedMemory->FSMState == FSM_TROT_SLOW)
        {
            mDelay[leg] = 3;
        }
        else if (sharedMemory->FSMState == FSM_OVERLAP_TROT_FAST)
        {
            mDelay[leg] = 1;
        }
        else
        {
            mDelay[leg] = 0;
        }

        if (mbContactState[leg])
        {
            mDelayStep[leg]++;
            if (mDelayStep[leg] > mDelay[leg])
            {
                contactLegNum++;
                RobotMath::GetJacobian2(jacobian, motorPositions[leg], leg);
                globalBasePosition += mGlobalContactFootPos[leg] - rotWorld2Body * mBodyBase2FootPosition[leg];
                globalBaseVelocity += rotWorld2Body * (RobotMath::getSkewMatrix(mBodyBase2FootPosition[leg]) * angularVel + jacobian * jointVelocities[leg]);
                groundBasePosition += mGroundContactFootPos[leg] - rotBody2Ground.transpose() * mBodyBase2FootPosition[leg];
                groundBaseVelocity += rotBody2Ground.transpose() * (RobotMath::getSkewMatrix(mBodyBase2FootPosition[leg]) * angularVel + jacobian * jointVelocities[leg]);
            }
        }
    }
    if (contactLegNum == 0)
    {
        std::cout << "[STATE ESTIMATOR] contact leg num is zero." << std::endl;
        globalBaseVelocity = mGlobalBaseVelocity + mGravity * LOW_CONTROL_dT;
        globalBasePosition = mGlobalBasePosition + globalBaseVelocity * LOW_CONTROL_dT;
        groundBasePosition = mGroundPosition;
        groundBaseVelocity = mGroundVelocity;
    }
    else
    {
        globalBasePosition = globalBasePosition / contactLegNum;
        globalBaseVelocity = globalBaseVelocity / contactLegNum;
        groundBasePosition = groundBasePosition / contactLegNum;
        groundBaseVelocity = groundBaseVelocity / contactLegNum;
    }
    Vec3<double> LPFfilteredGlobalBasePosition;
    Vec3<double> LPFfilteredGlobalBaseVelocity;
    Vec3<double> MAFfilteredBodyBasePosition;
    Vec3<double> MAFfilteredBodyBaseVelocity;
    LPFfilteredGlobalBasePosition = globalPosLPF.GetFilteredVar(globalBasePosition);
    LPFfilteredGlobalBaseVelocity = globalVelLPF.GetFilteredVar(globalBaseVelocity);
    MAFfilteredBodyBasePosition = globalPosMAF.GetFilteredVar(LPFfilteredGlobalBasePosition);
    MAFfilteredBodyBaseVelocity = globalVelMAF.GetFilteredVar(LPFfilteredGlobalBaseVelocity);

    Vec3<double> LPFfilteredLocalBasePosition;
    Vec3<double> LPFfilteredLocalBaseVelocity;
    Vec3<double> MAFfilteredLocalBodyBasePosition;
    Vec3<double> MAFfilteredLocalBodyBaseVelocity;
    LPFfilteredLocalBasePosition = localPosLPF.GetFilteredVar(groundBasePosition);
    LPFfilteredLocalBaseVelocity = localVelLPF.GetFilteredVar(groundBaseVelocity);
    MAFfilteredLocalBodyBasePosition = localPosMAF.GetFilteredVar(LPFfilteredLocalBasePosition);
    MAFfilteredLocalBodyBaseVelocity = localVelMAF.GetFilteredVar(LPFfilteredLocalBaseVelocity);

    mGroundPosition = MAFfilteredLocalBodyBasePosition;
    mGroundVelocity = MAFfilteredLocalBodyBaseVelocity;

    mGlobalBasePosition = MAFfilteredBodyBasePosition;
    mGlobalBaseVelocity = MAFfilteredBodyBaseVelocity;

}

void StateEstimator::getRobotState()
{
    for (int idx = 0; idx < 4; idx++)
    {
        mBodyBase2FootPosition[idx] = sharedMemory->bodyBase2FootPosition[idx];
        mGlobalFootPosition[idx] = sharedMemory->globalFootPosition[idx];
        mGlobalBaseQuaternion[idx] = sharedMemory->globalBaseQuaternion[idx];
        mbPrevGateTableState[idx] = mbGateTableState[idx];
        mbGateTableState[idx] = sharedMemory->gaitTable[idx];
    }

    for (int idx = 0; idx < 3; idx++)
    {
        mGlobalBaseVelocity[idx] = sharedMemory->globalBaseVelocity[idx];
        mGlobalBaseEulerAngle[idx] = sharedMemory->globalBaseEulerAngle[idx];
        mBodyBaseAngularVelocity[idx] = sharedMemory->bodyBaseAngularVelocity[idx];
    }

    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        mMotorPosition[idx] = sharedMemory->motorPosition[idx];
        mMotorVelocity[idx] = sharedMemory->motorVelocity[idx];
        mMotorDesiredTorque[idx] = HWData->sensor.motor[idx].torque;
        mMotorDesiredVelocity[idx] = HWData->sensor.motor[idx].vel;
    }

    mStandPeriod = sharedMemory->standPeriod;
    mSwingPeriod = sharedMemory->swingPeriod;
}

void StateEstimator::updateRobotState()
{
    sharedMemory->globalBasePosition = mGlobalBasePosition;
    sharedMemory->globalBaseVelocity = mGlobalBaseVelocity;
    sharedMemory->bodyBasePosition = mGroundPosition;
    sharedMemory->bodyBaseVelocity = mGroundVelocity;
    sharedMemory->globalBaseAngularVelocity = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion) * sharedMemory->bodyBaseAngularVelocity;
    sharedMemory->globalYaw = mGlobalYaw;

    sharedMemory->estimatedLocalSlopeRPY = mEstimatedLocalSlopeRPY;
    sharedMemory->estimatedLocalSlopeQuat = mEstimatedLocalSlopeQuat;
    sharedMemory->estimatedGlobalSlopeRPY = mEstimatedGlobalSlopeRPY;
    sharedMemory->estimatedGlobalSlopeQuat = mEstimatedGlobalSlopeQuat;

    for (int idx = 0; idx < 4; idx++)
    {
        sharedMemory->globalFootPosition[idx] = mGlobalFootPosition[idx];
        sharedMemory->bodyBase2FootPosition[idx] = mBodyBase2FootPosition[idx];
        sharedMemory->contactState[idx] = mbContactState[idx];
        sharedMemory->groundContactFootPos[idx] = mGroundContactFootPos[idx];
    }
}

/// for debugging
void StateEstimator::showOutputs()
{
//    for(int i=0; i<4; i++)
//    {
//        std::cout << std::setw(14) << std::right <<" [gait table] : " <<sharedMemory->gaitTable[i] << " [res] : " <<sharedMemory->tempRes[i];
//    }

    for (int i = 0; i < 3; i++)
    {
        std::cout << std::setw(14) << std::right << mGlobalBasePosition[i] << ", ";
    }
    std::cout << std::endl;

}

Eigen::Vector3d StateEstimator::fitPlaneToPoints(const Eigen::Vector3d points[4])
{
    Eigen::MatrixXd A(4, 3);
    Eigen::VectorXd b(4);

    for (size_t i = 0; i < 4; ++i)
    {
        A(i, 0) = points[i].x();
        A(i, 1) = points[i].y();
        A(i, 2) = 1.0;
        b(i) = points[i].z();
    }

    Eigen::Vector3d planeParams = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    Eigen::Vector3d normal(planeParams[0], planeParams[1], -1);
    normal.normalize();

    Eigen::Vector3d referenceVector(0, 0, 1);

    if (normal.dot(referenceVector) < 0)
    {
        normal = -normal;
    }

    return normal;
}

Eigen::Matrix3d StateEstimator::alignRectangleAxes(const Eigen::Vector3d& normal, const Eigen::Vector3d points[4])
{
    Eigen::Vector3d xAxis = (points[0] - points[2]).normalized();
    Eigen::Vector3d yAxis = (points[0] - points[1]).normalized();
    Eigen::Vector3d zAxis = normal;

    yAxis = zAxis.cross(xAxis).normalized();
    xAxis = yAxis.cross(zAxis).normalized();

    // 최종 회전 행렬 구성
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix.col(0) = xAxis;
    rotationMatrix.col(1) = yAxis;
    rotationMatrix.col(2) = zAxis;

    return rotationMatrix;
}