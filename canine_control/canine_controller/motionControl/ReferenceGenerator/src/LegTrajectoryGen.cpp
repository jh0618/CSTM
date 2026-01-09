//
// Created by ys on 24. 6. 25.
//

#include "LegTrajectoryGenerator/LegTrajectoryGen.hpp"

//#define H
LegTrajectoryGen::LegTrajectoryGen()
    : HomeMotion()
    , bIsFirstRunStand{ true, true, true, true }
    , bIsRunTrot{ false, false, false, false }
    , mStandPgain{ 0.0, 0.0, 0.0 }
    , mStandDgain{ 1.0, 1.0, 1.0 }
{
    InitializeVariable();
}

void LegTrajectoryGen::InitializeVariable()
{
    sharedMemory = SharedMemory::getInstance();

    mHipPosition[FL_IDX] << HIP_CENTER_POSITION_X, HIP_CENTER_POSITION_Y, 0;
    mHipPosition[FR_IDX] << HIP_CENTER_POSITION_X, -HIP_CENTER_POSITION_Y, 0;
    mHipPosition[HL_IDX] << -HIP_CENTER_POSITION_X, HIP_CENTER_POSITION_Y, 0;
    mHipPosition[HR_IDX] << -HIP_CENTER_POSITION_X, -HIP_CENTER_POSITION_Y, 0;
    mShoulderPosition[FL_IDX] << SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[FR_IDX] << SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mShoulderPosition[HL_IDX] << -SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[HR_IDX] << -SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mHip2ShoulderPosition[FL_IDX] = mShoulderPosition[FL_IDX] - mHipPosition[FL_IDX];
    mHip2ShoulderPosition[FR_IDX] = mShoulderPosition[FR_IDX] - mHipPosition[FR_IDX];
    mHip2ShoulderPosition[HL_IDX] = mShoulderPosition[HL_IDX] - mHipPosition[HL_IDX];
    mHip2ShoulderPosition[HR_IDX] = mShoulderPosition[HR_IDX] - mHipPosition[HR_IDX];

    mBasePosition.setZero();
    mBaseVelocity.setZero();
    mBaseQuaternion.setZero();

    for (int leg = 0; leg < 4; leg++)
    {
        mCount[leg] = 0;
        mContactLeg[leg] = false;
        mSwingUpPhase[leg] = false;
        mMotorPosition[leg].setZero();
        mMotorVelocity[leg].setZero();
        mBodyFootPosition[leg].setZero();
        mSwingFootDesiredPosition[leg].setZero();
        mSwingFootDesiredVelocity[leg].setZero();
        mHipFootReferencePosition[leg].setZero();
        mHipFootReferenceVelocity[leg].setZero();
        mHipFootReferenceAcceleration[leg].setZero();
    }

    /// PD
    mStandJointPos.setZero();
    mStandJointVel.setZero();

}

void LegTrajectoryGen::getState()
{
    mBasePosition = sharedMemory->bodyBasePosition;
    mBaseVelocity = sharedMemory->bodyBaseVelocity;

    mBaseDesiredVelocity = sharedMemory->bodyBaseDesiredVelocity;
    mBaseDesiredAngularVelocity = sharedMemory->bodyBaseDesiredAngularVelocity;


    for (int leg = 0; leg < 4; leg++)
    {
        mBaseQuaternion[leg] = sharedMemory->globalBaseQuaternion[leg];
        for (int idx = 0; idx < 3; idx++)
        {
            mMotorPosition[leg][idx] = sharedMemory->motorPosition[leg * 3 + idx];
            mMotorVelocity[leg][idx] = sharedMemory->motorVelocity[leg * 3 + idx];
        }
        mBodyFootPosition[leg] = sharedMemory->bodyBase2FootPosition[leg];
        mGaitTable[leg] = sharedMemory->gaitTable[leg];
        if ((sharedMemory->contactState[leg] && !mSwingUpPhase[leg]) || mGaitTable[leg])
        {
            mContactLeg[leg] = true;
        }
        else
        {
            mContactLeg[leg] = false;
        }

    }
}

void LegTrajectoryGen::SetLegMotion()
{
    getState();
    generateLegTrajectory();
    updateState();
}

void LegTrajectoryGen::checkGaitTransition()
{
    static int prevGait[4];
    for(int idx = 0; idx < 4; idx++){
        if(prevGait[idx] == 0 && mGaitTable[idx] == 1){
            mCount[idx] = 0;
        }
        prevGait[idx] = mGaitTable[idx];
    }
}

void LegTrajectoryGen::generateLegTrajectory()
{
    checkGaitTransition();
//    sharedMemory->stumbleLegIndex = 6;
//    if(sharedMemory->stumble.recoveryStage == STUMBLE::NO_ACT)
//    {
//        sharedMemory->stumble.bStumble = false;
//        sharedMemory->stumble.num = 0;
//    }
    for (int leg = 0; leg < 4; leg++)
    {
        if (mGaitTable[leg] && mContactLeg[leg]) // contact
        {
            LandingAlgorithm(leg);
        }
        else if (mGaitTable[leg] && !mContactLeg[leg]) // late landing
        {
            LandingAlgorithm(leg);
//            LateLandingAlgorithm(leg);// but not use this now.
        }
        else if (!mGaitTable[leg] && mContactLeg[leg])// early landing or Swing Start
        {
            if (mCount[leg] < sharedMemory->swingPeriod * 1 / 2 / LOW_CONTROL_dT) // Swing Up Phase
            {
                SwingAlgorithm(leg);
                mCount[leg]++;
            }
            else // swing down phase, early landing
            {
                SwingAlgorithm(leg);
//                LandingAlgorithm(leg);
            }
        }
        else // swing leg
        {
            SwingAlgorithm(leg);
            mCount[leg]++;
        }
//        if(sharedMemory->stumble.recoveryStage == STUMBLE::NO_ACT)
//        {
//            sharedMemory->stumble.bLeg[leg] = false;
//        }
//        if(!mGaitTable[leg])
//        {
//            /// error of desired position and current position
//            Eigen::Matrix3d rotSB = RobotMath::GetSlopeRotationMat(sharedMemory->estimatedLocalSlopeRPY).transpose();
//            Eigen::Vector3d footDesiredPosition = rotSB * sharedMemory->bodyBase2FootDesiredPosition[leg];
//            Eigen::Vector3d footPosition = rotSB * sharedMemory->bodyBase2FootPosition[leg];
//
//            if(sharedMemory->stumble.recoveryStage == STUMBLE::NO_ACT && (footDesiredPosition - footPosition).norm() > 0.07)
//            {
//                sharedMemory->stumbleLegIndex = leg;
//                sharedMemory->stumble.bLeg[leg] = true;
//                sharedMemory->stumble.num++;
//                sharedMemory->stumble.bStumble = true;
//            }
//        }
    }
//    if(sharedMemory->stumble.recoveryStage == STUMBLE::NO_ACT && sharedMemory->stumble.bStumble)
//    {
//        std::cout<<"start stopping"<<std::endl;
//        sharedMemory->stumble.bRecovery = true;
//        sharedMemory->stumble.recoveryStage = STUMBLE::STOPPING;
//    }
}

void LegTrajectoryGen::updateState()
{
    std::copy(mHipFootReferencePosition, mHipFootReferencePosition + 4, sharedMemory->hipFootReferencePosition);
    std::copy(mHipFootReferenceVelocity, mHipFootReferenceVelocity + 4, sharedMemory->hipFootReferenceVelocity);
    std::copy(mHipFootReferenceAcceleration, mHipFootReferenceAcceleration + 4, sharedMemory->hipFootReferenceAcceleration);
    std::copy(mSwingUpPhase, mSwingUpPhase + 4, sharedMemory->swingUpPhase);
}

void LegTrajectoryGen::calculateSwingJointPDTorque(const int& leg)
{
    SwingLegTrajectory.GetPositionTrajectory(mSwingFootDesiredPosition[leg], leg);
    SwingLegTrajectory.GetVelocityTrajectory(mSwingFootDesiredVelocity[leg], leg);
    SwingLegTrajectory.GetAccelerationTrajectory(mHipFootReferenceAcceleration[leg], leg);
#ifdef H
    mSwingFootDesiredPosition[leg] = mHip2ShoulderPosition[leg] + mSwingFootDesiredPosition[leg];
#else
    Mat3<double> rot = RobotMath::GetSlopeRotationMat(sharedMemory->globalBaseEulerAngle);

    mSwingFootDesiredPosition[leg] = rot.transpose() * mSwingFootDesiredPosition[leg];
    mSwingFootDesiredVelocity[leg] = rot.transpose() * mSwingFootDesiredVelocity[leg];
    mHipFootReferenceAcceleration[leg] = rot.transpose() * mHipFootReferenceAcceleration[leg];
#endif
    mHipFootReferencePosition[leg] = mSwingFootDesiredPosition[leg]; // 평지 + hip 기준 !!!!
    RobotMath::GetLegInvKinematics(mJointDesiredPosition[leg], mHipFootReferencePosition[leg], leg);
    for (int idx = 0; idx < 3; idx++)
    {
        sharedMemory->motorDesiredPosition[leg * 3 + idx] = mJointDesiredPosition[leg][idx];
    }
    sharedMemory->bodyBase2FootDesiredPosition[leg] = mHipFootReferencePosition[leg] + mHipPosition[leg];

    mHipFootReferenceVelocity[leg] = mSwingFootDesiredVelocity[leg];
    Mat3<double> jacobian;
    RobotMath::GetJacobian2(jacobian, mJointDesiredPosition[leg], leg);
    mJointDesiredVelocity[leg] = -jacobian.inverse() * mSwingFootDesiredVelocity[leg];
    for (int idx = 0; idx < 3; idx++)
    {
        sharedMemory->motorDesiredVelocity[leg * 3 + idx] = mJointDesiredVelocity[leg][idx];
    }
    sharedMemory->bodyBase2FootDesiredVelocity[leg] = mSwingFootDesiredVelocity[leg];
//swing joint PD
    for (int mt = 0; mt < 3; mt++)
    {
        sharedMemory->pdTorque[leg][mt] = sharedMemory->swingPgain[leg][mt] * (mJointDesiredPosition[leg][mt] - mMotorPosition[leg][mt])
            + sharedMemory->swingDgain[leg][mt] * (mJointDesiredVelocity[leg][mt] - mMotorVelocity[leg][mt]);
    }
}

void LegTrajectoryGen::SwingAlgorithm(const int& leg)
{
    if (mCount[leg] < (sharedMemory->swingPeriod * 2 / 3 / LOW_CONTROL_dT))
    {
        SwingLegTrajectory.SetParameters();
        Mat3<double> yawRateRot;
        Vec3<double> raibertHeuristic;
        Vec3<double> desiredHipFootPosition;
        Vec3<double> centrifugalPosition;
        double raibertGain;
        raibertGain = 0.0;
        double yawRate = sharedMemory->standPeriod / 2 * mBaseDesiredAngularVelocity[2];
        yawRateRot << cos(yawRate), -sin(yawRate), 0,
            sin(yawRate), cos(yawRate), 0,
            0, 0, 1;

        if (mBasePosition[2] <= 0.05)
        {
            mBasePosition[2] = 0.05;
        }
        centrifugalPosition = 0.5 * sqrt(abs(mBasePosition[2]) / 9.81) * mBaseVelocity.cross(mBaseDesiredAngularVelocity);
//                raibertHeuristic = mBodyBaseVelocity * sharedMemory->standPeriod/2 + raibertGain * (mBodyBaseVelocity - mBaseDesiredVelocity);
        raibertHeuristic = yawRateRot * (mHipPosition[leg] + mBaseVelocity * sharedMemory->standPeriod / 2 + raibertGain * (mBaseVelocity - mBaseDesiredVelocity)) - mHipPosition[leg];
#ifdef H
        desiredHipFootPosition = raibertHeuristic + centrifugalPosition;
#else
        desiredHipFootPosition = mHip2ShoulderPosition[leg] + raibertHeuristic + centrifugalPosition;
#endif
        if (bIsRunTrot[leg])
        {
            mSwingUpPhase[leg] = true;
            mDesiredFootPGain <<
                700, 0, 0,
                0, 700, 0,
                0, 0, 700;
            mDesiredFootDGain <<
                100, 0, 0,
                0, 100, 0,
                0, 0, 100;
            Vec3<double> currentHipFootPosition;
#ifdef H
            currentHipFootPosition = mBodyFootPosition[leg]-mShoulderPosition[leg];
#else
            Mat3<double> rot = RobotMath::GetSlopeRotationMat(sharedMemory->globalBaseEulerAngle);

            currentHipFootPosition = rot * mHipFootReferencePosition[leg];
#endif
//                    SwingLegTrajectory.SetFootTrajectory(mHipFootPosition[leg], desiredHipFootPosition, leg); //TODO: should be checked
            SwingLegTrajectory.SetFootTrajectory(currentHipFootPosition, desiredHipFootPosition, leg);
            bIsRunTrot[leg] = false;
            bIsFirstRunStand[leg] = true;
        }
        else
        {
            SwingLegTrajectory.ReplanningXY(desiredHipFootPosition, leg);
        }
    }
    else
    {
        if (mSwingUpPhase[leg])
        {
            mDesiredFootPGain <<
                700, 0, 0,
                0, 700, 0,
                0, 0, 100;
            mDesiredFootDGain <<
                100, 0, 0,
                0, 100, 0,
                0, 0, 100;
            mSwingUpPhase[leg] = false;
        }
    }
    calculateSwingJointPDTorque(leg);
}

void LegTrajectoryGen::LandingAlgorithm(const int& leg)
{
    if (bIsFirstRunStand[leg])
    {
        bIsFirstRunStand[leg] = false;
        bIsRunTrot[leg] = true;
        sharedMemory->isFirstStand[leg] = true;
    }
    Vec3<double> deltaOrientation = mBaseDesiredAngularVelocity * LOW_CONTROL_dT;
    double cy = cos(deltaOrientation[2] * 0.5);
    double sy = sin(deltaOrientation[2] * 0.5);
    double cp = cos(deltaOrientation[1] * 0.5);
    double sp = sin(deltaOrientation[1] * 0.5);
    double cr = cos(deltaOrientation[0] * 0.5);
    double sr = sin(deltaOrientation[0] * 0.5);

    Vec4<double> desiredQuaternion;

    desiredQuaternion[0] = cr * cp * cy + sr * sp * sy;
    desiredQuaternion[1] = sr * cp * cy - cr * sp * sy;
    desiredQuaternion[2] = cr * sp * cy + sr * cp * sy;
    desiredQuaternion[3] = cr * cp * sy - sr * sp * cy;

    mHipFootReferencePosition[leg] = mBodyFootPosition[leg] - mHipPosition[leg] + LOW_CONTROL_dT * (mBaseDesiredAngularVelocity.cross(mHipPosition[leg] + mBaseDesiredVelocity));
    sharedMemory->bodyBase2FootDesiredPosition[leg] = mHipFootReferencePosition[leg] + mHipPosition[leg];
    /// joint space PD
    RobotMath::GetLegInvKinematics(mStandJointPos, mHipFootReferencePosition[leg], leg);
    mJointDesiredPosition[leg] = mStandJointPos;
    for (int idx = 0; idx < 3; idx++)
    {
        sharedMemory->motorDesiredPosition[leg * 3 + idx] = mStandJointPos[idx];
    }
    mHipFootReferenceVelocity[leg] = -mBaseDesiredAngularVelocity.cross(RobotMath::GetBaseRotationMat(desiredQuaternion) * mBodyFootPosition[leg]) - RobotMath::GetBaseRotationMat(desiredQuaternion) * mBaseDesiredVelocity;
    Mat3<double> jacobian;
    RobotMath::GetJacobian2(jacobian, mJointDesiredPosition[leg], leg);
    mStandJointVel = -jacobian.inverse() * mHipFootReferenceVelocity[leg];
    mJointDesiredVelocity[leg] = mStandJointVel;
    /// joint space PD
    for (int idx = 0; idx < 3; idx++)
    {
        sharedMemory->motorDesiredVelocity[leg * 3 + idx] = mStandJointVel[idx];
    }
    sharedMemory->bodyBase2FootDesiredVelocity[leg] = mHipFootReferenceVelocity[leg];

    calculateStandJointPDTorque(leg);
}

void LegTrajectoryGen::LateLandingAlgorithm(const int& leg)
{
    //TODO : Need to make Algorithm
}

void LegTrajectoryGen::calculateStandJointPDTorque(const int& leg)
{

    /// PD control
    for (int mt = 0; mt < 3; mt++)
    {
        sharedMemory->pdTorque[leg][mt] = mStandPgain[mt] * (mStandJointPos[mt] - mMotorPosition[leg][mt])
            + mStandDgain[mt] * (mStandJointVel[mt] - mMotorVelocity[leg][mt]);
    }
}