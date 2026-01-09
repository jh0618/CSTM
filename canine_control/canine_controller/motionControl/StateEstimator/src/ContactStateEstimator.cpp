//
// Created by cha on 24. 1. 24.
//

#include "ContactStateEstimator.hpp"

ContactStateEstimator::ContactStateEstimator(RigidBodyDynamics::Model* model, const double& dt)
    : mModel(model)
    , mDt(dt)
{
    mGain = Eigen::Matrix<double, 18, 18>::Identity() * 100;
    mMassMatrix = Eigen::MatrixXd::Zero(18, 18);
    mPrevMassMatrix = Eigen::MatrixXd::Zero(18, 18);

    mQ = Eigen::VectorXd::Zero(19);
    mQd = Eigen::VectorXd::Zero(18);
    mMotorTorque = Eigen::VectorXd::Zero(18);

    mBeta = Eigen::VectorXd::Zero(18);
    mDisturbanceTorque = Eigen::VectorXd::Zero(18);
    mMomentum = Eigen::VectorXd::Zero(18);

    mNonlinearEff = Eigen::VectorXd::Zero(18);

    for (int idx = 0; idx < 4; idx++)
    {
        mbContactState[idx] = true;
        mbGaitTableState[idx] = true;
        mbPrevGaitTableState[idx] = true;
        mTimeStep[idx] = 0.0;
    }

    mErrorCovariance.setZero();
    mMeasurementError.setZero();
    mOutputMat.setIdentity();
    for (int idx = 0; idx < 4; idx++)
    {
        mErrorCovariance(idx, idx) = 0.5; // Gait Table dependency, 높아지면 의존을 덜함
        mMeasurementError(idx, idx) = 0.1; // Momentum Observer dependency, 높아지면 의존을 덜함
    }
    sharedMemory = SharedMemory::getInstance();
}

void ContactStateEstimator::GetContactState(const double* baseQuaternion,
                                            const double* baseAngularVelocity,
                                            const double* baseLinearVelocity,
                                            const double* jointPosition,
                                            const double* jointVelocity,
                                            const double* desiredTorque,
                                            const bool* gaitTable,
                                            const double& swingPeriod,
                                            const double& standPeriod,
                                            bool* contactState,
                                            double* timeStep)
{
    // update Q
    mQ.block<3, 1>(0, 0) = Eigen::Vector3d::Zero();
    mQ[3] = baseQuaternion[1]; /// q_x
    mQ[4] = baseQuaternion[2]; /// q_y
    mQ[5] = baseQuaternion[3]; /// q_z
    mQ[18] = baseQuaternion[0]; /// q_w
    std::copy_n(jointPosition, 12, mQ.data() + 6);

    // update Qdot
    std::copy_n(baseLinearVelocity, 3, mQd.data());
    std::copy_n(baseAngularVelocity, 3, mQd.data() + 3);
    std::copy_n(jointVelocity, 12, mQd.data() + 6);

    std::copy_n(desiredTorque, 12, mMotorTorque.data() + 6);

    std::copy_n(gaitTable, 4, mbGaitTableState);

    mSwingPeriod = swingPeriod;
    mStandPeriod = standPeriod;
    /// algorithm
    this->periodChecking(); // for KF
    this->calculateResidual(); // for MO
    this->contactEstimation(); // for KF

    /// output

    std::copy_n(mbContactState, 4, contactState);
    std::copy_n(mTimeStep, 4, timeStep);

    std::copy_n(mContactTorque, 4, sharedMemory->contactResidualTorque);
    std::copy_n(mEstimatedGRF, 4, sharedMemory->estimatedGRF);
}

void ContactStateEstimator::calculateResidual()
{
    mNonlinearEff.setZero();
    RigidBodyDynamics::NonlinearEffects(*mModel, mQ, mQd, mNonlinearEff);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*mModel, mQ, mMassMatrix, true);

    mBeta = mNonlinearEff - (mMassMatrix - mPrevMassMatrix) / mDt * mQd;
    mMomentum = mMomentum + mMotorTorque * mDt - mBeta * mDt + mDisturbanceTorque * mDt;
    mDisturbanceTorque = mGain * (-mMomentum + mMassMatrix * mQd);
    Eigen::Matrix3d jacobian;
    for (int idx = 0; idx < 4; idx++)
    {
        RobotMath::GetJacobian2(jacobian, mQ.block<3, 1>(6 + idx * 3, 0), idx);
        mEstimatedGRF[idx] = (-jacobian.transpose()).inverse() * mDisturbanceTorque.block<3, 1>(6 + idx * 3, 0);
    }
    auto torque = mDisturbanceTorque.array() * mDisturbanceTorque.array();
    for (int idx = 0; idx < 4; idx++)
    {
        mContactTorque[idx] = std::sqrt(torque[6 + idx * 3] + torque[6 + idx * 3 + 1] + torque[6 + idx * 3 + 2]);
    }

    mPrevMassMatrix = mMassMatrix;
}

void ContactStateEstimator::contactEstimation()
{
    for (int idx = 0; idx < 4; idx++)
    {
        mStates[idx] = GaitTableCDF(mSwingPeriod, mStandPeriod, mTimeStep[idx], mbGaitTableState[idx]);
        mMeasurements[idx] = ContactForceCDF(mContactTorque[idx]);
    }

    mKalmanGain = mErrorCovariance * mOutputMat.transpose() * (mOutputMat * mErrorCovariance * mOutputMat.transpose() + mMeasurementError).inverse();
    mEstimatedStates = mStates + mKalmanGain * (mMeasurements - mOutputMat * mStates);

    if (!mbContactState[FL_IDX] && mEstimatedStates[FL_IDX] > 0.8)
    {
        mbContactState[FL_IDX] = true;
    }
    else if (mbContactState[FL_IDX] == true && mEstimatedStates[FL_IDX] < 0.4)
    {
        mbContactState[FL_IDX] = false;
    }
    if (!mbContactState[FR_IDX] && mEstimatedStates[FR_IDX] > 0.8)
    {
        mbContactState[FR_IDX] = true;
    }
    else if (mbContactState[FR_IDX] == true && mEstimatedStates[FR_IDX] < 0.4)
    {
        mbContactState[FR_IDX] = false;
    }
    if (!mbContactState[HL_IDX] && mEstimatedStates[HL_IDX] > 0.8)
    {
        mbContactState[HL_IDX] = true;
    }
    else if (mbContactState[HL_IDX] == true && mEstimatedStates[HL_IDX] < 0.4)
    {
        mbContactState[HL_IDX] = false;
    }
    if (!mbContactState[HR_IDX] && mEstimatedStates[HR_IDX] > 0.8)
    {
        mbContactState[HR_IDX] = true;
    }
    else if (mbContactState[HR_IDX] == true && mEstimatedStates[HR_IDX] < 0.4)
    {
        mbContactState[HR_IDX] = false;
    }

    /// 가만히 있을때 컨텍 on 으로 만들기 위해
    for (int idx = 0; idx < 4; idx++)
    {
        if (mTimeStep[idx] > 0.5 || sharedMemory->FSMState == FSM_INITIAL || sharedMemory->FSMState == FSM_READY)
        {
            mbContactState[idx] = true;
        }
    }
}

void ContactStateEstimator::periodChecking()
{
    bool gaitTransition[4];
    for (int idx = 0; idx < 4; idx++)
    {
        gaitTransition[idx] = mbGaitTableState[idx] - mbPrevGaitTableState[idx];
        if (gaitTransition[idx] != 0)
        {
            mTimeStep[idx] = 0.0;
        }
    }
    for (int idx = 0; idx < 4; idx++)
    {
        mbPrevGaitTableState[idx] = mbGaitTableState[idx];
        mTimeStep[idx] += mDt;
    }
}

void ContactStateEstimator::showOutput()
{
    if (sharedMemory->gaitState == TROT_FAST or sharedMemory->gaitState == TROT_SLOW or sharedMemory->gaitState == OVERLAP_TROT_FAST)
    {
        std::cout << "[LF]:"
            << std::setw(5) << mTimeStep[0] << "s "
            << std::setw(10) << mEstimatedStates[FL_IDX] << " "
            << std::setw(2) << sharedMemory->gaitTable[FL_IDX] << " "
            << std::setw(2) << mbContactState[FL_IDX] << " "
            << std::setw(9) << sharedMemory->simulContactForceFL << " "
            << std::setw(10) << mContactTorque[FL_IDX]
            << ", [RF]:"
            << std::setw(5) << mTimeStep[1] << "s "
            << std::setw(10) << mEstimatedStates[FR_IDX] << " "
            << std::setw(2) << sharedMemory->gaitTable[FR_IDX] << " "
            << std::setw(2) << mbContactState[FR_IDX] << " "
            << std::setw(9) << sharedMemory->simulContactForceFR << " "
            << std::setw(10) << mContactTorque[FR_IDX]
            << ", [HL]:"
            << std::setw(5) << mTimeStep[2] << "s "
            << std::setw(10) << mEstimatedStates[HL_IDX] << " "
            << std::setw(2) << sharedMemory->gaitTable[HL_IDX] << " "
            << std::setw(2) << mbContactState[HL_IDX] << " "
            << std::setw(9) << sharedMemory->simulContactForceHL << " "
            << std::setw(10) << mContactTorque[HL_IDX]
            << ", [HR]:"
            << std::setw(5) << mTimeStep[3] << "s "
            << std::setw(10) << mEstimatedStates[HR_IDX] << " "
            << std::setw(2) << sharedMemory->gaitTable[HR_IDX] << " "
            << std::setw(2) << mbContactState[HR_IDX] << " "
            << std::setw(9) << sharedMemory->simulContactForceHR << " "
            << std::setw(10) << mContactTorque[HR_IDX]
            << std::endl;
        ///
    }
}

double GaitTableCDF(double swingPeriod, double standPeriod, double time, bool gaitTableState)
{
    double result;
    result = !gaitTableState * (std::erfc((time - 0) / std::sqrt(0.0025 * 2)) + std::erfc((swingPeriod - time) / std::sqrt(0.0025 * 2))) / 2
        + gaitTableState * (-2 + std::erfc((0 - time) / std::sqrt(0.0025 * 2)) + std::erfc((time - standPeriod) / std::sqrt(0.0025 * 2))) / 2;

    return result;
}

double ContactForceCDF(double contactForce)
{
    double result;
    result = std::erfc((-contactForce + 7) / std::sqrt(5 * 2)) / 2;

    return result;
}