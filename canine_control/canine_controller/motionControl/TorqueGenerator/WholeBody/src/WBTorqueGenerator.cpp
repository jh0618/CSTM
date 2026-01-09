//
// Created by hwayoung on 23. 12. 15.
//

#include "WBTorqueGenerator/WBTorqueGenerator.hpp"

extern std::string modelFile;
CanineFilter::TauLPF tauLPF(LOW_CONTROL_dT, 50);

WBTorqueGenerator::WBTorqueGenerator()
    : mTorqueLimit(30)
    , mMu(0.6)
{
    sharedMemory = SharedMemory::getInstance();
    InitializeVariable();
    mDesiredFootPGain <<
        700, 0, 0,
        0, 700, 0,
        0, 0, 700;
    mDesiredFootDGain <<
        20, 0, 0,
        0, 20, 0,
        0, 0, 20;

    mFrictionBlock <<
        1, 0, mMu,
        -1, 0, mMu,
        0, 1, mMu,
        0, -1, mMu,
        0, 0, 1;
    mKpLinear = { 30, 30, 70 };
    mKdLinear = { 10, 10, 10 };
    mKpOrientation = { 200, 200, 200 };
    mKdOrientation = { 20, 20, 20 };
}

void WBTorqueGenerator::DoControl()
{
    updateState();
    updateRBDL();
    calculateWBTorque();
    setTorque();
}

double* WBTorqueGenerator::GetTorque()
{
    return mTorque;
}

void WBTorqueGenerator::InitializeVariable()
{
    RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), &mModel, true);
    mMassMatrix = Eigen::MatrixXd::Zero(18, 18);
    mNonlinearEff = Eigen::VectorXd::Zero(18);
    mOptimalQ = Eigen::VectorXd::Zero(18);
    mOptimalTau = Eigen::VectorXd::Zero(12);

    mBasePosition.setZero();
    mBaseDesiredPosition.setZero();
    mBaseVelocity.setZero();
    mBaseDesiredVelocity.setZero();
    mBaseQuaternion.setZero();
    mBaseDesiredQuaternion.setZero();
    mBodyBaseAngularVelocity.setZero();
    mBodyBaseDesiredAngularVelocity.setZero();

    mNumberOfContact = 0;

    mHipPosition[FL_IDX] << HIP_CENTER_POSITION_X, HIP_CENTER_POSITION_Y, 0;
    mHipPosition[FR_IDX] << HIP_CENTER_POSITION_X, -HIP_CENTER_POSITION_Y, 0;
    mHipPosition[HL_IDX] << -HIP_CENTER_POSITION_X, HIP_CENTER_POSITION_Y, 0;
    mHipPosition[HR_IDX] << -HIP_CENTER_POSITION_X, -HIP_CENTER_POSITION_Y, 0;

    for (int leg = 0; leg < 4; leg++)
    {
        mMotorPosition[leg].setZero();
        mMotorVelocity[leg].setZero();
        mMotorDesiredPosition[leg].setZero();
        mMotorDesiredVelocity[leg].setZero();
        mBodyFootPosition[leg].setZero();

        mGaitTable[leg] = false;
        mContactLeg[leg] = false;
        mSwingUpPhase[leg] = false;
        mLegJacobian[leg].setZero();
        mLegPrevJacobian[leg].setZero();
        mLegJacobianDot[leg].setZero();

        mHipFootPosition[leg].setZero();
        mHipFootVelocity[leg].setZero();
        mHipFootReferencePosition[leg].setZero();
        mHipFootReferenceVelocity[leg].setZero();
        mHipFootReferenceAcceleration[leg].setZero();


        mHipFootDesiredAcc[leg].setZero();
    }
    for (int idx = 0; idx < LEG_MOTOR_NUM; idx++)
    {
        mTorque[idx] = 0;
    }
}

void WBTorqueGenerator::updateState()
{
    // copy sharedMemory Data
    mBasePosition = sharedMemory->globalBasePosition;
    mBaseVelocity = sharedMemory->globalBaseVelocity;
    mBodyBaseAngularVelocity = sharedMemory->bodyBaseAngularVelocity;

    mBaseDesiredPosition = sharedMemory->globalBaseDesiredPosition;
    mBaseDesiredVelocity = sharedMemory->globalBaseDesiredVelocity;

    if ((mBaseDesiredPosition - mBasePosition).norm() >= 0.4)
    {
        mBaseDesiredPosition = (mBaseDesiredPosition - mBasePosition).normalized() * 0.4 + mBasePosition;
    }
    mBodyBaseDesiredAngularVelocity = sharedMemory->bodyBaseDesiredAngularVelocity;
    mBaseEulerAngle = sharedMemory->globalBaseEulerAngle;

    for (int leg = 0; leg < 4; leg++)
    {
        mBaseDesiredQuaternion[leg] = sharedMemory->globalBaseDesiredQuaternion[leg];
        mBaseQuaternion[leg] = sharedMemory->globalBaseQuaternion[leg];
        mGaitTable[leg] = sharedMemory->gaitTable[leg];
        mSwingUpPhase[leg] = sharedMemory->swingUpPhase[leg];
        mBodyFootPosition[leg] = sharedMemory->bodyBase2FootPosition[leg];

        for (int idx = 0; idx < 3; idx++)
        {
            mMotorPosition[leg][idx] = sharedMemory->motorPosition[leg * 3 + idx];
            mMotorVelocity[leg][idx] = sharedMemory->motorVelocity[leg * 3 + idx];
            mMotorDesiredPosition[leg][idx] = sharedMemory->motorDesiredPosition[leg * 3 + idx];
            mMotorDesiredVelocity[leg][idx] = sharedMemory->motorDesiredVelocity[leg * 3 + idx];
        }

        mHipFootReferencePosition[leg] = sharedMemory->hipFootReferencePosition[leg];
        mHipFootReferenceVelocity[leg] = sharedMemory->hipFootReferenceVelocity[leg];
        mHipFootReferenceAcceleration[leg] = sharedMemory->hipFootReferenceAcceleration[leg];
    }

    // State Update
    mNumberOfContact = 0;
    for (int leg = 0; leg < 4; leg++)
    {
        mLegPrevJacobian[leg] = mLegJacobian[leg];
        RobotMath::GetJacobian2(mLegJacobian[leg], mMotorPosition[leg], leg);
        mLegJacobianDot[leg] = (mLegJacobian[leg] - mLegPrevJacobian[leg]) / LOW_CONTROL_dT;
        mHipFootPosition[leg] = mBodyFootPosition[leg] - mHipPosition[leg];
        mHipFootVelocity[leg] = mLegJacobian[leg] * -mMotorVelocity[leg];
        if ((sharedMemory->contactState[leg] && !mSwingUpPhase[leg]) || mGaitTable[leg])
        {
            mContactLeg[leg] = true;
            mNumberOfContact++;
        }
        else
        {
            mContactLeg[leg] = false;
        }
        sharedMemory->bodyBase2FootVelocity[leg] = mHipFootVelocity[leg];
    }
}

void WBTorqueGenerator::updateRBDL()
{
    mQ.block<3, 1>(0, 0) = mBasePosition;

    mQDot.block<3, 1>(0, 0) = mBaseVelocity;
    mQDot.block<3, 1>(3, 0) = mBodyBaseAngularVelocity;

    mQ[3] = mBaseQuaternion[1]; /// q_x
    mQ[4] = mBaseQuaternion[2]; /// q_y
    mQ[5] = mBaseQuaternion[3]; /// q_z
    mQ[18] = mBaseQuaternion[0]; /// q_w

    for (int leg = 0; leg < 4; leg++)
    {
        mQ.block<3, 1>(6 + 3 * leg, 0) = mMotorPosition[leg];
        mQDot.block<3, 1>(6 + 3 * leg, 0) = mMotorVelocity[leg];
    }
    mRef_Q.block<3, 1>(0, 0) = mBaseDesiredPosition;
    mRef_QDot.block<3, 1>(0, 0) = mBaseDesiredVelocity;
    mRef_QDot.block<3, 1>(3, 0) = mBodyBaseDesiredAngularVelocity;

    mRef_Q[3] = mBaseDesiredQuaternion[1]; /// q_x
    mRef_Q[4] = mBaseDesiredQuaternion[2]; /// q_y
    mRef_Q[5] = mBaseDesiredQuaternion[3]; /// q_z
    mRef_Q[18] = mBaseDesiredQuaternion[0]; /// q_w

    for (int leg = 0; leg < 4; leg++)
    {
        mRef_Q.block<3, 1>(6 + 3 * leg, 0) = mMotorDesiredPosition[leg];
        mRef_QDot.block<3, 1>(6 + 3 * leg, 0) = mMotorDesiredVelocity[leg];
    }
}

void WBTorqueGenerator::calculateWBTorque()
{
    setObj();
    setEquality();
    setInequality();
    setQpOasesMat();
    solveQP();
}


void WBTorqueGenerator::setObjWeight()
{
    Eigen::VectorXd gains = Eigen::VectorXd(18 + 3 * mNumberOfContact);
    /// base linear acc x,y,z
    gains[0] = 10;
    gains[1] = 10;
    gains[2] = 10;
    /// base angular acc r, p, y
    gains[3] = 20;
    gains[4] = 20;
    gains[5] = 20;
    /// leg task-space acc x,y,z
    for (int leg = 0; leg < 4; leg++) //TODO: 힘 2중으로 줌?
    {
        if (mContactLeg[leg])
        {
            gains[6 + 3 * leg + 0] = 2;
            gains[6 + 3 * leg + 1] = 2;
            gains[6 + 3 * leg + 2] = 2;
        }
        else
        {
            gains[6 + 3 * leg + 0] = 2;
            gains[6 + 3 * leg + 1] = 2;
            gains[6 + 3 * leg + 2] = 2;
        }
    }

    /// contact force x,y,z (3*contact)
    for (int leg = 0; leg < mNumberOfContact; leg++)
    {
        gains[18 + 3 * leg + 0] = 1e-5;//x
        gains[18 + 3 * leg + 1] = 1e-5;//y
        gains[18 + 3 * leg + 2] = 1e-5;//z
    }
    for (int idx = 0; idx < gains.size(); idx++)
    {
        mObjGain(idx, idx) = gains[idx];
    }
}

void WBTorqueGenerator::setObj()
{
    mObjectA = Eigen::MatrixXd::Identity(18 + 3 * mNumberOfContact, 18 + 3 * mNumberOfContact);
    mObjectB = Eigen::VectorXd::Zero(18 + 3 * mNumberOfContact);

    // Base Q_dd
    mObjectB.block<3, 1>(0, 0) = mKpLinear * (mBaseDesiredPosition - mBasePosition).array() + mKdLinear * (mBaseDesiredVelocity - mBaseVelocity).array();
    Eigen::Vector3d orientationError;
    orientationError = RobotMath::OrientationError(RobotMath::GetBaseRotationMat(mBaseDesiredQuaternion), RobotMath::GetBaseRotationMat(mBaseQuaternion));
    mObjectB.block<3, 1>(3, 0) = mKpOrientation * orientationError.array() + mKdOrientation * (mBodyBaseDesiredAngularVelocity - mBodyBaseAngularVelocity).array();

    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, 18);

    for (int leg = 0; leg < 4; leg++)
    {
        if (mContactLeg[leg])
        {
            RigidBodyDynamics::CalcPointJacobian(mModel, mRef_Q, 3 * leg + 5, { 0, 0, HIP_PITCH2KNEE_PITCH_POSITION_Z }, jacobian);
            mObjectA.block<3, 18>(6 + leg * 3, 0) = jacobian;
            mObjectB.block<3, 1>(6 + leg * 3, 0) = -RigidBodyDynamics::CalcPointAcceleration(mModel, mRef_Q, mRef_QDot, Eigen::VectorXd::Zero(18), 3 * leg + 5, { 0, 0, HIP_PITCH2KNEE_PITCH_POSITION_Z }); // jacobian_dot * Q_Dot
        }
        else
        {
            /// task space
            mHipFootDesiredAcc[leg] = mHipFootReferenceAcceleration[leg] +
                mDesiredFootPGain * (mHipFootReferencePosition[leg] - mHipFootPosition[leg]) +
                mDesiredFootDGain * (mHipFootReferenceVelocity[leg] - mHipFootVelocity[leg]);
            mObjectA.block(6 + leg * 3, 6 + leg * 3, 3, 3) = -mLegJacobian[leg];
            mObjectB.block(6 + leg * 3, 0, 3, 1)  = mHipFootDesiredAcc[leg] +mLegJacobianDot[leg]*mMotorVelocity[leg];
        }
    }
}

void WBTorqueGenerator::setEquality()
{
    mEqualityA = Eigen::MatrixXd(6, 18 + 3 * mNumberOfContact);
    mEqualityB = Eigen::MatrixXd(6, 1);
    mContactJacobian = Eigen::MatrixXd(3 * mNumberOfContact, 18);

    mEqualityA.setZero();
    mEqualityB.setZero();
    mContactJacobian.setZero();
    mMassMatrix.setZero();
    mNonlinearEff.setZero();

    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, 18);
    int c = 0;
    for (int leg = 0; leg < 4; leg++)
    {
        if (mContactLeg[leg])
        {
            jacobian.setZero();
            RigidBodyDynamics::CalcPointJacobian(mModel, mQ, 3 * leg + 5, { 0, 0, HIP_PITCH2KNEE_PITCH_POSITION_Z }, jacobian);
//            RigidBodyDynamics::CalcPointJacobian(mModel, mRef_Q, 3*leg+5, {0,0,HIP_PITCH2KNEE_PITCH_POSITION_Z}, jacobian);
            mContactJacobian.block<3, 18>(3 * c, 0) = jacobian;
            c++;
        }
    }

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(mModel, mQ, mMassMatrix, true);
    RigidBodyDynamics::NonlinearEffects(mModel, mQ, mQDot, mNonlinearEff);
    mEqualityA.block<6, 18>(0, 0) = mMassMatrix.block<6, 18>(0, 0);
    mEqualityA.block(0, 18, 6, 3 * mNumberOfContact) = -mContactJacobian.transpose().block(0, 0, 6, 3 * mNumberOfContact);
    mEqualityB = -mNonlinearEff.block<6, 1>(0, 0);

}

void WBTorqueGenerator::setInequality()
{
    mInequalityA = Eigen::MatrixXd(6 + 12 + 5 * mNumberOfContact, 18 + 3 * mNumberOfContact);
    mInequalityUbA = Eigen::VectorXd(6 + 12 + 5 * mNumberOfContact);
    mInequalityLbA = Eigen::VectorXd(6 + 12 + 5 * mNumberOfContact);
    mInequalityUb = Eigen::VectorXd(18 + 3 * mNumberOfContact);
    mInequalityLb = Eigen::VectorXd(18 + 3 * mNumberOfContact);
    mInequalityA.setZero();
    mInequalityUbA.setConstant(120);
    mInequalityLbA.setZero();
    mInequalityUb.setConstant(60);
    mInequalityLb.setConstant(-60);

    mInequalityLb.block<3, 1>(0, 0).setConstant(-50);
    mInequalityUb.block<3, 1>(0, 0).setConstant(50);
    mInequalityLb.block<3, 1>(3, 0).setConstant(-50);
    mInequalityUb.block<3, 1>(3, 0).setConstant(50);
    mInequalityLb.block<12, 1>(6, 0).setConstant(-200);
    mInequalityUb.block<12, 1>(6, 0).setConstant(200);

    /// equality
    mInequalityA.block(0, 0, 6, 18 + 3 * mNumberOfContact) = mEqualityA;
    mInequalityLbA.block<6, 1>(0, 0) = mEqualityB;
    mInequalityUbA.block<6, 1>(0, 0) = mEqualityB;
    for (int idx = 0; idx < mNumberOfContact; idx++)
    {
        /// friction Cone
        mInequalityA.block(6 + idx * 5, 18 + idx * 3, 5, 3) = mFrictionBlock;
        mInequalityUbA[6 + idx * 5 + 4] = 200;
        /// Fz bound
        mInequalityLb[18 + 3 * idx + 2] = 0;
        mInequalityUb[18 + 3 * idx + 2] = 200;
    }
    /// torque limit
    mInequalityA.block<12, 18>(6 + 5 * mNumberOfContact, 0) = mMassMatrix.block<12, 18>(6, 0);
    mInequalityA.block(6 + 5 * mNumberOfContact, 18, 12, 3 * mNumberOfContact) = -mContactJacobian.transpose().block(6, 0, 12, 3 * mNumberOfContact);
    for (int idx = 0; idx < 12; idx++)
    {
        mInequalityLbA[6 + 5 * mNumberOfContact + idx] = -mTorqueLimit - mNonlinearEff[6 + idx];
        mInequalityUbA[6 + 5 * mNumberOfContact + idx] = mTorqueLimit - mNonlinearEff[6 + idx];
    }
}

void WBTorqueGenerator::setQpOasesMat()
{
    mObjGain = Eigen::MatrixXd::Zero(18 + 3 * mNumberOfContact, 18 + 3 * mNumberOfContact);
    mHMatQp = Eigen::MatrixXd::Zero(18 + 3 * mNumberOfContact, 18 + 3 * mNumberOfContact);
    mGMatQp = Eigen::VectorXd::Zero(18 + 3 * mNumberOfContact);
    setObjWeight();
    // base linear acc(3), base angular acc(3), leg acc(4*3), torque(12), force(3*contact)
    mHMatQp = mObjectA.transpose() * mObjGain * mObjectA;
    /// make g
    mGMatQp = -mObjectA.transpose() * mObjGain * mObjectB;

    /// transform mat 2 real
    free(mHRealQp);
    free(mGRealQp);
    free(mARealQP);
    free(mLbARealQP);
    free(mUbARealQP);
    free(mLbRealQP);
    free(mUbRealQP);
    free(mXRealQP);

    mHRealQp = (qpOASES::real_t*)malloc((mHMatQp.rows()) * (mHMatQp.cols()) * sizeof(qpOASES::real_t));
    mGRealQp = (qpOASES::real_t*)malloc((mGMatQp.size()) * sizeof(qpOASES::real_t));
    mARealQP = (qpOASES::real_t*)malloc((mInequalityA.rows()) * (mInequalityA.cols()) * sizeof(qpOASES::real_t));
    mLbARealQP = (qpOASES::real_t*)malloc((mInequalityLbA.size()) * sizeof(qpOASES::real_t));
    mUbARealQP = (qpOASES::real_t*)malloc((mInequalityUbA.size()) * sizeof(qpOASES::real_t));
    mLbRealQP = (qpOASES::real_t*)malloc((mInequalityLb.size()) * sizeof(qpOASES::real_t));
    mUbRealQP = (qpOASES::real_t*)malloc((mInequalityUb.size()) * sizeof(qpOASES::real_t));
    mXRealQP = (qpOASES::real_t*)malloc((18 + 3 * mNumberOfContact) * sizeof(qpOASES::real_t));

    transformMat2Real(mHRealQp, mHMatQp);
    transformMat2Real(mGRealQp, mGMatQp);
    transformMat2Real(mARealQP, mInequalityA);
    transformMat2Real(mLbARealQP, mInequalityLbA);
    transformMat2Real(mUbARealQP, mInequalityUbA);
    transformMat2Real(mLbRealQP, mInequalityLb);
    transformMat2Real(mUbRealQP, mInequalityUb);
}

void WBTorqueGenerator::solveQP()
{
    qpOASES::int_t nWSR = 10000;
    qpOASES::QProblem problem(18 + 3 * mNumberOfContact, 18 + 5 * mNumberOfContact);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem.setOptions(op);
    problem.init(mHRealQp, mGRealQp, mARealQP, mLbRealQP, mUbRealQP, mLbARealQP, mUbARealQP, nWSR);

    int result = problem.getPrimalSolution(mXRealQP);
    if (result != qpOASES::SUCCESSFUL_RETURN)
    {
        printf("[WBC] failed to solve!\n");
    }
    else
    {
        mOptimalF = Eigen::VectorXd::Zero(3 * mNumberOfContact);
        mContactTau = Eigen::VectorXd::Zero(3 * mNumberOfContact);

        for (int idx = 0; idx < 3 * mNumberOfContact; idx++)
        {
            mOptimalF[idx] = mXRealQP[18 + idx];
        }
        mContactTau = -mContactJacobian.transpose().block(6, 0, 12, 3 * mNumberOfContact) * mOptimalF;

        for (int idx = 0; idx < 18; idx++)
        {
            mOptimalQ[idx] = mXRealQP[idx];
        }
        mOptimalTau = mMassMatrix.block<12, 18>(6, 0) * mOptimalQ + mContactTau + mNonlinearEff.block<12, 1>(6, 0);
        Vec3<double> ref_qdd[4];
        mOptimalQ.block(0, 0, 6, 1).setZero();
        for (int leg = 0; leg < 4; leg++)
        {
            for (int motor = 0; motor < 3; motor++)
            {
                mTorque[leg * 3 + motor] = sharedMemory->pdTorque[leg][motor] + mOptimalTau[leg * 3 + motor];
            }
        }
    }
    int cnt = 0;
    for (int leg = 0; leg < 4; leg++)
    {
        if (mContactLeg[leg])
        {
            sharedMemory->solvedGRF[leg] = mOptimalF.block<3, 1>(cnt * 3, 0);
            cnt++;
        }
        else
        {
            sharedMemory->solvedGRF[leg] = Eigen::Vector3d::Zero();
        }
    }
}

void WBTorqueGenerator::setTorque()
{
    double* rawFilteredTau = tauLPF.GetFilteredVar(mTorque);
    Eigen::Map<Eigen::Array<double, 12, 1>> filteredTau(rawFilteredTau);
    filteredTau = filteredTau.max(-1 * mTorqueLimit);
    filteredTau = filteredTau.min(mTorqueLimit);
    std::copy(filteredTau.data(), filteredTau.data() + 12, sharedMemory->motorDesiredTorque);
}

void WBTorqueGenerator::transformMat2Real(qpOASES::real_t* result, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> source)
{
    int idx = 0;
    for (int row = 0; row < source.rows(); row++)
    {
        for (int col = 0; col < source.cols(); col++)
        {
            result[idx] = source(row, col);
            idx++;
        }
    }
}