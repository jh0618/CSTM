//
// Created by ys on 24. 7. 2.
//

#include "PDTorqueGenerator.hpp"

PDTorqueGenerator::PDTorqueGenerator()
    : TorqueLimit(30)
{
    Kp.setConstant(100);
    Kd.setConstant(3);
    InitializeVariable();
}

void PDTorqueGenerator::InitializeVariable()
{
    sharedMemory = SharedMemory::getInstance();
    mJointReferenceTorque.setZero();
    mJointReferencePosition.setZero();
    mJointReferenceVelcoity.setZero();
    mJointPosition.setZero();
    mJointVelocity.setZero();
}

double* PDTorqueGenerator::GetTorque()
{
    return mJointReferenceTorque.data();
}

void PDTorqueGenerator::calculatePDTorque()
{
    updateCurrent();
    calculateTorque();
    setTorqueLimit();
}

void PDTorqueGenerator::updateCurrent()
{
    std::copy(sharedMemory->motorPosition, sharedMemory->motorPosition + LEG_MOTOR_NUM, mJointPosition.data());
    std::copy(sharedMemory->motorVelocity, sharedMemory->motorVelocity + LEG_MOTOR_NUM, mJointVelocity.data());
    std::copy(sharedMemory->motorDesiredPosition, sharedMemory->motorDesiredPosition + LEG_MOTOR_NUM, mJointReferencePosition.data());
    std::copy(sharedMemory->motorDesiredVelocity, sharedMemory->motorDesiredVelocity + LEG_MOTOR_NUM, mJointReferenceVelcoity.data());
}

void PDTorqueGenerator::calculateTorque()
{
    mJointReferenceTorque = Kp * (mJointReferencePosition - mJointPosition) + Kd * (mJointReferenceVelcoity - mJointVelocity);
}

void PDTorqueGenerator::setTorqueLimit()
{
    mJointReferenceTorque = mJointReferenceTorque.max(-1 * TorqueLimit);
    mJointReferenceTorque = mJointReferenceTorque.min(TorqueLimit);
}