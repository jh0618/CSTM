//
// Created by hs on 22. 10. 6.
//

#include <ControlMain/LowControlMain.hpp>


LowControlMain::LowControlMain()
    : mAlpha(0), mRefTime(0), bStandUp(false), bStandDown(false)
{
    for (int idx = 0; idx < LEG_MOTOR_NUM; idx++)
    {
        mCalTorque[idx] = 0;
    }
    sharedMemory = SharedMemory::getInstance();
}

void LowControlMain::ControllerFunction()
{

    switch (sharedMemory->LowControlState)
    {
    case STATE_LOW_CONTROL_STOP:
    {
        resetParameters();
        break;
    }
    case STATE_LOW_CONTROL_START:
    {
        QuadMotionGenerator::SetWalkingTrajectory();
        WBController.DoControl();
        break;
    }
    case STATE_LOW_HOME_STAND_UP_START:
    {
        std::cout << "[LOW CONTROL] Home up start!" << std::endl;
        sharedMemory->LowControlState = STATE_LOW_HOME_CONTROL;
        QuadMotionGenerator::InitHomeUp();
        mRefTime = sharedMemory->localTime + STAND_UP_TIME;
        mAlpha = 0;
        bStandUp = true;
        break;
    }
    case STATE_LOW_HOME_STAND_DOWN_START:
    {
        std::cout << "[LOW CONTROL] Home down start!" << std::endl;
        sharedMemory->LowControlState = STATE_LOW_HOME_CONTROL;
        QuadMotionGenerator::InitHomeDown();
        mRefTime = sharedMemory->localTime + STAND_DOWN_TIME;
        mAlpha = 1;
        bStandDown = true;
        break;
    }
    case STATE_LOW_HOME_CONTROL:
    {
        QuadMotionGenerator::SetHomeTrajectory();
        PDController.calculatePDTorque();
        WBController.DoControl();
        if (mAlpha > 1)
        {
            std::cout << "[LOW CONTROL] Home up finished." << std::endl;
            sharedMemory->LowControlState = STATE_LOW_CONTROL_START;
            sharedMemory->bIsEndHome = true;
            bStandDown = false;
            bStandUp = false;
            mAlpha = 1;
        }
        else if (mAlpha < 0)
        {
            std::cout << "[LOW CONTROL] Home down finished." << std::endl;
            bStandDown = false;
            bStandUp = false;
            mAlpha = 0;
        }
        else
        {
            if (bStandUp)
            {
                mAlpha = (STAND_UP_TIME - mRefTime + sharedMemory->localTime) / STAND_UP_TIME;
            }
            if (bStandDown)
            {
                mAlpha = 1 - ((STAND_DOWN_TIME - mRefTime + sharedMemory->localTime) / STAND_DOWN_TIME);
            }
        }
        for (int index = 0; index < LEG_MOTOR_NUM; index++)
        {
            mCalTorque[index] = mAlpha * WBController.GetTorque()[index] + (1 - mAlpha) * PDController.GetTorque()[index];
            if (mCalTorque[index] > TORQUE_LIMIT)
            {
                mCalTorque[index] = TORQUE_LIMIT;
            }
            else if (mCalTorque[index] < -TORQUE_LIMIT)
            {
                mCalTorque[index] = -TORQUE_LIMIT;
            }
            sharedMemory->motorDesiredTorque[index] = mCalTorque[index];
        }
        break;
    }
    case STATE_LOW_RECOVERY_READY:
    {
        std::cout << "[LOW CONTROL] recovery ready." << std::endl;
        sharedMemory->LowControlState = STATE_LOW_RECOVERY_CONTROL;
        break;
    }
    case STATE_LOW_RECOVERY_CONTROL:
    {
        break;
    }
    case STATE_LOW_E_STOP:
        for(int idx = 0; idx< MOTOR_NUM; idx++)
        {
            sharedMemory->motorDesiredTorque[idx] = 0.0;
        }
        break;
    default:
        break;
    }
}

void LowControlMain::resetParameters()
{
    bStandDown = false;
    bStandUp = false;

    sharedMemory->isNan = false;

    for (int index = 0; index < MOTOR_NUM; index++)
    {
        sharedMemory->motorDesiredPosition[index] = sharedMemory->motorPosition[index];
        sharedMemory->motorDesiredVelocity[index] = 0;
        sharedMemory->motorDesiredTorque[index] = 0;
    }

    sharedMemory->globalBaseDesiredPosition = sharedMemory->globalBasePosition;
    sharedMemory->globalBaseDesiredVelocity.setZero();
    sharedMemory->globalBaseDesiredEulerAngle.setZero();
    sharedMemory->globalBaseDesiredEulerAngle[2] = sharedMemory->globalBaseEulerAngle[2];
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
    sharedMemory->bodyBaseDesiredAngularVelocity.setZero();
    sharedMemory->bodyBaseDesiredVelocity.setZero();
    sharedMemory->globalBaseDesiredAngularVelocity.setZero();
    sharedMemory->globalBaseDesiredVelocity.setZero();
    sharedMemory->globalReferencePosition.setZero();
    for (int index = 0; index < 4; index++)
    {
        sharedMemory->pdTorque[index].setZero();
        sharedMemory->mpcTorque[index].setZero();
        sharedMemory->bodyBase2FootDesiredVelocity[index].setZero();
        sharedMemory->swingPgain[index] << 700.0, 700.0, 700.0;
        sharedMemory->swingDgain[index] << 60.0, 60.0, 60.0;
        sharedMemory->isFirstHome[index] = true;
    }
}