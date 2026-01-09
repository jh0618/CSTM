#include "Command.hpp"


Command::Command()
    : mCmdVelThreshold(0.001)
    , mTrotStopThreshold(0.1)
{
    sharedMemory = SharedMemory::getInstance();
    HWData = _HWD_::getInstance();
    mHoldCommand = false;

    mStumbleRecoveryTime = 0.0;
    mbStumbleRecovery = false;
    mGlobalStumbleStableBasePosition.setZero();

    mHipPosition[FL_IDX] << HIP_CENTER_POSITION_X, HIP_CENTER_POSITION_Y, 0;
    mHipPosition[FR_IDX] << HIP_CENTER_POSITION_X, -HIP_CENTER_POSITION_Y, 0;
    mHipPosition[HL_IDX] << -HIP_CENTER_POSITION_X, HIP_CENTER_POSITION_Y, 0;
    mHipPosition[HR_IDX] << -HIP_CENTER_POSITION_X, -HIP_CENTER_POSITION_Y, 0;

    mShoulderPosition[FL_IDX] << SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[FR_IDX] << SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mShoulderPosition[HL_IDX] << -SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[HR_IDX] << -SHOULD_X_POS, -SHOULD_Y_POS, 0;

    mHip2ShoulderPosition[FL_IDX] = Eigen::Vector3d(SHOULD_X_POS - HIP_CENTER_POSITION_X, SHOULD_Y_POS - HIP_CENTER_POSITION_Y, 0);
    mHip2ShoulderPosition[FR_IDX] = Eigen::Vector3d(SHOULD_X_POS - HIP_CENTER_POSITION_X, -SHOULD_Y_POS + HIP_CENTER_POSITION_Y, 0);
    mHip2ShoulderPosition[HL_IDX] = Eigen::Vector3d(-SHOULD_X_POS + HIP_CENTER_POSITION_X, SHOULD_Y_POS - HIP_CENTER_POSITION_Y, 0);
    mHip2ShoulderPosition[HR_IDX] = Eigen::Vector3d(-SHOULD_X_POS + HIP_CENTER_POSITION_X, -SHOULD_Y_POS + HIP_CENTER_POSITION_Y, 0);

    mStumbleRecoveryTime = 0;
}

void Command::commandFunction()
{
    if (CommandMapping::newCommand)
    {
        mHoldCommand = true;
        CommandMapping::newCommand = false;
        int incomingCommand = sharedMemory->command.userCommand;
        switch (incomingCommand)
        {
        case MOTOR_START:
            sharedMemory->LowControlState = STATE_LOW_CONTROL_STOP;
            sharedMemory->canState[FL_IDX] = CAN_INIT;
            sharedMemory->canState[FR_IDX] = CAN_INIT;
            sharedMemory->canState[HL_IDX] = CAN_INIT;
            sharedMemory->canState[HR_IDX] = CAN_INIT;
            sharedMemory->FSMState = FSM_READY;
            mHoldCommand = false;
            break;
        case MOTOR_OFF:
            sharedMemory->LowControlState = STATE_LOW_CONTROL_STOP;
            sharedMemory->canState[FL_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[FR_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[HL_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[HR_IDX] = CAN_MOTOR_OFF;
            mHoldCommand = false;
            break;
        case STAND_UP:
            sharedMemory->LowControlState = STATE_LOW_HOME_STAND_UP_START;
            sharedMemory->FSMState = FSM_STAND_UP;
            sharedMemory->canState[FL_IDX] = CAN_SET_TORQUE;
            sharedMemory->canState[FR_IDX] = CAN_SET_TORQUE;
            sharedMemory->canState[HL_IDX] = CAN_SET_TORQUE;
            sharedMemory->canState[HR_IDX] = CAN_SET_TORQUE;
            sharedMemory->bIsEndHome = false;
            mHoldCommand = false;
            break;
        case SIT_DOWN:
            sharedMemory->LowControlState = STATE_LOW_HOME_STAND_DOWN_START;
            sharedMemory->FSMState = FSM_SIT_DOWN;
            sharedMemory->canState[FL_IDX] = CAN_SET_TORQUE;
            sharedMemory->canState[FR_IDX] = CAN_SET_TORQUE;
            sharedMemory->canState[HL_IDX] = CAN_SET_TORQUE;
            sharedMemory->canState[HR_IDX] = CAN_SET_TORQUE;
            sharedMemory->bIsEndHome = false;
            mHoldCommand = false;
            break;
        case CHANGE_GAIT_STAND:
            sharedMemory->gaitChangeFlag = true;
            sharedMemory->command.gaitCommand = DESIRED_GAIT_STAND;
            mHoldCommand = false;
            break;
        case CHANGE_GAIT_TROT_SLOW:
            sharedMemory->gaitChangeFlag = true;
            sharedMemory->command.gaitCommand = DESIRED_GAIT_TROT_SLOW;
            mHoldCommand = false;
            break;
        case CHANGE_GAIT_TROT_FAST:
            sharedMemory->gaitChangeFlag = true;
            sharedMemory->command.gaitCommand = DESIRED_GAIT_TROT_FAST;
            mHoldCommand = false;
            break;
        case CHANGE_GAIT_OVERLAP_TROT_FAST:
            sharedMemory->gaitChangeFlag = true;
            sharedMemory->command.gaitCommand = DESIRED_GAIT_OVERLAP_TROT_FAST;
            mHoldCommand = false;
            break;
        case TROT_STOP:
            sharedMemory->FSMState = FSM_TROT_STOP;
            sharedMemory->gaitChangeFlag = true;
            sharedMemory->command.gaitCommand = DESIRED_GAIT_TROT_SLOW;
            mHoldCommand = false;
            break;
        case EMERGENCY_STOP:
            sharedMemory->LowControlState = STATE_LOW_E_STOP;
            sharedMemory->FSMState = FSM_EMERGENCY_STOP;
            sharedMemory->canState[FL_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[FR_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[HL_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[HR_IDX] = CAN_MOTOR_OFF;
            break;
        case RESTART:
            sharedMemory->FSMState = FSM_RESTART;
            sharedMemory->LowControlState = STATE_LOW_E_STOP;
            sharedMemory->canState[FL_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[FR_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[HL_IDX] = CAN_MOTOR_OFF;
            sharedMemory->canState[HR_IDX] = CAN_MOTOR_OFF;
            mHoldCommand = false;
            std::cout<<"[COMMAND] program is EXITing"<<std::endl;
            sleep(1);
            exit(0);
            break;
        default:
            mHoldCommand = false;
            break;
        }
    }
    else
    {
        if(mHoldCommand && sharedMemory->FSMState == FSM_EMERGENCY_STOP && !sharedMemory->motorStatus)
        {
            mHoldCommand = false;
        }
    }
    if (!mHoldCommand)
    {
        mappingCommand();
    }
}

void Command::mappingCommand()
{
    setJoyInput();
//    CommandMapping::RobotRestart();
    switch (sharedMemory->FSMState)
    {
    case FSM_INITIAL:
        CommandMapping::FSMStart();
        break;
    case FSM_READY:
        CommandMapping::FSMEmergencyStop();
        CommandMapping::FSMStandUp();
        CommandMapping::FSMMotorOff();
        break;
    case FSM_STAND_UP:
        FSMStandUpFunction();
        CommandMapping::FSMEmergencyStop();
        CommandMapping::FSMMotorOff();
        break;
    case FSM_SIT_DOWN:
        FSMSitDownFunction();
        CommandMapping::FSMEmergencyStop();
        CommandMapping::FSMMotorOff();
        break;
    case FSM_STAND:
        FSMConstStandFunction();
        CommandMapping::FSMEmergencyStop();
        CommandMapping::FSMMotorOff();
        CommandMapping::FSMSitDown();
        CommandMapping::FSMTrotSlow();
        CommandMapping::FSMTrotFast();
        CommandMapping::FSMTrotOverlap();
        break;
    case FSM_TROT_STOP:
        FSMTrotStopFunction();
        CommandMapping::FSMTrotForceStop();
        CommandMapping::FSMEmergencyStop();
        break;
    case FSM_TROT_SLOW:
        FSMTrotSlowFunction();
        CommandMapping::FSMEmergencyStop();
        CommandMapping::FSMMotorOff();
        CommandMapping::FSMTrotStop();
        CommandMapping::FSMTrotFast();
        CommandMapping::FSMTrotOverlap();
        break;
    case FSM_TROT_FAST:
        FSMTrotFastFunction();
        CommandMapping::FSMEmergencyStop();
        CommandMapping::FSMMotorOff();
        CommandMapping::FSMTrotStop();
        CommandMapping::FSMTrotSlow();
        CommandMapping::FSMTrotOverlap();
        break;
    case FSM_OVERLAP_TROT_FAST:
        FSMTrotOverlapFunction();
        CommandMapping::FSMEmergencyStop();
        CommandMapping::FSMMotorOff();
        CommandMapping::FSMTrotStop();
        CommandMapping::FSMTrotSlow();
        CommandMapping::FSMTrotFast();
        break;
    case FSM_EMERGENCY_STOP:
        CommandMapping::FSMRestart();
        break;
    case FSM_RESTART:
        break;
    default:
        CommandMapping::FSMEmergencyStop();
        break;
    }
}

void Command::FSMStandUpFunction()
{
    if (sharedMemory->bIsEndHome)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = CHANGE_GAIT_STAND;
    }
}

void Command::FSMSitDownFunction()
{
    if (sharedMemory->bIsEndHome)
    {
        sharedMemory->FSMState = FSM_READY;
    }
}

void Command::FSMConstStandFunction()
{
//    mJoyAngVel = mJoyAngVel.array().max(-0.7);
//    mJoyAngVel = mJoyAngVel.array().min(0.7);
//
//    sharedMemory->commandAngularVelocity = mJoyAngVel;

//    for (int index = 0; index < 3; index++)
//    {
//        if ((sharedMemory->globalBaseDesiredEulerAngle[index] > PI / 15 + CommandMapping::BaseReferenceEulerPosition[index]) && (sharedMemory->commandAngularVelocity[index] > 0.0))
//        {
//            sharedMemory->commandAngularVelocity[index] = 0.0;
//        }
//        else if ((sharedMemory->globalBaseDesiredEulerAngle[index] < -PI / 15 + CommandMapping::BaseReferenceEulerPosition[index]) && (sharedMemory->commandAngularVelocity[index] < 0.0))
//        {
//            sharedMemory->commandAngularVelocity[index] = 0.0;
//        }
//    }

    if(sharedMemory->stumble.recoveryStage == STUMBLE::BALANCING)
    {
        std::cout<<"BALANCING recovery"<<std::endl;
        Eigen::Vector3d groundBody;
        Eigen::Vector3d groundContactCenter;
        Eigen::Matrix3d rotWB = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion);
        Eigen::Matrix3d rotBS = RobotMath::GetSlopeRotationMat(sharedMemory->estimatedLocalSlopeRPY);
        Eigen::Matrix3d rotWS = rotWB * rotBS;
        Eigen::Vector3d worldZ = rotWS.transpose()* Eigen::Vector3d(0, 0, 1);
        double l_gravity = sharedMemory->bodyBasePosition[2]/worldZ[2];
        groundBody = sharedMemory->bodyBasePosition - rotWS.transpose()*Eigen::Vector3d(0, 0, l_gravity);
        groundBody[2] = 0;
        groundContactCenter = rotBS.transpose() * (sharedMemory->bodyBase2FootPosition[0] + sharedMemory->bodyBase2FootPosition[1] + sharedMemory->bodyBase2FootPosition[2] + sharedMemory->bodyBase2FootPosition[3])/4;
        groundContactCenter[2] = 0;
        mJoyLinVel = groundContactCenter-groundBody;

        mJoyAngVel = sharedMemory->globalBaseEulerAngle - sharedMemory->estimatedGlobalSlopeRPY;

        Vec3<double> temp = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion) * (sharedMemory->bodyBase2FootPosition[FL_IDX]-sharedMemory->bodyBase2FootPosition[HL_IDX] + sharedMemory->bodyBase2FootPosition[FR_IDX]-sharedMemory->bodyBase2FootPosition[HR_IDX])/2;
        double deltaYaw = atan2(temp[1],temp[0]);

        // Calculate the number of full revolutions (in multiples of 2π)
        double revolutions = std::floor(sharedMemory->globalBaseEulerAngle[2] / (2 * M_PI));
        double baseAngle = revolutions * 2 * M_PI;

        // Calculate the possible angles close to the multi-turn angle
        double possibleAngles[3] = {
            baseAngle + deltaYaw,
            baseAngle + deltaYaw + 2 * M_PI,
            baseAngle + deltaYaw - 2 * M_PI
        };

        // Find the closest angle to the multi-turn angle
        double closestAngle = possibleAngles[0];
        double minDifference = std::abs(closestAngle - sharedMemory->globalBaseEulerAngle[2]);

        for (int i = 1; i < 3; ++i) {
            double difference = std::abs(possibleAngles[i] - sharedMemory->globalBaseEulerAngle[2]);
            if (difference < minDifference)
            {
                closestAngle = possibleAngles[i];
                minDifference = difference;
            }
        }

        mJoyAngVel[2] = closestAngle - sharedMemory->globalBaseEulerAngle[2];
//        mJoyAngVel.setZero();

        if(mJoyLinVel.norm() < 0.01 && mJoyLinVel.norm() < 0.01)
        {
            sharedMemory->stumble.bRecovery = false;
            sharedMemory->stumble.recoveryStage = STUMBLE::NO_ACT;
//            sharedMemory->stumble.recoveryStage = STUMBLE::FOOT_RECOVERY;
        }

        mJoyAngVel = mJoyAngVel.array().max(-0.5);
        mJoyAngVel = mJoyAngVel.array().min(0.5);
        mJoyLinVel = mJoyLinVel.array().max(-0.5);
        mJoyLinVel = mJoyLinVel.array().min(0.5);
        if ((sharedMemory->commandLinearVelocity - mJoyLinVel).norm() >= 0.5)
        {
            sharedMemory->commandLinearVelocity = (mJoyLinVel - sharedMemory->commandLinearVelocity).normalized() * mCmdVelThreshold + sharedMemory->commandLinearVelocity;
        }
        else
        {
            sharedMemory->commandLinearVelocity = mJoyLinVel;
        }

        sharedMemory->commandAngularVelocity[2] = mJoyAngVel[2];
        for (int index = 0; index < 3; index++)
        {
            if ((sharedMemory->globalBaseDesiredEulerAngle[index] > PI / 15 + CommandMapping::BaseReferenceEulerPosition[index]) && (sharedMemory->commandAngularVelocity[index] > 0.0))
            {
                sharedMemory->commandAngularVelocity[index] = 0.0;
            }
            else if ((sharedMemory->globalBaseDesiredEulerAngle[index] < -PI / 15 + CommandMapping::BaseReferenceEulerPosition[index]) && (sharedMemory->commandAngularVelocity[index] < 0.0))
            {
                sharedMemory->commandAngularVelocity[index] = 0.0;
            }
        }
//        sharedMemory->commandAngularVelocity.setZero();
//        sharedMemory->commandLinearVelocity.setZero();
    }
    else
    {
        sharedMemory->commandAngularVelocity.setZero();
        sharedMemory->commandLinearVelocity.setZero();
    }
}

void Command::FSMTrotSlowFunction()
{
    /// stumble strategy
//    Eigen::Matrix3d rotWB = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion);
//    Eigen::Matrix3d rotBS = RobotMath::GetSlopeRotationMat(sharedMemory->estimatedLocalSlopeRPY);
//    Eigen::Matrix3d rotWS = rotWB * rotBS;
//
//    Eigen::Vector3d groundShoulder[4];
//    Eigen::Vector3d heading;
//    double yaw = 0;
//    heading.setZero();
//
//    for(int leg=0; leg<4; leg++)
//    {
//        Eigen::Vector3d worldZ = rotWS.transpose()* Eigen::Vector3d(0, 0, 1);
//        double l_gravity = sharedMemory->bodyBasePosition[2]/worldZ[2];
//        groundShoulder[leg] = sharedMemory->bodyBasePosition + rotBS.transpose() * mShoulderPosition[leg] - rotWS.transpose()*Eigen::Vector3d(0, 0, l_gravity);
//        groundShoulder[leg][2] = 0;
//
//        if(sharedMemory->stumble.bStumble)
//        {
//            sharedMemory->stumble.bRecovery = true;
//        }
//    }
//
//    sharedMemory->stumble.num = 0;
//
//    if(sharedMemory->stumble.bRecovery)
//    {
//        for(int leg=0; leg<4; leg++)
//        {
//            if(sharedMemory->stumble.bLeg[leg])
//            {
//                sharedMemory->stumble.bHeadingLeg[leg] = true;
//            }
//
//            if(sharedMemory->stumble.bHeadingLeg[leg])
//            {
//                sharedMemory->stumble.num++;
//                heading = heading + groundShoulder[leg] - sharedMemory->groundContactFootPos[leg];
//
//                if(heading.norm() > 0.07)
//                {
//                    yaw = yaw + atan2(heading[1], heading[0]);
//                }
//            }
//        }
//
//        mJoyLinVel = -heading/sharedMemory->stumble.num * 3;
//        mJoyAngVel[2] = yaw/sharedMemory->stumble.num/5;
//    }
//
//    if(heading.norm() < 0.01)
//    {
//        sharedMemory->stumble.bRecovery = false;
//        for(int leg=0; leg<4; leg++)
//        {
//            sharedMemory->stumble.bHeadingLeg[leg] = false;
//            mStumbleRelaxTime = sharedMemory->localTime + 1.0;
//        }
//    }
//
    /// stumble strategy end

    /// stumble strategy version 3
    if(sharedMemory->stumble.recoveryStage == STUMBLE::STOPPING)
    {
        std::cout<<"STOPPING recovery"<<std::endl;
        Eigen::Vector3d groundBody;
        Eigen::Matrix3d rotWB = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion);
        Eigen::Matrix3d rotBS = RobotMath::GetSlopeRotationMat(sharedMemory->estimatedLocalSlopeRPY);
        Eigen::Matrix3d rotWS = rotWB * rotBS;
        Eigen::Vector3d worldZ = rotWS.transpose()* Eigen::Vector3d(0, 0, 1);
        double l_gravity = sharedMemory->bodyBasePosition[2]/worldZ[2];
        groundBody = sharedMemory->bodyBasePosition - rotWS.transpose()*Eigen::Vector3d(0, 0, l_gravity);
        mJoyLinVel = -groundBody;
//        mJoyLinVel.setZero();
        mJoyAngVel.setZero();
    }
    ///

    mJoyAngVel = mJoyAngVel.array().max(-0.7);
    mJoyAngVel = mJoyAngVel.array().min(0.7);
    mJoyLinVel = mJoyLinVel.array().max(-0.7);
    mJoyLinVel = mJoyLinVel.array().min(0.7);

    if(sharedMemory->stumble.recoveryStage == STUMBLE::STOPPING)
    {
        if((sharedMemory->commandLinearVelocity - mJoyLinVel).norm() >= 0.5)
        {
            sharedMemory->commandLinearVelocity = (mJoyLinVel - sharedMemory->commandLinearVelocity).normalized() + sharedMemory->commandLinearVelocity;
        }
        else
        {
            sharedMemory->commandLinearVelocity = mJoyLinVel;
        }
    }
    else
    {
        if ((sharedMemory->commandLinearVelocity - mJoyLinVel).norm() >= mCmdVelThreshold)
        {
            sharedMemory->commandLinearVelocity = (mJoyLinVel - sharedMemory->commandLinearVelocity).normalized() * mCmdVelThreshold + sharedMemory->commandLinearVelocity;
        }
        else
        {
            sharedMemory->commandLinearVelocity = mJoyLinVel;
        }
    }
    sharedMemory->commandAngularVelocity[2] = mJoyAngVel[2];

    if(sharedMemory->stumble.recoveryStage == STUMBLE::STOPPING && sharedMemory->gaitState == STAND)
    {
        sharedMemory->FSMState = FSM_STAND;
        sharedMemory->stumble.recoveryStage = STUMBLE::BALANCING;
    }

//    if(sharedMemory->localTime < mStumbleRecoveryTime)
//    {
//        sharedMemory->commandLinearVelocity.setZero();
//        sharedMemory->commandAngularVelocity.setZero();
//    }
}

void Command::FSMTrotFastFunction()
{
    mJoyAngVel = mJoyAngVel.array().max(-0.7);
    mJoyAngVel = mJoyAngVel.array().min(0.7);
    mJoyLinVel = mJoyLinVel.array().max(-1.2);
    mJoyLinVel = mJoyLinVel.array().min(1.2);
    if ((sharedMemory->commandLinearVelocity - mJoyLinVel).norm() >= mCmdVelThreshold)
    {
        sharedMemory->commandLinearVelocity = (mJoyLinVel - sharedMemory->commandLinearVelocity).normalized() * mCmdVelThreshold + sharedMemory->commandLinearVelocity;
    }
    else
    {
        sharedMemory->commandLinearVelocity = mJoyLinVel;
    }
    sharedMemory->commandAngularVelocity[2] = mJoyAngVel[2];
}

void Command::FSMTrotOverlapFunction()
{
    mJoyAngVel = mJoyAngVel.array().max(-0.7);
    mJoyAngVel = mJoyAngVel.array().min(0.7);
    mJoyLinVel = mJoyLinVel.array().max(-1.0);
    mJoyLinVel = mJoyLinVel.array().min(1.0);
    if ((sharedMemory->commandLinearVelocity - mJoyLinVel).norm() >= mCmdVelThreshold)
    {
        sharedMemory->commandLinearVelocity = (mJoyLinVel - sharedMemory->commandLinearVelocity).normalized() * mCmdVelThreshold + sharedMemory->commandLinearVelocity;
    }
    else
    {
        sharedMemory->commandLinearVelocity = mJoyLinVel;
    }
    sharedMemory->commandAngularVelocity[2] = mJoyAngVel[2];
}

void Command::setJoyInput()
{
    static int cnt;
    if (HWData->gamepad.newCommand)
    {
        if ((int)HWData->gamepad.joyCommand != GAMEPAD_NO_INPUT && mLastJoyCommand != (int)HWData->gamepad.joyCommand)
        {
            CommandMapping::JoystickCommand = (int)HWData->gamepad.joyCommand;
            mLastJoyCommand = CommandMapping::JoystickCommand;
        }
        else
        {
            CommandMapping::JoystickCommand = GAMEPAD_NO_INPUT;
        }
        if ((int)HWData->gamepad.joyCommand == GAMEPAD_NO_INPUT && mLastJoyCommand != (int)HWData->gamepad.joyCommand)
        {
            CommandMapping::JoystickCommand = (int)HWData->gamepad.joyCommand;
            mLastJoyCommand = CommandMapping::JoystickCommand;
        }
        mJoyLinVel = HWData->gamepad.userLinVel;
        mJoyAngVel = HWData->gamepad.userAngVel;
        HWData->gamepad.newCommand = false;
        cnt = 0;
    }
    else
    {
        CommandMapping::JoystickCommand = GAMEPAD_NO_INPUT;
        if (cnt <= 1000)
        {
            mJoyLinVel = HWData->gamepad.userLinVel;
            mJoyAngVel = HWData->gamepad.userAngVel;
        }
        else
        {
            mJoyLinVel.setZero();
            mJoyAngVel.setZero();
        }
        cnt++;
    }
}

void Command::FSMTrotStopFunction()
{
    mJoyLinVel.setZero();
    mJoyAngVel.setZero();
    if ((sharedMemory->commandLinearVelocity - mJoyLinVel).norm() >= mCmdVelThreshold)
    {
        sharedMemory->commandLinearVelocity = (mJoyLinVel - sharedMemory->commandLinearVelocity).normalized() * mCmdVelThreshold + sharedMemory->commandLinearVelocity;
    }
    else
    {
        sharedMemory->commandLinearVelocity = mJoyLinVel;
    }
    if (sharedMemory->commandLinearVelocity.norm() <= mTrotStopThreshold && sharedMemory->globalBaseVelocity.norm() <= mTrotStopThreshold && sharedMemory->bodyBaseAngularVelocity.norm() <= mTrotStopThreshold)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = CHANGE_GAIT_STAND;
    }
}