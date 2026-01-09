//
// Created by ys on 24. 6. 23.
//

#include "CommandMapping.hpp"


void CommandMapping::FSMStart()
{
    if (JoystickCommand == GAMEPAD_START)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = MOTOR_START;
    }
}
void CommandMapping::FSMEmergencyStop()
{
    if (JoystickCommand == GAMEPAD_EMERGENCY_STOP || sharedMemory->bStateDiverged)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = EMERGENCY_STOP;
    }
}

void CommandMapping::FSMStandUp()
{
    if (JoystickCommand == GAMEPAD_STAND_UP)
    {
        CommandMapping::newCommand = true;
        BaseReferenceEulerPosition = sharedMemory->globalBaseEulerAngle;
        sharedMemory->command.userCommand = STAND_UP;
    }
}

void CommandMapping::FSMSitDown()
{
    if (JoystickCommand == GAMEPAD_SIT_DOWN)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = SIT_DOWN;
    }
}

void CommandMapping::FSMMotorOff()
{

}

void CommandMapping::FSMTrotStop()
{
    if (JoystickCommand == GAMEPAD_TROT_STOP && sharedMemory->FSMState != FSM_TROT_STOP)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = TROT_STOP;
        BaseReferenceEulerPosition = sharedMemory->globalBaseEulerAngle;
        sharedMemory->globalBaseDesiredPosition[0] = sharedMemory->globalBasePosition[0];
        sharedMemory->globalBaseDesiredPosition[1] = sharedMemory->globalBasePosition[1];
    }
}

void CommandMapping::FSMTrotSlow()
{
    if(sharedMemory->stumble.bRecovery && JoystickCommand == GAMEPAD_TROT_SLOW)
    {
        CommandMapping::recoveryCount++;
    }
    if(recoveryCount > 2)
    {
        sharedMemory->stumble.bRecovery = false;
        sharedMemory->stumble.recoveryStage = STUMBLE::NO_ACT;
    }
    if (!sharedMemory->stumble.bRecovery && JoystickCommand == GAMEPAD_TROT_SLOW)
    {
        CommandMapping::recoveryCount = 0;
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = CHANGE_GAIT_TROT_SLOW;
    }
}

void CommandMapping::FSMTrotFast()
{
    if (JoystickCommand == GAMEPAD_TROT_FAST)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = CHANGE_GAIT_TROT_FAST;
    }
}

void CommandMapping::FSMTrotOverlap()
{
    if (JoystickCommand == GAMEPAD_TROT_OVERLAP)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = CHANGE_GAIT_OVERLAP_TROT_FAST;
    }
}

void CommandMapping::FSMRestart()
{
    if (JoystickCommand == GAMEPAD_RESTART)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = RESTART;
    }
}

void CommandMapping::RobotRestart()
{
    if (JoystickCommand == GAMEPAD_RESTART)
    {
        sharedMemory->isRobotRestart = true;
    }
}

void CommandMapping::FSMTrotForceStop()
{
    if (JoystickCommand == GAMEPAD_TROT_STOP)
    {
        CommandMapping::newCommand = true;
        sharedMemory->command.userCommand = CHANGE_GAIT_STAND;
    }
}