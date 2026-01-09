#include "console/JoystickCommand.hpp"

JoystickCommand::JoystickCommand()
{
    joystickCommand = GAMEPAD_NO_INPUT;
    gamepad = Gamepad::getInstance();
    sharedMemory = SharedMemory::getInstance();
    sharedMemory->gamepad.controllerName = std::string(gamepad->name_of_joystick);
}

void JoystickCommand::JoystickCommandFunction()
{
    while (true)
    {
        gamepad->Read();
        sendJoystickinfo();
        mappingFunction();

        sharedMemory->gamepad.joyCommand = joystickCommand;
        std::copy(bodyAngVel_ref, bodyAngVel_ref + 3, sharedMemory->gamepad.userAngVel.data());
        std::copy(bodyLinVel_ref, bodyLinVel_ref + 3, sharedMemory->gamepad.userLinVel.data());
    }
}

void JoystickCommand::sendJoystickinfo()
{
    sharedMemory->gamepad.joystick.LeftStickX = (double)gamepad->mJoystickAxis[0] / 30767;
    sharedMemory->gamepad.joystick.LeftStickY = -(double)gamepad->mJoystickAxis[1] / 30767;
    sharedMemory->gamepad.joystick.LeftTrigger = gamepad->mJoystickAxis[2] / 30767;
    sharedMemory->gamepad.joystick.RightStickX = (double)gamepad->mJoystickAxis[3] / 30767;
    sharedMemory->gamepad.joystick.RightStickY = -(double)gamepad->mJoystickAxis[4] / 30767;
    sharedMemory->gamepad.joystick.RightTrigger = gamepad->mJoystickAxis[5] / 30767;
    sharedMemory->gamepad.joystick.DpadX = gamepad->mJoystickAxis[6] / 30767;
    sharedMemory->gamepad.joystick.DpadY = -gamepad->mJoystickAxis[7] / 30767;
    sharedMemory->gamepad.button.A = gamepad->mJoystickButton[0];
    sharedMemory->gamepad.button.B = gamepad->mJoystickButton[1];
    sharedMemory->gamepad.button.X = gamepad->mJoystickButton[2];
    sharedMemory->gamepad.button.Y = gamepad->mJoystickButton[3];
    sharedMemory->gamepad.button.LB = gamepad->mJoystickButton[4];
    sharedMemory->gamepad.button.RB = gamepad->mJoystickButton[5];
    sharedMemory->gamepad.button.Back = gamepad->mJoystickButton[6];
    sharedMemory->gamepad.button.Start = gamepad->mJoystickButton[7];
    sharedMemory->gamepad.button.Guide = gamepad->mJoystickButton[8];
    sharedMemory->gamepad.button.LeftStick = gamepad->mJoystickButton[9];
    sharedMemory->gamepad.button.RightStick = gamepad->mJoystickButton[10];
}

void JoystickCommand::mappingFunction()
{
    joystickCommand = GAMEPAD_NO_INPUT;
//    mappingStart();
//    mappingEmergencyStop();
//    mappingStandUp();
//    mappingStandDown();
//    mappingMotorOff();
//    mappingTrotStop();
//    mappingTrotSlow();
//    mappingTrotFast();
//    mappingTrotOverlap();
//    mappingRecovery();
    mappingJoystick();
    mappingRestart();
    switch (sharedMemory->FSMState)
    {
    case FSM_INITIAL:
        mappingStart();
        mappingEmergencyStop();
        break;
    case FSM_READY:
        mappingStandUp();
        mappingEmergencyStop();
        break;
    case FSM_STAND_UP:
        mappingEmergencyStop();
        break;
    case FSM_SIT_DOWN:
        mappingEmergencyStop();
        break;
    case FSM_STAND:
        mappingStandDown();
        mappingTrotSlow();
        mappingTrotFast();
        mappingTrotOverlap();
        mappingEmergencyStop();
        break;
    case FSM_TROT_STOP:
        mappingTrotStop();
        mappingEmergencyStop();
        break;
    case FSM_TROT_SLOW:
        mappingTrotStop();
        mappingTrotFast();
        mappingTrotOverlap();
        mappingEmergencyStop();
        break;
    case FSM_TROT_FAST:
        mappingTrotStop();
        mappingTrotSlow();
        mappingTrotFast();
        mappingTrotOverlap();
        mappingEmergencyStop();
        break;
    case FSM_OVERLAP_TROT_FAST:
        mappingTrotStop();
        mappingTrotSlow();
        mappingTrotFast();
        mappingEmergencyStop();
        break;
    case FSM_EMERGENCY_STOP:
        mappingRestart();
        break;
    case FSM_RESTART:
        break;
    default:
        break;
    }
}

void JoystickCommand::mappingStart()
{
    if (sharedMemory->gamepad.button.B || sharedMemory->gamepad.gui.GUIButton == GUI_START)
    {
        joystickCommand = GAMEPAD_START;
    }
}

void JoystickCommand::mappingEmergencyStop()
{
    if ((sharedMemory->gamepad.joystick.LeftTrigger > 0.5 && sharedMemory->gamepad.joystick.RightTrigger > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_E_STOP)
    {
        joystickCommand = GAMEPAD_EMERGENCY_STOP;
    }
}

void JoystickCommand::mappingStandUp()
{
    if ((sharedMemory->gamepad.joystick.RightTrigger > 0.5 && sharedMemory->gamepad.joystick.DpadY > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_HOME_UP)
    {
        joystickCommand = GAMEPAD_STAND_UP;
    }
}

void JoystickCommand::mappingStandDown()
{
    if ((sharedMemory->gamepad.joystick.RightTrigger > 0.5 && sharedMemory->gamepad.joystick.DpadY < -0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_HOME_DOWN)
    {
        joystickCommand = GAMEPAD_SIT_DOWN;
    }
}

void JoystickCommand::mappingTrotStop()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadX > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_STOP)
    {
        joystickCommand = GAMEPAD_TROT_STOP;
    }
}

void JoystickCommand::mappingTrotSlow()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadX < -0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_SLOW)
    {
        joystickCommand = GAMEPAD_TROT_SLOW;
    }
}

void JoystickCommand::mappingTrotFast()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadY < -0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_FAST)
    {
        joystickCommand = GAMEPAD_TROT_FAST;
    }
}

void JoystickCommand::mappingTrotOverlap()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadY > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_OVERLAP)
    {
        joystickCommand = GAMEPAD_TROT_OVERLAP;
    }
}

void JoystickCommand::mappingRestart()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_RESTART || sharedMemory->gamepad.button.B)
    {
        joystickCommand = GAMEPAD_RESTART;
    }
}

void JoystickCommand::mappingJoystick()
{
    switch (sharedMemory->FSMState)
    {
    case FSM_STAND:
    {
//        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 0.6;
//        bodyLinVel_ref[1] = -sharedMemory->gamepad.joystick.LeftStickX * 0.4;
//        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    case FSM_TROT_SLOW:
    {
        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 0.6;
        bodyLinVel_ref[1] = -sharedMemory->gamepad.joystick.LeftStickX * 0.4;
        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    case FSM_TROT_FAST:
    {
        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 1.0; // 1 m/s
        bodyLinVel_ref[1] = -sharedMemory->gamepad.joystick.LeftStickX * 0.5;
        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    case FSM_OVERLAP_TROT_FAST:
    {
        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 1.0;
        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    default:
        break;

    }
}