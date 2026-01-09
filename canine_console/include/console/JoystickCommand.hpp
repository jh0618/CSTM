#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "SharedMemory.hpp"
#include "Gamepad.hpp"


class JoystickCommand
{
public:
    JoystickCommand();
    void JoystickCommandFunction();
    double bodyLinVel_ref[3];
    double bodyAngVel_ref[3];
    int8_t joystickCommand;
    int FSMState;

private:
    void mappingFunction();
    void sendJoystickinfo();

    void mappingStart();
    void mappingEmergencyStop();
    void mappingStandUp();
    void mappingStandDown();
    void mappingTrotStop();
    void mappingTrotSlow();
    void mappingTrotFast();
    void mappingTrotOverlap();
    void mappingRestart();
    void mappingJoystick();

private:

    SharedMemory* sharedMemory;
    Gamepad* gamepad;

};


#endif //RAISIM_COMMAND_H
