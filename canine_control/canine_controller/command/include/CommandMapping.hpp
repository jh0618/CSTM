//
// Created by ys on 24. 6. 23.
//

#ifndef RBQ_COMMANDMAPPING_HPP
#define RBQ_COMMANDMAPPING_HPP

#include "SharedMemory.hpp"
#include "iostream"

namespace CommandMapping
{

    inline SharedMemory* sharedMemory = SharedMemory::getInstance();
    inline Eigen::Vector3d BaseReferenceEulerPosition;
    inline int JoystickCommand;
    inline bool newCommand = false;
    inline int recoveryCount = 0;
    void FSMStart();
    void FSMEmergencyStop();
    void FSMStandUp();
    void FSMSitDown();
    void FSMMotorOff();
    void FSMTrotStop();
    void FSMTrotSlow();
    void FSMTrotFast();
    void FSMTrotOverlap();
    void FSMRestart();
    void RobotRestart(); /// no use
    void FSMTrotForceStop();
}


#endif //RBQ_COMMANDMAPPING_HPP
