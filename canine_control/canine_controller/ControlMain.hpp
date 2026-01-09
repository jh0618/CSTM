//
// Created by ys on 24. 2. 11.
//

#ifndef CAMEL_CANINE_CONTROLMAIN_HPP
#define CAMEL_CANINE_CONTROLMAIN_HPP

#include <array>


class HighControlMain;

class LowControlMain;

class StateEstimator;

class DivergenceEstimator;

class SharedMemory;

class _HWD_;

class ControlMain
{
public:
    static ControlMain* getInstance()
    {
        static ControlMain instance;
        return &instance;
    };
    void startThread();
    void doHardControl();
    void doHighControl();
private:
    ControlMain();

    std::array<double, 12> mSimulJointPosition;
    std::array<double, 12> mSimulJointVelocity;
    std::array<double, 12> mSimulJointTorque;
    std::array<double, 4> mSimulBaseQuat;
    std::array<double, 3> mSimulBaseAngularVelocity;
    std::array<double, 12> mJointDesiredTorque;

private:
    HighControlMain* mHighControl;
    LowControlMain* mLowControl;
    StateEstimator* mStateEstimator;
    DivergenceEstimator* mDivergenceEsitmator;
    SharedMemory* sharedMemory;
    _HWD_* HWData;
    void copyHardData();
    void doStateEstimation();
    void doLowControl();
    void sendMotorTorque();
};

void* RT_High_Controller_Thread(void* arg);
void* RT_Low_Controller_Thread(void* arg);
void* RT_Logger_Thread(void* arg);
void* NRT_COMMAND_Thread(void* arg);


#endif //CAMEL_CANINE_CONTROLMAIN_HPP
