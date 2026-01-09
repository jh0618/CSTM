//
// Created by hs on 22. 10. 6.
//

#ifndef RAISIM_LOWCONTROLMAIN_HPP
#define RAISIM_LOWCONTROLMAIN_HPP

#include <stdint.h>

#include "SharedMemory.hpp"
#include "EnumClasses.hpp"
#include "EigenTypes.hpp"

#include "WBTorqueGenerator/WBTorqueGenerator.hpp"
#include <ControlUtils/Gait.hpp>
#include "QuadMotionGenerator/QuadMotionGenerator.hpp"
#include "PDTorqueGenerator.hpp"

class LowControlMain {
public:
    LowControlMain();

    void ControllerFunction();

private:
    void resetParameters();

private:
    SharedMemory *sharedMemory;
    double mAlpha;
    double mCalTorque[LEG_MOTOR_NUM];
    double mRefTime;
    bool bStandUp;
    bool bStandDown;
    WBTorqueGenerator WBController;
    PDTorqueGenerator PDController;
};

#endif //RAISIM_CONTROLSTATE_H
