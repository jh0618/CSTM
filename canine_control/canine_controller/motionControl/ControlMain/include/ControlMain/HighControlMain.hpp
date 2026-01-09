//
// Created by hs on 23. 2. 9.
//

#ifndef RAISIM_HIGHCONTROLMAIN_HPP
#define RAISIM_HIGHCONTROLMAIN_HPP

#include "SharedMemory.hpp"
#include "EnumClasses.hpp"
#include "EigenTypes.hpp"

#include <ControlUtils/Gait.hpp>

class HighControlMain
{
public:
    HighControlMain();

    void ControllerFunction();

private:
    void startGaitSelectPanel();
    void startGaitChangePanel(OffsetGait* currentGait);
    void updateGaitState();

private:
    SharedMemory* sharedMemory;
    uint64_t mIteration;
    bool mbFirstLateLanding[4];
    OffsetGait mStand, mTrotSlow, mTrotFast, mOverlapTrotFast;
    OffsetGait mTrotStair;
    double mStumbleRecoverySwingTime;
    bool mbStumbleSwing;
    int mStumbleLeg;
};

#endif //RAISIM_HIGHCONTROLMAIN_HPP
