//
// Created by jh on 11/6/24.
//

#include "DivergenceEstimator.hpp"

DivergenceEstimator::DivergenceEstimator()
    : mbDiverged(false)
{
    sharedMemory = SharedMemory::getInstance();
}

void DivergenceEstimator::DivergenceEstimatorFunction()
{
    if((sharedMemory->FSMState != FSM_INITIAL) && (sharedMemory->FSMState != FSM_EMERGENCY_STOP) && (sharedMemory->FSMState != FSM_RESTART))
    {
        if((sharedMemory->FSMState != FSM_READY) && (sharedMemory->FSMState != FSM_STAND_UP) && (sharedMemory->FSMState != FSM_SIT_DOWN))
        {
            checkGlobalBodyHeight(); // body2foot height
        }
        checkOrientation();
        checkBody2FootPosition();
        setRobotState();
    }
}

void DivergenceEstimator::checkOrientation()
{
    if(std::abs(sharedMemory->globalBaseEulerAngle[0]) > 40 * D2R)
    {
        mbDiverged = true;
        std::cout<<"[E-STOP] roll-over."<<std::endl;
    }

    if(std::abs(sharedMemory->globalBaseEulerAngle[1]) > 40 * D2R)
    {
        mbDiverged = true;
        std::cout<<"[E-STOP] pitch-over."<<std::endl;
    }
}

void DivergenceEstimator::checkBody2FootPosition()
{

    for (int leg = 0 ; leg < 4 ; leg++)
    {
        int sign = 1;
        if(leg < 2)
        {
            sign = -1;
        }
        if((std::abs(sharedMemory->bodyBase2FootPosition[leg][0] + sign * HIP_CENTER_POSITION_X) > 0.3) && (std::abs(sharedMemory->bodyBase2FootPosition[leg][2]) < 0.1))
        {
            mbDiverged = true;
            std::cout<<"[E-STOP] unsafe foot position."<<leg<<std::endl;
        }
    }
}

void DivergenceEstimator::setRobotState()
{
    sharedMemory->bStateDiverged = mbDiverged;
}

void DivergenceEstimator::checkGlobalBodyHeight()
{
    if(sharedMemory->bodyBasePosition[2] < 0.15)
    {
        mbDiverged = true;
        std::cout<<"[E-STOP] low body height."<<std::endl;
    }
}