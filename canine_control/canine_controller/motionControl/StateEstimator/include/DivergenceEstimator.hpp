//
// Created by jh on 11/6/24.
//

#ifndef DIVERGENCEESTIMATOR_HPP
#define DIVERGENCEESTIMATOR_HPP

#include "EnumClasses.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"

class DivergenceEstimator
{
public:
    DivergenceEstimator();
    void DivergenceEstimatorFunction();

private:
    SharedMemory* sharedMemory;
    void checkOrientation();
    void checkBody2FootPosition();
    void checkGlobalBodyHeight();
    void setRobotState();
    bool mbDiverged;
};


#endif //DIVERGENCEESTIMATOR_HPP
