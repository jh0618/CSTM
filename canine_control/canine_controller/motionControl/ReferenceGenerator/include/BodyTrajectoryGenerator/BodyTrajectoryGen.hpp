//
// Created by hwayoung on 24. 3. 12.
//

#ifndef CAMEL_CANINE_BODYTRAJECTORYGEN_HPP
#define CAMEL_CANINE_BODYTRAJECTORYGEN_HPP

#include "EnumClasses.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "TrajectoryGenerator.hpp"
#include "Filter.hpp"

class BodyTrajectoryGen
{
public:
    BodyTrajectoryGen();
    void InitUpTrajectory();
    void InitDownTrajectory();
    void GenerateBaseTrajectory();
private:
    void UpdateDesiredRotation();
    void UpdateDesiredLinear();
    SharedMemory* sharedMemory;
    Eigen::Vector3d mGlobalBaseDesiredEulerRate;
    Eigen::Matrix3d mRotWorld2Body;
    bool mbStumbleRecovery;
    Eigen::Vector3d mGlobalStumbleStableBasePosition;
    int mNonStumbleCount;
};

#endif //CAMEL_CANINE_BODYTRAJECTORYGEN_HPP
