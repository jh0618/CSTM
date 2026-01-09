//
// Created by ys on 24. 8. 7.
//

#include "QuadMotionGenerator/QuadMotionGenerator.hpp"

void QuadMotionGenerator::InitHomeUp()
{
    bodyTrajectoryGen.InitUpTrajectory();
    legTrajectoryGen.initHomeUpTrajectory();
}

void QuadMotionGenerator::InitHomeDown()
{
    bodyTrajectoryGen.InitDownTrajectory();
    legTrajectoryGen.initHomeDownTrajectory();
}

void QuadMotionGenerator::SetHomeTrajectory()
{
    bodyTrajectoryGen.GenerateBaseTrajectory();
    legTrajectoryGen.SetHomeTrajectory();
}

void QuadMotionGenerator::SetWalkingTrajectory()
{
    bodyTrajectoryGen.GenerateBaseTrajectory();
    legTrajectoryGen.SetLegMotion();
}