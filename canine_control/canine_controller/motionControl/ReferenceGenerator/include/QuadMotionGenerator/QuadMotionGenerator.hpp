//
// Created by ys on 24. 8. 7.
//

#ifndef RBQ_QUADMOTIONGENERATOR_HPP
#define RBQ_QUADMOTIONGENERATOR_HPP

#include "LegTrajectoryGenerator/LegTrajectoryGen.hpp"
#include "BodyTrajectoryGenerator/BodyTrajectoryGen.hpp"

namespace QuadMotionGenerator{
    static LegTrajectoryGen legTrajectoryGen;
    static BodyTrajectoryGen bodyTrajectoryGen;
    void InitHomeUp();
    void InitHomeDown();
    void SetHomeTrajectory();
    void SetWalkingTrajectory();
}

#endif //RBQ_QUADMOTIONGENERATOR_HPP
