//
// Created by ys on 24. 7. 2.
//

#ifndef RBQ_PDTORQUEGENERATOR_HPP
#define RBQ_PDTORQUEGENERATOR_HPP

#include <Eigen/Dense>
#include "SharedMemory.hpp"

class PDTorqueGenerator
{
public:
    PDTorqueGenerator();
    void calculatePDTorque();
    double* GetTorque();
private:
    void InitializeVariable();
    void updateCurrent();
    void calculateTorque();
    void setTorqueLimit();
    const double TorqueLimit;
    SharedMemory* sharedMemory;
    Eigen::Array<double, 12, 1> Kp;
    Eigen::Array<double, 12, 1> Kd;

    Eigen::Array<double, 12, 1> mJointReferenceTorque;
    Eigen::Array<double, 12, 1> mJointReferencePosition;
    Eigen::Array<double, 12, 1> mJointReferenceVelcoity;
    Eigen::Array<double, 12, 1> mJointPosition;
    Eigen::Array<double, 12, 1> mJointVelocity;

};


#endif //RBQ_PDTORQUEGENERATOR_HPP
