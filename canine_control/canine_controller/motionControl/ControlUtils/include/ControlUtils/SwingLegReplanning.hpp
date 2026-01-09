//
// Created by jaehoon on 23. 9. 15.
//

#ifndef CAMEL_CANINE_SWINGLEGREPLANNING_HPP
#define CAMEL_CANINE_SWINGLEGREPLANNING_HPP


#include <Eigen/Eigen>
#include "SharedMemory.hpp"

class SwingLegReplanning
{
public:
    SwingLegReplanning();
    void updateCubicTrajectory(double currentPosition,double goalPosition,double currentVelocity, double goalVelocity, double currentTime,double timeDuration);
    void updateLineTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration);
    void CubicReplanning(double originalPosition, double goalPosition);
    void LineReplanning(double originalPosition, double goalPosition);
    void Replanning(double goalPosition, double currentTime);
    void ReplanningGoalVel(double goalPosition, double goalVelocity, double currentTime);
    void calculateCubicCoefficient();
    double getCubicPositionTrajectory(double currentTime);
    double getCubicVelocityTrajectory(double currentTime);
    double getCubicAccelerationTrajectory(double currentTime);
    double getLinePositionTrajectory(double currentTime);
    double getLineVelocityTrajectory(double currentTime);

private:
    void updateMatrixA(double currentTime);
    SharedMemory* sharedMemory;

    Eigen::MatrixXd mCubicMatrixA = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd mCubicCoefficient = Eigen::MatrixXd(4, 1);
    Eigen::MatrixXd mCubicFunctionValue = Eigen::MatrixXd(4, 1);
    double mLineCoefficient[2];
    double mReferenceTime;
    double mTimeDuration;
};

#endif //CAMEL_CANINE_SWINGLEGREPLANNING_HPP
