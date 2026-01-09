//
// Created by cha on 24. 1. 24.
//

#ifndef CAMEL_CANINE_CONTACTSTATEESTIMATOR_HPP
#define CAMEL_CANINE_CONTACTSTATEESTIMATOR_HPP

#include <iostream>
#include <iomanip>
#include <math.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "EnumClasses.hpp"
#include "SharedMemory.hpp"
#include "GlobalParameters.hpp"
#include "RobotMath.hpp"

double GaitTableCDF(double swingPeriod, double standPeriod, double time, bool gaitTableState);
double ContactForceCDF(double contactForce);

class ContactStateEstimator
{
public:
    ContactStateEstimator(RigidBodyDynamics::Model* model, const double& dt);
    void GetContactState(const double* baseQuaternion,
                         const double* baseAngularVelocity,
                         const double* baseLinearVelocity,
                         const double* jointPosition,
                         const double* jointVelocity,
                         const double* desiredTorque,
                         const bool* gaitTable,
                         const double& swingPeriod,
                         const double& standPeriod,
                         bool* contactState,
                         double* timeStep);
private:
    void calculateResidual();
    void contactEstimation();
    void periodChecking();
    void showOutput();

private:
    SharedMemory* sharedMemory;
    Eigen::Vector3d mEstimatedGRF[4];
    bool mbContactState[4];
    bool mbGaitTableState[4];
    bool mbPrevGaitTableState[4];

    const double mDt;
    double mContactTorque[4];
    double mSwingPeriod;
    double mStandPeriod;
    double mTimeStep[4];

    Eigen::MatrixXd mGain;

    Eigen::VectorXd mDesiredQd;
    Eigen::Vector4d mEstimatedStates;
    Eigen::Vector4d mMeasurements;
    Eigen::Vector4d mStates;
    Eigen::VectorXd mBeta;
    Eigen::VectorXd mMotorTorque;
    Eigen::VectorXd mMomentum;
    Eigen::VectorXd mQ;
    Eigen::VectorXd mQd;
    Eigen::VectorXd mDisturbanceTorque;
    Eigen::VectorXd mNonlinearEff;

    Eigen::Matrix<double, 4, 4> mErrorCovariance;
    Eigen::Matrix<double, 4, 4> mKalmanGain;
    Eigen::Matrix<double, 4, 4> mMeasurementError;
    Eigen::Matrix<double, 4, 4> mOutputMat;

    RigidBodyDynamics::Math::MatrixNd mMassMatrix;
    RigidBodyDynamics::Math::MatrixNd mPrevMassMatrix;
    RigidBodyDynamics::Model* mModel;
};


#endif //CAMEL_CANINE_CONTACTSTATEESTIMATOR_HPP
