//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include "EnumClasses.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "Filter.hpp"
#include "ContactStateEstimator.hpp"

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

class StateEstimator
{
public:
    StateEstimator(RigidBodyDynamics::Model* model);
    void StateEstimatorFunction();

private:
    ContactStateEstimator contactStateEstimator;
    SharedMemory* sharedMemory;

    void getContactState();
    void getRobotBodyFootPosition();
    void getRobotGlobalFootPosition();
    void getRobotLinearState();
    void getEstimatedGroundSlope();
    void showOutputs();
    void getRobotState();
    void updateRobotState();
    Eigen::Vector3d fitPlaneToPoints(const Eigen::Vector3d points[4]);
    Eigen::Matrix3d alignRectangleAxes(const Eigen::Vector3d& normal, const Eigen::Vector3d points[4]);

private:
    Eigen::Vector3d mGravity = { 0, 0, -9.81 };
    const double mDt;
    bool mbContactState[4];
    bool mbPrevContactState[4];

    Mat4<double> mTransMat[4];

    RigidBodyDynamics::Model* mModel;
    HWD* HWData;

    /// leg kinematics
    bool mbIsFirstRun;
    bool mbIsFirstCon[4];

    int mAlpha;
    const int mAlphaStep;

    Vec3<double> mGlobalBaseVelBuff[4];
    Vec3<double> mGlobalContactFootPos[4];
    Eigen::Vector3d mGroundContactFootPos[4];

    bool mbGateTableState[4];
    bool mbPrevGateTableState[4];
    int mDelay[4];
    int mDelayStep[4];
    double mMotorPosition[MOTOR_NUM];
    double mMotorVelocity[MOTOR_NUM];
    double mMotorDesiredVelocity[MOTOR_NUM];
    double mMotorDesiredTorque[MOTOR_NUM];
    double mStandPeriod;
    double mSwingPeriod;
    double mTimeStep[4];

    Vec3<double> mBodyBase2FootPosition[4];
    Vec3<double> mBodyBaseAngularVelocity;
    Vec3<double> mGlobalBaseEulerAngle;
    Vec3<double> mGlobalBasePosition;
    Vec3<double> mGlobalBaseVelocity;
    Vec3<double> mGlobalFootPosition[4];
    Vec3<double> mGroundPosition;
    Vec3<double> mGroundVelocity;

    Vec4<double> mGlobalBaseQuaternion;

    Eigen::Vector3d mEstimatedGlobalSlopeRPY;
    Eigen::Vector4d mEstimatedGlobalSlopeQuat;
    Eigen::Vector3d mEstimatedLocalSlopeRPY;
    Eigen::Vector4d mEstimatedLocalSlopeQuat;

    double mGlobalYaw;
};

#endif //RAISIM_STATEESTIMATOR_HPP
