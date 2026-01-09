//
// Created by ys on 24. 6. 25.
//

#ifndef RBQ_LEGTRAJECTORYGEN_HPP
#define RBQ_LEGTRAJECTORYGEN_HPP

#include "SharedMemory.hpp"
#include "EnumClasses.hpp"
#include "CubicTrajectoryGenerator.hpp"
// #include "ControlUtils/CubicSwingLegLocal.hpp"
// #include "ControlUtils/SwingLeg.hpp"
// #include "ControlUtils/SwingLegLinear.hpp"
#include "ControlUtils/CubicSwingLeg.hpp"
#include "RobotMath.hpp"

class HomeMotion
{
public:
    HomeMotion();
    void InitializeVariable();

    void SetHomeTrajectory();
    void initHomeUpTrajectory();
    void initHomeDownTrajectory();

    Eigen::Vector3d mJointRefPos[4];
    Eigen::Vector3d mJointRefVel[4];

private:
    void switchHomePhase();
    void getState();
    void setJointTrajectory();
    CubicTrajectoryGenerator mCubicTrajectoryGen[LEG_MOTOR_NUM];
    int HomePhase;
    double mRefTime;
    SharedMemory* sharedMemory;
    Eigen::Vector3d mJointPos[4];

    enum HOME_UP_PHASE
    {
        HOME_NO_ACT,
        HOME_STAND_UP_PHASE1,
        HOME_STAND_UP_PHASE2,
        HOME_STAND_UP_PHASE3,
        HOME_STAND_DOWN_PHASE1,
        HOME_STAND_DOWN_PHASE2,
        HOME_STAND_DOWN_PHASE3,
    };
};

class LegTrajectoryGen : public HomeMotion
{
public:
    LegTrajectoryGen();
    void SetLegMotion();

private:
    void getState();
    void checkGaitTransition();
    void generateLegTrajectory();
    void SwingAlgorithm(const int& leg);
    void LandingAlgorithm(const int& leg);
    void LateLandingAlgorithm(const int& leg);
    void calculateStandJointPDTorque(const int& leg);
    void calculateSwingJointPDTorque(const int& leg);
    void updateState();
    void InitializeVariable();

private:
    SharedMemory* sharedMemory;
//     CubicSwingLegLocal SwingLegTrajectory;
//     SwingLeg SwingLegTrajectory;
//     SwingLegLinear SwingLegTrajectory;
    CubicSwingLeg SwingLegTrajectory;

    int mCount[4];
    int mGaitTable[4];
    bool bIsRunTrot[4];
    bool bIsFirstRunStand[4];
    bool mSwingUpPhase[4];
    bool mContactLeg[4];
    Vec3<double> mBasePosition;
    Vec3<double> mBaseVelocity;
    Vec4<double> mBaseQuaternion;
    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Vec3<double> mBodyFootPosition[4];
    Vec3<double> mSwingFootDesiredPosition[4];
    Vec3<double> mSwingFootDesiredVelocity[4];
    Vec3<double> mHipFootReferencePosition[4];
    Vec3<double> mHipFootReferenceVelocity[4];
    Vec3<double> mHipFootReferenceAcceleration[4];
    Vec3<double> mHipPosition[4];
    Vec3<double> mShoulderPosition[4];
    Vec3<double> mHip2ShoulderPosition[4];
    Mat3<double> mDesiredFootPGain;
    Mat3<double> mDesiredFootDGain;
    Vec3<double> mBaseDesiredVelocity;
    Vec3<double> mBaseDesiredAngularVelocity;
    Vec3<double> mJointDesiredPosition[4];
    Vec3<double> mJointDesiredVelocity[4];
    Vec3<double> mStandJointPos;
    Vec3<double> mStandJointVel;
    Vec3<double> mStandPgain;
    Vec3<double> mStandDgain;
};

#endif //RBQ_LEGTRAJECTORYGEN_HPP
