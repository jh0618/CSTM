//
// Created by hwayoung on 23. 12. 15.
//

#ifndef CAMEL_CANINE_WBCONTROLLER_HPP
#define CAMEL_CANINE_WBCONTROLLER_HPP

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <qpOASES.hpp>


#include "SharedMemory.hpp"
#include "EnumClasses.hpp"
#include "EigenTypes.hpp"
#include <RobotMath.hpp>
#include <Filter.hpp>
#include <ControlUtils/CubicSwingLegLocal.hpp>
#include <ControlUtils/SwingLeg.hpp>

class WBTorqueGenerator
{
public:
    WBTorqueGenerator();
    void DoControl();
    double* GetTorque();

private:
    SharedMemory* sharedMemory;
    void InitializeVariable();
    void setObjWeight();
    void updateState();
    void updateRBDL();
    void calculateWBTorque();
    void setObj();
    void setEquality();
    void setInequality();
    void setQpOasesMat();
    void solveQP();
    void setTorque();
    static void transformMat2Real(qpOASES::real_t* result, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> source);

    RigidBodyDynamics::Model mModel;

    const int mTorqueLimit;
    double mMu;
    int mNumberOfContact;
    bool mGaitTable[4];
    bool mContactLeg[4];
    bool mSwingUpPhase[4];
    Eigen::Array3d mKpLinear;
    Eigen::Array3d mKdLinear;
    Eigen::Array3d mKpOrientation;
    Eigen::Array3d mKdOrientation;

    Vec3<double> mBasePosition;
    Vec3<double> mBaseVelocity;
    Vec4<double> mBaseQuaternion;

    Eigen::Vector3d mBaseEulerAngle;

    Vec3<double> mMotorPosition[4];
    Vec3<double> mMotorVelocity[4];
    Eigen::Vector3d mBodyFootPosition[4];
    Eigen::Vector3d mHipPosition[4];

    Eigen::Vector3d mBodyBaseAngularVelocity;

    Vec3<double> mBaseDesiredPosition;
    Vec3<double> mBaseDesiredVelocity;
    Vec4<double> mBaseDesiredQuaternion;

    Vec3<double> mMotorDesiredPosition[4];
    Vec3<double> mMotorDesiredVelocity[4];

    Vec3<double> mBodyBaseDesiredAngularVelocity;

    Mat3<double> mLegJacobian[4];
    Mat3<double> mLegPrevJacobian[4];
    Mat3<double> mLegJacobianDot[4];

    Vec3<double> mHipFootPosition[4];
    Vec3<double> mHipFootVelocity[4];
    Vec3<double> mHipFootReferencePosition[4];
    Vec3<double> mHipFootReferenceVelocity[4];

    Vec3<double> mHipFootReferenceAcceleration[4];

    Mat3<double> mDesiredFootPGain;
    Mat3<double> mDesiredFootDGain;

    Vec3<double> mHipFootDesiredAcc[4];

    ///QP
    Eigen::Matrix<double, 19, 1> mQ;
    Eigen::Matrix<double, 19, 1> mRef_Q;
    Eigen::Matrix<double, 18, 1> mQDot;
    Eigen::Matrix<double, 18, 1> mRef_QDot;
    Eigen::MatrixXd mObjectA;
    Eigen::VectorXd mObjectB;
    Eigen::MatrixXd mEqualityA;
    Eigen::VectorXd mEqualityB;
    Eigen::VectorXd mCotactJacobianDot;
    Eigen::MatrixXd mContactJacobian;
    Eigen::MatrixXd mMassMatrix;
    Eigen::VectorXd mNonlinearEff;
    Eigen::MatrixXd mInequalityA;
    Eigen::VectorXd mInequalityUbA;
    Eigen::VectorXd mInequalityLbA;
    Eigen::VectorXd mInequalityUb;
    Eigen::VectorXd mInequalityLb;
    Eigen::MatrixXd mObjGain;
    Eigen::MatrixXd mHMatQp;
    Eigen::VectorXd mGMatQp;
    Eigen::VectorXd mOptimalQ;
    Eigen::VectorXd mOptimalF;
    Eigen::VectorXd mOptimalTau;
    Eigen::VectorXd mContactTau;
    Eigen::Matrix<double, 5, 3> mFrictionBlock;

    qpOASES::real_t* mHRealQp{ };
    qpOASES::real_t* mGRealQp{ };
    qpOASES::real_t* mARealQP{ };
    qpOASES::real_t* mLbARealQP{ };
    qpOASES::real_t* mUbARealQP{ };
    qpOASES::real_t* mLbRealQP{ };
    qpOASES::real_t* mUbRealQP{ };
    qpOASES::real_t* mXRealQP{ };

    double mTorque[LEG_MOTOR_NUM];
};

#endif //CAMEL_CANINE_WBCONTROLLER_HPP
