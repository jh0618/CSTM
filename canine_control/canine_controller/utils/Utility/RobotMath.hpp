//
// Created by hs on 22. 10. 14.
//

#ifndef CANINE_ROBOTMATH_HPP
#define CANINE_ROBOTMATH_HPP

#include <math.h>
#include <iostream>

#include "EigenTypes.hpp"
#include "EnumClasses.hpp"
#include "GlobalParameters.hpp"

namespace RobotMath{

    Mat3<double> GetBaseRotationMat(const Vec4<double>& quat);
    Eigen::Matrix3d GetSlopeRotationMat(const Eigen::Vector3d& rpy);
    Mat3<double> getRmatrix(const Vec3<double>& w, const double& rTheta);
    Mat3<double> GetBaseRotationMatInverse(const Vec4<double>& quat);
    Vec3<double> OrientationError(Eigen::Matrix3d RDesired, Eigen::Matrix3d R);
    Mat4<double> GetGlobal2BodyTransMat(const Vec4<double>& quat, const Vec3<double>& pos);
    Mat4<double> GetBody2GlobalTransMat(const Vec4<double>& quat, const Vec3<double>& pos);
    void TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const double& hip, const double& thi, const double& cal);
    void TransformQuat2Euler(const Vec4<double>& quat, double* euler);
    void GetJacobian(Eigen::Matrix<double, 3, 3>& J, const Eigen::Matrix<double, 3, 1>& pos, int side);
    void GetJacobian2(Eigen::Matrix<double, 3, 3>& J, const Eigen::Matrix<double, 3, 1>& pos, int side);
    Eigen::Matrix<double, 3, 3> getSkewMatrix(Vec3<double> r);
    int8_t NearZero(float a);
    int8_t NearOne(float a);
    Eigen::Matrix<double, 6, 1> getScrew(const Vec3<double>& w, const Vec3<double>& q);
    Mat4<double> getTmatrix(const Eigen::VectorXd& Screw, const double& rTheta);
    inline Eigen::Matrix<double, 6, 6> getAdjMatrix(const Mat4<double>& T);
    inline Mat4<double> getInvTMatrix(const Mat4<double>& T);
    void GetLegInvKinematics(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg);

    Eigen::Vector3d rot2RPY(const Eigen::Matrix3d& mat);
    Eigen::Vector4d rpy2QUAT(const Eigen::Vector3d& rpy);

}

#endif //CANINE_ROBOTMATH_HPP
