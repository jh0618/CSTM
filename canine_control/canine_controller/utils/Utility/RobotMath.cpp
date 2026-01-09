//
// Created by hs on 22. 10. 14.
//

#include "RobotMath.hpp"

const Mat3<double> I3 = Eigen::Matrix3d::Identity();

Mat3<double> RobotMath::GetBaseRotationMat(const Vec4<double>& quat)
{
    Eigen::Quaterniond q(quat(0), quat(1), quat(2), quat(3));
    Mat3<double> Rot = q.matrix();
    return Rot;
}

Mat3<double> RobotMath::GetBaseRotationMatInverse(const Vec4<double>& quat)
{
    Mat3<double> BaseRot;
    BaseRot = GetBaseRotationMat(quat).transpose();
    return BaseRot;
}

Vec3<double> RobotMath::OrientationError(Eigen::Matrix3d RDesired, Eigen::Matrix3d R)
{
    // Orientation Error
    Eigen::Matrix3d err_R = R.transpose() * RDesired; // error in rotation matrix
//    Eigen::Matrix3d err_R = RDesired*R.transpose(); // error in rotation matrix
    double err_W_theta = (err_R.trace() - 1.0) / 2.0;

    if (err_W_theta >= 1.0)
    {
        err_W_theta = 1.0 - 1e-6;
    }
    else if (err_W_theta <= -1.0)
    {
        err_W_theta = -1.0 + 1e-6;
    }
    err_W_theta = acos(err_W_theta);

    Eigen::Vector3d err_W_vec;
    if (fabs(err_W_theta) < 1e-3)
    {
        err_W_vec = Eigen::Vector3d::Zero(3, 1);
    }
    else
    {
        err_W_vec(0, 0) = 1.0 / 2.0 / sin(err_W_theta) * (err_R(2, 1) - err_R(1, 2));
        err_W_vec(1, 0) = 1.0 / 2.0 / sin(err_W_theta) * (err_R(0, 2) - err_R(2, 0));
        err_W_vec(2, 0) = 1.0 / 2.0 / sin(err_W_theta) * (err_R(1, 0) - err_R(0, 1));
    }
    return err_W_theta * err_W_vec;
}


Mat4<double> RobotMath::GetGlobal2BodyTransMat(const Vec4<double>& quat, const Vec3<double>& pos)
{
    Mat4<double> BaseRot = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(quat(0), quat(1), quat(2), quat(3));
    BaseRot.block<3, 3>(0, 0) = q.matrix();
    BaseRot.block<3, 1>(0, 3) = pos;
    return BaseRot;
}

Mat4<double> RobotMath::GetBody2GlobalTransMat(const Vec4<double>& quat, const Vec3<double>& pos)
{
    Mat4<double> BaseRot;
    BaseRot = GetGlobal2BodyTransMat(quat, pos).inverse();
    return BaseRot;
}

Eigen::Matrix<double, 6, 1> RobotMath::getScrew(const Vec3<double>& w, const Vec3<double>& q)
{
    Eigen::Matrix<double, 6, 1> Screw;
    Mat3<double> w_skew;
    w_skew = getSkewMatrix(w);
    Vec3<double> v;
    v = -w_skew * q;
    Screw.block<3, 1>(0, 0) = w;

    Screw.block<3, 1>(3, 0) = v;
    return Screw;
}

Mat3<double> RobotMath::getRmatrix(const Vec3<double>& w, const double& rTheta)
{
    Mat3<double> w_skew;
    Mat3<double> rotationMatrix;
    w_skew = getSkewMatrix(w);
    rotationMatrix = I3 + sin(rTheta) * w_skew + (1 - cos(rTheta)) * w_skew * w_skew;
    return rotationMatrix;
}

Mat4<double> RobotMath::getTmatrix(const Eigen::VectorXd& Screw, const double& rTheta)
{
    Vec3<double> w = Screw.block<3, 1>(0, 0);
    Mat3<double> R = getRmatrix(w, rTheta);

    Vec3<double> p;
    Mat3<double> w_skew = getSkewMatrix(w);
    Vec3<double> v = Screw.block<3, 1>(3, 0);
    p = (I3 * rTheta + (1 - cos(rTheta)) * w_skew + (rTheta - sin(rTheta)) * w_skew * w_skew) * v;

    Mat4<double> T = Eigen::MatrixXd::Identity(4, 4);
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;
    return T;
}

static int L_or_R(int lnum)
{
    return (-2 * (lnum % 2) + 1); // Left = +1, Right = -1
}

static int F_or_H(int lnum)
{
    return (2 * (lnum < 2) - 1); // Front = +1    Hind = -1
}

void RobotMath::TransMatBody2Foot(Mat4<double>* Base2Foot, LEG_INDEX legIndex, const double& hip, const double& thi, const double& cal)
{
    Mat4<double> T1;
    Mat4<double> T2;
    Mat4<double> T3;

    Mat4<double> InitMat = Eigen::Matrix4d::Identity();
    Eigen::Vector3d w_x = { 1, 0, 0 };
    Eigen::Vector3d w_y = { 0, 1, 0 };

    Eigen::Vector3d base2leg = { F_or_H(legIndex) * BASE2HIP_ROLL_POSITION_X, L_or_R(legIndex) * BASE2HIP_ROLL_POSITION_Y, 0 };
    Eigen::Vector3d l1 = { F_or_H(legIndex) * HIP_ROLL2HIP_PITCH_POSITION_X, L_or_R(legIndex) * HIP_CENTER2PELVIS_POSITION_Y, 0 };
    Eigen::Vector3d l2 = { 0, 0, -THIGH_LENGTH_Z };
    Eigen::Vector3d l3 = { 0, 0, -CALF_LENGTH_Z };
    Eigen::Vector3d footThickness = { 0, 0, FOOT2GROUND_Z };

    InitMat.block<3, 1>(0, 3) = base2leg + l1 + l2 + l3 + footThickness;

    T1 = getTmatrix(getScrew(w_x, base2leg), hip);
    T2 = getTmatrix(getScrew(w_y, base2leg + l1), thi);
    T3 = getTmatrix(getScrew(w_y, base2leg + l1 + l2), cal);

    *Base2Foot = T1 * T2 * T3 * InitMat;
}

template<class T>
T t_min(T a, T b)
{
    if (a < b)
    {
        return a;
    }
    return b;
}

template<class T>
T sq(T a)
{
    return a * a;
}

void RobotMath::TransformQuat2Euler(const Vec4<double>& quat, double* euler)
{
    //edge case!
    float as = t_min(-2. * (quat[1] * quat[3] - quat[0] * quat[2]), .99999);
    euler[0] = atan2(2.f * (quat[2] * quat[3] + quat[0] * quat[1]), sq(quat[0]) - sq(quat[1]) - sq(quat[2]) + sq(quat[3]));
    euler[1] = asin(as);
    euler[2] = atan2(2.f * (quat[1] * quat[2] + quat[0] * quat[3]), sq(quat[0]) + sq(quat[1]) - sq(quat[2]) - sq(quat[3]));
}

void RobotMath::GetJacobian(Eigen::Matrix<double, 3, 3>& J, const Eigen::Matrix<double, 3, 1>& pos, int leg)
{
    double s1 = std::sin(pos[0]);
    double s2 = std::sin(pos[1]);

    double c1 = std::cos(pos[0]);
    double c2 = std::cos(pos[1]);

    double s32 = std::sin(pos[1] + pos[2]);
    double c32 = std::cos(pos[1] + pos[2]);

    if (leg == FR_IDX || leg == HR_IDX)
    {
        J << 0,
            THIGH_LENGTH_Z * c2 + CALF_LENGTH_Z * c32,
            CALF_LENGTH_Z * c32,

            (-1) * HIP_CENTER2PELVIS_POSITION_Y * s1 - THIGH_LENGTH_Z * c1 * c2 - CALF_LENGTH_Z * c1 * c32,
            THIGH_LENGTH_Z * s1 * s2 + CALF_LENGTH_Z * s1 * s32,
            CALF_LENGTH_Z * s1 * s32,

            HIP_CENTER2PELVIS_POSITION_Y * c1 - THIGH_LENGTH_Z * s1 * c2 - CALF_LENGTH_Z * s1 * c32,
            -THIGH_LENGTH_Z * c1 * s2 - CALF_LENGTH_Z * c1 * s32,
            -CALF_LENGTH_Z * c1 * s32;
    }
    else
    {
        J << 0,
            THIGH_LENGTH_Z * c2 + CALF_LENGTH_Z * c32,
            CALF_LENGTH_Z * c32,

            HIP_CENTER2PELVIS_POSITION_Y * s1 - THIGH_LENGTH_Z * c1 * c2 - CALF_LENGTH_Z * c1 * c32,
            THIGH_LENGTH_Z * s1 * s2 + CALF_LENGTH_Z * s1 * s32,
            CALF_LENGTH_Z * s1 * s32,

            (-1) * HIP_CENTER2PELVIS_POSITION_Y * c1 - THIGH_LENGTH_Z * s1 * c2 - CALF_LENGTH_Z * s1 * c32,
            -THIGH_LENGTH_Z * c1 * s2 - CALF_LENGTH_Z * c1 * s32,
            -CALF_LENGTH_Z * c1 * s32;
    }
}

inline Eigen::Matrix<double, 6, 6> RobotMath::getAdjMatrix(const Mat4<double>& T)
{
    Mat3<double> R = T.block<3, 3>(0, 0);
    Vec3<double> p = T.block<3, 1>(0, 3);
    Eigen::Matrix<double, 6, 6> Adj;
    Adj.setZero();
    Adj.block<3, 3>(0, 0) = R;
    Adj.block<3, 3>(3, 0) = getSkewMatrix(p) * R;
    Adj.block<3, 3>(3, 3) = R;
    return Adj;
}

inline Mat4<double> RobotMath::getInvTMatrix(const Mat4<double>& T)
{
    Mat3<double> R = T.block<3, 3>(0, 0);
    Mat4<double> Tbs;
    Tbs.block<3, 3>(0, 0) = R.transpose();
    Tbs.block<3, 1>(0, 3) = -R.transpose() * T.block<3, 1>(0, 3);
    Tbs.block<1, 4>(3, 0) << 0, 0, 0, 1;
    return Tbs;
}

void RobotMath::GetJacobian2(Eigen::Matrix<double, 3, 3>& J, const Eigen::Matrix<double, 3, 1>& pos, int leg)
{
    Mat4<double> T1;
    Mat4<double> T2;
    Mat4<double> T3;

    Eigen::Vector3d w_x = { 1, 0, 0 };
    Eigen::Vector3d w_y = { 0, 1, 0 };

    Eigen::Vector3d base2leg = { F_or_H(leg) * BASE2HIP_ROLL_POSITION_X, L_or_R(leg) * BASE2HIP_ROLL_POSITION_Y, 0 };
    Eigen::Vector3d l1 = { F_or_H(leg) * HIP_ROLL2HIP_PITCH_POSITION_X, L_or_R(leg) * HIP_ROLL2HIP_PITCH_POSITION_Y, 0 };
    Eigen::Vector3d l2 = { 0, 0, -THIGH_LENGTH_Z };
    Eigen::Vector3d l3 = { 0, 0, -CALF_LENGTH_Z };
    Eigen::Vector3d footThickness = { 0, 0, FOOT2GROUND_Z };

    Eigen::Matrix<double, 6, 1> S1 = getScrew(w_x, base2leg);
    Eigen::Matrix<double, 6, 1> S2 = getScrew(w_y, base2leg + l1);
    Eigen::Matrix<double, 6, 1> S3 = getScrew(w_y, base2leg + l1 + l2);

    T1 = getTmatrix(S1, pos[0]);
    T2 = getTmatrix(S2, pos[1]);


    Eigen::Matrix<double, 6, 3> Js;

    Js.block<6, 1>(0, 0) = S1;
    Js.block<6, 1>(0, 1) = getAdjMatrix(T1) * S2;
    Js.block<6, 1>(0, 2) = getAdjMatrix(T1 * T2) * S3;

    Eigen::Matrix4d Tsb;
    TransMatBody2Foot(&Tsb, LEG_INDEX(leg), pos[0], pos[1], pos[2]);

    J = -Tsb.block<3, 3>(0, 0) * (getAdjMatrix(getInvTMatrix(Tsb)) * Js).block<3, 3>(3, 0);
}

Mat3<double> RobotMath::getSkewMatrix(Vec3<double> r)
{
    Eigen::Matrix3d cm;
    cm << 0.0, -r(2), r(1),
        r(2), 0.0, -r(0),
        -r(1), r(0), 0.0;
    return cm;
}

int8_t RobotMath::NearZero(float a)
{
    return (a < 0.01 && a > -.01);
}

int8_t RobotMath::NearOne(float a)
{
    return NearZero(a - 1);
}

void RobotMath::GetLegInvKinematics(Vec3<double>& jointPos, Vec3<double> footPos, const int& leg)
{
    double alpha;
    double beta;
    double rLimit = 0.43; // max shoulder2foot length

    double absXYZ = sqrt(footPos[0] * footPos[0] + footPos[1] * footPos[1] + footPos[2] * footPos[2]);
    if (absXYZ > rLimit)
    {
        footPos[0] = footPos[0] * rLimit / absXYZ;
        footPos[1] = footPos[1] * rLimit / absXYZ;
        footPos[2] = footPos[2] * rLimit / absXYZ;
        std::cout << "[ROBOT MATH] ik limit is occurred in leg : " << leg << std::endl;
    }

    if (leg == FR_IDX || leg == HR_IDX)
    {
        alpha = acos(abs(footPos[1]) / sqrt(pow(footPos[1], 2) + pow(footPos[2], 2)));
        beta = acos(HIP_CENTER2PELVIS_POSITION_Y / sqrt(pow(footPos[1], 2) + pow(footPos[2], 2)));
        if (footPos[1] >= 0)
        {
            jointPos[0] = PI - beta - alpha;
        }
        else
        {
            jointPos[0] = alpha - beta;
        }
    }
    else
    {
        alpha = acos(abs(footPos[1]) / sqrt(pow(footPos[1], 2) + pow(footPos[2], 2)));
        beta = acos(HIP_CENTER2PELVIS_POSITION_Y / sqrt(pow(footPos[1], 2) + pow(footPos[2], 2)));
        if (footPos[1] >= 0)
        {
            jointPos[0] = beta - alpha;
        }
        else
        {
            jointPos[0] = alpha + beta - PI;
        }
    }

    double zdot = -sqrt(pow(footPos[1], 2) + pow(footPos[2], 2) - pow(HIP_CENTER2PELVIS_POSITION_Y, 2));
    double d = sqrt(pow(footPos[0], 2) + pow(zdot, 2));
    double phi = acos(abs(footPos[0]) / d);
    double psi = acos(pow(d, 2) / (2 * THIGH_LENGTH_Z * d));

    if (footPos[0] < 0)
    {
        jointPos[1] = PI / 2 - phi + psi;
    }
    else if (footPos[0] == 0)
    {
        jointPos[1] = psi;
    }
    else
    {
        jointPos[1] = phi + psi - PI / 2;
    }
    jointPos[2] = -acos((pow(d, 2) - 2 * pow(CALF_LENGTH_Z, 2)) / (2 * CALF_LENGTH_Z * CALF_LENGTH_Z));
}

Eigen::Vector3d RobotMath::rot2RPY(const Eigen::Matrix3d& mat)
{
    Eigen::Vector3d rpy;

    if (mat(0, 0) == 0 && mat(1, 0) == 0)
    {
        rpy(0) = std::atan2(mat(0, 1), mat(1, 1));
        rpy(1) = PI / 2;
        rpy(2) = 0.0;
        std::cout << "rot2RPY Warning!! pitch is PI/2" << std::endl;
    }
    else
    {
        rpy(0) = std::atan2(mat(2, 1), mat(2, 2));
        rpy(1) = std::atan2(-mat(2, 0), std::sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0)));
        rpy(2) = std::atan2(mat(1, 0), mat(0, 0));
    }
    return rpy;
}

Eigen::Vector4d RobotMath::rpy2QUAT(const Eigen::Vector3d& rpy)
{
    Eigen::Vector4d QUAT;

    double cr = cos(rpy.x() / 2), cp = cos(rpy.y() / 2), cy = cos(rpy.z() / 2);
    double sr = sin(rpy.x() / 2), sp = sin(rpy.y() / 2), sy = sin(rpy.z() / 2);
    QUAT(0) = cr * cp * cy + sr * sp * sy;
    QUAT(1) = sr * cp * cy - cr * sp * sy;
    QUAT(2) = cr * sp * cy + sr * cp * sy;
    QUAT(3) = cr * cp * sy - sr * sp * cy;

    return QUAT;
}
Eigen::Matrix3d RobotMath::GetSlopeRotationMat(const Eigen::Vector3d& rpy){
    Eigen::Matrix3d rot;
    double rc = cos(rpy[0]);
    double rs = sin(rpy[0]);
    double pc2 = cos(rpy[1]);
    double ps = sin(rpy[1]);
    rot << pc2, ps * rs, ps * rc,
        0.0, rc, -rs,
        -ps, pc2 * rs, pc2 * rc;

    return rot;
}