//
// Created by ys on 24. 2. 16.
//

#ifndef RBCANINE_CUSTOM_STRUCT_HPP
#define RBCANINE_CUSTOM_STRUCT_HPP

#include<Eigen/Dense>
#include"JoystickInfo.hpp"

#define MOTOR_NUM 12
#define MAX_COMMAND_DATA 10
#define MAX_CUSTOM_DATA     20

typedef struct _SENSOR_INFO_
{
    typedef struct _MOTOR_INFO_
    {
        double pos;
        double vel;
        double torque;
        char temp;
    } MOTOR_INFO;
    typedef struct _IMU_INFO_
    {
        Eigen::Vector4d quat;
        Eigen::Vector3d rpy;
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;
        double imu_offset_roll;
        double imu_offset_pitch;
    } IMU_INFO;

    MOTOR_INFO motor[MOTOR_NUM];
    IMU_INFO imu;
} SENSOR_INFO;
typedef struct _GAMEPAD_
{
    AXIS joystick;
    BUTTON button;
    GUI_BUTTON gui;
    Eigen::Vector3d userLinVel;
    Eigen::Vector3d userAngVel;
    bool newCommand;
    int8_t joyCommand;
    uint8_t state;
    std::string controllerName;
} GAMEPAD, * pGAMEPAD;

typedef struct _UI_COMMAND_
{
    int userCommand;
    int gaitCommand;
    char userParamChar[MAX_COMMAND_DATA];
    int userParamInt[MAX_COMMAND_DATA];
    double userParamDouble[MAX_COMMAND_DATA];
} UI_COMMAND, * pUI_COMMAND;

typedef struct _CUSTOM_DATA_
{
    double customVariableDouble[MAX_CUSTOM_DATA];
    int customVariableInt[MAX_CUSTOM_DATA];
} CUSTOM_DATA, * pCUSTOM_DATA;

typedef struct _WBParam_
{
    double angGain[3];
    double linGain[3];
    double accSwingGain[3];
    double accStandGain[3];
    double grfGain[12];
    double baseLinKp[3];
    double baseLinKd[3];
    double baseAngKp[3];
    double baseAngKd[3];
    double footTaskKp[3];
    double footTaskKd[3];
} WBParam;

typedef struct _YAMLPARAM_
{

    bool isFirstYamlInputFlag;
    bool isValueChanged;
    bool isYamlSave;

    // Yaml Parameters
    double jointStandKp[3];
    double jointStandKd[3];
    double jointLiftUpKp[3];
    double jointLiftUpKd[3];
    double jointLiftDownKp[3];
    double jointLiftDownKd[3];

    double taskStandKp[3];
    double taskStandKd[3];
    double taskLiftUpKp[3];
    double taskLiftUpKd[3];
    double taskLiftDownKp[3];
    double taskLiftDownKd[3];

    double alpha;
    double mu;
    int fmax;
    int payload;
    double comOffset[3];
    double mpcWeightAngPos[3];
    double mpcWeightLinPos[3];
    double mpcWeightAngVel[3];
    double mpcWeightLinVel[3];

    WBParam wbParam;

} YAMLPARAM, * pYAMLPARAM;
typedef struct _STAIR_KEY_FRAME_
{
    bool bReady;
    bool bControl;
    bool bEnd;
    bool bGenerateTrajectory;
    Eigen::Vector3d bodyBaseDesiredPosition;
    Eigen::Vector3d globalBaseDesiredEulerAngle;
    Eigen::Vector3d bodyBase2FootDesiredPosition[4];
} STAIR_KEY_FRAME, * pSTAIR_KEY_FRAME;

#endif //RBCANINE_CUSTOM_STRUCT_HPP
