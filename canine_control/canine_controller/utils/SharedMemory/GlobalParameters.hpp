//
// Created by ys on 24. 2. 26.
//

#ifndef RBCANINE_GLOBALPARAMETERS_HPP
#define RBCANINE_GLOBALPARAMETERS_HPP

#define RT_MS       2
#define O_RDWR             02
//#define MAX_BYTE_PER_MSG    84
//#define SPI_SPEED 4000000

constexpr int MOTOR_NUM_LEG_PER_CAN = 3;
#define MOTOR_NUM 12
#define MAX_LEG 4

#define CMD_dT              0.002
#define HIGH_CONTROL_dT     0.02
#define LOW_CONTROL_dT      0.002
#define IMU_dT              0.005
#define VISUAL_dT           0.001
#define ESTIMATOR_dT        0.002
#define MAX_COMMAND_DATA    10
#define MAX_CUSTOM_DATA     20
#define PI                  3.141592
#define MPC_HORIZON         5
#define CMD_INIT_CONTROL            0xC3

#define R2D         57.2957802
#define D2R         0.0174533


constexpr int LEG_MOTOR_NUM = 12;

constexpr double GRAVITY = -9.81;
constexpr double BODYMASS = 20.45;
constexpr double BODY_INERTIA[3] = {0.1347, 0.75599, 0.83681}; // Com offset: 0.02 m
constexpr double TORQUE_LIMIT = 30;
constexpr double STAND_UP_TIME = 2.0;
constexpr double STAND_DOWN_TIME = 3.0;

/// This exist for transform matrix
constexpr double BASE2HIP_ROLL_POSITION_X = 0.1935; //BASE2HIP_ROLL_POSITION_X / BASE2HIP_POSITION_X
constexpr double BASE2HIP_ROLL_POSITION_Y = 0.075;
constexpr double HIP_ROLL2HIP_PITCH_POSITION_X = 0.075;
constexpr double HIP_ROLL2HIP_PITCH_POSITION_Y = -0.003;
constexpr double HIP_PITCH2KNEE_PITCH_POSITION_Y = 0.085;
constexpr double HIP_PITCH2KNEE_PITCH_POSITION_Z = -0.23;
constexpr double KNEE_PITCH2FOOT_POSITION_Z = -0.23;
constexpr double FOOT2GROUND_Z = -0.0; //TODO: should be checked

/// This exist for jacobian matrix
constexpr double HIP_CENTER2PELVIS_POSITION_Y = HIP_ROLL2HIP_PITCH_POSITION_Y+HIP_PITCH2KNEE_PITCH_POSITION_Y;
constexpr double THIGH_LENGTH_Z = -HIP_PITCH2KNEE_PITCH_POSITION_Z;
constexpr double CALF_LENGTH_Z = -KNEE_PITCH2FOOT_POSITION_Z;

// TODO have to check
constexpr double HIP_CENTER_POSITION_X = BASE2HIP_ROLL_POSITION_X + HIP_ROLL2HIP_PITCH_POSITION_X;
constexpr double HIP_CENTER_POSITION_Y = BASE2HIP_ROLL_POSITION_Y;
constexpr double SHOULD_X_POS = HIP_CENTER_POSITION_X;
constexpr double SHOULD_Y_POS = BASE2HIP_ROLL_POSITION_Y + HIP_ROLL2HIP_PITCH_POSITION_Y + HIP_PITCH2KNEE_PITCH_POSITION_Y;

constexpr int MOTOR_FLHR_ID = 0x141;
constexpr int MOTOR_FLHP_ID = 0x142;
constexpr int MOTOR_FLKP_ID = 0x143;
constexpr int MOTOR_FRHR_ID = 0x141;
constexpr int MOTOR_FRHP_ID = 0x142;
constexpr int MOTOR_FRKP_ID = 0x143;
constexpr int MOTOR_HLHR_ID = 0x141;
constexpr int MOTOR_HLHP_ID = 0x142;
constexpr int MOTOR_HLKP_ID = 0x143;
constexpr int MOTOR_HRHR_ID = 0x141;
constexpr int MOTOR_HRHP_ID = 0x142;
constexpr int MOTOR_HRKP_ID = 0x143;

constexpr double FLHR_POS_OFFSET = -0.244346;
constexpr double FLHP_POS_OFFSET = 1.91114;
constexpr double FLKP_POS_OFFSET = -3.12065;
constexpr double FRHR_POS_OFFSET = -0.336848;
constexpr double FRHP_POS_OFFSET = -2.70351;
constexpr double FRKP_POS_OFFSET = 2.4679;
constexpr double HLHR_POS_OFFSET = -0.383972;
constexpr double HLHP_POS_OFFSET = 1.90066;
constexpr double HLKP_POS_OFFSET = -3.16428;
constexpr double HRHR_POS_OFFSET = -0.329867;
constexpr double HRHP_POS_OFFSET = -2.52898;
constexpr double HRKP_POS_OFFSET = 2.47662;

#endif //RBCANINE_GLOBALPARAMETERS_HPP
