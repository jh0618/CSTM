//
// Created by jaehoon on 24. 5. 11.
//

#ifndef CAMEL_CAN_UPBOARD_COMMANDLIST_HPP
#define CAMEL_CAN_UPBOARD_COMMANDLIST_HPP

constexpr int CAN_CMD_READ_ENCODER = 0x90;
constexpr int CAN_CMD_READ_ERROR = 0x9a;
constexpr int CAN_CMD_READ_LOOP_GAIN = 0x30;
constexpr int CAN_CMD_TURN_ON = 0x88;
constexpr int CAN_CMD_TURN_OFF = 0x80;
constexpr int CAN_CMD_SET_LOOP_GAIN = 0x32;
constexpr int CAN_CMD_SET_TORQUE = 0xa1;
constexpr int CAN_CMD_SET_ENCODER_ZERO = 0x19;

enum MOTOR_INDEX
{
    FLHR_IDX = 0,
    FLHP_IDX,
    FLKP_IDX,
    FRHR_IDX,
    FRHP_IDX,
    FRKP_IDX,
    HLHR_IDX,
    HLHP_IDX,
    HLKP_IDX,
    HRHR_IDX,
    HRHP_IDX,
    HRKP_IDX,
};


#endif //CAMEL_CAN_UPBOARD_COMMANDLIST_HPP
