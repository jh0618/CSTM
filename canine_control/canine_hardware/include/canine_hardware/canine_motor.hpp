//
// Created by rt on 24. 1. 31.
//

#ifndef CAMEL_CANINE_SPI2CAN_H
#define CAMEL_CANINE_SPI2CAN_H

#include <vector>
#include <QByteArray>
#include <linux/spi/spidev.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <sys/ioctl.h>
#include <termios.h>
#include <Eigen/Dense>
#include <QDebug>
#include "ThreadGenerator.hpp"
#include "SharedMemory.hpp"
#include "GlobalParameters.hpp"

#define MAX_BYTE_PER_MSG    108
#define SPI_SPEED 4000000

typedef struct{
    unsigned char header;
    unsigned char dlc;
    unsigned short id;
    unsigned char data[8];
} ST_CAN;

class SPI2CAN
{
public:
    SPI2CAN();
    void SPI2CANFunction();

private:
    int spi2canInit();
    void spi2canSendRead();
    void spi2canSend1();
    void spi2canSend2();
    void setMessage(int i);
    void spi2canRead();
    void checkConnection();
    void mapReadValue();
    static void setCanMsg(int bno, unsigned char data[8]);
    void readEncoder();
    void readMotorErrorStatus();
    void turnOnMotor();
    void turnOffMotor();
    void setTorque();
    void flushData();

private:
    HWD* HWData;
    SharedMemory* sharedMemory;
    int thread_id;
    unsigned char mProtocol[MOTOR_NUM][8];
    unsigned char mRawData[MOTOR_NUM][8];
    pthread_t thread_handler;
    static void* spi2can_thread(void* arg);

    int spi_1_fd;
    struct spi_ioc_transfer spi1_tr;
    unsigned char tx_1[MAX_BYTE_PER_MSG] = {0,};
    unsigned char rx_1[MAX_BYTE_PER_MSG] = {0,};
    QByteArray recv_buf1;

    int spi_2_fd;
    struct spi_ioc_transfer spi2_tr;
    unsigned char tx_2[MAX_BYTE_PER_MSG] = {0,};
    unsigned char rx_2[MAX_BYTE_PER_MSG] = {0,};
    QByteArray recv_buf2;

    // i12 gpio selection
    int pin37_gpio26_fd;
    int pin38_gpio20_fd;
    int pin26_gpio7_fd;

    static pthread_mutex_t mutex_reference[MOTOR_NUM];
    static ST_CAN  reference_msg[MOTOR_NUM];
    ST_CAN general_send_msgs1[6];
    ST_CAN general_send_msgs2[6];

    int mDisconnectedCount;
    bool mbEmergencyOff;
    double enc2rad;
    double torque2int[MOTOR_NUM];
    int mMotorId[MOTOR_NUM];
    int mEncoder[MOTOR_NUM];
    int mEncoderMultiturnNum[MOTOR_NUM];
    int mEncoderTemp[MOTOR_NUM];
    int mEncoderPast[MOTOR_NUM];
    int mEncoderRaw[MOTOR_NUM];
    int mEncoderOffset[MOTOR_NUM];
    int mGearRatio[MOTOR_NUM];
    int mMotorTemperature[MOTOR_NUM];
    int mMotorErrorCode[MOTOR_NUM];
    double mAxis[MOTOR_NUM];
    double mAngularPositionOffset[MOTOR_NUM];
    double mAngularPosition[MOTOR_NUM];
    double mAngularVelocity[MOTOR_NUM];
    double mCurrentTorque[MOTOR_NUM];
    double mMotorVoltage[MOTOR_NUM];
    int mIdx;
};

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

constexpr int CAN_CMD_READ_ENCODER = 0x90;
constexpr int CAN_CMD_READ_ERROR = 0x9a;
constexpr int CAN_CMD_TURN_ON = 0x88;
constexpr int CAN_CMD_TURN_OFF = 0x80;
constexpr int CAN_CMD_SET_TORQUE = 0xa1;


#endif //CAMEL_CANINE_SPI2CAN_H
