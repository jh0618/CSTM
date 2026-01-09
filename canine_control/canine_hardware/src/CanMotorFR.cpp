#include <canine_hardware/CanMotorFR.hpp>
#include "camel-tools/ThreadGenerator.hpp"

CanMotorFR::CanMotorFR(std::string canName)
    : mCanName(canName)
{
    HWData = HWD::getInstance();
    sharedMemory = SharedMemory::getInstance();
    enc2rad = 2.0 * 3.141592 / 65535;
    mGearRatio = 9;
    torque2int[FRHR_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = 24.0385;
    torque2int[FRHP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = 24.0385;
    torque2int[FRKP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = 24.0385;

    mMotorId[FRHR_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = MOTOR_FRHR_ID;
    mMotorId[FRHP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = MOTOR_FRHP_ID;
    mMotorId[FRKP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = MOTOR_FRKP_ID;

    mAxis[FRHR_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = 1.0;
    mAxis[FRHP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = -1.0;
    mAxis[FRKP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = -1.0;

    mAngularPositionOffset[FRHR_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = FRHR_POS_OFFSET;
    mAngularPositionOffset[FRHP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = FRHP_POS_OFFSET;
    mAngularPositionOffset[FRKP_IDX - FR_IDX * MOTOR_NUM_LEG_PER_CAN] = FRKP_POS_OFFSET;

    for (int index = 0; index < MOTOR_NUM_LEG_PER_CAN; index++)
    {
        mEncoder[index] = 0;
        mEncoderMultiturnNum[index] = 0;
        mEncoderTemp[index] = 35000;
        mEncoderPast[index] = 35000;
        mEncoderRaw[index] = 0;
        mEncoderOffset[index] = 0;
        mMotorTemperature[index] = 0;
        mMotorErrorCode[index] = 0;
        mAngularPosition[index] = 0;
        mAngularVelocity[index] = 0;
        mCurrentTorque[index] = 0;
        mMotorVoltage[index] = 0;
    }

    mMotorIdOffset = 0x141; // should be fixed to 0x141
    mMotorIdxOffset = 3;
    mDisconnectedCount = 0;
    TXfailed = false;
}

void CanMotorFR::CanFunction()
{
    switch (sharedMemory->canState[FR_IDX])
    {
    case CAN_NO_ACT:
    {
        usleep(10);
        break;
    }
    case CAN_MOTOR_ON:
    {
        for (int index = 0; index < MOTOR_NUM_LEG_PER_CAN; index++)
        {
            mEncoderTemp[index] = 35000;
            mEncoderPast[index] = 35000;
        }
        turnOnMotor();
        sharedMemory->canState[FR_IDX] = CAN_READ_ERROR;
        break;
    }
    case CAN_INIT:
    {
        canInit();
        sharedMemory->canState[FR_IDX] = CAN_MOTOR_ON;
        break;
    }
    case CAN_MOTOR_OFF:
    {
        turnOffMotor();
        for (int index = 0; index < MOTOR_NUM_LEG_PER_CAN; index++)
        {
            mEncoderTemp[index] = 35000;
            mEncoderPast[index] = 35000;
        }
        sharedMemory->canState[FR_IDX] = CAN_READ_ENCODER;
        break;
    }
    case CAN_SET_TORQUE:
    {
        setTorque();
        break;
    }
    case CAN_READ_ERROR:
    {
        readMotorErrorStatus();
//        setEncoderZeroPosition(0);// 0: hip roll, 1: hip pitch, 2: knee pitch
        sharedMemory->canState[FR_IDX] = CAN_READ_ENCODER;
        break;
    }
    case CAN_READ_ENCODER:
    {
        readEncoder();
//        readLoopGain();
//        setLoopGain(100, 100, 50, 40, 50, 25);
        break;
    }
    default:
        break;
    }
}

void CanMotorFR::canInit()
{
    std::string command3 =
            "sudo ip link set " + mCanName + " up type can bitrate 1000000";
    const char* c3 = command3.c_str();
    system(c3);
    std::string command2 =
            "sudo ip link set " + mCanName + " txqueuelen 2048";
    const char* c2 = command2.c_str();
    system(c2);
    mSock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mSock == -1)
    {
        perror("Fail to create can socket for ");
        std::cout << mCanName << std::endl;
        return;
    }
    std::cout << "Success to create can socket for " << mCanName << std::endl;

    // TODO: check non-sleep loop for CAN Read, If we Implement this, we have to check when CAN read error.
    int flags = fcntl(mSock, F_GETFL, 0);
    if (flags == -1)
    {
        perror("Fail to get socket flags");
        close(mSock);
        return;
    }

    if (fcntl(mSock, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        perror("Fail to set socket non-blocking");
        close(mSock);
        return;
    }
    // TODO: check non-sleep loop for CAN Read

    struct ifreq ifr;
    const char* canName = mCanName.c_str();
    strcpy(ifr.ifr_name, canName);
    int ret = ioctl(mSock, SIOCGIFINDEX, &ifr);
    if (ret == -1)
    {
        perror("Fail to get can interface index -");
        return;
    }
    std::cout << "Success to get can interface index: " << ifr.ifr_ifindex << std::endl;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(mSock, (struct sockaddr*)&addr, sizeof(addr));
    if (ret == -1)
    {
        perror("Fail to bind can socket -");
        return;
    }
    std::cout << "Success to bind can socket" << std::endl;
}

void CanMotorFR::canSend(int motorIndex, const u_int8_t* data)
{
    if(motorIndex == 0)
    {
        for(int i = 0 ; i < 3 ; i++)
        {
            bConnected[i] = false;
        }
        TXfailed = false;
    }

    u_int32_t tempid = mMotorId[motorIndex] & 0x1fffffff;
    mFrame.can_id = tempid;
    memcpy(mFrame.data, data, sizeof(data));
    mFrame.can_dlc = sizeof(data);
    int tx_bytes = write(mSock, &mFrame, sizeof(mFrame));
    if (tx_bytes == -1)
    {
        perror("Fail to transmit can");
        TXfailed = true;
        return;
    }
}

void CanMotorFR::canRead()
{
    usleep(500);
    read(mSock, &mFrame, sizeof(mFrame));
    int id = mFrame.can_id - mMotorIdOffset; //TODO: should be checked
    for(int i = 0 ; i<8 ; i++)
    {
        mRawData[id][i] = mFrame.data[i];
    }
    bConnected[id] = true;
}

void CanMotorFR::mapReadValue()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM_LEG_PER_CAN; motorIndex++)
    {
        int command = mRawData[motorIndex][0];
        int shmIdx = motorIndex + mMotorIdxOffset;
        switch (command)
        {
            case CAN_CMD_READ_ENCODER:
            {
                mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
                mEncoderTemp[motorIndex] = mRawData[motorIndex][2] + mRawData[motorIndex][3] * 256;
                mEncoderRaw[motorIndex] = mRawData[motorIndex][4] + mRawData[motorIndex][5] * 256;
                mEncoderOffset[motorIndex] = mRawData[motorIndex][6] + mRawData[motorIndex][7] * 256;
                if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000))
                {
                    mEncoderMultiturnNum[motorIndex] += 1;
                }
                else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000))
                {
                    mEncoderMultiturnNum[motorIndex] -= 1;
                }
                mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
                mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;
                HWData->sensor.motor[shmIdx].pos = mAxis[motorIndex] * (mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex]);
                sharedMemory->FRMotorStatus = true;
            }
                break;
            case CAN_CMD_READ_ERROR:
            {
                mMotorTemperature[motorIndex] = mRawData[motorIndex][1];
                mMotorVoltage[motorIndex] = (mRawData[motorIndex][3] + mRawData[motorIndex][4] * 256) * 0.1;
                mMotorErrorCode[motorIndex] = mRawData[motorIndex][7];
                HWData->sensor.motor[shmIdx].temp = mMotorTemperature[motorIndex];
                HWData->sensor.motor[shmIdx].voltage = mMotorVoltage[motorIndex];
                HWData->sensor.motor[shmIdx].error_code = mMotorErrorCode[motorIndex];
            }
                break;
            case CAN_CMD_READ_LOOP_GAIN:
            {
                u_int8_t angleKp = mRawData[motorIndex][2];
                u_int8_t angleKi = mRawData[motorIndex][3];
                u_int8_t speedKp = mRawData[motorIndex][4];
                u_int8_t speedKi = mRawData[motorIndex][5];
                u_int8_t currentKp = mRawData[motorIndex][6];
                u_int8_t currentKi = mRawData[motorIndex][7];
                printf("Motor idx : %d\n", motorIndex);
                printf("angleKp : %d\n", angleKp);
                printf("angleKi : %d\n", angleKi);
                printf("speedKp : %d\n", speedKp);
                printf("speedKi : %d\n", speedKi);
                printf("currentKp : %d\n", currentKp);
                printf("currentKi : %d\n\n", currentKi);
            }
                break;
            case CAN_CMD_TURN_ON:
            {
//                std::cout<<"[CAN-FR] Turned on motor: "<<motorIndex<<std::endl;
            }
                break;
            case CAN_CMD_TURN_OFF:
            {
//                std::cout<<"[CAN-FR] Turned off motor: "<<motorIndex<<std::endl;
            }
                break;
            case CAN_CMD_SET_LOOP_GAIN:
            {
                std::cout<<"[SPI] Set loop gain. Motor idx: "<<motorIndex<<std::endl;
            }
                break;
            case CAN_CMD_SET_TORQUE:
            {
                int16_t currentTorque;
                int16_t angularVelocity;

                mMotorTemperature[motorIndex] = mRawData[motorIndex][1];
                currentTorque = mRawData[motorIndex][2] + mRawData[motorIndex][3] * 256;
                mCurrentTorque[motorIndex] = currentTorque / torque2int[motorIndex];
                angularVelocity = mRawData[motorIndex][4] + mRawData[motorIndex][5] * 256;
                mAngularVelocity[motorIndex] = angularVelocity * D2R / mGearRatio;

                mEncoderPast[motorIndex] = mEncoderTemp[motorIndex];
                mEncoderTemp[motorIndex] = mRawData[motorIndex][6] + mRawData[motorIndex][7] * 256;
                if ((mEncoderTemp[motorIndex] < 10000) && (mEncoderPast[motorIndex] > 50000))
                {
                    mEncoderMultiturnNum[motorIndex] += 1;
                }
                else if ((mEncoderTemp[motorIndex] > 50000) && (mEncoderPast[motorIndex] < 10000))
                {
                    mEncoderMultiturnNum[motorIndex] -= 1;
                }
                mEncoder[motorIndex] = mEncoderTemp[motorIndex] + 65535 * mEncoderMultiturnNum[motorIndex];
                mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio;
                HWData->sensor.motor[shmIdx].temp = mMotorTemperature[motorIndex];
                HWData->sensor.motor[shmIdx].voltage = mMotorVoltage[motorIndex];
                HWData->sensor.motor[shmIdx].torque = mAxis[motorIndex] * mCurrentTorque[motorIndex];
                HWData->sensor.motor[shmIdx].vel = mAxis[motorIndex] * mAngularVelocity[motorIndex];
                HWData->sensor.motor[shmIdx].pos = mAxis[motorIndex] * (mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex]);
            }
                break;
            case CAN_CMD_SET_ENCODER_ZERO:
            {
                std::cout<<"[SPI] Encoder zero position is set to current position. Motor idx: "<<shmIdx<<std::endl;
            }
                break;
            default:
            {
                std::cout<<"[CAN-FR] Command is not matched. Motor idx: "<<motorIndex<<"\tCommand : "<<command<<std::endl;
                bConnected[motorIndex] = false;
            }
                break;
        }
    }
    checkConnection();
}

void CanMotorFR::checkConnection()
{
    bool RXdisConnected = false;
    for (int id = 0; id < 3; id++)
    {
        if (!bConnected[id])
        {
            RXdisConnected = true;
        }
    }
    if (RXdisConnected || TXfailed)
    {
        mDisconnectedCount++;
    }
    else
    {
        mDisconnectedCount = 0;
    }

    if (mDisconnectedCount == 10)
    {
        sharedMemory->bStateDiverged = true;
        std::cerr<<"[CAN-FR] CAN Communication error."<<std::endl;
    }
}

void CanMotorFR::readEncoder()
{
    u_int8_t data[8] = { 0X90, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };

    canSend(0, data);
    canRead();
    canSend(1, data);
    canRead();
    canSend(2, data);
    canRead();
    mapReadValue();
}

void CanMotorFR::readLoopGain()
{
    u_int8_t data[8] = { 0X30, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };

    canSend(0, data);
    canRead();
    canSend(1, data);
    canRead();
    canSend(2, data);
    canRead();
    mapReadValue();
}

void CanMotorFR::readMotorErrorStatus()
{
    u_int8_t data[8] = { 0X9a, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };

    canSend(0, data);
    canRead();
    canSend(1, data);
    canRead();
    canSend(2, data);
    canRead();
    mapReadValue();
}

void CanMotorFR::turnOffMotor()
{

    u_int8_t data[8] = { 0x80, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };

    canSend(0, data);
    canRead();
    canSend(1, data);
    canRead();
    canSend(2, data);
    canRead();
    mapReadValue();
    sharedMemory->FRMotorStatus = false;
}

void CanMotorFR::stopMotor()
{
    u_int8_t data[8] = { 0x81, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };

    canSend(0, data);
    canRead();
    canSend(1, data);
    canRead();
    canSend(2, data);
    canRead();
    mapReadValue();
    sharedMemory->FRMotorStatus = false;
}

void CanMotorFR::turnOnMotor()
{
    u_int8_t data[8] = { 0x88, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };

    canSend(0, data);
    canRead();
    canSend(1, data);
    canRead();
    canSend(2, data);
    canRead();
    mapReadValue();
}

void CanMotorFR::setTorque()
{
    u_int8_t data[3][8];
    double desiredTorque[MOTOR_NUM_LEG_PER_CAN];

    for (int motorIndex = 0; motorIndex < MOTOR_NUM_LEG_PER_CAN; motorIndex++)
    {
        desiredTorque[motorIndex] = HWData->motorDesiredTorque[motorIndex + mMotorIdxOffset];
//        desiredTorque[motorIndex] = 0.0;
        u_int16_t desiredCurrent = round(mAxis[motorIndex] * torque2int[motorIndex] * desiredTorque[motorIndex]);
        u_int8_t currentLowerData = desiredCurrent % 256;
        u_int8_t currentUpperData = (desiredCurrent / 256) % 256;
        data[motorIndex][0] = 0Xa1;
        data[motorIndex][1] = 0X00;
        data[motorIndex][2] = 0X00;
        data[motorIndex][3] = 0X00;
        data[motorIndex][4] = currentLowerData;
        data[motorIndex][5] = currentUpperData;
        data[motorIndex][6] = 0X00;
        data[motorIndex][7] = 0X00;

    }

    canSend(0, data[0]);
    canRead();
    canSend(1, data[1]);
    canRead();
    canSend(2, data[2]);
    canRead();
    mapReadValue();
}

void CanMotorFR::setLoopGain(int angleKp, int angleKi, int speedKp, int speedKi, int currentKp, int currentKi)
{
    u_int8_t data[3][8];
    u_int8_t angleKp_uint = angleKp;
    u_int8_t angleKi_uint = angleKi;
    u_int8_t speedKp_uint = speedKp;
    u_int8_t speedKi_uint = speedKi;
    u_int8_t currentKp_uint = currentKp;
    u_int8_t currentKi_uint = currentKi;

    for (int motorIndex = 0; motorIndex < MOTOR_NUM_LEG_PER_CAN; motorIndex++)
    {
        data[motorIndex][0] = 0x32;
        data[motorIndex][1] = 0x00;
        data[motorIndex][2] = angleKp_uint;
        data[motorIndex][3] = angleKi_uint;
        data[motorIndex][4] = speedKp_uint;
        data[motorIndex][5] = speedKi_uint;
        data[motorIndex][6] = currentKp_uint;
        data[motorIndex][7] = currentKi_uint;
    }

    canSend(0, data[0]);
    canRead();
    canSend(1, data[1]);
    canRead();
    canSend(2, data[2]);
    canRead();
    mapReadValue();
}

void CanMotorFR::setEncoderZeroPosition(int motorIdx)
{
    u_int8_t data[3][8];

    for (int motorIndex = 0; motorIndex < MOTOR_NUM_LEG_PER_CAN; motorIndex++)
    {
        data[motorIndex][0] = 0x88;
        data[motorIndex][1] = 0x00;
        data[motorIndex][2] = 0x00;
        data[motorIndex][3] = 0x00;
        data[motorIndex][4] = 0x00;
        data[motorIndex][5] = 0x00;
        data[motorIndex][6] = 0x00;
        data[motorIndex][7] = 0x00;
    }

    if(motorIdx < 12)
    {
        data[motorIdx][0] = 0x19;
    }
    else
    {
        perror("[SPI2CAN] motor idx should less than 12.");
    }

    canSend(0, data[0]);
    canRead();
    canSend(1, data[1]);
    canRead();
    canSend(2, data[2]);
    canRead();
    mapReadValue();
}
