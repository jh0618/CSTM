//
// Created by rt on 24. 1. 31.
//

#include <canine_hardware/canine_motor.hpp>

pthread_mutex_t SPI2CAN::mutex_reference[MOTOR_NUM];
ST_CAN SPI2CAN::reference_msg[MOTOR_NUM];
int loop_time;
bool bConnected[16];
SPI2CAN::SPI2CAN()
{
    HWData = HWD::getInstance();
    sharedMemory = SharedMemory::getInstance();
    mIdx = 0;
    mDisconnectedCount = 0;
    mbEmergencyOff = false;
    enc2rad = 2.0 * 3.141592 / 65535;

    mGearRatio[FLHR_IDX] = 9;
    mGearRatio[FLHP_IDX] = 9;
    mGearRatio[FLKP_IDX] = 9;
    mGearRatio[FRHR_IDX] = 9;
    mGearRatio[FRHP_IDX] = 9;
    mGearRatio[FRKP_IDX] = 9;
    mGearRatio[HLHR_IDX] = 9;
    mGearRatio[HLHP_IDX] = 9;
    mGearRatio[HLKP_IDX] = 9;
    mGearRatio[HRHR_IDX] = 9;
    mGearRatio[HRHP_IDX] = 9;
    mGearRatio[HRKP_IDX] = 9;

    torque2int[FLHR_IDX] = 24.0385;
    torque2int[FLHP_IDX] = 24.0385;
    torque2int[FLKP_IDX] = 24.0385;
    torque2int[FRHR_IDX] = 24.0385;
    torque2int[FRHP_IDX] = 24.0385;
    torque2int[FRKP_IDX] = 24.0385;
    torque2int[HLHR_IDX] = 24.0385;
    torque2int[HLHP_IDX] = 24.0385;
    torque2int[HLKP_IDX] = 24.0385;
    torque2int[HRHR_IDX] = 24.0385;
    torque2int[HRHP_IDX] = 24.0385;
    torque2int[HRKP_IDX] = 24.0385;

    mMotorId[FLHR_IDX] = MOTOR_FLHR_ID;
    mMotorId[FLHP_IDX] = MOTOR_FLHP_ID;
    mMotorId[FLKP_IDX] = MOTOR_FLKP_ID;
    mMotorId[FRHR_IDX] = MOTOR_FRHR_ID;
    mMotorId[FRHP_IDX] = MOTOR_FRHP_ID;
    mMotorId[FRKP_IDX] = MOTOR_FRKP_ID;
    mMotorId[HLHR_IDX] = MOTOR_HLHR_ID;
    mMotorId[HLHP_IDX] = MOTOR_HLHP_ID;
    mMotorId[HLKP_IDX] = MOTOR_HLKP_ID;
    mMotorId[HRHR_IDX] = MOTOR_HRHR_ID;
    mMotorId[HRHP_IDX] = MOTOR_HRHP_ID;
    mMotorId[HRKP_IDX] = MOTOR_HRKP_ID;

    mAxis[FLHR_IDX] = 1.0;
    mAxis[FLHP_IDX] = 1.0;
    mAxis[FLKP_IDX] = 1.0;
    mAxis[FRHR_IDX] = 1.0;
    mAxis[FRHP_IDX] = -1.0;
    mAxis[FRKP_IDX] = -1.0;
    mAxis[HLHR_IDX] = -1.0;
    mAxis[HLHP_IDX] = 1.0;
    mAxis[HLKP_IDX] = 1.0;
    mAxis[HRHR_IDX] = -1.0;
    mAxis[HRHP_IDX] = -1.0;
    mAxis[HRKP_IDX] = -1.0;

    mAngularPositionOffset[FLHR_IDX] = FLHR_POS_OFFSET;
    mAngularPositionOffset[FLHP_IDX] = FLHP_POS_OFFSET;
    mAngularPositionOffset[FLKP_IDX] = FLKP_POS_OFFSET;
    mAngularPositionOffset[FRHR_IDX] = FRHR_POS_OFFSET;
    mAngularPositionOffset[FRHP_IDX] = FRHP_POS_OFFSET;
    mAngularPositionOffset[FRKP_IDX] = FRKP_POS_OFFSET;
    mAngularPositionOffset[HLHR_IDX] = HLHR_POS_OFFSET;
    mAngularPositionOffset[HLHP_IDX] = HLHP_POS_OFFSET;
    mAngularPositionOffset[HLKP_IDX] = HLKP_POS_OFFSET;
    mAngularPositionOffset[HRHR_IDX] = HRHR_POS_OFFSET;
    mAngularPositionOffset[HRHP_IDX] = HRHP_POS_OFFSET;
    mAngularPositionOffset[HRKP_IDX] = HRKP_POS_OFFSET;

    for (int index = 0; index < MOTOR_NUM; index++)
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

    // ---------- rainbow-robotics 240605
    // Pin number=37, UP Pinout=GPIO 26
    pin37_gpio26_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    write(pin37_gpio26_fd, "26", 2);
    close(pin37_gpio26_fd);

    pin37_gpio26_fd = open("/sys/class/gpio/export", O_WRONLY);
    if(pin37_gpio26_fd == -1){
        perror("Unable to open /sys/class/gpio/export");
        printf("Unable to open\n");
        exit(1);
    }
    if(write(pin37_gpio26_fd, "26", 2) != 2){
        perror("Error writing to /sys/class/gpio/export");
        printf("Error writing to\n");
        exit(1);
    }
    close(pin37_gpio26_fd);

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio26/direction
    pin37_gpio26_fd = open("/sys/class/gpio/gpio26/direction", O_WRONLY);
    if(pin37_gpio26_fd == -1){
        perror("Unable to open /sys/class/gpio/gpio26/direction");
        printf("Unable to open\n");
        exit(1);
    }
    if(write(pin37_gpio26_fd, "out", 3) != 3){
        perror("Error writing to /sys/class/gpio/gpio26/direction");
        printf("Error writing to\n");
        exit(1);
    }
    close(pin37_gpio26_fd);
    // pin37_gpio26

    // Pin number=38, UP Pinout=GPIO 20
    pin38_gpio20_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    write(pin38_gpio20_fd, "20", 2);
    close(pin38_gpio20_fd);

    pin38_gpio20_fd = open("/sys/class/gpio/export", O_WRONLY);
    if(pin38_gpio20_fd == -1){
        perror("Unable to open /sys/class/gpio/export");
        printf("Unable to open\n");
        exit(1);
    }
    if(write(pin38_gpio20_fd, "20", 2) != 2){
        perror("Error writing to /sys/class/gpio/export");
        printf("Error writing to\n");
        exit(1);
    }
    close(pin38_gpio20_fd);

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio20/direction
    pin38_gpio20_fd = open("/sys/class/gpio/gpio20/direction", O_WRONLY);
    if(pin38_gpio20_fd == -1){
        perror("Unable to open /sys/class/gpio/gpio20/direction");
        printf("Unable to open\n");
        exit(1);
    }
    if(write(pin38_gpio20_fd, "out", 3) != 3){
        perror("Error writing to /sys/class/gpio/gpio20/direction");
        printf("Error writing to\n");
        exit(1);
    }
    close(pin38_gpio20_fd);


    pin26_gpio7_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    write(pin26_gpio7_fd, "7", 1);
    close(pin26_gpio7_fd);

    pin26_gpio7_fd = open("/sys/class/gpio/export", O_WRONLY);
    if(pin26_gpio7_fd == -1){
        perror("Unable to open /sys/class/gpio/export");
        printf("Unable to open\n");
        exit(1);
    }
    if(write(pin26_gpio7_fd, "7", 1) != 1){
        perror("Error writing to /sys/class/gpio/export");
        printf("Error writing to\n");
        exit(1);
    }
    close(pin26_gpio7_fd);

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio7/direction
    pin26_gpio7_fd = open("/sys/class/gpio/gpio7/direction", O_WRONLY);
    if(pin26_gpio7_fd == -1){
        perror("Unable to open /sys/class/gpio/gpio7/direction");
        printf("Unable to open\n");
        exit(1);
    }
    if(write(pin26_gpio7_fd, "out", 3) != 3){
        perror("Error writing to /sys/class/gpio/gpio7/direction");
        printf("Error writing to\n");
        exit(1);
    }
    close(pin26_gpio7_fd);

    // pin38_gpio20
    // ----------

    spi_1_fd = -1;

    reference_msg[0].header = 0x89;
    reference_msg[1].header = 0x89;
    reference_msg[2].header = 0x89;

    reference_msg[3].header = 0x77;
    reference_msg[4].header = 0x77;
    reference_msg[5].header = 0x77;

    reference_msg[6].header = 0x89;
    reference_msg[7].header = 0x89;
    reference_msg[8].header = 0x89;

    reference_msg[9].header  = 0x77;
    reference_msg[10].header = 0x77;
    reference_msg[11].header = 0x77;

//    thread_id = generate_rt_thread_hard(thread_handler, spi2can_thread, "spi2can", 5, 99, this);
//    thread_id = generate_rt_thread(thread_handler, spi2can_thread, "spi2can", 6, 99, this);

}

void *SPI2CAN::spi2can_thread(void* arg)
{
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    const long PERIOD_US = 2.0 * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_1;
    struct timespec TIME_2;
    clock_gettime(CLOCK_REALTIME,&TIME_NEXT);
    unsigned long dead_miss_cnt = 0;

    SPI2CAN *spi = (SPI2CAN*)arg;
    std::cout << "Generated spi2can thread"<<std::endl;
    while(true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_gettime(CLOCK_REALTIME, &TIME_1);

        spi->SPI2CANFunction();

        clock_gettime(CLOCK_REALTIME, &TIME_2);
        sharedMemory->threadElapsedTime[3] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);

        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[SPI2CAN] RT Deadline Miss : " << timediff_us(&TIME_NEXT,&TIME_NOW)*0.001 <<" ms"<< std::endl;
        }

    }
}

void SPI2CAN::SPI2CANFunction()
{
    mIdx++;
    loop_time++;
    if(mbEmergencyOff)
    {
        std::cout<<"[SPI2CAN] Emergency stop."<<std::endl;
        turnOffMotor();
    }
    else
    {
        switch (sharedMemory->SPI2CANState)
        {
            case CAN_NO_ACT:
            {
                usleep(10);
                break;
            }
            case CAN_MOTOR_ON:
            {
                std::cout << "[SPI2CAN] Motor turn on\n";
                turnOnMotor();
                sharedMemory->SPI2CANStatus = true;
                sharedMemory->SPI2CANState = CAN_READ_ERROR;
                break;
            }
            case CAN_INIT:
            {
                std::cout << "[SPI2CAN] Motor can init\n";
                spi2canInit();
                sharedMemory->SPI2CANState = CAN_MOTOR_ON;
                break;
            }
            case CAN_MOTOR_OFF:
            {
                turnOffMotor();
                sharedMemory->SPI2CANState = CAN_READ_ENCODER;
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
                sharedMemory->isRobotControl = true;
                sharedMemory->SPI2CANState = CAN_READ_ENCODER;
                break;
            }
            case CAN_READ_ENCODER:
            {
                readEncoder();
//                if(loop_time == 7)
//                {
//                    sharedMemory->SPI2CANState = CAN_NO_ACT;
//                }
                break;
            }
            default:
                break;
        }
    }
}

int SPI2CAN::spi2canInit()
{
    unsigned char spi_mode = SPI_MODE_0;
    unsigned char spi_bits_per_word = 8;
    int spi_speed = SPI_SPEED;
    unsigned char lsb = 0x01;

    memset(&spi1_tr, 0, sizeof(struct spi_ioc_transfer));
    memset(&spi2_tr, 0, sizeof(struct spi_ioc_transfer));

    for(int i=0; i<MOTOR_NUM; i++)
    {
        pthread_mutex_init(&mutex_reference[i], NULL);
        reference_msg[i].id = 0x141+i;
        reference_msg[i].dlc = 0x08;
    }

    spi1_tr.tx_buf = (unsigned long)tx_1;
    spi1_tr.rx_buf = (unsigned long)rx_1;
    spi1_tr.len = MAX_BYTE_PER_MSG;
    spi1_tr.speed_hz = SPI_SPEED;
    spi1_tr.delay_usecs = 0;
    spi1_tr.bits_per_word = spi_bits_per_word;
    spi1_tr.cs_change = 1;

    spi2_tr.tx_buf = (unsigned long)tx_2;
    spi2_tr.rx_buf = (unsigned long)rx_2;
    spi2_tr.len = MAX_BYTE_PER_MSG;
    spi2_tr.speed_hz = SPI_SPEED;
    spi2_tr.delay_usecs = 0;
    spi2_tr.bits_per_word = spi_bits_per_word;
    spi2_tr.cs_change = 1;

    int rv = 0;
//    spi_1_fd = open("/dev/spidev0.0", O_RDWR);
    spi_1_fd = open("/dev/spidev1.0", O_RDWR);
    if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 0.0");
    rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");
    rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");
    rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");
    rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");
    rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
    rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
    rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");


    spi_2_fd = open("/dev/spidev1.1", O_RDWR);
    if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 0.0");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");

    // ---------- rainbow-robotics 240605
    pin26_gpio7_fd = open("/sys/class/gpio/gpio7/value", O_WRONLY);
    if(pin26_gpio7_fd == -1){
        perror("Unable to open /sys/class/gpio/gpio7/value");
        printf("Unable to open\n");
        exit(1);
    }
    write(pin26_gpio7_fd, "1", 1);

    pin37_gpio26_fd = open("/sys/class/gpio/gpio26/value", O_WRONLY);
    if(pin37_gpio26_fd == -1){
        perror("Unable to open /sys/class/gpio/gpio26/value");
        printf("Unable to open\n");
        exit(1);
    }
    write(pin37_gpio26_fd, "1", 1);

    pin38_gpio20_fd = open("/sys/class/gpio/gpio20/value", O_WRONLY);
    if(pin38_gpio20_fd == -1){
        perror("Unable to open /sys/class/gpio/gpio20/value");
        printf("Unable to open\n");
        exit(1);
    }
    write(pin38_gpio20_fd, "1", 1);

    flushData();

    return rv;
}

void SPI2CAN::setCanMsg(int bno, unsigned char* data)
{
    for(int i=0; i<8; i++)
    {
        reference_msg[bno].data[i] = data[i];
    }
}

void SPI2CAN::setMessage(int i)
{
    setCanMsg(i, mProtocol[i]);
    pthread_mutex_lock(&mutex_reference[i]);
    memcpy(&(tx_1[i * 12]), &reference_msg[i], 12);
    pthread_mutex_unlock(&mutex_reference[i]);

    setCanMsg(i + 6, mProtocol[i + 6]);
    pthread_mutex_lock(&mutex_reference[i + 6]);
    memcpy(&(tx_2[i * 12]), &reference_msg[i + 6], 12);
    pthread_mutex_unlock(&mutex_reference[i + 6]);
}

void SPI2CAN::spi2canSendRead()
{
    for(int i = 0 ; i < MOTOR_NUM ; i++)
    {
        bConnected[i] = false;
//        std::cout<<loop_time<<" "<<i<<" bConnected:"<<bConnected[i]<<std::endl;
    }
    spi2canSend1();
    spi2canRead();
//    for(int i = 0 ; i < 12 ; i++)
//    {
//        std::cout<<loop_time<<" "<<i<<" bConnected1:"<<bConnected[i]<<std::endl;
//    }
    spi2canSend2();
    spi2canRead();
//    for(int i = 0 ; i < 12 ; i++)
//    {
//        std::cout<<loop_time<<" "<<i<<" bConnected2:"<<bConnected[i]<<std::endl;
//    }
    checkConnection();
}

void SPI2CAN::spi2canSend1()
{
    memset(&(tx_1[0]), 0, MAX_BYTE_PER_MSG);
    memset(&(tx_2[0]), 0, MAX_BYTE_PER_MSG);
    setMessage(0); //0 6
    setMessage(1); //1 7
    setMessage(3); //3 9
    setMessage(4); //4 10
    memset(&(tx_1[12*8]), 0, 12);
    memset(&(tx_2[12*8]), 0, 12);
}

void SPI2CAN::spi2canSend2()
{
    memset(&(tx_1[0]), 0, MAX_BYTE_PER_MSG);
    memset(&(tx_2[0]), 0, MAX_BYTE_PER_MSG);
    setMessage(2); //2 8
    setMessage(5); //5 11
    memset(&(tx_1[12*8]), 0, 12);
    memset(&(tx_2[12*8]), 0, 12);
}

void SPI2CAN::spi2canRead()
{
    usleep(100);

    write(pin37_gpio26_fd, "0", 1);
    {
        ioctl(spi_1_fd, SPI_IOC_MESSAGE(1), &spi1_tr);   // pin37_gpio26_fd 7
        recv_buf1.append((const char*)rx_1, MAX_BYTE_PER_MSG);
    }
    write(pin37_gpio26_fd, "1", 1);

    write(pin38_gpio20_fd, "0", 1);
    {
        ioctl(spi_2_fd, SPI_IOC_MESSAGE(1), &spi2_tr);   // pin38_gpio20_fd 7
        recv_buf2.append((const char*)rx_2, MAX_BYTE_PER_MSG);
    }
    write(pin38_gpio20_fd, "1", 1);

//    qDebug() << loop_time << "BUF1 (Hex):" << recv_buf1.toHex();
//    qDebug() << loop_time << "BUF2 (Hex):" << recv_buf2.toHex();

    while(recv_buf1.size() >= 12)
    {
        if(uchar(recv_buf1[0]) == 0x89)
        {
            int dlc = recv_buf1[1];
            unsigned int id = (ushort)(recv_buf1[2])-0x41;
            unsigned char recv_data1[8];
            if(id<6)
            {
                for(int j=0; j<8; j++)
                {
                    mRawData[id][j] = recv_buf1[4+j];
                }
                bConnected[id] = true;
                recv_buf1.remove(0, 12);
            }
        }
        else
        {
            recv_buf1.remove(0, 1);
        }
    }

    while(recv_buf2.size() >= 12)
    {
        if(uchar(recv_buf2[0]) == 0x89)
        {
            int dlc = recv_buf2[1];
            unsigned int id = (ushort)(recv_buf2[2])-0x41;
            unsigned char recv_data2[8];
            if(id >= 6 && id <= 11 )
            {
                for(int j=0; j<8; j++)
                {
                    mRawData[id][j] = recv_buf2[4+j];
                }
                bConnected[id] = true;
                recv_buf2.remove(0, 12);
            }
        }
        else
        {
            recv_buf2.remove(0, 1);
        }
    }

    mapReadValue();
}

void SPI2CAN::checkConnection()
{
    bool disConnected = false;
    int failedNum = 0;
    for (int id = 0; id < 12; id++)
    {
        if (!bConnected[id])
        {
            std::cout << loop_time << " [SPI2CAN] Motor communication is disconnected. id: " << id << std::endl;
            disConnected = true;
            failedNum++;
            sharedMemory->SPI2CANConnectionFailedID = id;
        }
    }
    if (disConnected)
    {
        mDisconnectedCount++;
    }
    else
    {
        mDisconnectedCount = 0;
    }

    if (mDisconnectedCount == 10)
    {
        mbEmergencyOff = true;
    }

    if(sharedMemory->SPI2CANConnectionFailedNum == 0)
    {
        sharedMemory->SPI2CANConnectionFailedNum = failedNum;
    }
}

void SPI2CAN::flushData()
{
    for (int i = 0 ; i < 2 ; i++)
    {
        static QByteArray leftoverData1;
        static QByteArray leftoverData2;
        int rv1 = ioctl(spi_1_fd, SPI_IOC_MESSAGE(1), &spi1_tr);
        int rv2 = ioctl(spi_2_fd, SPI_IOC_MESSAGE(1), &spi2_tr);

        if (rv1 == -1 || rv2 == -1)
        {
            std::cout << "ioctl error!\n";
            return;
        }

        QByteArray newData1(reinterpret_cast<const char*>(rx_1), MAX_BYTE_PER_MSG);
        QByteArray newData2(reinterpret_cast<const char*>(rx_2), MAX_BYTE_PER_MSG);
        leftoverData1.append(newData1);
        leftoverData2.append(newData2);

        qDebug() << loop_time << "Flush data (spi1):" << leftoverData1.toHex();
        qDebug() << loop_time << "Flush data (spi2):" << leftoverData2.toHex();
        if(leftoverData1.size() >= 1)
        {
            leftoverData1.remove(0, MAX_BYTE_PER_MSG);
            recv_buf1 = leftoverData1;
        }

        if(leftoverData2.size() >= 1)
        {
            leftoverData2.remove(0, MAX_BYTE_PER_MSG);
            recv_buf2 = leftoverData2;
        }
    }
}

void SPI2CAN::mapReadValue()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        int command = mRawData[motorIndex][0];
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
            mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio[motorIndex];
            HWData->sensor.motor[motorIndex].pos = mAxis[motorIndex] * (mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex]);
        }
            break;
        case CAN_CMD_READ_ERROR:
        {
            mMotorTemperature[motorIndex] = mRawData[motorIndex][1];
            mMotorVoltage[motorIndex] = (mRawData[motorIndex][3] + mRawData[motorIndex][4] * 256) * 0.1;
            mMotorErrorCode[motorIndex] = mRawData[motorIndex][7];
            HWData->sensor.motor[motorIndex].temp = mMotorTemperature[motorIndex];
            HWData->sensor.motor[motorIndex].voltage = mMotorVoltage[motorIndex];
            HWData->sensor.motor[motorIndex].error_code = mMotorErrorCode[motorIndex];
        }
            break;
        case CAN_CMD_TURN_ON:
        {
//                std::cout<<"[SPI2CAN] Turned on motor: "<<motorIndex<<std::endl;
        }
            break;
        case CAN_CMD_TURN_OFF:
        {
//                std::cout<<"[SPI2CAN] Turned off motor: "<<motorIndex<<std::endl;
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
            mAngularVelocity[motorIndex] = angularVelocity * D2R / mGearRatio[motorIndex];

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
            mAngularPosition[motorIndex] = mEncoder[motorIndex] * enc2rad / mGearRatio[motorIndex];
            HWData->sensor.motor[motorIndex].temp = mMotorTemperature[motorIndex];
            HWData->sensor.motor[motorIndex].voltage = mMotorVoltage[motorIndex];
            HWData->sensor.motor[motorIndex].torque = mAxis[motorIndex] * mCurrentTorque[motorIndex];
            HWData->sensor.motor[motorIndex].vel = mAxis[motorIndex] * mAngularVelocity[motorIndex];
            HWData->sensor.motor[motorIndex].pos = mAxis[motorIndex] * (mAngularPosition[motorIndex] + mAngularPositionOffset[motorIndex]);
        }
            break;
        default:
        {
            std::cout << "[SPI2CAN] Command is not matched. Motor idx: " << motorIndex << "\tCommand : " << command << std::endl;
        }
            break;
        }
    }
}

void SPI2CAN::readEncoder()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        mProtocol[motorIndex][0] = 0x90;
        mProtocol[motorIndex][1] = 0x00;
        mProtocol[motorIndex][2] = 0x00;
        mProtocol[motorIndex][3] = 0x00;
        mProtocol[motorIndex][4] = 0x00;
        mProtocol[motorIndex][5] = 0x00;
        mProtocol[motorIndex][6] = 0x00;
        mProtocol[motorIndex][7] = 0x00;
    }

    spi2canSendRead();
}

void SPI2CAN::readMotorErrorStatus()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        mProtocol[motorIndex][0] = 0x9a;
        mProtocol[motorIndex][1] = 0x00;
        mProtocol[motorIndex][2] = 0x00;
        mProtocol[motorIndex][3] = 0x00;
        mProtocol[motorIndex][4] = 0x00;
        mProtocol[motorIndex][5] = 0x00;
        mProtocol[motorIndex][6] = 0x00;
        mProtocol[motorIndex][7] = 0x00;
    }

    spi2canSendRead();
}

void SPI2CAN::turnOnMotor()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        mProtocol[motorIndex][0] = 0x88;
        mProtocol[motorIndex][1] = 0x00;
        mProtocol[motorIndex][2] = 0x00;
        mProtocol[motorIndex][3] = 0x00;
        mProtocol[motorIndex][4] = 0x00;
        mProtocol[motorIndex][5] = 0x00;
        mProtocol[motorIndex][6] = 0x00;
        mProtocol[motorIndex][7] = 0x00;
    }

    spi2canSendRead();
    sharedMemory->motorStatus = true;
}

void SPI2CAN::turnOffMotor()
{
    for (int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        mProtocol[motorIndex][0] = 0x80;
        mProtocol[motorIndex][1] = 0x00;
        mProtocol[motorIndex][2] = 0x00;
        mProtocol[motorIndex][3] = 0x00;
        mProtocol[motorIndex][4] = 0x00;
        mProtocol[motorIndex][5] = 0x00;
        mProtocol[motorIndex][6] = 0x00;
        mProtocol[motorIndex][7] = 0x00;
    }

    spi2canSendRead();
    sharedMemory->motorStatus = false;
}

void SPI2CAN::setTorque()
{
    double desiredTorque[MOTOR_NUM];
    std::copy_n(HWData->motorDesiredTorque, MOTOR_NUM, desiredTorque);

    for (int motorIndex = 0; motorIndex < MOTOR_NUM; motorIndex++)
    {
        u_int16_t desiredCurrent = round(mAxis[motorIndex] * torque2int[motorIndex] * desiredTorque[motorIndex]);
//        u_int16_t desiredCurrent = 0.0;
        u_int8_t currentLowerData;
        u_int8_t currentUpperData;

        currentLowerData = desiredCurrent % 256;
        desiredCurrent = desiredCurrent / 256;
        currentUpperData = desiredCurrent % 256;

        mProtocol[motorIndex][0] = 0xa1;
        mProtocol[motorIndex][1] = 0x00;
        mProtocol[motorIndex][2] = 0x00;
        mProtocol[motorIndex][3] = 0x00;

        mProtocol[motorIndex][4] = currentLowerData;
        mProtocol[motorIndex][5] = currentUpperData;
        mProtocol[motorIndex][6] = 0x00;
        mProtocol[motorIndex][7] = 0x00;
    }

    spi2canSendRead();
}
