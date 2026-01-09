//
// Created by ys on 24. 8. 24.
//

#include "canine_hardware/canine_imu.hpp"
#include "SharedMemory.hpp"


void* NRTIMUThread(void* arg)
{
    HWD* HWData = HWD::getInstance();
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    struct timespec TIME_1;
    struct timespec TIME_2;
    std::cout << "[MAIN FSM] Generated IMU NRT Thread" <<std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(IMU_dT * 1e6);
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    const char *SERIAL_PORT = "/dev/ttyUSB0";
    //const char *SERIAL_PORT = "/dev/ttyS5";
    const speed_t BAUD_RATE = B921600;

    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    int multiTurn = 0;
    int buf_cnt = 0;

    char *rawData;

    double parsedData[9];
    double eulerAngle[3];
    double angularVelocity[3];
    double linearAcceleration[3];
    double baseSingleTurnEulerAngle[3];
    double globalBaseQuaternion[4];
    double globalBaseEulerAngle[3];
    double bodyBaseAngularVelocity[3];
    double tempYaw = 0.0;
    double yawOffset = 0.0;
    int nullingCount = 0;

    bool isFirstRun = true;

    if (serial_fd == -1) {
        std::cerr << "Error: Unable to open serial port of the IMU." << std::endl;
        exit(0);
    }

    struct termios serial_options;
    tcgetattr(serial_fd, &serial_options);

    cfsetospeed(&serial_options, BAUD_RATE);
    cfsetispeed(&serial_options, BAUD_RATE);

    serial_options.c_cflag &= ~PARENB;   // No parity
    serial_options.c_cflag &= ~CSTOPB;   // 1 stop bit
    serial_options.c_cflag &= ~CSIZE;
    serial_options.c_cflag |= CS8;       // 8 data bits
    serial_options.c_cflag &= ~CRTSCTS;  // No hardware flow control
    serial_options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    serial_options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    serial_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical mode
    serial_options.c_oflag &= ~OPOST;    // Raw output

    tcsetattr(serial_fd, TCSANOW, &serial_options);

    double baseEulerAngle[3];
    Eigen::Quaternion<double> quaternion;

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_gettime(CLOCK_REALTIME,&TIME_1);

        char buffer[128];
        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer));

        buf_cnt = 0;
        if (bytes_read > 1) {
            rawData = strtok(buffer, "*,");

            while (rawData != NULL) {
                parsedData[buf_cnt] = atof(rawData);
                rawData = strtok(NULL, ",");
                buf_cnt++;
            }

            for (int idx = 0; idx < 3; idx++) {
                eulerAngle[idx] = parsedData[idx];
                angularVelocity[idx] = parsedData[idx + 3];
                linearAcceleration[idx] = parsedData[idx + 6];
            }

            baseEulerAngle[0] = -eulerAngle[0] * 3.141592 / 180.0;
            baseEulerAngle[1] = eulerAngle[1] * 3.141592 / 180.0;
            baseEulerAngle[2] = -eulerAngle[2] * 3.141592 / 180.0;

            if(baseEulerAngle[1] < -3.141592/2)
            {
                baseEulerAngle[1] = baseEulerAngle[1] + 3.141592;
            }
            else if(baseEulerAngle[1] >= 3.141592/2)
            {
                baseEulerAngle[1] = baseEulerAngle[1] - 3.141592;
            }

            if(nullingCount < 500)
            {
                yawOffset = yawOffset + baseEulerAngle[2]/500;
                nullingCount += 1;
            }
            else
            {
                baseSingleTurnEulerAngle[0] = baseEulerAngle[0];
                baseSingleTurnEulerAngle[1] = baseEulerAngle[1];
                baseSingleTurnEulerAngle[2] = baseEulerAngle[2] - yawOffset;

                double cy = cos(baseSingleTurnEulerAngle[2] * 0.5);
                double sy = sin(baseSingleTurnEulerAngle[2] * 0.5);
                double cp = cos(baseSingleTurnEulerAngle[1] * 0.5);
                double sp = sin(baseSingleTurnEulerAngle[1] * 0.5);
                double cr = cos(baseSingleTurnEulerAngle[0] * 0.5);
                double sr = sin(baseSingleTurnEulerAngle[0] * 0.5);

                if (tempYaw > 178 && eulerAngle[2] < 0) {
                    multiTurn++;
                }
                else if (tempYaw < -178 && eulerAngle[2] > 0) {
                    multiTurn--;
                }

                globalBaseQuaternion[0] = cr * cp * cy + sr * sp * sy;
                globalBaseQuaternion[1] = sr * cp * cy - cr * sp * sy;
                globalBaseQuaternion[2] = cr * sp * cy + sr * cp * sy;
                globalBaseQuaternion[3] = cr * cp * sy - sr * sp * cy;

                globalBaseEulerAngle[0] = baseSingleTurnEulerAngle[0];
                globalBaseEulerAngle[1] = baseSingleTurnEulerAngle[1];
                globalBaseEulerAngle[2] = baseSingleTurnEulerAngle[2] - multiTurn * 2.0 * 3.141592;

                bodyBaseAngularVelocity[0] = -angularVelocity[0] * 3.14 / 180;
                bodyBaseAngularVelocity[1] = -angularVelocity[1] * 3.14 / 180;
                bodyBaseAngularVelocity[2] = angularVelocity[2] * 3.14 / 180;

                std::copy_n(globalBaseQuaternion,4,HWData->sensor.imu.quat.data());
                std::copy_n(globalBaseEulerAngle,3,HWData->sensor.imu.rpy.data());
                std::copy_n(bodyBaseAngularVelocity,3,HWData->sensor.imu.gyro.data());
                HWData->sensor.imu.acc.setZero(); // not use

                tempYaw = eulerAngle[2];
            }
        }

        clock_gettime(CLOCK_REALTIME,&TIME_2);
        sharedMemory->threadElapsedTime[7] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
    }
}