//
// Created by jaehoon on 23. 7. 19.
//

#include "communication/ConsoleCommunication.hpp"


ConsoleCommunication* console = ConsoleCommunication::getInstance();

ConsoleCommunication* ConsoleCommunication::getInstance()
{
    static ConsoleCommunication instance;
    return &instance;
}

ConsoleCommunication::ConsoleCommunication()
{
    HWData = _HWD_::getInstance();
    sharedMemory = SharedMemory::getInstance();
}

void ConsoleCommunication::unPackingUDPmsg(char* msg)
{
    static int8_t joystickCommand;
    bool wrong;
    double BodyLinVel_ref[3];
    double BodyAngVel_ref[3];
    /// unpacking part
    memcpy(&joystickCommand, &msg[2], sizeof(int8_t));
    memcpy(&BodyLinVel_ref, &msg[3], sizeof(BodyLinVel_ref));
    memcpy(&BodyAngVel_ref, &msg[27], sizeof(BodyAngVel_ref));
    wrong = true;
    if (joystickCommand >= GAMEPAD_NO_INPUT && joystickCommand <= GAMEPAD_RESTART)
    {
        HWData->gamepad.joyCommand = joystickCommand;
    }
    else
    {
        HWData->gamepad.joyCommand = GAMEPAD_NO_INPUT;
        wrong = false;
    }
    for (int idx = 0; idx < 3; idx++)
    {
        if (abs(BodyLinVel_ref[idx]) <= 3)
        {
            HWData->gamepad.userLinVel[idx] = BodyLinVel_ref[idx];
        }
        else
        {
            HWData->gamepad.userLinVel[idx] = 0;
            wrong = false;
        }
        if (abs(BodyAngVel_ref[idx] <= 3))
        {
            HWData->gamepad.userAngVel[idx] = BodyAngVel_ref[idx];
        }
        else
        {
            HWData->gamepad.userAngVel[idx] = 0;
            wrong = false;
        }
    }
    if (wrong)
    {
        HWData->gamepad.newCommand = true;
    }
    else
    {
        HWData->gamepad.newCommand = false;
    }
}

void ConsoleCommunication::serializeSharedMemoryInfo(QDataStream& stream)
{
    stream << sharedMemory->command.userCommand;
    stream << sharedMemory->command.gaitCommand;

    // HWD
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->gaitTable), MPC_HORIZON * 4 * sizeof(int));

    stream << sharedMemory->motorStatus;

    stream << sharedMemory->FSMState;

    stream << sharedMemory->localTime;
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorErrorStatus[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorTemp[i];
    }

    // Serialize double arrays
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorVoltage[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorPosition[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorVelocity[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorTorque[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorDesiredPosition[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorDesiredVelocity[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        stream << sharedMemory->motorDesiredTorque[i];
    }
    stream << sharedMemory->globalBasePosition(0) << sharedMemory->globalBasePosition(1) << sharedMemory->globalBasePosition(2);
    stream << sharedMemory->globalBaseVelocity(0) << sharedMemory->globalBaseVelocity(1) << sharedMemory->globalBaseVelocity(2);
    stream << sharedMemory->bodyBaseVelocity(0) << sharedMemory->bodyBaseVelocity(1) << sharedMemory->bodyBaseVelocity(2);
    stream << sharedMemory->globalBaseQuaternion(0) << sharedMemory->globalBaseQuaternion(1) << sharedMemory->globalBaseQuaternion(2) << sharedMemory->globalBaseQuaternion(3);
    stream << sharedMemory->globalBaseEulerAngle(0) << sharedMemory->globalBaseEulerAngle(1) << sharedMemory->globalBaseEulerAngle(2);
    stream << sharedMemory->bodyBaseAngularVelocity(0) << sharedMemory->bodyBaseAngularVelocity(1) << sharedMemory->bodyBaseAngularVelocity(2);
    stream << sharedMemory->globalBaseAngularVelocity(0) << sharedMemory->globalBaseAngularVelocity(1) << sharedMemory->globalBaseAngularVelocity(2);
    stream << HWData->sensor.imu.acc(0) << HWData->sensor.imu.acc(1) << HWData->sensor.imu.acc(2);

    stream << sharedMemory->globalBaseDesiredPosition(0) << sharedMemory->globalBaseDesiredPosition(1) << sharedMemory->globalBaseDesiredPosition(2);
    stream << sharedMemory->globalBaseDesiredVelocity(0) << sharedMemory->globalBaseDesiredVelocity(1) << sharedMemory->globalBaseDesiredVelocity(2);
    stream << sharedMemory->globalBaseDesiredQuaternion(0) << sharedMemory->globalBaseDesiredQuaternion(1) << sharedMemory->globalBaseDesiredQuaternion(2) << sharedMemory->globalBaseDesiredQuaternion(3);
    stream << sharedMemory->globalBaseDesiredEulerAngle(0) << sharedMemory->globalBaseDesiredEulerAngle(1) << sharedMemory->globalBaseDesiredEulerAngle(2);
    stream << sharedMemory->bodyBaseDesiredAngularVelocity(0) << sharedMemory->bodyBaseDesiredAngularVelocity(1) << sharedMemory->bodyBaseDesiredAngularVelocity(2);
    stream << sharedMemory->globalBaseDesiredAngularVelocity(0) << sharedMemory->globalBaseDesiredAngularVelocity(1) << sharedMemory->globalBaseDesiredAngularVelocity(2);
    stream << sharedMemory->bodyBaseDesiredVelocity(0) << sharedMemory->bodyBaseDesiredVelocity(1) << sharedMemory->bodyBaseDesiredVelocity(2);


    for (int i = 0; i < 4; ++i)
    {
        stream << sharedMemory->pdTorque[i](0) << sharedMemory->pdTorque[i](1) << sharedMemory->pdTorque[i](2);
        stream << sharedMemory->mpcTorque[i](0) << sharedMemory->mpcTorque[i](1) << sharedMemory->mpcTorque[i](2);
        stream << sharedMemory->globalFootPosition[i](0) << sharedMemory->globalFootPosition[i](1) << sharedMemory->globalFootPosition[i](2);
        stream << sharedMemory->bodyBase2FootPosition[i](0) << sharedMemory->bodyBase2FootPosition[i](1) << sharedMemory->bodyBase2FootPosition[i](2);
        stream << sharedMemory->bodyBase2FootVelocity[i](0) << sharedMemory->bodyBase2FootVelocity[i](1) << sharedMemory->bodyBase2FootVelocity[i](2);
        stream << sharedMemory->bodyBase2FootDesiredPosition[i](0) << sharedMemory->bodyBase2FootDesiredPosition[i](1) << sharedMemory->bodyBase2FootDesiredPosition[i](2);
        stream << sharedMemory->bodyBase2FootDesiredVelocity[i](0) << sharedMemory->bodyBase2FootDesiredVelocity[i](1) << sharedMemory->bodyBase2FootDesiredVelocity[i](2);
        stream << sharedMemory->solvedGRF[i](0) << sharedMemory->solvedGRF[i](1) << sharedMemory->solvedGRF[i](2);
    }
    for (int i = 0; i < 6; ++i)
    {
        stream << sharedMemory->qddot[i](0) << sharedMemory->qddot[i](1) << sharedMemory->qddot[i](2);
    }

    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->contactState), 4 * sizeof(bool));
    stream.writeRawData(reinterpret_cast<char*>(HWData->threadElapsedTime), 11 * sizeof(double));
    stream.writeRawData(reinterpret_cast<char*>(sharedMemory->contactResidualTorque), 4 * sizeof(double));

    stream << sharedMemory->stumble.bRecovery;
}

void ConsoleCommunication::getIPaddress(const std::string& IP_address)
{
    sharedMemory->consoleAddress = IP_address;
    sharedMemory->consoleConnection = true;
};

void* NRT_UDP_Receive_Data(void* arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;

    SharedMemory* sharedMemory = SharedMemory::getInstance();

    int serverSocket;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;

    // Open socket
    while ((serverSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        qWarning() << "[COMM] UDP socket failed";
    }
    std::cout << "[COMM] UDP socket on" << std::endl;

    // Config
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(60000);                    // DO NOT CHANGE THIS PORT

    while (bind(serverSocket, (const struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        qWarning() << "[COMM] UDP bind failed";
    }
    std::cout << "[COMM] UDP bind on" << std::endl;


    char buffer[1024];
    socklen_t len = sizeof(clientAddr);

    std::cout << "Waiting for Client...!\n";

    int n = recvfrom(serverSocket, (char*)buffer, 1024, MSG_WAITALL, (struct sockaddr*)&clientAddr, &len);
    buffer[n] = '\0';

    std::cout << "Received message from " << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;
    console->getIPaddress(inet_ntoa(clientAddr.sin_addr));
    sleep(1);


    while (!sharedMemory->isRobotRestart)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

        memset(&clientAddr, 0, sizeof(clientAddr));
        socklen_t addrLen = sizeof(clientAddr);

        char msg[53];
        recvfrom(serverSocket, msg, sizeof(msg), MSG_CONFIRM, (sockaddr*)&clientAddr, &addrLen);
        console->unPackingUDPmsg(msg);
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        sharedMemory->threadElapsedTime[10] = timediff_us(&TIME_NEXT, &TIME_NOW) * 1e-3;
        usleep(100);
    }
    close(serverSocket);
}

void* NRT_TCP_Send_Data(void* arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;

    SharedMemory* sharedMemory = SharedMemory::getInstance();

    int cnt = 0;
    while (!sharedMemory->consoleConnection)
    {
        if (cnt >= 10)
        {
            cnt = 0;
            qWarning() << "[COMM] TCP - Waiting for client to connect...";
            usleep(1000000); // 1초 대기
        }
        cnt++;
    }

    QHostAddress serverAddress(sharedMemory->consoleAddress.c_str());
    const quint16 serverPort = 60001;

    std::unique_ptr<QTcpSocket> socket(new QTcpSocket());

    while (!sharedMemory->isRobotRestart)
    {
        if (socket->state() != QAbstractSocket::ConnectedState)
        {
            socket->abort();
            socket->connectToHost(serverAddress, serverPort);
            if (!socket->waitForConnected(5000))
            {
                qWarning() << "[COMM] TCP - Could not connect to server";
                usleep(1000000); // 1초 대기 후 재시도
                continue;
            }
        }

        clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

        QByteArray byteArray;
        QDataStream stream(&byteArray, QIODevice::WriteOnly);

        // 데이터를 직렬화하는 부분
        console->serializeSharedMemoryInfo(stream);

        // 데이터 크기와 실제 데이터를 전송합니다.
        QByteArray requestData;
        QDataStream out(&requestData, QIODevice::WriteOnly);
        out << (qint64)byteArray.size();  // 데이터 크기 추가
        requestData.append(byteArray);

        qint64 bytesWritten = 0;
        while (bytesWritten < requestData.size())
        {
            qint64 written = socket->write(requestData.mid(bytesWritten));
            if (written == -1)
            {
                qWarning() << "[COMM] TCP - Error during write: " << socket->errorString();
                break;
            }
            bytesWritten += written;
            if (!socket->waitForBytesWritten(5000)) // 5초 대기하여 데이터가 모두 쓰일 때까지 기다립니다.
            {
                qWarning() << "[COMM] TCP - Data sending timeout";
                break;
            }
        }

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        sharedMemory->threadElapsedTime[9] = timediff_us(&TIME_NEXT, &TIME_NOW) * 1e-3;
        usleep(20000); // 20ms 대기
    }

    socket->disconnectFromHost();
    if (socket->state() != QAbstractSocket::UnconnectedState)
    {
        socket->waitForDisconnected(5000); // 5초 대기하여 소켓이 종료될 때까지 기다립니다.
    }

    return nullptr;
}