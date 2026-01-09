#include "Communication/TCP_Receiver.hpp"

TCP_Receiver::TCP_Receiver(boost::asio::io_context& io_context, short port)
    : socket_(io_context), acceptor_(io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)), data_(1024)  // 초기 버퍼 크기
{
    sharedMemory = SharedMemory::getInstance();
    startAccept();
    last_receive_time_ = std::chrono::steady_clock::now(); // 초기 시간 설정
}

void TCP_Receiver::startAccept()
{
    acceptor_.async_accept(socket_,
        boost::bind(&TCP_Receiver::handleAccept, this, boost::asio::placeholders::error));
}

void TCP_Receiver::handleAccept(const boost::system::error_code& error)
{
    if (!error)
    {
        doReadHeader();
    }
    else
    {
        std::cerr << "[TCP] Accept error: " << error.message() << std::endl;
        startAccept();  // 다음 클라이언트 연결을 대기
    }
}

void TCP_Receiver::doReadHeader()
{
    auto self(shared_from_this());
    boost::asio::async_read(socket_, boost::asio::buffer(data_, sizeof(int64_t)),
        [this, self](boost::system::error_code ec, std::size_t length)
        {
            if (!ec && length == sizeof(int64_t))
            {
                int64_t dataSize = 0;
                std::memcpy(&dataSize, data_.data(), sizeof(int64_t));
                dataSize = ntohll(dataSize);  // 필요한 경우 바이트 오더 변환

                if (dataSize > 0 && dataSize <= 65536)
                {  // 최대 데이터 크기 설정
                    data_.resize(dataSize);
                    doReadBody(dataSize);
                }
                else
                {
                    std::cerr << "[TCP] Invalid data size received: " << dataSize << std::endl;
                    socket_.close();
                    startAccept();
                }
            }
            else
            {
                if (ec == boost::asio::error::eof)
                {
                    std::cerr << "[TCP] Connection closed by client" << std::endl;
                    sharedMemory->isTCPConnected = false;
                }
                else
                {
                    std::cerr << "[TCP] Error reading data size: " << ec.message() << std::endl;
                }
                socket_.close();
                startAccept();
            }
        });
}

void TCP_Receiver::doReadBody(std::size_t dataSize)
{
    auto self(shared_from_this());
    boost::asio::async_read(socket_, boost::asio::buffer(data_),
        [this, self, dataSize](boost::system::error_code ec, std::size_t length)
        {
            if (!ec && length == dataSize)
            {
                try
                {
                    std::vector<char> dataBuffer(data_.begin(), data_.begin() + dataSize);
                    unpackingTCPmsg(dataBuffer);
                    sharedMemory->isTCPConnected = true;
                    logReceiveTime();  // 수신 시간 기록
                    doReadHeader();  // 다시 헤더를 읽음
                }
                catch (const std::exception& e)
                {
                    std::cerr << "[TCP] Exception in unpackingTCPmsg: " << e.what() << std::endl;
                    socket_.close();
                    startAccept();
                }
            }
            else
            {
                if (ec == boost::asio::error::eof)
                {
                    std::cerr << "[TCP] Connection closed by client" << std::endl;
                    sharedMemory->isTCPConnected = false;
                }
                else
                {
                    std::cerr << "[TCP] Error reading data: " << ec.message() << std::endl;
                }
                socket_.close();
                startAccept();  // 새로운 클라이언트 연결 대기
            }
        });
}

void TCP_Receiver::logReceiveTime()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_receive_time_);

    last_receive_time_ = now;
    sharedMemory->TCP_Duration = duration.count();
}

void TCP_Receiver::unpackingTCPmsg(const std::vector<char>& dataBuffer)
{
    const char* dataPtr = dataBuffer.data();

    // Unpack sharedMemory->userCommand
    int userCommandNetworkOrder;
    memcpy(&userCommandNetworkOrder, dataPtr, sizeof(userCommandNetworkOrder));
    sharedMemory->command.userCommand = ntohl(userCommandNetworkOrder);
    dataPtr += sizeof(userCommandNetworkOrder);


    // Unpack sharedMemory->gaitCommand
    int gaitCommandNetworkOrder;
    memcpy(&gaitCommandNetworkOrder, dataPtr, sizeof(gaitCommandNetworkOrder));
    sharedMemory->command.gaitCommand = ntohl(gaitCommandNetworkOrder);
    dataPtr += sizeof(gaitCommandNetworkOrder);

    // Unpack sharedMemory->gaitTable
    std::memcpy(sharedMemory->gaitTable,dataPtr, MPC_HORIZON*4*sizeof(int));
    dataPtr += MPC_HORIZON*4*sizeof(int);

    memcpy(&sharedMemory->motorStatus, dataPtr, sizeof(sharedMemory->motorStatus));
    dataPtr += sizeof(sharedMemory->motorStatus);

    int stateNetworkOrder;
    memcpy(&stateNetworkOrder, dataPtr, sizeof(stateNetworkOrder));
    sharedMemory->FSMState = ntohl(stateNetworkOrder);
    dataPtr += sizeof(stateNetworkOrder);

    uint64_t localTimeNetworkOrder;
    memcpy(&localTimeNetworkOrder, dataPtr, sizeof(localTimeNetworkOrder));
    sharedMemory->localTime = ntohd(localTimeNetworkOrder);
    dataPtr += sizeof(localTimeNetworkOrder);

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        int motorErrorStatusNetworkOrder;
        memcpy(&motorErrorStatusNetworkOrder, dataPtr, sizeof(motorErrorStatusNetworkOrder));
        sharedMemory->motorErrorStatus[i] = ntohl(motorErrorStatusNetworkOrder);
        dataPtr += sizeof(motorErrorStatusNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        int motorTempNetworkOrder;
        memcpy(&motorTempNetworkOrder, dataPtr, sizeof(motorTempNetworkOrder));
        sharedMemory->motorTemp[i] = ntohl(motorTempNetworkOrder);
        dataPtr += sizeof(motorTempNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorVoltageNetworkOrder;
        memcpy(&motorVoltageNetworkOrder, dataPtr, sizeof(motorVoltageNetworkOrder));
        sharedMemory->motorVoltage[i] = ntohd(motorVoltageNetworkOrder);
        dataPtr += sizeof(motorVoltageNetworkOrder);
    }
    const double PI = 3.141592;
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorPositionNetworkOrder;
        memcpy(&motorPositionNetworkOrder, dataPtr, sizeof(motorPositionNetworkOrder));
        sharedMemory->motorPosition[i] = ntohd(motorPositionNetworkOrder);
        dataPtr += sizeof(motorPositionNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorVelocityNetworkOrder;
        memcpy(&motorVelocityNetworkOrder, dataPtr, sizeof(motorVelocityNetworkOrder));
        sharedMemory->motorVelocity[i] = ntohd(motorVelocityNetworkOrder);
        dataPtr += sizeof(motorVelocityNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorTorqueNetworkOrder;
        memcpy(&motorTorqueNetworkOrder, dataPtr, sizeof(motorTorqueNetworkOrder));
        sharedMemory->motorTorque[i] = ntohd(motorTorqueNetworkOrder);
        dataPtr += sizeof(motorTorqueNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredPositionNetworkOrder;
        memcpy(&motorDesiredPositionNetworkOrder, dataPtr, sizeof(motorDesiredPositionNetworkOrder));
        sharedMemory->motorDesiredPosition[i] = ntohd(motorDesiredPositionNetworkOrder);
        dataPtr += sizeof(motorDesiredPositionNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredVelocityNetworkOrder;
        memcpy(&motorDesiredVelocityNetworkOrder, dataPtr, sizeof(motorDesiredVelocityNetworkOrder));
        sharedMemory->motorDesiredVelocity[i] = ntohd(motorDesiredVelocityNetworkOrder);
        dataPtr += sizeof(motorDesiredVelocityNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredTorqueNetworkOrder;
        memcpy(&motorDesiredTorqueNetworkOrder, dataPtr, sizeof(motorDesiredTorqueNetworkOrder));
        sharedMemory->motorDesiredTorque[i] = ntohd(motorDesiredTorqueNetworkOrder);
        dataPtr += sizeof(motorDesiredTorqueNetworkOrder);
    }

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBasePosition);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseVelocity);
    unpackingEigenVector4d(dataPtr, sharedMemory->globalBaseQuaternion);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseEulerAngle);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseAcceleration);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredPosition);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredVelocity);
    unpackingEigenVector4d(dataPtr, sharedMemory->globalBaseDesiredQuaternion);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredEulerAngle);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseDesiredAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseDesiredVelocity);


    for (int i = 0; i < 4; ++i)
    {
        unpackingEigenVector3d(dataPtr, sharedMemory->pdTorque[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->mpcTorque[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->globalFootPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootVelocity[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootDesiredPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootDesiredVelocity[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->solvedGRF[i]);
    }

    for (int i = 0; i < 6; ++i)
    {
        double x, y, z;
        std::memcpy(&x, dataPtr, sizeof(double));
        dataPtr += sizeof(double);
        std::memcpy(&y, dataPtr, sizeof(double));
        dataPtr += sizeof(double);
        std::memcpy(&z, dataPtr, sizeof(double));
        dataPtr += sizeof(double);
        sharedMemory->qddot[i] = Eigen::Vector3d(x, y, z);
    }

    std::memcpy(sharedMemory->contactState, dataPtr, 4 * sizeof(bool));
    dataPtr += 4 * sizeof(bool);

    // HWData->threadElapsedTime를 언패킹
    std::memcpy(sharedMemory->threadElapsedTime, dataPtr, 11 * sizeof(double));
    dataPtr += 11 * sizeof(double);

    // sharedMemory->contactResidualTorque를 언패킹
    std::memcpy(sharedMemory->contactResidualTorque, dataPtr, 4 * sizeof(double));
    dataPtr += 4 * sizeof(double);

    memcpy(&sharedMemory->stumbleRecovery, dataPtr, sizeof(sharedMemory->stumbleRecovery));
    dataPtr += sizeof(sharedMemory->stumbleRecovery);
}


void TCP_Receiver::unpackingEigenVector3d(const char*& dataPtr, Eigen::Vector3d& vec)
{
    for (int i = 0; i < 3; ++i)
    {
        uint64_t networkOrder;
        memcpy(&networkOrder, dataPtr, sizeof(networkOrder));
        vec[i] = ntohd(networkOrder);
        dataPtr += sizeof(networkOrder);
    }
}

void TCP_Receiver::unpackingEigenVector4d(const char*& dataPtr, Eigen::Vector4d& vec)
{
    for (int i = 0; i < 4; ++i)
    {
        uint64_t networkOrder;
        memcpy(&networkOrder, dataPtr, sizeof(networkOrder));
        vec[i] = ntohd(networkOrder);
        dataPtr += sizeof(networkOrder);
    }
}

uint64_t TCP_Receiver::htonll(uint64_t value)
{
    if (__BYTE_ORDER == __LITTLE_ENDIAN)
    {
        return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
    }
    else
    {
        return value;
    }
}

uint64_t TCP_Receiver::ntohll(uint64_t value)
{
    if (__BYTE_ORDER == __LITTLE_ENDIAN)
    {
        return ((uint64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
    }
    else
    {
        return value;
    }
}

uint64_t TCP_Receiver::htond(double hostDouble)
{
    union
    {
        double d;
        uint64_t i;
    } u;
    u.d = hostDouble;
    return htobe64(u.i);
}

double TCP_Receiver::ntohd(uint64_t net64)
{
    union
    {
        uint64_t i;
        double d;
    } u;
    u.i = be64toh(net64);
    return u.d;
}

