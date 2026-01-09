#ifndef TCP_RECEIVER_HPP
#define TCP_RECEIVER_HPP

#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <memory>
#include "SharedMemory.hpp"  // SharedMemory 클래스가 정의된 헤더 파일을 포함합니다.
#include <Eigen/Dense>
#include <cstring>
#include <arpa/inet.h>  // htonl, ntohl 함수가 포함된 헤더

class TCP_Receiver : public std::enable_shared_from_this<TCP_Receiver> {
public:
    TCP_Receiver(boost::asio::io_context& io_context, short port);
    void startAccept();

private:
    void handleAccept(const boost::system::error_code& error);
    void doReadHeader();
    void doReadBody(std::size_t dataSize);
    void unpackingTCPmsg(const std::vector<char>& dataBuffer);
    void logReceiveTime();

    void unpackingEigenVector3d(const char*& dataPtr, Eigen::Vector3d& vec);
    void unpackingEigenVector4d(const char*& dataPtr, Eigen::Vector4d& vec);

    uint64_t htonll(uint64_t value);
    uint64_t ntohll(uint64_t value);
    uint64_t htond(double hostDouble);
    double ntohd(uint64_t net64);

    boost::asio::ip::tcp::socket socket_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::vector<char> data_;
    SharedMemory* sharedMemory;

    std::chrono::steady_clock::time_point last_receive_time_;
};

#endif // TCP_RECEIVER_HPP
