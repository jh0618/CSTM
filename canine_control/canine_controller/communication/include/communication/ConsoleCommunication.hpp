//
// Created by jaehoon on 23. 7. 19.
//

#ifndef CAMEL_RAISIM_PROJECTS_TCPCOMMUNICATION_HPP
#define CAMEL_RAISIM_PROJECTS_TCPCOMMUNICATION_HPP

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <QTcpServer>
#include <QTcpSocket>
#include <QDataStream>
#include <QApplication>
#include <QtCore>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <SharedMemory.hpp>
#include <ThreadGenerator.hpp>

class ConsoleCommunication
{
public:
    ConsoleCommunication(const ConsoleCommunication&) = delete;
    ConsoleCommunication& operator=(const ConsoleCommunication&) = delete;
    static ConsoleCommunication* getInstance();
private:

    ConsoleCommunication();
public:
    void serializeSharedMemoryInfo(QDataStream& stream);

    void unPackingUDPmsg(char* msg);

    void getIPaddress(const std::string& IP_address);
private:
    _HWD_* HWData;
    SharedMemory* sharedMemory;
};

void* NRT_TCP_Send_Data(void* arg);

void* NRT_UDP_Receive_Data(void* arg);

#endif //CAMEL_RAISIM_PROJECTS_TCPCOMMUNICATION_HPP
