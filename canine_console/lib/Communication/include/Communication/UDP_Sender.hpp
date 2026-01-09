//
// Created by ys on 24. 7. 3.
//

#ifndef RBQ_UDP_SENDER_HPP
#define RBQ_UDP_SENDER_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "SharedMemory.hpp"

class UDP_Sender{
public:
    UDP_Sender();
    void SendData();
private:
    SharedMemory* sharedMemory;
    int clientSocket;
    struct sockaddr_in serverAddr;
    socklen_t addr_size;

    char msg[53];
    bool open();
    bool UDP_Open;
    void packageUDPmsg(char* msg);
};

#endif //RBQ_UDP_SENDER_HPP
