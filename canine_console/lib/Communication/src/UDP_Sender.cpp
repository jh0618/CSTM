//
// Created by ys on 24. 7. 3.
//

#include "Communication/UDP_Sender.hpp"

UDP_Sender::UDP_Sender()
{
    UDP_Open = open();
    sharedMemory = SharedMemory::getInstance();
}
bool UDP_Sender::open(){

    memset(msg, '0', sizeof(msg));

    clientSocket = socket(PF_INET, SOCK_DGRAM, 0);

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(60000);
    serverAddr.sin_addr.s_addr = inet_addr(INIT_IP.c_str());
    memset(serverAddr.sin_zero, '\0', sizeof(serverAddr.sin_zero));
    addr_size = sizeof(serverAddr);
    return 1;
}

//data sender!!
void UDP_Sender::packageUDPmsg(char* msg)
{
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    // header
    msg[0] = 0xFF;
    msg[1] = 0xFE;
    memcpy(&msg[2], &sharedMemory->gamepad.joyCommand, sizeof(int8_t));
    memcpy(&msg[3], sharedMemory->gamepad.userLinVel.data(), sizeof(double[3]));
    memcpy(&msg[27], sharedMemory->gamepad.userAngVel.data(), sizeof(double[3]));
    // tail
    msg[51] = 0x00;
    msg[52] = 0x01;
}

void UDP_Sender::SendData()
{
    while (true)
    {
        packageUDPmsg(msg);
        sendto(clientSocket, msg, sizeof(msg), 0, (struct sockaddr*)&serverAddr, addr_size);
        usleep(100);
    }
}