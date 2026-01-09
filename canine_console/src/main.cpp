//
// Created by ys on 24. 6. 6.
//
#include "threadGenerator.hpp"
#include "console/JoystickCommand.hpp"
#include "Communication/TCP_Receiver.hpp"
#include "Communication/UDP_Sender.hpp"
#include "mainwindow.hpp"

void* sendRobotCommand_udp(void* arg)
{
    UDP_Sender udpSender;
    udpSender.SendData();
}

void* receiveRobotStatus_tcp(void* arg)
{
    try
    {
        boost::asio::io_context io_context;
        std::shared_ptr<TCP_Receiver> server = std::make_shared<TCP_Receiver>(io_context, 60001);
        io_context.run();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

void* ReadJoystick(void* arg)
{
    JoystickCommand JoyCommand;
    JoyCommand.JoystickCommandFunction();
}

void StartCommunication()
{
    pthread_t UDPthread;
    pthread_t TCPthread;
    pthread_t JoystickReader;

    generateNrtThread(JoystickReader, ReadJoystick, "joystick_thread", 4, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 5, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 6, NULL);
}

class CommunicationThread : public QThread
{
public:
    void run() override
    {
        StartCommunication();
    }
};

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    CommunicationThread communicationThread;
    communicationThread.start();

    w.show();
    return a.exec();
}
