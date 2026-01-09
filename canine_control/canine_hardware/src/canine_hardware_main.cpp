//
// Created by ys on 24. 8. 24.
//

#include "canine_hardware/canine_hardware.hpp"

CanMotorFL canFL("can11");
CanMotorFR canFR("can12");
CanMotorHL canHL("can13");
CanMotorHR canHR("can14");
std::string modelFile = std::string(URDF_RSC_DIR) + "canine_blue.urdf";

void* NRTCANFL(void* arg);
void* NRTCANFR(void* arg);
void* NRTCANHL(void* arg);
void* NRTCANHR(void* arg);

int main(int argc, char* argv[])
{
    QCoreApplication a(argc, argv);
    pthread_t uart_Thread;
    pthread_t NRTThreadCANFL;
    pthread_t NRTThreadCANFR;
    pthread_t NRTThreadCANHL;
    pthread_t NRTThreadCANHR;
    pthread_t high_controller_thread;
    pthread_t low_controller_thread;
    pthread_t logger_thread;
    ControlMain::getInstance()->startThread();
    generate_rt_thread_hard(uart_Thread, NRTIMUThread, "uart_Thread", 0, 40, NULL);
    generate_rt_thread(high_controller_thread, RT_High_Controller_Thread, "HCtrl_Thread", 5, 79, NULL);
    generate_rt_thread(low_controller_thread, RT_Low_Controller_Thread, "LCtrl_Thread", 6, 99, NULL);
    generate_rt_thread(logger_thread, RT_Logger_Thread, "Logger_Thread", 1, 20, NULL);
    generate_rt_thread(NRTThreadCANFL, NRTCANFL, "nrt_m_FL", 8, 98, NULL);
    generate_rt_thread(NRTThreadCANFR, NRTCANFR, "nrt_m_FR", 8, 98, NULL);
    generate_rt_thread(NRTThreadCANHL, NRTCANHL, "nrt_m_HL", 9, 98, NULL);
    generate_rt_thread(NRTThreadCANHR, NRTCANHR, "nrt_m_HR", 9, 98, NULL);
    return a.exec();
}

void* NRTCANFL(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-LF NRT Thread : " << "non-sleep loop" <<std::endl;
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    struct timespec TIME_1;
    struct timespec TIME_2;
    while (true)
    {
        clock_gettime(CLOCK_REALTIME,&TIME_1);
        canFL.CanFunction();
        clock_gettime(CLOCK_REALTIME,&TIME_2);
        sharedMemory->threadElapsedTime[3] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
//        std::cout<<"can-fl time: "<<sharedMemory->threadElapsedTime[3]<<" ms"<<std::endl;
    }
}

void* NRTCANFR(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-RF NRT Thread : " << "non-sleep loop" <<std::endl;
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    struct timespec TIME_1;
    struct timespec TIME_2;
    while (true)
    {
        clock_gettime(CLOCK_REALTIME,&TIME_1);
        canFR.CanFunction();
        clock_gettime(CLOCK_REALTIME,&TIME_2);
        sharedMemory->threadElapsedTime[4] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
//        std::cout<<"can-fr time: "<<sharedMemory->threadElapsedTime[4]<<" ms"<<std::endl;
    }
}

void* NRTCANHL(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-LB NRT Thread : " << "non-sleep loop" <<std::endl;
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    struct timespec TIME_1;
    struct timespec TIME_2;
    while (true)
    {
        clock_gettime(CLOCK_REALTIME,&TIME_1);
        canHL.CanFunction();
        clock_gettime(CLOCK_REALTIME,&TIME_2);
        sharedMemory->threadElapsedTime[5] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
//        std::cout<<"can-hl time: "<<sharedMemory->threadElapsedTime[5]<<" ms"<<std::endl;
    }
}

void* NRTCANHR(void* arg)
{
    std::cout << "[MAIN FSM] Generated CAN-RB NRT Thread : " << "non-sleep loop" <<std::endl;
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    struct timespec TIME_1;
    struct timespec TIME_2;
    while (true)
    {
        clock_gettime(CLOCK_REALTIME,&TIME_1);
        canHR.CanFunction();
        clock_gettime(CLOCK_REALTIME,&TIME_2);
        sharedMemory->threadElapsedTime[6] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
//        std::cout<<"can-hr time: "<<sharedMemory->threadElapsedTime[6]<<" ms"<<std::endl;
    }
}