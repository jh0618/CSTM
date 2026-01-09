//
// Created by ys on 24. 5. 13.
//
#include "ControlMain.hpp"
#include "SharedMemory.hpp"
#include "ThreadGenerator.hpp"
#include "SimulVisualizer/SimulVisualizer.hpp"

std::string modelFile = std::string(URDF_RSC_DIR) + "canine_blue.urdf";
SimulVisualizer simulVisualizer;

void* RT_Simulation_Thread(void* arg);
void* NRT_Simulator_Thread(void* arg);

int main(int argc, char* argv[])
{
    pthread_t simulation_thread;
    pthread_t simulator_thread;
    pthread_t high_controller_thread;
    pthread_t low_controller_thread;
    pthread_t logger_thread;
    ControlMain::getInstance()->startThread();
    generate_rt_thread_hard(simulation_thread, RT_Simulation_Thread, "Simul_Thread", 5, 99, NULL);
    generate_rt_thread(high_controller_thread, RT_High_Controller_Thread, "HCtrl_Thread", 6, 79, NULL);
    generate_rt_thread(low_controller_thread, RT_Low_Controller_Thread, "LCtrl_Thread", 7, 99, NULL);
    generate_rt_thread(logger_thread, RT_Logger_Thread, "Logger_Thread", 6, 20, NULL);
//    generate_nrt_thread(simulator_thread,NRT_Simulator_Thread,"Simulator_thread",8,NULL);
    while(true){
        sleep(10000000);
    }
}


void* RT_Simulation_Thread(void* arg)
{
    HWD* HWData = _HWD_::getInstance();
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_1;
    struct timespec TIME_2;
    const long PERIOD_US = long(VISUAL_dT * 1e6);
    std::cout << "[SIMULATION] Generated SIMULATOR RT Thread : " << 1 / double(PERIOD_US) * 1e6 << " Hz" << std::endl;
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (!sharedMemory->isRobotRestart)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        clock_gettime(CLOCK_REALTIME, &TIME_1);
        simulVisualizer.updateCurrent();
        simulVisualizer.updateVisual();
        clock_gettime(CLOCK_REALTIME, &TIME_2);
        HWData->threadElapsedTime[2] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[SIMULATION] Deadline Miss, SIMULATOR RT Thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* NRT_Simulator_Thread(void* arg){
    simulVisualizer.openSimulation();
}