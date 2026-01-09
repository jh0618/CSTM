//
// Created by ys on 24. 2. 11.
//

#include "ControlMain.hpp"
#include "SharedMemory.hpp"
#include "ThreadGenerator.hpp"
#include "ControlMain/HighControlMain.hpp"
#include "ControlMain/LowControlMain.hpp"
#include "StateEstimator.hpp"
#include "DivergenceEstimator.hpp"
#include "command/include/Command.hpp"
#include "DataAnalysis.hpp"
#include "communication/ConsoleCommunication.hpp"

extern std::string modelFile;
SharedMemory* sharedMemory = SharedMemory::getInstance();
HWD* HWData = HWD::getInstance();
DataAnalysis Logger;


ControlMain::ControlMain()
{
    mHighControl = new HighControlMain;
    mLowControl = new LowControlMain;
    RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
    RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, true);
    mStateEstimator = new StateEstimator(model);
    mDivergenceEsitmator = new DivergenceEstimator();
    sharedMemory = SharedMemory::getInstance();
    HWData = _HWD_::getInstance();
}

void ControlMain::startThread()
{
    pthread_t Command_Thread;
    pthread_t QtClient_Thread;
    pthread_t QtServer_Thread;
    generate_nrt_thread(Command_Thread, NRT_COMMAND_Thread, "Comm_Thread", 0, NULL);
    generate_nrt_thread(QtClient_Thread, NRT_TCP_Send_Data, "client_Thread", 0, NULL);
    generate_nrt_thread(QtServer_Thread, NRT_UDP_Receive_Data, "server_thread", 0, NULL);
}

void ControlMain::doHighControl()
{
    mHighControl->ControllerFunction();
}

void ControlMain::doLowControl()
{
    mLowControl->ControllerFunction();
}

void ControlMain::doStateEstimation()
{
    mStateEstimator->StateEstimatorFunction();
    mDivergenceEsitmator->DivergenceEstimatorFunction();
}

void ControlMain::doHardControl()
{

    static uint64_t mIteration;
    sharedMemory->localTime = int(mIteration) * LOW_CONTROL_dT;
    mIteration++;
    copyHardData();
    if(sharedMemory->isRobotControl)
    {
        doStateEstimation();
        doLowControl();
        sendMotorTorque();
    }
    if (sharedMemory->FLMotorStatus && sharedMemory->FRMotorStatus && sharedMemory->HLMotorStatus && sharedMemory->HRMotorStatus)
    {
        sharedMemory->motorStatus = true;
    }
    else
    {
        sharedMemory->motorStatus = false;
    }
}

void ControlMain::copyHardData()
{
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        sharedMemory->motorPosition[idx] = HWData->sensor.motor[idx].pos;
        sharedMemory->motorVelocity[idx] = HWData->sensor.motor[idx].vel;
        sharedMemory->motorTemp[idx] = HWData->sensor.motor[idx].temp;
        sharedMemory->motorVoltage[idx] = HWData->sensor.motor[idx].voltage;
        sharedMemory->motorTorque[idx] = HWData->sensor.motor[idx].torque;
        sharedMemory->motorErrorStatus[idx] = HWData->sensor.motor[idx].error_code;
    }
    sharedMemory->globalBaseEulerAngle = HWData->sensor.imu.rpy;
    sharedMemory->globalBaseQuaternion = HWData->sensor.imu.quat;
    sharedMemory->bodyBaseAngularVelocity = HWData->sensor.imu.gyro;
    sharedMemory->globalBaseAcceleration = HWData->sensor.imu.acc;
}

void ControlMain::sendMotorTorque()
{
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        HWData->motorDesiredPos[idx] = sharedMemory->motorDesiredPosition[idx];
        HWData->motorDesiredTorque[idx] = sharedMemory->motorDesiredTorque[idx];
    }
}

void* NRT_COMMAND_Thread(void* arg)
{
    Command command;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    while (!sharedMemory->isRobotRestart)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
        command.commandFunction();
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        sharedMemory->threadElapsedTime[10] = timediff_us(&TIME_NEXT, &TIME_NOW) * 1e-3;
        usleep(1000);
    }
}

void* RT_High_Controller_Thread(void* arg)
{
    ControlMain* controlMain = ControlMain::getInstance();
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_1;
    struct timespec TIME_2;
    const long PERIOD_US = long(HIGH_CONTROL_dT * 1e6);
    std::cout << "[MAIN FSM] Generated High Controller RT Thread : " << 1 / double(PERIOD_US) * 1e6 << " Hz"
        << std::endl;
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while (!sharedMemory->isRobotRestart)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        clock_gettime(CLOCK_REALTIME, &TIME_1);
        controlMain->doHighControl();

        clock_gettime(CLOCK_REALTIME, &TIME_2);
        HWData->threadElapsedTime[1] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, High Controller RT Thread : "
                << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* RT_Low_Controller_Thread(void* arg)
{
    ControlMain* controlMain = ControlMain::getInstance();
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_1;
    struct timespec TIME_2;
    const long PERIOD_US = long(LOW_CONTROL_dT * 1e6);
    std::cout << "[MAIN FSM] Generated Low Controller RT Thread : " << 1 / double(PERIOD_US) * 1e6 << " Hz"
        << std::endl;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while (!sharedMemory->isRobotRestart)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        clock_gettime(CLOCK_REALTIME, &TIME_1);
        controlMain->doHardControl();
        clock_gettime(CLOCK_REALTIME, &TIME_2);
        HWData->threadElapsedTime[2] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, Low Controller RT Thread : "
                << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

void* RT_Logger_Thread(void* arg)
{
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_1;
    struct timespec TIME_2;
    double logger_dt = 0.02;
    const long PERIOD_US = long(logger_dt * 1e6); // 200Hz 짜리 쓰레드
    std::cout << "[MAIN FSM] Generated LOGGER RT Thread : " << 1 / double(logger_dt) << " Hz" <<std::endl;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함
        clock_gettime(CLOCK_REALTIME, &TIME_1);

        if(sharedMemory->FSMState != FSM_INITIAL)
        {
            Logger.SaveRobotState();
        }

        clock_gettime(CLOCK_REALTIME, &TIME_2);
        sharedMemory->threadElapsedTime[1] = timediff_us(&TIME_1, &TIME_2) * 1e-3;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0)
        {
            std::cout << "[MAIN FSM] Deadline Miss, Logger RT Thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}