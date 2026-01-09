//
// Created by ys on 24. 2. 15.
//
#include"threadGenerator.hpp"

int generateNrtThread(pthread_t& thread_nrt, void* (* thread_func)(void*), const char* name, int cpuNum, void* arg)
{
    pthread_attr_t attr;
    cpu_set_t cpuSet;
    int returnValue;

    returnValue = pthread_attr_init(&attr);
    if (returnValue)
    {
        std::cout << "[Thread Generator] init pthread attributes failed\n";
        return false;
    }

    if (cpuNum >= 0)
    {
        CPU_ZERO(&cpuSet);
        CPU_SET(cpuNum, &cpuSet);
        returnValue = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuSet);
        if (returnValue)
        {
            std::cout << "[Thread Generator] pthread cpu allocation failed\n";
            return false;
        }
    }

    returnValue = pthread_create(&thread_nrt, &attr, thread_func, arg);
    if (returnValue)
    {
        std::cout << "[Thread Generator] create pthread failed\n";
        return false;
    }

    returnValue = pthread_setname_np(thread_nrt, name);
    if (returnValue)
    {
        std::cout << "[Thread Generator] thread naming failed\n";
        return false;
    }
    return true;
}

int generateRtThread(pthread_t& thread_rt, void* (* thread_func)(void*), const char* name, int cpuNum, int priority,
                     void* arg)
{
    pthread_attr_t attr;
    cpu_set_t cpuSet;
    int returnValue;

    returnValue = pthread_attr_init(&attr);
    if (returnValue)
    {
        std::cout << "[Thread Generator] init pthread attributes failed\n";
        return false;
    }

    if (cpuNum >= 0)
    {
        CPU_ZERO(&cpuSet);
        CPU_SET(cpuNum, &cpuSet);
        returnValue = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuSet);
        if (returnValue)
        {
            std::cout << "[Thread Generator] pthread cpu allocation failed\n";
            return false;
        }
    }

    returnValue = pthread_create(&thread_rt, &attr, thread_func, arg);
    if (returnValue)
    {
        std::cout << "[Thread Generator] create pthread failed\n";
        return false;
    }

    returnValue = pthread_setname_np(thread_rt, name);
    if (returnValue)
    {
        std::cout << "[Thread Generator] thread naming failed\n";
        return false;
    }
    return true;
}

int timeDifferentMs(struct timespec* before, struct timespec* after)
{
    return (after->tv_sec - before->tv_sec) * 1e6 + (after->tv_nsec - before->tv_nsec) / 1000;
}
