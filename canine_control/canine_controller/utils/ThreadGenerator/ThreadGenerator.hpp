//
// Created by ys on 24. 2. 15.
//

#ifndef RBCANINE_THREADGENERATOR_HPP
#define RBCANINE_THREADGENERATOR_HPP

#include <iostream>
#include <pthread.h>
#include <limits.h>

int generate_nrt_thread(pthread_t& thread_nrt, void* (* thread_func)(void*), const char* name, int cpu_no, void* arg);
int generate_rt_thread(pthread_t& thread_rt, void* (* thread_func)(void*), const char* name, int cpu_no, int priority, void* arg);
int generate_rt_thread_hard(pthread_t& thread_rt, void* (* thread_func)(void*), const char* name, int cpu_no, int priority, void* arg);

void timespec_add_us(struct timespec* t, long us);
int timespec_cmp(struct timespec* a, struct timespec* b);
int timediff_us(struct timespec* before, struct timespec* after);


#endif //RBCANINE_THREADGENERATOR_HPP
