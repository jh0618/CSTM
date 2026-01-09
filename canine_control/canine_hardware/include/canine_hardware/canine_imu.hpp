//
// Created by ys on 24. 8. 24.
//

#ifndef CAMEL_CANINE_CANINE_IMU_HPP
#define CAMEL_CANINE_CANINE_IMU_HPP

#include "ThreadGenerator.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

void* NRTIMUThread(void* arg);

#endif //CAMEL_CANINE_CANINE_IMU_HPP
