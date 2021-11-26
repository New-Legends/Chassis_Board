#ifndef MOTOR_H
#define MOTOR_H

#include "can_receive.h"

//m3508电机
class M3508_motor_c
{
    const motor_measure_t *motor_measure;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;

    void init(motor_measure_t *motor_measure_);
} ;







#endif