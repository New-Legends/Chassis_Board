#ifndef MOTOR_H
#define MOTOR_H

#include "Pid.h"
#include "Can_receive.h"

//m3508电机
class M3508_motor
{
public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;


    fp32 accel;
    fp32 speed;
    fp32 speed_set;

    fp32 set_current;
    int16_t give_current;

    void init(const motor_measure_t *motor_measure_);
} ;









#endif