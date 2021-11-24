//
// Created by WSJ on 2021/11/2.
//

#ifndef CLASSIS_BOARD_PID_H
#define CLASSIS_BOARD_PID_H

#ifdef __cplusplus

#include "struct_typedef.h"
enum PID_MODE
{
    PID_SPEED = 0,
    PID_POSITION
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;
    fp32 error;
    fp32 last_error;

    fp32 error_delta; //微分项,速度环下为error之间的差值,速度环下为陀螺仪角速度值

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

} pid_data_t;



class pid {
public:
    uint8_t mode;
    pid_data_t data; 
    
    void pid_init(uint8_t mode, const fp32 *PID, fp32 max_out, fp32 max_iout);
    fp32 speed_angle_calc(fp32 fdb, fp32 set); 
    fp32 angle_angle_calc(fp32 fdb, fp32 set, fp32 error_delta);
};

#endif
#endif //CLASSIS_BOARD_PID_H
