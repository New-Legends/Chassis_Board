#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote_control.h"
#include "motor.h"

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,  //chassis will follow yaw gimbal motor relative angle.底盘会跟随云台相对角度
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW, //chassis will have yaw angle(chassis_yaw) close-looped control.底盘有底盘角度控制闭环
    CHASSIS_VECTOR_NO_FOLLOW_YAW,      //chassis will have rotation speed control. 底盘有旋转速度控制
    CHASSIS_VECTOR_RAW,                //control-current will be sent to CAN bus derectly.

} chassis_mode_e;










class chassis
{
public:
    const RC_ctrl_t *chassis_RC; //底盘使用的遥控器指针

    chassis_mode_e chassis_mode; // 底盘控制状态机
    chassis_mode_e last_chassis_mode; //底盘上次控制状态机

    m3508_motor_c M3508_motor_c; //底盘电机数据

    void init();
    


};

extern Chassis chassis;



#endif