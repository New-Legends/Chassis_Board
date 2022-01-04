#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"

#include "Remote_control.h"
#include "Can_receive.h"

//底盘遥控器是否开启 正常上下板通信是关闭状态的
#define CHASSIS_REMOTE_OPEN 0

class Communicat
{
public:
    void init();

    void run();

};

extern Remote_control remote_control;
extern Can_receive can_receive;

extern Communicat communicat;

#endif