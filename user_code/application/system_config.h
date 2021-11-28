//
// Created by WSJ on 2021/11/2.
//

#ifndef CLASSIS_BOARD_SYSTEM_CONFIG_H
#define CLASSIS_BOARD_SYSTEM_CONFIG_H


#ifdef  __cplusplus
extern "C" {
#endif

//底盘电机无电流输出
#define CHASSIS_NO_CURRENT 0



extern void Task_start(void);
extern void System_Resource_Init(void);

#ifdef  __cplusplus
}
#endif

#endif //CLASSIS_BOARD_SYSTEM_CONFIG_H
