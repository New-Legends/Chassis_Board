#ifndef COMMUNICAT_TASK_H
#define COMMUNICAT_TASK_H

#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"


//任务开始空闲一段时间
#define COMMUNICAT_TASK_INIT_TIME 30

//底盘任务控制间隔 2ms
#define COMMUNICAT_CONTROL_TIME_MS 10

/**
  * @brief          test_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void communicat_task(void *pvParameters);






#endif