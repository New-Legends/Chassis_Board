#ifndef MY_TEST_TASK_H
#define MY_TEST_TASK_H

#include "cmsis_os.h"
#include "main.h"

#include "bsp_led.h"
#include "pid.h"

//chassis motor speed PID
//底盘电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_MODE PID_SPEED
#define MOTIVE_MOTOR_SPEED_PID_KP 6000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.1f
#define MOTIVE_MOTOR_SPEED_PID_KD 2.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 6000.0f





/**
  * @brief          test_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void my_test_task(void *pvParameters);

#endif 