#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void pid::init(uint8_t mode_, const fp32 *pid_parm, fp32 *error, fp32 *set, fp32 *ref, fp32 *error_delta)
{
    mode = mode;
    data.Kp = pid_parm[0];
    data.Ki = pid_parm[1];
    data.Kd = pid_parm[2];
    data.max_iout = pid_parm[4];
    data.max_out = pid_parm[3];

    data.set = set;
    data.ref = ref;
    data.error = *set - *ref;

    if (data.mode == PID_ANGLE)
        data.error_delta = error_delta;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
void pid::pid_calc()
{   
    data.last_error = data.error;
    data.error = *data.set - *data.ref;
    if (mode == PID_SPEED)
        *data.error_delta = data.error - data.last_error;

    
    data.Pout = data.Kp * data.error;
    data.Iout += data.Ki * data.error;
    data.Dout = data.Kd * (*data.error_delta);

    LimitMax(data.Iout, data.max_iout);
    LimitMax(data.out, data.max_out);
}

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void pid::pid_clear()
{
    mode = 0;
    data.Kp = 0;
    data.Ki = 0;
    data.Kd = 0;
    data.max_out = 0;
    data.max_iout = 0;

    data.set = 0;
    data.ref = 0;
    data.error = 0;

    data.error_delta = 0;
}