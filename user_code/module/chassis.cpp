#include "Chassis.h"
#include "Communicate.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "arm_math.h"
#include "Referee.h"
#include "stdlib.h"
#include "time.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif


//底盘模块 对象
Chassis chassis;

Referee referee;

//扭腰控制数据
fp32 swing_angle = 0.0f;
uint8_t swing_switch = 0;
uint8_t key_pressed_num_ctrl = 0;

//小陀螺控制数据
fp32 top_angle = 0;
bool_t top_switch = 0;

//45度角对敌数据
fp32 pisa_angle = 0; //保留45度对敌前的云台相对底盘角度
bool_t pisa_switch = 0;

//未受击打的不规则运动初始
int16_t Irregular_motion[10];
int8_t Irregular_motion_num = 0;
int16_t Irregular_motion_sign[10];
int16_t tim = 1;

/**
  * @brief          初始化变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     
  * @retval         none
  */
void Chassis::init()
{
    //获取遥控器指针
    chassis_RC = remote_control.get_remote_control_point();
    chassis_last_key_v = 0;

    //设置初试状态机
    chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    last_chassis_behaviour_mode = chassis_behaviour_mode;

    chassis_mode = CHASSIS_VECTOR_RAW;
    last_chassis_mode = chassis_mode;

    //初始化底盘电机
        //动力电机数据
        chassis_motive_motor.init(can_receive.get_chassis_motive_motor_measure_point());
        //初始化pid
        fp32 motive_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
        chassis_motive_motor.speed_pid.init(PID_SPEED, motive_speed_pid_parm, &chassis_motive_motor.speed, &chassis_motive_motor.speed_set, NULL);
        chassis_motive_motor.speed_pid.pid_clear();

    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //用一阶滤波代替斜波函数生成
    chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //初始化角度Z轴PID
    fp32 z_angle_pid_parm[5] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT};
    chassis_wz_angle_pid.init(PID_ANGLE, z_angle_pid_parm, &chassis_relative_angle, &chassis_relative_angle_set, NULL);
    chassis_wz_angle_pid.pid_clear();
    //速度限幅设置
    y.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    y.max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;


    chassis_control_way = RC ;
    left_light_sensor = 0 ;
    right_light_sensor = 0;
    direction = LEFT ;
    referee.field_event_outpost=0;
    
    //不规则运动初始化
    srand(tim);   //初始化种子为随机值
	for(Irregular_motion_num=0;Irregular_motion_num<10;Irregular_motion_num++)
	{
			Irregular_motion[Irregular_motion_num] = rand()%2500+500;
            Irregular_motion_sign[Irregular_motion_num] = Irregular_motion[Irregular_motion_num];
	}
	Irregular_motion_num = 9;
    //更新一下数据
    feedback_update();
}

/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     
  * @retval         none
  */
void Chassis::set_mode() {
    chassis_behaviour_mode_set();
}

/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     
  * @retval         none
  */
void Chassis::mode_change_control_transit()
{
    if (last_chassis_mode == chassis_mode)
    {
        return;
    }

    last_chassis_mode = chassis_mode;
}

/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     
  * @retval         none
  */
void Chassis::feedback_update()
{   
    //记录上一次遥控器值
    chassis_last_key_v =chassis_RC->key.v;
    chassis_yaw_set = chassis_yaw;

    //更新电机数据
        //更新动力电机速度，加速度是速度的PID微分
    chassis_motive_motor.speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_motive_motor.motor_measure->speed_rpm;
    chassis_motive_motor.accel = *chassis_motive_motor.speed_pid.data.error_delta * CHASSIS_CONTROL_FREQUENCE;

    //更新底盘 y 速度值,右手坐标系
    //TODO 速度的更新可能要进行修改
    
    y.speed =chassis_motive_motor.speed * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    

        
    //TODO 还未完善
    //底盘相对于云台的角度,由云台发送过来
    chassis_relative_angle = can_receive.chassis_receive.gimbal_yaw_angle;

    // //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    // chassis_yaw = rad_format(*(chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_yaw_motor->relative_angle);
    // chassis_pitch = rad_format(*(chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_pitch_motor->relative_angle);
    // chassis_roll = *(chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
    
    //更新光电数据
    left_light_sensor = !(HAL_GPIO_ReadPin(left_light_sensor_GPIO_Port, left_light_sensor_Pin));
    right_light_sensor = !(HAL_GPIO_ReadPin(right_light_sensor_GPIO_Port, right_light_sensor_Pin));
}

/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     
  * @retval         none
  */
void Chassis::set_contorl() {
    fp32  vy_set = 0.0f;

    //获取控制设置值
    chassis_behaviour_control_set( &vy_set);

    if (chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //速度限幅
        y.speed_set = fp32_constrain(vy_set, y.min_speed, y.max_speed);
    }
    else if (chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //在原始模式，设置值是发送到CAN总线
        y.speed_set = vy_set;
        chassis_cmd_slow_set_vy.out = 0.0f;
    }
}


/**
  * @brief          解算数据,并进行pid计算
  * @param[out]     
  * @retval         none
  */
void Chassis::solve() {
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed =  0.0f; //动力电机目标速度
    wheel_speed = y.speed_set;


    if (chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
            chassis_motive_motor.current_give = (int16_t)(wheel_speed);
        
        //raw控制直接返回
        return;
    }

    //计算动力电机控制最大速度，并限制其最大速度
    
        chassis_motive_motor.speed_set = wheel_speed;
        temp = fabs(chassis_motive_motor.speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    


    //计算pid
    
        //计算动力电机的输出电流
        chassis_motive_motor.current_set = chassis_motive_motor.speed_pid.pid_calc();
    
}

/**
  * @brief          底盘功率控制
  * @param[in]     
  * @retval         none
  */
void Chassis::power_ctrl() {
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 chassis_power_limit = 0.0f;

    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = 0;
    referee.get_robot_id(&robot_id);


    if (toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if (robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        referee.get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        referee.get_chassis_power_limit(&chassis_power_limit);

        //功率超过上限 和缓冲能量小于50j,因为缓冲能量小于50意味着功率超过上限
        if (chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if (chassis_power_buffer > 5.0f)
            {
                //缩小WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                // only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //缩小
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            //功率大于WARNING_POWER
            if (chassis_power > chassis_power_limit - WARNING_POWER_DISTANCE)
            {
                fp32 power_scale;
                //功率小于上限
                if (chassis_power < chassis_power_limit)
                {
                    //缩小
                    power_scale = (chassis_power_limit - chassis_power) / (chassis_power_limit - (chassis_power_limit - WARNING_POWER_DISTANCE));
                }
                //功率大于上限
                else
                {
                    power_scale = 0.0f;
                }

                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //功率小于WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    total_current = 0.0f;
    //计算原本电机电流设定
    total_current += fabs(chassis_motive_motor.current_set);

    if (total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        //对动力电机进行功率控制
        chassis_motive_motor.current_set *= current_scale;

    }
}

/**
  * @brief         输出电流
  * @param[in]     
  * @retval         none
  */
void Chassis::output()
{
    //赋值电流值
    
    chassis_motive_motor.current_give = (int16_t)(chassis_motive_motor.current_set);
    

    //电流输出控制,通过调整宏定义控制    
#if CHASSIS_MOTIVE_MOTOR_NO_CURRENT
        chassis_motive_motor.current_give = 0;
#endif

    

    can_receive.can_cmd_chassis_motive_motor(chassis_motive_motor.current_give);
}

/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]     
  * @retval         none
  */
void Chassis::chassis_behaviour_mode_set()
{
    last_chassis_behaviour_mode = chassis_behaviour_mode;
    last_chassis_mode = chassis_mode;

    //遥控器设置模式
    if (switch_is_up(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        //chassis_behaviour_mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
        chassis_control_way = AUTO;
    }
    else if (switch_is_mid(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
        chassis_control_way = RC;
    }
    else if (switch_is_down(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }

    //添加自己的逻辑判断进入新模式

    //根据行为模式选择一个底盘控制模式
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_mode = CHASSIS_VECTOR_RAW;
    }
}

/**
  * @brief          设置控制量.根据不同底盘控制模式，参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[in]       包括底盘所有信息.
  * @retval         none
  */
void Chassis::chassis_behaviour_control_set(fp32 *vy_set ) {

    if ( vy_set == NULL  )
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control( vy_set); 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control( vy_set); 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control( vy_set); 
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control( vy_set); 
    }
}

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @retval         返回空
  */
void Chassis::chassis_zero_force_control( fp32 *vy_can_set)
{
    if (vy_can_set == NULL)
    {
        return;
    }
    *vy_can_set = 0.0f;
}

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @retval         返回空
  */
void Chassis::chassis_no_move_control(fp32 *vy_set)
{
    if (vy_set == NULL)
    {
        return;
    }
    *vy_set = 0.0f;
}

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @retval         返回空
  */

/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      数据
  * @retval         返回空
  */
void Chassis::chassis_no_follow_yaw_control( fp32 *vy_set) {

    if ( vy_set == NULL )
    {
        return;
    }

    chassis_rc_to_control_vector( vy_set);

}

/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      数据
  * @retval         none
  */
void Chassis::chassis_open_set_control( fp32 *vy_set) { 
    if (vy_set == NULL )
    {
        return;
    }

    *vy_set = -chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @retval         none
  */
void Chassis::chassis_rc_to_control_vector( fp32 * vy_set) {
    if ( vy_set == NULL)
    {
        return;
    }
    if (chassis_control_way==RC){
        int16_t  vy_channel;
        fp32  vy_set_channel;
        //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
        rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

        vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

        //一阶低通滤波代替斜波作为底盘速度输入
        chassis_cmd_slow_set_vy.first_order_filter_cali(vy_set_channel);
        
        //停止信号，不需要缓慢加速，直接减速到零
        if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
        {
            chassis_cmd_slow_set_vy.out = 0.0f;
        }

        *vy_set = chassis_cmd_slow_set_vy.out;
    }
    else if(chassis_control_way==AUTO){
        int flag = 0;           //被击打开关
        int up_time = 0;        //加速时间
        if(referee.if_hit()){
            flag = 1;
        }
        referee.output_state();
        if(referee.field_event_outpost == 1){//前哨站存活,停在右边
            if(left_light_sensor == TRUE && right_light_sensor == TRUE)
            {
                direction = NO_MOVE;
            }
            else if(left_light_sensor == FALSE && right_light_sensor == TRUE)
            {
                direction = NO_MOVE;
            }
            else if(left_light_sensor == TRUE && right_light_sensor == FALSE)
            {
                direction = RIGHT;
            }
            else if(left_light_sensor == FALSE && right_light_sensor == FALSE)
            {
                direction = direction ;
            }
        }
        if(referee.field_event_outpost == 0){//前哨站被击毁，开始巡逻
            //底盘基础巡逻轨迹
            /*
            左边识别 右边识别    静止不动
            左边未识别 右边识别  方向向左
            左边识别 右边未识别  方向向右
            左边未识别 右边未识别 保持原状态
            */
            if(left_light_sensor == TRUE && right_light_sensor == TRUE)
            {
                direction = NO_MOVE;
            }
            else if(left_light_sensor == FALSE && right_light_sensor == TRUE)
            {
                direction = LEFT;
            }
            else if(left_light_sensor == TRUE && right_light_sensor == FALSE)
            {
                direction = RIGHT;
            }
            else if(left_light_sensor == FALSE && right_light_sensor == FALSE)
            {
                  
                //不规则运动
                if(!referee.if_hit() && Irregular_motion[Irregular_motion_num] > 0)
                {
                    Irregular_motion[Irregular_motion_num]--;
                }
                if(!referee.if_hit() && Irregular_motion[Irregular_motion_num] == 0)
                {
                    if(direction == LEFT)
                    {
                        direction = RIGHT;
                    }
                    else
                    {
                        direction = LEFT;
                    }
                    if(Irregular_motion_num >= 0)
                    {
                        Irregular_motion_num--;
                    }
                    else
                    {
                        srand(tim++);
                        for(Irregular_motion_num=0;Irregular_motion_num<10;Irregular_motion_num++){
			                    Irregular_motion[Irregular_motion_num] = rand()%2500+500;  //随机数生成
                                Irregular_motion_sign[Irregular_motion_num] = Irregular_motion[Irregular_motion_num];
	                    }
	                    Irregular_motion_num = 9;
                    }
                }
                if(referee.if_hit()){
                    if(flag){
                        flag = 1;
                    
                        if(direction == LEFT){
                            direction = RIGHT;
                        }
                        if(direction == RIGHT){
                            direction = LEFT;
                        }
                    }
                }
                direction = direction;
            }
        }

        //受击打加速
        if(flag)
        {
            *vy_set = CHASSIS_HIGH_SPEED;
        }
        else
        {
            *vy_set = CHASSIS_MID_SPEED;
        }
        if(!referee.if_hit()){
            if(flag){
                up_time++;
            }
            if(up_time>500){
                flag = 0;
            }
        }
        //根据方向设置输出
        if(direction == LEFT)
            *vy_set = *vy_set;
        else if(direction == RIGHT)
            *vy_set = -*vy_set;
        else if(direction == NO_MOVE)
            *vy_set = 0;

        
    }
    
}


/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
fp32 Chassis::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}