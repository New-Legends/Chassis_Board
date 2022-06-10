#ifndef CHASSIS_H
#define CHASSIS_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Referee.h"


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//遥控器控制
#define RC   0  
//自动程序控制
#define AUTO 1  
//底盘运动方向
#define LEFT 0
#define RIGHT 1
#define NO_MOVE 2

//有无光电
#define guangdian 1

//跑轨时间
#define time_gui 1500

//底盘动力电机无电流输出
#define CHASSIS_MOTIVE_MOTOR_NO_CURRENT 0
//底盘舵向电机无电流输出
#define CHASSIS_RUDDER_MOTOR_NO_CURRENT 0

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3

//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 0

//初试yaw轴角度
#define INIT_YAW_SET 0.0f

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘小陀螺按键
#define TOP_KEY KEY_PRESSED_OFFSET_F

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f //4
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f //1.5
//底盘巡逻速度等级
#define CHASSIS_LOW_SPEED 0.4*NORMAL_MAX_CHASSIS_SPEED_Y
#define CHASSIS_MID_SPEED 1.1*NORMAL_MAX_CHASSIS_SPEED_Y
#define CHASSIS_HIGH_SPEED 1.5*NORMAL_MAX_CHASSIS_SPEED_Y


#define right_light_sensor_Pin GPIO_PIN_9
#define right_light_sensor_GPIO_Port GPIOE
#define left_light_sensor_Pin GPIO_PIN_11
#define left_light_sensor_GPIO_Port GPIOE


//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

#define SWING_KEY ((chassis_RC->key.v & KEY_PRESSED_OFFSET_C) && !(chassis_last_key_v & KEY_PRESSED_OFFSET_C))
#define PISA_KEY ((chassis_RC->key.v & KEY_PRESSED_OFFSET_X) && !(chassis_last_key_v & KEY_PRESSED_OFFSET_X))
#define SWING_KEY ((chassis_RC->key.v & KEY_PRESSED_OFFSET_C) && !(chassis_last_key_v & KEY_PRESSED_OFFSET_C))
#define PISA_KEY ((chassis_RC->key.v & KEY_PRESSED_OFFSET_X) && !(chassis_last_key_v & KEY_PRESSED_OFFSET_X))

// #define TOP_KEY ((chassis_RC->key.v & KEY_PRESSED_OFFSET_F) && !(chassis_last_key_v & KEY_PRESSED_OFFSET_F))


#define PISA_DELAY_TIME 500
#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上



//chassis motor speed PID
//底盘电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_KP 6000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 2.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 6000.0f

//chassis follow angle PID
//底盘旋转跟随PID
// #define CHASSIS_FOLLOW_GIMBAL_PID_KP 11.0f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 2.0f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 10.0f

//功率控制参数
#define POWER_DEFAULT_LIMIT 30.0f  //默认功率限制
#define WARNING_POWER_DISTANCE 10.0f //距离超过率的距离
#define WARNING_POWER_BUFF 150.0f    //警告缓存

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 16000.0f
#define POWER_TOTAL_CURRENT_LIMIT 20000.0f

typedef enum
{
    CHASSIS_ZERO_FORCE,                  //chassis will be like no power,底盘无力, 跟没上电那样
    CHASSIS_NO_MOVE,                     //chassis will be stop,底盘保持不动
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //chassis will follow gimbal, usually in infantry,正常步兵底盘跟随云台
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, //chassis will follow chassis yaw angle, usually in engineer,
                                         //because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
                                         //if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
                                         //工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，
                                         //如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度 在chassis_feedback_update函数中
    CHASSIS_NO_FOLLOW_YAW,               //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
                                         //底盘不跟随角度，角度是开环的，但轮子是有速度环
    CHASSIS_OPEN,                        //the value of remote control will mulitiply a value, get current value that will be sent to can bus
                                         // 遥控器的值乘以比例成电流值 直接发送到can总线上
} chassis_behaviour_e;

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,  //底盘会跟随云台相对角度
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW, //底盘有底盘角度控制闭环
    CHASSIS_VECTOR_NO_FOLLOW_YAW,      // 底盘有旋转速度控制
    CHASSIS_VECTOR_RAW,                //电流之间控制,开环

} chassis_mode_e;

typedef enum
{
    chushihua,  //初始化跑到最左边
    jigui, //记录轨道长度
    wancheng,
} Chushijigui;


struct speed_t
{
    fp32 speed;
    fp32 speed_set;

    fp32 max_speed;
    fp32 min_speed;
};



class Chassis {
public:
    const RC_ctrl_t *chassis_RC; //底盘使用的遥控器指针
    uint16_t chassis_last_key_v;  //遥控器上次按键

    chassis_behaviour_e chassis_behaviour_mode; //底盘行为状态机
    chassis_behaviour_e last_chassis_behaviour_mode; //底盘上次行为状态机
    Chushijigui chushijigui;//记录轨道长度

    chassis_mode_e chassis_mode; //底盘控制状态机
    chassis_mode_e last_chassis_mode; //底盘上次控制状态机

    M3508_motor chassis_motive_motor; //底盘动力电机数据


    First_order_filter chassis_cmd_slow_set_vy;        //使用一阶低通滤波减缓设定值

    // Pid chassis_wz_angle_pid;        //底盘角度pid

    speed_t x;
    speed_t y;
    speed_t z;

    fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad
    fp32 chassis_relative_angle_set; //设置相对云台控制角度
    fp32 chassis_yaw_set;

    fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
    fp32 chassis_pitch; //.陀螺仪和云台电机叠加的pitch角度
    fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度

    //巡逻会用到的数据
    bool_t chassis_control_way; //底盘控制方式
    bool_t left_light_sensor;  //左侧光电传感器 0为未感应到 1为感应到
    bool_t right_light_sensor;  //右侧光电传感器 0为未感应到 1为感应到
    uint16_t left_light_sensor_update_time; //光电传感器数据更新时间
    uint16_t right_light_sensor_update_time; //光电传感器数据更新时间
    uint8_t direction;          //底盘移动方向 分为NO_MOVE LEFT RIGHT 
    fp32 CHASSIS_MAX_SPEED;
    int speed_flag;
    int up_time ;        //加速时间
    int16_t guidao;      //轨道长度
    int16_t biaozhi;
    int8_t init_flag;
    int16_t shijian;



    //任务流程
    void init();

    void set_mode();

    void mode_change_control_transit();

    void feedback_update();

    void set_contorl();

    void solve();

    void power_ctrl();

    void output();

    //行为控制

    void chassis_behaviour_mode_set();

    void chassis_behaviour_control_set(fp32 *vy_set);

    void chassis_zero_force_control(fp32 *vy_can_set);

    void chassis_no_move_control(fp32 *vy_set);

    void chassis_no_follow_yaw_control(fp32 *vy_set);

    void chassis_open_set_control(fp32 *vy_set);

    //功能性函数
    void chassis_rc_to_control_vector(fp32 *vy_set);

    fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

    void chassis_motor_count_init(fp32 *vy_set);

};


extern Chassis chassis;


#endif