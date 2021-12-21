#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN hcan1
#define BOARD_COM_CAN hcan1

//底盘电机编号
enum chassis_motor_id_e
{
    //底盘动力电机接收
    FR_MOTOR = 0,
    FL_MOTOR,
    BL_MOTOR,
    BR_MOTOR,
};


/* CAN send and receive ID */
typedef enum
{
  //底盘电机接收ID  CAN2
  CAN_FR_MOTOR_ID = 0x201,
  CAN_FL_MOTOR_ID = 0x202,
  CAN_BL_MOTOR_ID = 0x203,
  CAN_BR_MOTOR_ID = 0x204,
  CAN_CHASSIS_ALL_ID = 0x200,

  CAN_RC_BOARM_COM_ID = 0x301,
  CAN_GIMBAL_BOARD_COM_ID = 302,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

//TODO 超电还未对接
// //rm motor data
// typedef struct
// {
//   fp32 input_vot;
//   fp32 supercap_vot;
//   fp32 input_current;
//   fp32 target_power;
// } super_cap_measure_t;

//底盘接收数据结构体
typedef struct
{
  //遥控器数据
  int16_t ch_1;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s1;
  uint8_t gimbal_behaviour;
  fp32    gimbal_yaw_angle;
} chassis_receive_t;

//云台发送数据结构体
typedef struct
{
  //遥控器数据
  int16_t ch_1;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s1;
  uint8_t gimbal_behaviour;
  fp32 gimbal_yaw_angle;
} gimbal_receive_t;

//底盘发送数据结构体
typedef struct
{

  //测试热量及ID
  uint16_t id1_17mm_cooling_limit;//17mm测速热量上限
  uint16_t id1_17mm_cooling_rate;//17mm测速热量冷却
  uint16_t id1_17mm_cooling_heat; //17mm测速实时热量
  uint8_t color;               //判断红蓝方
  uint8_t robot_id;            //机器人编号

  //测速速度及底盘模式
  uint16_t id1_17mm_speed_limi;//17mm测速射速上限
  uint16_t bullet_speed;       //17mm测速实时射速

  uint8_t chassis_behaviour;

} chassis_send_t;

class Can_receive {
public: 
  //反馈数据结构体
  motor_measure_t chassis_motor[4];

  chassis_receive_t chassis_receive;

  //做测试
  gimbal_receive_t gimbal_receive;

  //发送数据结构体
  CAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  void init();

  //电机数据接收
  void get_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

  void can_cmd_chassis_reset_ID();

  const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

  void get_rc_board_com(uint8_t data[8]);
  void get_gimbal_board_com(uint8_t data[8]);

  void send_rc_board_com(int16_t ch_1, int16_t ch_2, int16_t ch_3, uint16_t v);
};



#endif
