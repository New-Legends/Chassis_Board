#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN hcan2
#define BOARD_COM_CAN hcan1

//底盘动力电机编号
enum motive_chassis_motor_id_e
{
  //底盘动力电机接收
  MOTIVE_MOTOR = 0,
};


/* CAN send and receive ID */
typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MOTIVE = 0x204,
  CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  //板间通信ID
  CAN_RC_BOARM_COM_ID = 0x101,
  CAN_RC_COM_ID = 0X102,
  CAN_COOLING_BOARM_COM_1_ID = 0x303,
  CAN_17MM_SPEED_BOARD_COM_1_ID = 0x304,
  CAN_COOLING_BOARM_COM_2_ID = 0x305,
  CAN_17MM_SPEED_BOARD_COM_2_ID = 0x306,

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
  int16_t ch_0;
  int16_t ch_1;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s0;
  uint8_t s1;
  uint8_t gimbal_behaviour;
  fp32    gimbal_yaw_angle;
  int16_t gimbal_yaw_give;
} chassis_receive_t;


//底盘发送数据结构体
typedef struct
{
  //YAW轴数据和遥控器
  int16_t ch_0;
  int16_t ch_1;
  int8_t ch_2;
  int8_t s0;
  int8_t s1;
  //测试热量及ID
  uint16_t id1_17mm_cooling_limit;//17mm测速热量上限
  uint16_t id1_17mm_cooling_rate;//17mm测速热量冷却
  uint16_t id1_17mm_cooling_heat; //17mm测速实时热量
  uint16_t id2_17mm_cooling_limit;//17mm测速热量上限
  uint16_t id2_17mm_cooling_rate;//17mm测速热量冷却
  uint16_t id2_17mm_cooling_heat; //17mm测速实时热量
  uint8_t color;               //判断红蓝方
  uint8_t robot_id;            //机器人编号

  //测速速度及底盘模式
  uint16_t id1_17mm_speed_limi;//17mm测速射速上限
  uint16_t id1_bullet_speed;       //17mm测速实时射速
  uint16_t id2_17mm_speed_limi;//17mm测速射速上限
  uint16_t id2_bullet_speed;       //17mm测速实时射速

  uint16_t bullet_remaining_num_17mm;
  //YAW轴电机数据
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t give_current;
  uint8_t temperate;

} chassis_send_t;


class Can_receive {
public: 
  //动力电机反馈数据结构体
  motor_measure_t chassis_motive_motor;

  motor_measure_t chassis_yaw_motor;

  //发送数据结构体
  CAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  //板间通信
  //底盘接收信息
  chassis_receive_t chassis_receive;

  chassis_send_t chassis_send;

  void init();

  //电机数据接收
  void get_motive_motor_measure( uint8_t data[8]);

  void can_cmd_chassis_motive_motor(int16_t motor);      //动力电机数据

  void can_cmd_chassis_motive_motor_reset_ID();

  const motor_measure_t *get_chassis_motive_motor_measure_point();

  //板间通信函数
  void receive_rc_board_com(uint8_t data[8]);

  // 发送枪口1热量及ID
  void send_cooling_and_id_board_com_1(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint8_t color, uint8_t robot_id);
  //发送枪口1速度及底盘模式
  void send_17mm_speed_and_mode_board_com_1(uint16_t id1_17mm_speed_limit, uint16_t id1_bullet_speed, uint16_t base_HP, uint16_t bullet_remaining_num_17mm);
  // 发送枪口2热量及ID
  void send_cooling_and_id_board_com_2(uint16_t id2_17mm_cooling_limit, uint16_t id2_17mm_cooling_rate, uint16_t id2_17mm_cooling_heat, uint8_t color, uint8_t robot_id);
  //发送枪口2速度及底盘模式
  void send_17mm_speed_and_mode_board_com_2(uint16_t id2_17mm_speed_limit, uint16_t id2_bullet_speed, uint8_t chassis_behaviour, uint16_t base_HP);
  //发送下云台遥控器数据
  void send_rc_com(int16_t ch_0, int16_t ch_1 , int8_t ch_2, int8_t s0, int8_t s1);


};



#endif
