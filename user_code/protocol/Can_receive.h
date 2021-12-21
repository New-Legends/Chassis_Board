#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN hcan1

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

class Can_receive {
public: 
  //反馈数据结构体
  motor_measure_t chassis_motor[4];

  //发送数据结构体
  CAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  void init();

  void get_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

  void can_cmd_chassis_reset_ID();

  const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
};



#endif
