#include "can_receive.h"

#include "cmsis_os.h"
#include "main.h"

#include "bsp_can.h"
#include "can.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Can_receive::init()
{
    can_filter_init();
}
 
void Can_receive::get_motive_motor_measure(uint8_t data[8])
{
    chassis_motive_motor.last_ecd = chassis_motive_motor.ecd;
    chassis_motive_motor.ecd = (uint16_t)(data[0] << 8 | data[1]);
    chassis_motive_motor.speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    chassis_motive_motor.given_current = (uint16_t)(data[4] << 8 | data[5]);
    chassis_motive_motor.temperate = data[6];
}

void Can_receive::get_yaw_motor_measure(uint8_t data[8])
{
    chassis_yaw_motor.last_ecd = chassis_yaw_motor.ecd;
    chassis_yaw_motor.ecd = (uint16_t)(data[0] << 8 | data[1]);
    chassis_yaw_motor.speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    chassis_yaw_motor.given_current = (uint16_t)(data[4] << 8 | data[5]);
    chassis_yaw_motor.temperate = data[6]; 
}
/**
* @brief          发送电机控制电流(0x204)
* @param[in]      motor: (0x204) 3508电机控制电流, 范围 [-16384,16384]
* @retval         none
*/
void Can_receive::can_cmd_chassis_motive_motor( int16_t motor)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_MOTIVE_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = motor>>8;
    chassis_can_send_data[7] = motor;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
* @brief          发送YAW控制电流(0x205)
* @param[in]      motor: (0x205) 6020电机控制电流, 范围 [-16384,16384]
* @retval         none
*/
void Can_receive::can_cmd_yaw_motor( int16_t yaw)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = yaw >> 8;
    chassis_can_send_data[1] = yaw;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void Can_receive::can_cmd_chassis_motive_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          返回底盘动力电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *Can_receive::get_chassis_motive_motor_measure_point()
{
    return &chassis_motive_motor;
}

void Can_receive::receive_rc_board_com(uint8_t data[8])
{
    chassis_receive.ch_0 = (int16_t)(data[0] << 8 | data[1]);
    chassis_receive.ch_2 = (int16_t)(data[2] << 8 | data[3]);
    chassis_receive.ch_1 = (uint16_t)(data[4] << 8 | data[5]);
    chassis_receive.s0 = data[6];
}

void Can_receive::receive_yaw_motor(uint8_t data[8])
{
    chassis_receive.gimbal_yaw_give = (int16_t)(data[0] << 8 | data[1]);
}





void Can_receive::send_cooling_and_id_board_com_1(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint8_t color, uint8_t robot_id)
{
    //数据填充
    chassis_send.id1_17mm_cooling_limit = id1_17mm_cooling_limit;
    chassis_send.id1_17mm_cooling_rate = id1_17mm_cooling_rate;
    chassis_send.id1_17mm_cooling_heat = id1_17mm_cooling_heat;
    chassis_send.color = color;
    chassis_send.robot_id = robot_id;


    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_COOLING_BOARM_COM_1_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id1_17mm_cooling_limit >> 8;
    chassis_can_send_data[1] = id1_17mm_cooling_limit;
    chassis_can_send_data[2] = id1_17mm_cooling_rate >> 8;
    chassis_can_send_data[3] = id1_17mm_cooling_rate;
    chassis_can_send_data[4] = id1_17mm_cooling_heat >> 8;
    chassis_can_send_data[5] = id1_17mm_cooling_heat;
    chassis_can_send_data[6] = color;
    chassis_can_send_data[7] = robot_id;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Can_receive::send_cooling_and_id_board_com_2(uint16_t id2_17mm_cooling_limit, uint16_t id2_17mm_cooling_rate, uint16_t id2_17mm_cooling_heat, uint8_t color, uint8_t robot_id)
{
    //数据填充
    chassis_send.id2_17mm_cooling_limit = id2_17mm_cooling_limit;
    chassis_send.id2_17mm_cooling_rate = id2_17mm_cooling_rate;
    chassis_send.id2_17mm_cooling_heat = id2_17mm_cooling_heat;
    chassis_send.color = color;
    chassis_send.robot_id = robot_id;


    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_COOLING_BOARM_COM_2_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id2_17mm_cooling_limit >> 8;
    chassis_can_send_data[1] = id2_17mm_cooling_limit;
    chassis_can_send_data[2] = id2_17mm_cooling_rate >> 8;
    chassis_can_send_data[3] = id2_17mm_cooling_rate;
    chassis_can_send_data[4] = id2_17mm_cooling_heat >> 8;
    chassis_can_send_data[5] = id2_17mm_cooling_heat;
    chassis_can_send_data[6] = color;
    chassis_can_send_data[7] = robot_id;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Can_receive::send_17mm_speed_and_mode_board_com_1(uint16_t id1_17mm_speed_limit, uint16_t id1_bullet_speed, uint8_t chassis_behaviour,uint16_t base_HP)
{
    //数据填充
    chassis_send.id1_17mm_speed_limi = id1_17mm_speed_limit;
    chassis_send.id1_bullet_speed = id1_bullet_speed;
    chassis_send.chassis_behaviour = chassis_behaviour;


    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_17MM_SPEED_BOARD_COM_1_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id1_17mm_speed_limit >> 8;
    chassis_can_send_data[1] = id1_17mm_speed_limit;
    chassis_can_send_data[2] = id1_bullet_speed >> 8;
    chassis_can_send_data[3] = id1_bullet_speed;
    chassis_can_send_data[4] = chassis_behaviour;
    chassis_can_send_data[5] = base_HP >> 8;
    chassis_can_send_data[6] = base_HP;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Can_receive::send_17mm_speed_and_mode_board_com_2(uint16_t id2_17mm_speed_limit, uint16_t id2_bullet_speed, uint8_t chassis_behaviour,uint16_t base_HP)
{
    //数据填充
    chassis_send.id2_17mm_speed_limi = id2_17mm_speed_limit;
    chassis_send.id2_bullet_speed = id2_bullet_speed;
    chassis_send.chassis_behaviour = chassis_behaviour;


    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_17MM_SPEED_BOARD_COM_2_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id2_17mm_speed_limit >> 8;
    chassis_can_send_data[1] = id2_17mm_speed_limit;
    chassis_can_send_data[2] = id2_bullet_speed >> 8;
    chassis_can_send_data[3] = id2_bullet_speed;
    chassis_can_send_data[4] = chassis_behaviour;
    chassis_can_send_data[5] = base_HP >> 8;
    chassis_can_send_data[6] = base_HP;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

// void Can_receive::send_rc_board_com(int16_t ch_0, int16_t ch_1, int8_t s0){
//     //数据填充
//     chassis_send.ch_0 = ch_0;
//     chassis_send.ch_1 = ch_1;
//     chassis_send.s0 = s0;


//     uint32_t send_mail_box;
//     chassis_tx_message.StdId = CAN_CHASSIS_UNDER;
//     chassis_tx_message.IDE = CAN_ID_STD;
//     chassis_tx_message.RTR = CAN_RTR_DATA;
//     chassis_tx_message.DLC = 0x08;
//     chassis_can_send_data[0] = ch_0 >> 8;
//     chassis_can_send_data[1] = ch_0;
//     chassis_can_send_data[2] = ch_1 >> 8;
//     chassis_can_send_data[3] = ch_1;
//     chassis_can_send_data[4] = s0;
//     chassis_can_send_data[5] = 0;
//     chassis_can_send_data[6] = 0;
//     chassis_can_send_data[7] = 0;

//     HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
// }

// void Can_receive::send_yaw_motor(uint16_t ecd, int16_t speed_rpm, int16_t given_current, uint8_t temperate){
//     //数据填充
//     chassis_send.ecd = ecd;
//     chassis_send.speed_rpm = speed_rpm;
//     chassis_send.give_current = given_current;
//     chassis_send.temperate = temperate;


//     uint32_t send_mail_box;
//     chassis_tx_message.StdId = CAN_CHASSIS_YAW;
//     chassis_tx_message.IDE = CAN_ID_STD;
//     chassis_tx_message.RTR = CAN_RTR_DATA;
//     chassis_tx_message.DLC = 0x08;
//     chassis_can_send_data[0] = ecd >> 8;
//     chassis_can_send_data[1] = ecd;
//     chassis_can_send_data[2] = speed_rpm >> 8;
//     chassis_can_send_data[3] = speed_rpm;
//     chassis_can_send_data[4] = given_current >> 8;
//     chassis_can_send_data[5] = given_current;
//     chassis_can_send_data[6] = temperate;
//     chassis_can_send_data[7] = 0;

//     HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
// }