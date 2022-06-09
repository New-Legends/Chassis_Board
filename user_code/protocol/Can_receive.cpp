#include "can_receive.h"

#include "cmsis_os.h"
#include "main.h"

#include "bsp_can.h"
#include "can.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Can_receive can_receive;

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
    chassis_can_send_data[6] = motor >> 8;
    chassis_can_send_data[7] = motor;

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
    chassis_receive.s1 = data[7];
}


// void Can_receive::send_cooling_and_id_board_com_1(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint16_t stage_remain_time)
// {
//     //数据填充
//     chassis_send.id1_17mm_cooling_limit = id1_17mm_cooling_limit;
//     chassis_send.id1_17mm_cooling_rate = id1_17mm_cooling_rate;
//     chassis_send.id1_17mm_cooling_heat = id1_17mm_cooling_heat;
//     chassis_send.stage_remain_time = stage_remain_time;


//     uint32_t send_mail_box;
//     chassis_tx_message.StdId = CAN_COOLING_BOARM_COM_1_ID;
//     chassis_tx_message.IDE = CAN_ID_STD;
//     chassis_tx_message.RTR = CAN_RTR_DATA;
//     chassis_tx_message.DLC = 0x08;
//     chassis_can_send_data[0] = id1_17mm_cooling_limit >> 8;
//     chassis_can_send_data[1] = id1_17mm_cooling_limit;
//     chassis_can_send_data[2] = id1_17mm_cooling_rate >> 8;
//     chassis_can_send_data[3] = id1_17mm_cooling_rate;
//     chassis_can_send_data[4] = id1_17mm_cooling_heat >> 8;
//     chassis_can_send_data[5] = id1_17mm_cooling_heat;
//     chassis_can_send_data[6] = stage_remain_time >> 8;
//     chassis_can_send_data[7] = stage_remain_time;

//     HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
// }

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

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

// void Can_receive::send_17mm_speed_and_mode_board_com_1(uint16_t id1_17mm_speed_limit, uint16_t id1_bullet_speed, uint8_t HP, uint16_t bullet_remaining_num_17mm, uint8_t game_progress)
// {
//     //数据填充
//     chassis_send.id1_17mm_speed_limi = id1_17mm_speed_limit;
//     chassis_send.id1_bullet_speed = id1_bullet_speed;
//     chassis_send.bullet_remaining_num_17mm = bullet_remaining_num_17mm;
//     chassis_send.game_progress = game_progress;


//     uint32_t send_mail_box;
//     chassis_tx_message.StdId = CAN_17MM_SPEED_BOARD_COM_1_ID;
//     chassis_tx_message.IDE = CAN_ID_STD;
//     chassis_tx_message.RTR = CAN_RTR_DATA;
//     chassis_tx_message.DLC = 0x08;
//     chassis_can_send_data[0] = id1_17mm_speed_limit >> 8;
//     chassis_can_send_data[1] = id1_17mm_speed_limit;
//     chassis_can_send_data[2] = id1_bullet_speed >> 8;
//     chassis_can_send_data[3] = id1_bullet_speed;
//     chassis_can_send_data[4] = HP;
//     chassis_can_send_data[5] = bullet_remaining_num_17mm >> 8;
//     chassis_can_send_data[6] = bullet_remaining_num_17mm;
//     chassis_can_send_data[7] = game_progress;

//     HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
// }

void Can_receive::send_17mm_speed_and_mode_board_com_2(uint16_t id2_17mm_speed_limit, uint16_t id2_bullet_speed, uint8_t HP, uint16_t bullet_remaining_num_17mm, uint8_t biaozhi)
{
    //数据填充
    chassis_send.id2_17mm_speed_limi = id2_17mm_speed_limit;
    chassis_send.id2_bullet_speed = id2_bullet_speed;
    chassis_send.biaozhi = biaozhi;

    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_17MM_SPEED_BOARD_COM_2_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id2_17mm_speed_limit >> 8;
    chassis_can_send_data[1] = id2_17mm_speed_limit;
    chassis_can_send_data[2] = id2_bullet_speed >> 8;
    chassis_can_send_data[3] = id2_bullet_speed;
    chassis_can_send_data[4] = HP;
    chassis_can_send_data[5] = bullet_remaining_num_17mm >> 8;
    chassis_can_send_data[6] = bullet_remaining_num_17mm;
    chassis_can_send_data[7] = biaozhi;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

// void Can_receive::send_rc_com(int16_t ch_0, int16_t ch_1, int8_t ch_2, int8_t s0, int8_t s1){
//     //数据填充
//     chassis_send.ch_0 = ch_0;
//     chassis_send.ch_1 = ch_1;
//     chassis_send.ch_2 = ch_2;
//     chassis_send.s0 = s0;
//     chassis_send.s1 = s1;

//     uint32_t send_mail_box;
//     chassis_tx_message.StdId = CAN_RC_COM_ID;
//     chassis_tx_message.IDE = CAN_ID_STD;
//     chassis_tx_message.RTR = CAN_RTR_DATA;
//     chassis_tx_message.DLC = 0x08;
//     chassis_can_send_data[0] = ch_0 >> 8;
//     chassis_can_send_data[1] = ch_0;
//     chassis_can_send_data[2] = ch_1 >> 8;
//     chassis_can_send_data[3] = ch_1;
//     chassis_can_send_data[4] = ch_2 >> 8;
//     chassis_can_send_data[5] = ch_2;
//     chassis_can_send_data[6] = s0;
//     chassis_can_send_data[7] = s1;

//     HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
// }

