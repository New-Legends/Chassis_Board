#include "communicate.h"

#include "main.h"
#include "string.h"


#include "bsp_usart.h"
#include "bsp_led.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"
#include "fifo.h"

#ifdef __cplusplus
}
#endif

#include "Chassis.h"
#include "detect_task.h"

#include "Remote_control.h"
#include "Can_receive.h"
#include "Referee.h"
#include "Ui.h"

Remote_control remote_control;
Can_receive can_receive;
Ui      ui;
Communicate communicate;

void Communicate::init()
{

//TODO 这里最好使用指针赋值,减少计算量,后续需修改
#if CHASSIS_REMOTE_OPEN
    ;
#else
    remote_control.init();
#endif

    can_receive.init();

    referee.init();

    ui.init(&referee.Judge_Self_ID, &referee.Judge_SelfClient_ID);
}

void Communicate::run()
{
    referee.unpack();
    referee.determine_ID();

    // ui.run();


    //向云台发送裁判数据
    uint16_t temp_id1_17mm_cooling_limit, temp_id1_17mm_cooling_rate, temp_id1_17mm_cooling_heat;
    uint16_t temp_id2_17mm_cooling_limit, temp_id2_17mm_cooling_rate, temp_id2_17mm_cooling_heat;
    uint8_t temp_color, temp_robot_id;
    uint16_t temp_id1_17mm_speed_limit, temp_id2_17mm_speed_limit;
    fp32 temp_id1_bullet_speed, temp_id2_bullet_speed;
    uint16_t  temp_red_base_HP;
    uint16_t  temp_blue_base_HP;
    int16_t temp_ch_0,temp_ch_1,temp_ch_2;
    int8_t temp_s0,temp_s1;
    uint16_t temp_bullet_remaining_num_17mm;
    uint8_t temp_game_progress;
    uint8_t temp_HP;

    referee.get_shooter_17mm_cooling_limit_and_heat(&temp_id1_17mm_cooling_limit, &temp_id1_17mm_cooling_heat, &temp_id2_17mm_cooling_limit,&temp_id2_17mm_cooling_heat);
    referee.get_shooter_17mm_cooling_rate(&temp_id1_17mm_cooling_rate, &temp_id2_17mm_cooling_rate);
    referee.get_color(&temp_color);
    referee.get_robot_id(&temp_robot_id);
    referee.get_shooter_17mm_speed_limit_and_bullet_speed(&temp_id1_17mm_speed_limit, &temp_id1_bullet_speed, &temp_id2_17mm_speed_limit, &temp_id2_bullet_speed);
    temp_bullet_remaining_num_17mm = referee.bullet_remaining_t.bullet_remaining_num_17mm;
    temp_red_base_HP = referee.game_robot_HP_t.red_base_HP;
    temp_blue_base_HP = referee.game_robot_HP_t.blue_base_HP;
    if(temp_color == RED){
        if(temp_red_base_HP > 0){
            temp_HP = 1;
        }
        else{
            temp_HP = 0;
        }
    }
    else if(temp_color == BLUE){
        if(temp_blue_base_HP > 0){
            temp_HP = 1;
        }
        else{
            temp_HP = 0;
        }
    }
    temp_ch_0 = can_receive.chassis_receive.ch_0;
    temp_ch_1 = can_receive.chassis_receive.ch_1;
    temp_ch_2 = can_receive.chassis_receive.ch_2;
    temp_s0 = can_receive.chassis_receive.s0;
    temp_s1 = can_receive.chassis_receive.s1;
    temp_game_progress = referee.game_state.game_progress;


    can_receive.send_cooling_and_id_board_com_1(temp_id1_17mm_cooling_limit, temp_id1_17mm_cooling_rate, temp_id1_17mm_cooling_heat,
                                              temp_game_progress);
    can_receive.send_cooling_and_id_board_com_2(temp_id2_17mm_cooling_limit, temp_id2_17mm_cooling_rate, temp_id2_17mm_cooling_heat,
                                              temp_color, temp_robot_id);                              
    if(temp_color == RED){
        can_receive.send_17mm_speed_and_mode_board_com_1(temp_id1_17mm_speed_limit, temp_id1_bullet_speed, temp_HP, temp_bullet_remaining_num_17mm, temp_game_progress);
        can_receive.send_17mm_speed_and_mode_board_com_2(temp_id2_17mm_speed_limit, temp_id2_bullet_speed, temp_red_base_HP, temp_bullet_remaining_num_17mm);
    }
    else if(temp_color == BLUE){
        can_receive.send_17mm_speed_and_mode_board_com_1(temp_id1_17mm_speed_limit, temp_id1_bullet_speed, temp_HP, temp_bullet_remaining_num_17mm, temp_game_progress);
        can_receive.send_17mm_speed_and_mode_board_com_2(temp_id2_17mm_speed_limit, temp_id2_bullet_speed, temp_blue_base_HP, temp_bullet_remaining_num_17mm);
    }
    can_receive.send_rc_com(temp_ch_0,temp_ch_1,temp_ch_2,temp_s0,temp_s1);


//TODO 这里最好使用指针赋值,减少计算量,后续需修改
#if CHASSIS_REMOTE_OPEN
    remote_control.rc_ctrl.rc.ch[0] = can_receive.chassis_receive.ch_0;
    remote_control.rc_ctrl.rc.ch[2] = can_receive.chassis_receive.ch_2;
    remote_control.rc_ctrl.rc.ch[1] = can_receive.chassis_receive.ch_1;
    remote_control.rc_ctrl.rc.s[0] = can_receive.chassis_receive.s0;
    remote_control.rc_ctrl.rc.s[1] = can_receive.chassis_receive.s1;
#else
   ;
#endif
}

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{

    //TODO 设备检测未更新
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (hcan == &CHASSIS_CAN) //接底盘CAN 信息
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
                //底盘动力电机
                case CAN_MOTIVE:
                    can_receive.get_motive_motor_measure(rx_data);
                    //detect_hook(CHASSIS_MOTIVE_FR_MOTOR_TOE);
                    break;

                default:
                {
                    break;
                }
            }
        }
        else if(hcan == &BOARD_COM_CAN)
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
                case CAN_RC_BOARM_COM_ID:
                    can_receive.receive_rc_board_com(rx_data);
                    //detect_hook(BOARD_COM);
                    break;

                default:
                {
                    break;
                }
            }
        }
    }
    // TODO 设备检查未更新
    //遥控器串口
    void USART3_IRQHandler(void)
    {
        if (huart3.Instance->SR & UART_FLAG_RXNE) //接收到数据
        {
            __HAL_UART_CLEAR_PEFLAG(&huart3);
        }
        else if (USART3->SR & UART_FLAG_IDLE)
        {
            static uint16_t this_time_rx_len = 0;

            __HAL_UART_CLEAR_PEFLAG(&huart3);

            if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
            {
                /* Current memory buffer used is Memory 0 */

                //disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                //get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                //reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                //set memory buffer 1
                //设定缓冲区1
                hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

                //enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    remote_control.unpack(0);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(0);
                }
            }
            else
            {
                /* Current memory buffer used is Memory 1 */
                //disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                //get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                //reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                //set memory buffer 0
                //设定缓冲区0
                DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

                //enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
                    remote_control.unpack(1);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(1);
                }
            }
        }
    }

    // TODO 设备检查未更新
    //裁判串口数据
    void USART6_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART6->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart6);

            static uint16_t this_time_rx_len = 0;

            if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[0]), this_time_rx_len);
                detect_hook(REFEREE_TOE);
            } 
            else
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[1]), this_time_rx_len);
                detect_hook(REFEREE_TOE);
            }
        }
    }
}
#endif