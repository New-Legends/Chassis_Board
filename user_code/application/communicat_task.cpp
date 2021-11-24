#include "communicat_task.h"

#include "main.h"
#include "string.h"

#include "bsp_usart.h"
#include "bsp_led.h"



#include "remote_control.h"



extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t flag = 0;

remote_control_c remote_control;

/**
  * @brief          communucat_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void communicat_task(void *pvParameters)
{
  vTaskDelay(COMMUNICAT_TASK_INIT_TIME);


  remote_control.init();

  while (1)
  {
    flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    HAL_Delay(20);




  }
}

// TODO 设备检查未更新
//串口中断
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
        //detect_hook(DBUS_TOE);
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
        //detect_hook(DBUS_TOE);
        remote_control.sbus_to_usart1(1);
      }
    }
  }
}
