#include "my_test_task.h"

#include "bsp_led.h"

uint8_t flag = 0;

/**
  * @brief          test_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void my_test_task(void *pvParameters)
{

    while (1)
    {
        //aRGB_led_show(100);
        // HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        // HAL_Delay(200);

        flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

        HAL_Delay(200);
        // HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        // HAL_Delay(200);
    }
    
}