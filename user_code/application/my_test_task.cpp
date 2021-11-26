#include "my_test_task.h"

#include "pid.h"
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
      flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
      HAL_Delay(200);


  }
    
}