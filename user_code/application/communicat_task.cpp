#include "communicat_task.h"

#include "Communicat.h"

uint8_t communicat_flag = 0;

/**
* @brief          communucat_task
* @param[in]      pvParameters: NULL
* @retval         none
*/
void communicat_task(void *pvParameters)
{
  vTaskDelay(COMMUNICAT_TASK_INIT_TIME);

  communicat.init();

  while (1)
  {
    communicat_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    HAL_Delay(20);

  }
}






