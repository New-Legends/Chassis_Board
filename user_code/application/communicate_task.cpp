#include "communicate_task.h"

#include "Communicat.h"

uint8_t communicat_flag = 0;

/**
* @brief          communucat_task
* @param[in]      pvParameters: NULL
* @retval         none
*/
void communicate_task(void *pvParameters)
{
  vTaskDelay(COMMUNICATE_TASK_INIT_TIME);

  communicat.init();

  while (1)
  {
    communicat_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    
    communicat.run();

    //系统延时
    vTaskDelay(COMMUNICATE_CONTROL_TIME_MS);
  }
}






