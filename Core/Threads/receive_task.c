#include "receive_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include <stdlib.h>

osThreadId_t receiveTaskHandle;
const osThreadAttr_t receiveTask_attributes = {
  .name = "receiveTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osSemaphoreId_t receiveSemaphoreHandle;
const osSemaphoreAttr_t receiveSemaphore_attributes = {
    .name = "receiveSemaphore"
};

void MX_FREERTOS_Init(void) {
  receiveTaskHandle = osThreadNew(StartReceiveTask, NULL, &receiveTask_attributes);
  receiveSemaphoreHandle = osSemaphoreNew(1, 0, &receiveSemaphore_attributes);
};


