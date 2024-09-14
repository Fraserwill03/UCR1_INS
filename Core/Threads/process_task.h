#ifndef PROCESS_TASK_H
#define PROCESS_TASK_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "usart.h"
#include "fdcan.h"
#include "main.h"

void ProcessLogTask(void* argument);

extern osMessageQueueId_t dataQueueHandle;

#endif // PROCESS_TASK_H
