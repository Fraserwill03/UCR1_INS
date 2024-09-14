#ifndef PROCESS_TASK_H
#define PROCESS_TASK_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "usart.h"
#include "fdcan.h"
#include "main.h"

void ProcessLogTask(void);

typedef struct {
    uint32_t gnss_week;
    uint64_t gnss_seconds;
    uint64_t latitude;
    uint64_t longitude;
    uint64_t height;
    uint64_t north_velocity;
    uint64_t east_velocity;
    uint64_t up_velocity;
    uint64_t roll;
    uint64_t pitch;
    uint64_t azimuth;
    uint32_t status;
} imuratepvas_log_t;

typedef

#endif // PROCESS_TASK_H