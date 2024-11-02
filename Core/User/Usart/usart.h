/*
 * INS_usart.h
 *
 *  Created on: Nov 2, 2024
 *      Author: Will
 */

#ifndef USER_USART_USART_H_
#define USER_USART_USART_H_

/**
 * Includes
 * --------------------------------------- */
#include "cmsis_os.h"
#include "../Tasks/process_task.h"
#include "stm32g4xx_hal.h"


/**
 * Externs
 * --------------------------------------- */
extern osMessageQueueId_t dataQueueHandle;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


/**
 * Type definitions
 * --------------------------------------- */
typedef enum  {
    SYNC_BYTE_1_STATE,
    SYNC_BYTE_2_STATE,
    SYNC_BYTE_3_STATE,
    HEADER_LENGTH_STATE,
    MESSAGE_LENGTH_STATE,
    MESSAGE_ID_STATE,
    DMA_STATE
} sync_state_t;

typedef enum {
  SHORT,
  LONG
} header_type_t;


/**
 * Public function declarations
 * --------------------------------------- */

void start_ins_usart(void);

#endif /* USER_USART_USART_H_ */
