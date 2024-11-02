/*
 * INS_usart.c
 *
 *  Created on: Nov 2, 2024
 *      Author: Will
 */

/**
 * Private Includes
 * --------------------------------------------- */

#include <string.h>

#include "usart.h"


/**
 * Private Defines
 * --------------------------------------------- */

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x44
#define SYNC_BYTE_3_LONG 0x12
#define SYNC_BYTE_3_SHORT 0x13


/**
 * Private Variables
 * --------------------------------------------- */

static uint8_t sync_buf[4];
static uint8_t received_bytes = 0;
static uint8_t message_buf[MAX_RX_BUF];
static sync_state_t sync_state = SYNC_BYTE_1_STATE;
static uint16_t message_length = 0;
static header_type_t header_type = SHORT;
static uint8_t header_length = 0;


/**
 * Function Definitions
 * --------------------------------------------- */

/**
 * Starts uart reception
 */
void start_ins_usart(void) {
	HAL_UART_Receive_IT(&huart1, sync_buf, 1);  // Receive 1 byte to start the sync sequence
}


/**
 * Implements the HAL_UART_RxCpltCallback to parse INS data
 *
 * This essentially just enforces synchronization between the
 * OEM7600 and the ProcessLogTask thread by using the sync bytes
 * sent by the receiver along with the header information in order to ensure
 * we receive the correct amount of data.
 *
 * It receives single bytes using usart interrupt until we know the log
 * header and body length, then we receive the rest using DMA.
 *
 * This callback is also called once the DMA reception is complete, at which
 * point we place the data in an OSMessageQueue for the ProcessLogTask to pull from.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {

	  switch (sync_state) {
      case SYNC_BYTE_1_STATE:
        if (sync_buf[0] == SYNC_BYTE_1) {
          message_buf[received_bytes++] = SYNC_BYTE_1;
          sync_state = SYNC_BYTE_2_STATE;
        }
        HAL_UART_Receive_IT(&huart1, sync_buf, 1);
        break;

      case SYNC_BYTE_2_STATE:
        if (sync_buf[0] == SYNC_BYTE_2) {
          message_buf[received_bytes++] = SYNC_BYTE_2;
          sync_state = SYNC_BYTE_3_STATE;
        } else {
          received_bytes = 0;
          sync_state = SYNC_BYTE_1_STATE;
        }
        HAL_UART_Receive_IT(&huart1, sync_buf, 1);
        break;

      case SYNC_BYTE_3_STATE:
        if (sync_buf[0] == SYNC_BYTE_3_LONG) {
          message_buf[received_bytes++] = SYNC_BYTE_3_LONG;
          sync_state = HEADER_LENGTH_STATE;
          header_type = LONG;
        } else if (sync_buf[0] == SYNC_BYTE_3_SHORT) {
          message_buf[received_bytes++] = SYNC_BYTE_3_SHORT;
          sync_state = MESSAGE_LENGTH_STATE;
          header_type = SHORT;
          header_length = SHORT_HEADER_LENGTH; // Short headers have constant length
        } else {
          received_bytes = 0;
          sync_state = SYNC_BYTE_1_STATE;
        }
        HAL_UART_Receive_IT(&huart1, sync_buf, 1);
        break;

      case HEADER_LENGTH_STATE:
        header_length = sync_buf[0];
        message_buf[received_bytes++] = header_length;
        header_type = LONG;
        sync_state = MESSAGE_ID_STATE;

        HAL_UART_Receive_IT(&huart1, sync_buf, 2);
        break;

      case  MESSAGE_LENGTH_STATE:
        switch(header_type) {
          case SHORT:
            message_length = sync_buf[0];
            message_buf[received_bytes++] = message_length;
            sync_state = MESSAGE_ID_STATE;

            HAL_UART_Receive_IT(&huart1, sync_buf, 2);
            break;
          case LONG:
            message_length = (sync_buf[3] << 8) | sync_buf[2];
            memcpy(&message_buf[received_bytes], sync_buf, 4);
            received_bytes += 4;
            sync_state = DMA_STATE;
            uint16_t receive_length = message_length + header_length - received_bytes + CRC_LENGTH;
            HAL_UART_Receive_DMA(
              &huart1,
              &message_buf[received_bytes],
              receive_length
            );
            break;
        }
        break;

      case MESSAGE_ID_STATE:
        message_buf[received_bytes++] = sync_buf[0];
        message_buf[received_bytes++] = sync_buf[1];

        // Once sync bytes are received, message id and length are known, use DMA to receive the rest of the message
        switch(header_type) {
          case SHORT:
            sync_state = DMA_STATE;
            uint16_t receive_length = message_length + header_length - received_bytes + CRC_LENGTH;
            HAL_UART_Receive_DMA(
              &huart1,
              &message_buf[received_bytes],
			  receive_length
			);
            break;
          case LONG:
            sync_state = MESSAGE_LENGTH_STATE; // In long headers, message length is after a 2 more bytes, and is 2 bytes long
            HAL_UART_Receive_IT(&huart1, sync_buf, 4);
            break;
        }
        break;
      case DMA_STATE:
        // Send the buffer to the FreeRTOS queue
        osMessageQueuePut(dataQueueHandle, message_buf, 0, 0);

        // Restart reception
        sync_state = SYNC_BYTE_1_STATE;
        received_bytes = 0;
        header_length = 0;
        message_length = 0;

        HAL_UART_Receive_IT(&huart1, sync_buf, 1);
        break;
    }
  }
}
