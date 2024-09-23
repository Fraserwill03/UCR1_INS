#include "process_task.h"
#include "../vendor_generated/can_tools/ucr_01.h"

#define CRC32_POLYNOMIAL 0xEDB88320L

osMessageQueueId_t dataQueueHandle;

/* CAN Message Headers Start */

// GnssPosHeader header
FDCAN_TxHeaderTypeDef GnssPosHeader = {
  .Identifier = UCR_01_INS_GPS_FRAME_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_64,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_ON,
  .FDFormat = FDCAN_FD_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0
};

// GnssImuHeader header
FDCAN_TxHeaderTypeDef GnssImuHeader = {
  .Identifier = UCR_01_INS_IMU_FRAME_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_64,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_ON,
  .FDFormat = FDCAN_FD_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0
};

// TODO: BESTVEL header
//FDCAN_TxHeaderTypeDef BestvelHeader = {
//  .Identifier = UCR_01_GPS_BEST_VEL_FRAME_ID,
//  .IdType = FDCAN_STANDARD_ID,
//  .TxFrameType = FDCAN_DATA_FRAME,
//  .DataLength = FDCAN_DLC_BYTES_64,
//  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
//  .BitRateSwitch = FDCAN_BRS_ON,
//  .FDFormat = FDCAN_FD_CAN,
//  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
//  .MessageMarker = 0
//};
//

/* --------------------------------------------------------------------------

Calculate a CRC value to be used by CRC calculation functions.

-------------------------------------------------------------------------- */

unsigned long CRC32Value(int i) {

  int j;

  unsigned long ulCRC;

  ulCRC = i;

  for ( j = 8 ; j > 0; j-- ) {

    if ( ulCRC & 1 )

      ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;

    else

      ulCRC >>= 1;

  }

  return ulCRC;

}

 

/* --------------------------------------------------------------------------

Calculates the CRC-32 of a block of data all at once

ulCount - Number of bytes in the data block

ucBuffer - Data block

-------------------------------------------------------------------------- */

unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char *ucBuffer ) {

  unsigned long ulTemp1;

  unsigned long ulTemp2;

  unsigned long ulCRC = 0;

  while ( ulCount-- != 0 ) {

    ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;

    ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xFF );

    ulCRC = ulTemp1 ^ ulTemp2;

  }

  return( ulCRC );

}


void ProcessLogTask(void * argument) {
  static uint8_t received_data[MAX_RX_BUF];

  while(1) {
    osStatus_t status = osMessageQueueGet(dataQueueHandle, received_data, NULL, 0);
    if (status == osOK) {
      uint16_t message_id = (received_data[5] << 8) | received_data[4];

      switch(message_id) {
        case IMURATEPVAS_ID:
          struct ucr_01_ins_gps_t ins_gps;
          struct ucr_01_ins_imu_t ins_imu;

          uint32_t received_CRC;
          memcpy(&received_CRC, received_data + SHORT_HEADER_LENGTH + received_data[3], sizeof(received_CRC));

          uint32_t calculated_CRC = CalculateBlockCRC32(SHORT_HEADER_LENGTH + IMURATEPVAS_LENGTH, received_data);

          if(memcmp(&received_CRC, &calculated_CRC, sizeof(received_CRC)) != 0) {
            // CRC mismatch
            return;
          }

          // Skip header
          uint8_t *ptr = received_data + SHORT_HEADER_LENGTH;

          // All fields are in order so just copy the chunk
          memcpy(&ins_gps, ptr, sizeof(ins_gps));
          ptr += sizeof(ins_gps);

          // Copy all in order.
          // TODO: Validate that status is correct. It is sent as a 4 byte enum, but the enum values are only 0 through 14,
          // so we should only ever need the first byte...
          memcpy(&ins_imu, ptr, sizeof(ins_imu));
          ptr += sizeof(ins_imu);
          
          uint8_t ins_gps_data[UCR_01_INS_GPS_LENGTH];
          uint8_t ins_imu_data[UCR_01_INS_IMU_LENGTH];

          ucr_01_ins_gps_pack(ins_gps_data, &ins_gps, UCR_01_INS_GPS_LENGTH);
          ucr_01_ins_imu_pack(ins_imu_data, &ins_imu, UCR_01_INS_IMU_LENGTH);

          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &GnssPosHeader, ins_gps_data) != HAL_OK){
            // TODO: Handle error
          }
          
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &GnssImuHeader, ins_imu_data) != HAL_OK){
            // TODO: Handle error
          }
          break;
        case BESTVEL_ID:
          // TODO: Implement bestvel parsing
          break;
        default:
          // Unknown message
          return;
      }
    }
  }
}
