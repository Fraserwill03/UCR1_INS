#include "process_task.h"
#include "../vendor_generated/can_tools/ucr_01.h"

osMessageQueueId_t dataQueueHandle;

/* CAN Message Headers Start */

// GnssPosHeader header
FDCAN_TxHeaderTypeDef GnssPosHeader = {
  .Identifier = UCR_01_GNSS_POS_FRAME_ID,
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
  .Identifier = UCR_01_GNSS_IMU_FRAME_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_64,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_ON,
  .FDFormat = FDCAN_FD_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0
};

// BESTVEL header
FDCAN_TxHeaderTypeDef BestvelHeader = {
  .Identifier = UCR_01_GPS_BEST_VEL_FRAME_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_64,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_ON,
  .FDFormat = FDCAN_FD_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0
};
    

void ProcessLogTask(void) {
  static uint8_t received_data[MAX_RX_BUF];

  while(1) {
    osStatus_t status = osMessageQueueGet(dataQueueHandle, received_data, NULL, 0);
    if (status == osOK) {
      uint16_t message_id = (received_data[1] << 8) | received_data[0];

      switch(message_id) {
        case IMURATEPVAS_ID:
          struct ucr_01_gnss_pos_t gnss_pos;
          struct ucr_01_gnss_imu_t gnss_imu;

          // TODO: CRC

          // Skip remainder of header, week and seconds are at beginning of this log as well
          uint8_t *ptr = received_data + SHORT_HEADER_REMAINDER;
          // All fields are in order so just copy the chunk
          memcpy(&gnss_pos, ptr, sizeof(gnss_pos));
          ptr += sizeof(gnss_pos);

          // Again, all fields are in order so just copy the chunk
          memcpy(&gnss_imu, ptr, sizeof(gnss_imu));
          ptr += sizeof(gnss_imu);
          
          uint8_t gnss_pos_data[UCR_01_GNSS_POS_LENGTH];
          uint8_t gnss_imu_data[UCR_01_GNSS_IMU_LENGTH];

          ucr_01_gnss_pos_pack(gnss_pos_data, &gnss_pos, UCR_01_GNSS_POS_LENGTH);
          ucr_01_gnss_imu_pack(gnss_imu_data, &gnss_imu, UCR_01_GNSS_IMU_LENGTH);

          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &GnssPosHeader, gnss_pos_data) != HAL_OK){
            // TODO: Handle error
          }
          
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &GnssImuHeader, gnss_imu_data) != HAL_OK){
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