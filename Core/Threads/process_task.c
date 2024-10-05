#include "process_task.h"
#include "../vendor_generated/can_tools/ucr_01.h"

#define CRC32_POLYNOMIAL 0xEDB88320L

osMessageQueueId_t dataQueueHandle;

/* CAN Message Headers Start */

// IMURATEPVAS header
static FDCAN_TxHeaderTypeDef ImuratepvasHeader = {
  .Identifier = 0xFFFF,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = 0xFFFF,
  .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
  .BitRateSwitch = FDCAN_BRS_OFF,
  .FDFormat = FDCAN_CLASSIC_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0
};


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
  HAL_FDCAN_Start(&hfdcan1);

  while(1) {
    osStatus_t status = osMessageQueueGet(dataQueueHandle, received_data, NULL, 0);
    if (status == osOK) {
      uint16_t message_id = (received_data[5] << 8) | received_data[4];

      switch(message_id) {
        case IMURATEPVAS_ID:
          // Instantiate CAN types.
		  struct ucr_01_imuratepvas_week_t gnss_week;
		  struct ucr_01_imuratepvas_seconds_t gnss_seconds;
		  struct ucr_01_imuratepvas_lat_t ins_lat;
      struct ucr_01_imuratepvas_long_t ins_long;
      struct ucr_01_imuratepvas_height_t ins_height;
      struct ucr_01_imuratepvas_nvel_t north_vel;
      struct ucr_01_imuratepvas_evel_t east_vel;
      struct ucr_01_imuratepvas_uvel_t up_vel;
      struct ucr_01_imuratepvas_roll_t roll;
      struct ucr_01_imuratepvas_pitch_t pitch;
      struct ucr_01_imuratepvas_azimuth_t azimuth;
      struct ucr_01_imuratepvas_status_t status;


          uint32_t received_CRC;
          memcpy(&received_CRC, received_data + SHORT_HEADER_LENGTH + received_data[3], sizeof(received_CRC));

          uint32_t calculated_CRC = CalculateBlockCRC32(SHORT_HEADER_LENGTH + IMURATEPVAS_LENGTH, received_data);

          if(memcmp(&received_CRC, &calculated_CRC, sizeof(received_CRC)) != 0) {
            // CRC mismatch
            return;
          }

          // Skip header
          uint8_t *ptr = received_data + SHORT_HEADER_LENGTH;

          memcpy(&gnss_week.gnss_week, ptr, sizeof(gnss_week.gnss_week));
          ptr += sizeof(gnss_week.gnss_week);

          memcpy(&gnss_seconds.gnss_seconds, ptr, sizeof(gnss_seconds.gnss_seconds));
          ptr += sizeof(gnss_seconds.gnss_seconds);

          memcpy(&ins_lat.latitude, ptr, sizeof(ins_lat.latitude));
          ptr += sizeof(ins_lat.latitude);

          memcpy(&ins_long.longitude, ptr, sizeof(ins_long.longitude));
          ptr += sizeof(ins_long.longitude);

          memcpy(&ins_height.height, ptr, sizeof(ins_height.height));
          ptr += sizeof(ins_height.height);

          // TODO: Validate that status is correct. It is sent as a 4 byte enum, but the enum values are only 0 through 14,
          // so we should only ever need the first byte...
          memcpy(&north_vel.north_velocity, ptr, sizeof(north_vel.north_velocity));
          ptr += sizeof(north_vel.north_velocity);

          memcpy(&east_vel.east_velocity, ptr, sizeof(east_vel.east_velocity));
          ptr += sizeof(east_vel.east_velocity);

          memcpy(&up_vel.up_velocity, ptr, sizeof(up_vel.up_velocity));
          ptr += sizeof(up_vel.up_velocity);

          memcpy(&roll.roll, ptr, sizeof(roll.roll));
          ptr += sizeof(roll.roll);

          memcpy(&pitch.imu_pitch, ptr, sizeof(pitch.imu_pitch));
          ptr += sizeof(pitch.imu_pitch);

          memcpy(&azimuth.imu_azimuth, ptr, sizeof(azimuth.imu_azimuth));
          ptr += sizeof(azimuth.imu_azimuth);

          memcpy(&status.log_status, ptr, sizeof(status.log_status));
          ptr += sizeof(status.log_status);

          
          uint8_t gnss_week_data[UCR_01_IMURATEPVAS_WEEK_LENGTH];
          uint8_t gnss_seconds_data[UCR_01_IMURATEPVAS_SECONDS_LENGTH];
          uint8_t ins_lat_data[UCR_01_IMURATEPVAS_LAT_LENGTH];
          uint8_t ins_long_data[UCR_01_IMURATEPVAS_LONG_LENGTH];
          uint8_t ins_height_data[UCR_01_IMURATEPVAS_HEIGHT_LENGTH];
          uint8_t north_vel_data[UCR_01_IMURATEPVAS_NVEL_LENGTH];
          uint8_t east_vel_data[UCR_01_IMURATEPVAS_EVEL_LENGTH];
          uint8_t up_vel_data[UCR_01_IMURATEPVAS_UVEL_LENGTH];
          uint8_t roll_data[UCR_01_IMURATEPVAS_ROLL_LENGTH];
          uint8_t pitch_data[UCR_01_IMURATEPVAS_PITCH_LENGTH];
          uint8_t azimuth_data[UCR_01_IMURATEPVAS_AZIMUTH_LENGTH];
          uint8_t status_data[UCR_01_IMURATEPVAS_STATUS_LENGTH];

          ucr_01_imuratepvas_week_pack(gnss_week_data, &gnss_week, UCR_01_IMURATEPVAS_WEEK_LENGTH);
          ucr_01_imuratepvas_seconds_pack(gnss_seconds_data, &gnss_seconds, UCR_01_IMURATEPVAS_SECONDS_LENGTH);
          ucr_01_imuratepvas_lat_pack(ins_lat_data, &ins_lat, UCR_01_IMURATEPVAS_LAT_LENGTH);
          ucr_01_imuratepvas_long_pack(ins_long_data, &ins_long, UCR_01_IMURATEPVAS_LONG_LENGTH);
          ucr_01_imuratepvas_height_pack(ins_height_data, &ins_height, UCR_01_IMURATEPVAS_HEIGHT_LENGTH);
          ucr_01_imuratepvas_nvel_pack(north_vel_data, &north_vel, UCR_01_IMURATEPVAS_NVEL_LENGTH);
          ucr_01_imuratepvas_evel_pack(east_vel_data, &east_vel, UCR_01_IMURATEPVAS_EVEL_LENGTH);
          ucr_01_imuratepvas_uvel_pack(up_vel_data, &up_vel, UCR_01_IMURATEPVAS_UVEL_LENGTH);
          ucr_01_imuratepvas_roll_pack(roll_data, &roll, UCR_01_IMURATEPVAS_ROLL_LENGTH);
          ucr_01_imuratepvas_pitch_pack(pitch_data, &pitch, UCR_01_IMURATEPVAS_PITCH_LENGTH);
          ucr_01_imuratepvas_azimuth_pack(azimuth_data, &azimuth, UCR_01_IMURATEPVAS_AZIMUTH_LENGTH);
          ucr_01_imuratepvas_status_pack(status_data, &status, UCR_01_IMURATEPVAS_STATUS_LENGTH);


          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_WEEK_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_WEEK_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, gnss_week_data) != HAL_OK){
            // TODO: Handle error
          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_SECONDS_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_SECONDS_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, gnss_seconds_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_LAT_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_LAT_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, ins_lat_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_LONG_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_LONG_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, ins_long_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_HEIGHT_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_HEIGHT_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, ins_height_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_NVEL_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_NVEL_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, north_vel_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_EVEL_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_EVEL_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, east_vel_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_UVEL_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_UVEL_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, up_vel_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_ROLL_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_ROLL_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, roll_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_PITCH_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_PITCH_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, pitch_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_AZIMUTH_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_AZIMUTH_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, azimuth_data) != HAL_OK){

          }

          ImuratepvasHeader.Identifier = UCR_01_IMURATEPVAS_STATUS_FRAME_ID;
          ImuratepvasHeader.DataLength = UCR_01_IMURATEPVAS_STATUS_LENGTH;
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ImuratepvasHeader, status_data) != HAL_OK){

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
