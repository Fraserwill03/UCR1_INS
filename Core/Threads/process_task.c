#include "process_task.h"
#include "../vendor_generated/can_tools/ucr_01.h"


/**
 * Message IDs and lengths for UCR-01
 * 
 * Lengths should not include the header length or CRC length
 * -----------------------------------------------------------
 */
#define IMURATEPVAS_ID 1305
#define IMURATEPVAS_LENGTH 88
#define INSPVAS_ID 508
#define INSPVAS_LENGTH 88
#define BESTPOS_ID 42
#define BESTPOS_LENGTH 72
#define BESTGNSSPOS_ID 1429
#define BESTGNSSPOS_LENGTH 72
#define RAWIMUS_ID 325
#define RAWIMUS_LENGTH 40

/**
 * PRIVATE DEFINITIONS
 * 
 * -----------------------------------------------------------
 */
#define CRC32_POLYNOMIAL 0xEDB88320L
#define IMU_STATUS_MASK_ERROR_ALL 0x00000001


/**
 * PRIVATE VARIABLES
 * 
 * -----------------------------------------------------------
 */

osMessageQueueId_t dataQueueHandle;

/* CAN Message Headers Start */

// InsGpsHeader header
FDCAN_TxHeaderTypeDef InsGpsHeader = {
  .Identifier = UCR_01_INS_GPS_FRAME_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_48,
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

FDCAN_TxHeaderTypeDef GnssBestPosHeader = {
  .Identifier = UCR_01_GNSS_BESTPOS_FRAME_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_48,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_ON,
  .FDFormat = FDCAN_FD_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0
};

FDCAN_TxHeaderTypeDef RawImuHeader = {
  .Identifier = UCR_01_RAWIMU_FRAME_ID,
  .IdType = FDCAN_STANDARD_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_48,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_ON,
  .FDFormat = FDCAN_FD_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0
};
/**
 * PRIVATE FUNCTIONS
 * 
 * -----------------------------------------------------------
 */

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


/**
 * PUBLIC FUNCTIONS
 * 
 * -----------------------------------------------------------
 */

//static float gyro_x;
//static float gyro_y;
//static float gyro_z;
//
//static float accel_x;
//static float accel_y;
//static float accel_z;

void ProcessLogTask(void * argument) {
  static uint8_t received_data[MAX_RX_BUF];
  HAL_FDCAN_Start(&hfdcan1);
  uint32_t received_CRC;
  uint32_t calculated_CRC;
  uint32_t header_length;
  uint8_t *ptr = 0;

  while(1) {
    osStatus_t status = osMessageQueueGet(dataQueueHandle, received_data, NULL, 0);

    if (status == osOK) {
      uint16_t message_id = (received_data[5] << 8) | received_data[4];

      switch(message_id) {
        // These are sync and async version of same logs, so same parsing
        case IMURATEPVAS_ID:
        case INSPVAS_ID:
          struct ucr_01_ins_gps_t ins_gps;
          struct ucr_01_ins_imu_t ins_imu;

          memset(&received_CRC, 0, sizeof(received_CRC));
          memcpy(&received_CRC, received_data + SHORT_HEADER_LENGTH + received_data[3], sizeof(received_CRC));

          calculated_CRC = CalculateBlockCRC32(SHORT_HEADER_LENGTH + IMURATEPVAS_LENGTH, received_data);

          if(memcmp(&received_CRC, &calculated_CRC, sizeof(received_CRC)) != 0) {
            // CRC mismatch
            return;
          }

          // Skip header
          ptr = received_data + SHORT_HEADER_LENGTH;

          // TODO: Figure out if cantools can generate packed structs so we can copy chunks
          memcpy(&ins_gps.gnss_week, ptr, sizeof(ins_gps.gnss_week));
          ptr += sizeof(ins_gps.gnss_week);

          memcpy(&ins_gps.gnss_seconds, ptr, sizeof(ins_gps.gnss_seconds));
          ptr += sizeof(ins_gps.gnss_seconds);

          memcpy(&ins_gps.gnss_lat, ptr, sizeof(ins_gps.gnss_lat));
          ptr += sizeof(ins_gps.gnss_lat);

          memcpy(&ins_gps.gnss_long, ptr, sizeof(ins_gps.gnss_long));
          ptr += sizeof(ins_gps.gnss_long);

          memcpy(&ins_gps.gnss_height, ptr, sizeof(ins_gps.gnss_height));
          ptr += sizeof(ins_gps.gnss_height);

          memcpy(&ins_imu.north_vel, ptr, sizeof(ins_imu.north_vel));
          ptr += sizeof(ins_imu.north_vel);

          memcpy(&ins_imu.east_vel, ptr, sizeof(ins_imu.east_vel));
          ptr += sizeof(ins_imu.east_vel);

          memcpy(&ins_imu.up_vel, ptr, sizeof(ins_imu.up_vel));
          ptr += sizeof(ins_imu.up_vel);

          memcpy(&ins_imu.roll, ptr, sizeof(ins_imu.roll));
          ptr += sizeof(ins_imu.roll);

          memcpy(&ins_imu.pitch, ptr, sizeof(ins_imu.pitch));
          ptr += sizeof(ins_imu.pitch);

          memcpy(&ins_imu.azimuth, ptr, sizeof(ins_imu.azimuth));
          ptr += sizeof(ins_imu.azimuth);

          memcpy(&ins_imu.status, ptr, sizeof(ins_imu.status));
          ptr += sizeof(ins_imu.status);

          
          uint8_t ins_gps_data[UCR_01_INS_GPS_LENGTH];
          uint8_t ins_imu_data[UCR_01_INS_IMU_LENGTH];

          ucr_01_ins_gps_pack(ins_gps_data, &ins_gps, UCR_01_INS_GPS_LENGTH);
          ucr_01_ins_imu_pack(ins_imu_data, &ins_imu, UCR_01_INS_IMU_LENGTH);

          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &InsGpsHeader, ins_gps_data) != HAL_OK){
            // TODO: Handle error
          }
          
          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &GnssImuHeader, ins_imu_data) != HAL_OK){
            // TODO: Handle error
          }
          break;
        case BESTGNSSPOS_ID:
          struct ucr_01_gnss_bestpos_t gnss_bestpos;

          memset(&received_CRC, 0, sizeof(received_CRC));
          header_length = received_data[3];
          memcpy(&received_CRC, received_data + header_length + BESTGNSSPOS_LENGTH, sizeof(received_CRC));

          calculated_CRC = CalculateBlockCRC32(header_length + BESTGNSSPOS_LENGTH, received_data);

          if(memcmp(&received_CRC, &calculated_CRC, sizeof(received_CRC)) != 0) {
            // CRC mismatch
            return;
          }

          // This log only has gnss_week and ms in header
          memcpy(&gnss_bestpos.gnss_week, (received_data + 14), sizeof(gnss_bestpos.gnss_week));
          memcpy(&gnss_bestpos.gnss_ms, (received_data + 16), sizeof(gnss_bestpos.gnss_week));

          // Skip header
          ptr = received_data + header_length;

          // Copy remaining data
          memcpy(&gnss_bestpos.sol_stat, ptr, sizeof(gnss_bestpos.sol_stat));
          ptr += sizeof(gnss_bestpos.sol_stat);

          memcpy(&gnss_bestpos.pos_type, ptr, sizeof(gnss_bestpos.pos_type));
          ptr += sizeof(gnss_bestpos.pos_type);

          memcpy(&gnss_bestpos.lat, ptr, sizeof(gnss_bestpos.lat));
          ptr += sizeof(gnss_bestpos.lat);

          memcpy(&gnss_bestpos.lng, ptr, sizeof(gnss_bestpos.lng));
          ptr += sizeof(gnss_bestpos.lng);

          memcpy(&gnss_bestpos.hgt, ptr, sizeof(gnss_bestpos.hgt));
          ptr += sizeof(gnss_bestpos.hgt);
          
          // Skip Undulation and Datum ID
          ptr += 8;

          memcpy(&gnss_bestpos.lat_std_dev, ptr, sizeof(gnss_bestpos.lat_std_dev));
          ptr += sizeof(gnss_bestpos.lat_std_dev);

          memcpy(&gnss_bestpos.long_std_dev, ptr, sizeof(gnss_bestpos.long_std_dev));
          ptr += sizeof(gnss_bestpos.long_std_dev);

          memcpy(&gnss_bestpos.height_std_dev, ptr, sizeof(gnss_bestpos.height_std_dev));
          ptr += sizeof(gnss_bestpos.height_std_dev);

          uint8_t gnss_bestpos_data[UCR_01_GNSS_BESTPOS_LENGTH];
          ucr_01_gnss_bestpos_pack(gnss_bestpos_data, &gnss_bestpos, UCR_01_GNSS_BESTPOS_LENGTH);

          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &GnssBestPosHeader, gnss_bestpos_data) != HAL_OK){
            // TODO: Handle error
          }
          
          break;
        case RAWIMUS_ID:
		  struct ucr_01_rawimu_t rawimu;

          uint16_t crc_index = SHORT_HEADER_LENGTH + RAWIMUS_LENGTH;
          memset(&received_CRC, 0, sizeof(received_CRC));
          memcpy(&received_CRC, received_data + crc_index, sizeof(received_CRC));

          calculated_CRC = CalculateBlockCRC32(SHORT_HEADER_LENGTH + RAWIMUS_LENGTH, received_data);
          
          if(memcmp(&received_CRC, &calculated_CRC, sizeof(received_CRC)) != 0) {
            // CRC mismatch
            return;
          }

          // Skip header
          ptr = received_data + SHORT_HEADER_LENGTH;

          // Copy data
          memcpy(&rawimu.gnss_week, ptr, sizeof(rawimu.gnss_week));
          ptr += sizeof(rawimu.gnss_week);

          memcpy(&rawimu.gnss_seconds, ptr, sizeof(rawimu.gnss_seconds));
          ptr += sizeof(rawimu.gnss_seconds);

          uint32_t imu_status;
          memcpy(&imu_status, ptr, sizeof(imu_status));
          static uint32_t error_ctr;
          if(imu_status & IMU_STATUS_MASK_ERROR_ALL) {
            // IMU sensor failure https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMUSX.htm#G320
        	  error_ctr++;
          }
          // TODO: the status also provides bitfields for whether each accel and gyro has new data or not
          // I don't know if this means that if it is not new they give us zero or if they just don't update the value
          // This is probably more of a post-processing thing anyways... but leaving this note here as we have the option to add handling here

          ptr += sizeof(imu_status);

          memcpy(&rawimu.z_accel, ptr, sizeof(rawimu.z_accel));
          ptr += sizeof(rawimu.z_accel);

          memcpy(&rawimu.neg_y_accel, ptr, sizeof(rawimu.neg_y_accel));
          ptr += sizeof(rawimu.neg_y_accel);

          memcpy(&rawimu.x_accel, ptr, sizeof(rawimu.x_accel));
          ptr += sizeof(rawimu.x_accel);

          memcpy(&rawimu.z_gyro, ptr, sizeof(rawimu.z_gyro));
          ptr += sizeof(rawimu.z_gyro);

          memcpy(&rawimu.neg_y_gyro, ptr, sizeof(rawimu.neg_y_gyro));
          ptr += sizeof(rawimu.neg_y_gyro);

          memcpy(&rawimu.x_gyro, ptr, sizeof(rawimu.x_gyro));
          ptr += sizeof(rawimu.x_gyro);

          uint8_t rawimu_data[UCR_01_RAWIMU_LENGTH];
          ucr_01_rawimu_pack(rawimu_data, &rawimu, UCR_01_RAWIMU_LENGTH);

          if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &RawImuHeader, rawimu_data) != HAL_OK){
            // TODO: Handle error
          }
          
//          gyro_x = rawimu.z_gyro * (0.008/65536)/125;
//          gyro_y = rawimu.neg_y_gyro * (0.008/65536)/125;
//          gyro_z = rawimu.x_gyro * (0.008/65536)/125;
//
//          accel_x = rawimu.x_accel * (0.200/65536)*(9.80665 /1000)/(125);
//          accel_y = rawimu.neg_y_accel * (0.200/65536)*(9.80665 /1000)/(125);
//          accel_z = rawimu.z_accel * (0.200/65536)*(9.80665 /1000)/(125);



          break;
        default:
          // Unknown message
          return;
      }
    }
  }
}
