#include "process_task.h"

extern osMessageQueueId_t dataQueueHandle;

void ProcessLogTask(void) {
  static uint8_t received_data[MAX_RX_BUF];

  while(1) {
    osStatus_t status = osMessageQueueGet(dataQueueHandle, received_data, NULL, 0);
    if (status == osOK) {
      uint16_t message_id = (received_data[1] << 8) | received_data[0];

      switch(message_id) {
        case IMURATEPVAS_ID:
          uint8_t *ptr = received_data + SHORT_HEADER_REMAINDER;          
        
          imuratepvas_log_t log;
          memcpy(&(log.gnss_week), ptr, sizeof(log.gnss_week));
          ptr += sizeof(log.gnss_week);
          memcpy(&(log.gnss_seconds), ptr, sizeof(log.gnss_seconds));
          ptr += sizeof(log.gnss_seconds);
          memcpy(&(log.latitude), ptr, sizeof(log.latitude));
          ptr += sizeof(log.latitude);
          memcpy(&(log.longitude), ptr, sizeof(log.longitude));
          ptr += sizeof(log.longitude);
          memcpy(&(log.height), ptr, sizeof(log.height));
          ptr += sizeof(log.height);
          memcpy(&(log.north_velocity), ptr, sizeof(log.north_velocity));
          ptr += sizeof(log.north_velocity);
          memcpy(&(log.east_velocity), ptr, sizeof(log.east_velocity));
          ptr += sizeof(log.east_velocity);
          memcpy(&(log.up_velocity), ptr, sizeof(log.up_velocity));
          ptr += sizeof(log.up_velocity);
          memcpy(&(log.roll), ptr, sizeof(log.roll));
          ptr += sizeof(log.roll);
          memcpy(&(log.pitch), ptr, sizeof(log.pitch));
          ptr += sizeof(log.pitch);
          memcpy(&(log.azimuth), ptr, sizeof(log.azimuth));
          ptr += sizeof(log.azimuth);
          memcpy(&(log.status), ptr, sizeof(log.status));
          ptr += sizeof(log.status);
          // TODO: CRC checking
          
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