#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "Maxim.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "configuration.h"

#define CHARGE 0x35
#define DISCHARGE 0x2b
#define IDLE 0x2b

static const char Solax_Fox_Init_Data[8] = { 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const char Solax_Fox_Version_Data[8] = { 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const char Solax_Fox_Pack_Data[8] = { 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 };
static const char Solax_Fox_TimeStamp[2] = { 0x03, 0x06 };
static char fox_counter = 0;    
static char num_fox_slaves = 0; 
static bool announce = false;
static BMS_Data local_result;
static signed long CANmilliamps;

class CanBus {
public:
  char canread(twai_message_t rxFrame, QueueHandle_t tx_queue, BMS_Data *result);
  void summary(BMS_Data *result);

private:
  void CAB300(twai_message_t frame);
  void SAMSUNGSDI(twai_message_t frame);
  void BMS_ComLV(BMS_Data *result, QueueHandle_t tx_queue);
  void BMS_ComHV_Announce(BMS_Data *result, QueueHandle_t tx_queue);
  void BMS_ComHV_Pack_Data(BMS_Data *result, QueueHandle_t tx_queue);
  void BMS_ComHV_Data(BMS_Data *result, QueueHandle_t tx_queue);
  void BMS_ComHV_Version(BMS_Data *result, QueueHandle_t tx_queue);
  void BMS_ComHV_Cell_Data(BMS_Data *result, QueueHandle_t tx_queue);
  void BMS_ComHV_Temp_Data(BMS_Data *result, QueueHandle_t tx_queue);
};

bool filter_data_array(const char *arr1, const char *arr2, size_t length);
short max_charge(const BMS_Data *result);
short max_discharge(const BMS_Data *result);
void send_to_queue(QueueHandle_t tx_queue, twai_message_t *tx_msg);
#endif