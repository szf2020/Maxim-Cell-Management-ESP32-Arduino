
#ifndef ESP32_ARDUINO_H
#define ESP32_ARDUINO_H

// #define MAX17823 // Samsung SDI - comment out for MAX17852

// LIBRARY INCLUDES

#include <SPI.h>
#include <stdarg.h>
#include "Maxim.h"
#include "Initialisation.h"
#include "CAN_config.h"
#include <esp_task_wdt.h>


// #define MAX17823
#define EXAMPLE_TAG "TWAI Alert and Recovery"
#define WDT_TIMEOUT_MS 60000 // 1 minute
#define PRECHARGE 1
#define MAIN 2
twai_message_t rxFrame;

esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Bitmask of all cores
    .trigger_panic = true,
};

int modules = 0;
int PEC_VALUE = 0;
int PEC_check_status = 0;
int *SPI_return = 0;


Maxim maxim;
Initialisation initialisation;
PEC pec;
BMS_SPI bms_SPI;

void TWAI_Task(void *pvParameters);
void TWAI_Processing_Task(void *pvParameters);
void SPI_Task(void *pvParameters);
void interrupt_pin();
#endif
