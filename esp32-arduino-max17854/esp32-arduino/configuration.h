#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h" // For GPIO matrix signal index

#define PRECHARGE_PIN 9
#define MAIN_CONTACTOR_PIN 9
#define sck 5
#define miso 6
#define mosi 4
#define SS1 7               // chip select for MAX17841
#define SS2 7               // Optional Slave select if Dual UART is used
#define INT_MAX17841_1 8    // Shut down pin for MAX17841
#define SHDNL_MAX17841_1 10 // Shut down pin for MAX17841

#define CAN_TX_PIN 0
#define CAN_RX_PIN 1

#define TX_TASK_PRIO 9
#define CTRL_TASK_PRIO 10
#define ERR_DELAY_US 40  // Approximate time for arbitration phase at 25KBPS
#define ERR_PERIOD_US 16 // Approximate time for two bits at 25KBPS
#define EXAMPLE_TAG "TWAI Alert and Recovery"

#define CAB300a 0x3c0
#define CAB300b 0x3c1
#define CAB300c 0x3c2
#define SAMSUNG_SDI 0x10
#define FOX 0x1871
#define LV_INVERTER 0x305

#define PRECHARGE_DWELL_TIME_MS 200 // time between activating precharge and main contactors
// #define CAN_TX_INTERSPACE_MS 1 // time between subsequent transmitted CAN frames

// #define CANRX_DEBUG true  //
// #define CANTX_DEBUG true  //
// #define BALANCE_DEBUG true          //
#define BALANCING_CELLS_AS_KWH true // report balancing cells to FoxESS data instead of remaining kWh
#define AUTO_BALANCE true           // balancing method for MAX17845 slaves - 0 = legacy method

#if CONFIG_IDF_TARGET_ESP32C6
#define TWAI_TX_SIGNAL_IDX TWAI0_TX_IDX
#else
#define TWAI_TX_SIGNAL_IDX TWAI_TX_IDX
#endif


// Pack configuration
// #define CELL_CONFIGURATION CELL1 | CELL2 | CELL3 | CELL4 | CELL8 | CELL9 | CELL10 | CELL11 // ZOE 2021 modular = 1-4, 8-11
#define CELLS_PER_SLAVE 6
#define MAX_CELLS 108 // Cells per slave x Slaves

#define PANIC_MAX_CELL_MILLIVOLTS 4250 // opens contactors
#define MAX_CELL_MILLIVOLTS 4240  // shuts down charging
#define MIN_CELL_MILLIVOLTS 3300  // opens contactors
#define DELTA_CELL_MILLIVOLTS_MAX 100
#define MAX_SLAVE_VOLTAGE 26 // 6 cells per pack
#define MIN_SLAVE_VOLTAGE 20 // 6 cells per pack
#define BALANCE_MV_THRESHOLD 3700
#define BALANCE_MV_HYS 10
#define SLAVE_KWH 2.73
// #define PACK_VOLTS_DIVIDER 1 // Slaves are summed, this value divides the total

// These values halt charging and discharging
#define MAX_SOC 100
#define MIN_SOC 5

#define MAX_CHARGE 30    // rate in amps - desired W/min pack volts
#define MAX_DISCHARGE 30 // rate in amps


#define CELL1 (1<<1)
#define CELL2 (1<<2)
#define CELL3 (1<<3)
#define CELL4 (1<<4)
#define CELL5 (1<<5)
#define CELL6 (1<<6)
#define CELL7 (1<<7)
#define CELL8 (1<<8)
#define CELL9 (1<<9)
#define CELL10 (1<<10)
#define CELL11 (1<<11)
#define CELL12 (1<<12)



#endif