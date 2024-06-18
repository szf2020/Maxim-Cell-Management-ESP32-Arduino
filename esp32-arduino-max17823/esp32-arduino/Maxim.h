#ifndef MAXIM_H
#define MAXIM_H
#include "Arduino.h"
#include "PEC.h"
#include "bms_SPI.h"
#include "Max17841.h"
#include "configuration.h"
#include "Max17823.h"


// ADC consts
#define FULL_SCALE_DCIN 60.0
#define FULL_SCALE_DCIN_HEX 0x4000
#define VOLTAGE_REF 5.0
#define VOLTAGE_REF_HEX 0x4000

#define MAX_SLAVES 32
#define MAX_SLAVE_TEMP 100  // Maxim die temp

#define ERROR_CELL_DELTA_HIGH 1 << 0
#define ERROR_CELL_VOLT_HIGH 1 << 1
#define ERROR_CELL_VOLT_LOW 1 << 2
#define ERROR_TEMP_HIGH 1 << 3
#define ERROR_SLAVE_VOLT_HIGH 1 << 4
#define ERROR_SLAVE_VOLT_LOW 1 << 5
#define PEC_ERROR 1 << 6
#define INCOMPLETE 1 << 7

#define highVoltageCutOff 454  // 108 cells at 100% SoC
#define lowVoltageCutOff 367   // 108 cells at 0% SoC

typedef struct
{
  char errors[MAX_SLAVES + 1];
  char num_modules;
  char num_bal_cells;
  long milliamps;
  uint16_t cell_mv_min;
  uint16_t cell_mv_max;
  float cell_temp_min;
  float cell_temp_max;
  float pack_volts;
  float soc;
  uint16_t cell_millivolts[MAX_CELLS];  // Array to hold up to 108 cells
  int16_t die_temp[MAX_SLAVES];         // Array to hold MAX17823 IC temperatures
  int16_t cell_temp[MAX_SLAVES];         // Array to hold MAX17823 IC temperatures
  uint16_t balance_bits[MAX_SLAVES];    // Array to hold all cells to be balanced
} BMS_Data;

class Maxim {
  short bits_remainder[32];  // Manual balancing algo: for keeping track of alternating bit pattern
  char balance_counter = 0;
  unsigned short min_cell_adc_raw = 0x3fff;  // Automatic balancing algo, lowest of all slaves
  BMS_Data *data;

public:
  void init(char modules, BMS_Data *result);
  BMS_Data read_pack();
  void do_balance();
  void read_balance();
  void debug_balance();
  void display_error();
  void auto_balance();

private:
  int *spi_read(char command, char reg);
  int *spi_write(char module, char reg, short reg_data);
  void init_values();
  bool single_scan();
  void set_die_temp();
  void read_die_temp(char module);
  void read_cell_temp(char module);
  void calc_balance_bits(char module);
  void cell_V_cal();
  void cell_V_cal_slave(char modules);
  void block_V_cal_slave(char modules);
  void block_V_cal_all();
  void calculate_soc();
  void validate_cell_delta();
};

bool has_errors(const BMS_Data *result);
bool data_ready(const BMS_Data *result);
inline short toggle_adjacent_bits(const short bits, short *bits_remainder);
inline void print_shunts(unsigned short value);
inline void print_b16(unsigned short value);
bool cell_balance_conditions(short min_cell, short cell_mv);
void cell_debug(BMS_Data *local_result);

#endif
