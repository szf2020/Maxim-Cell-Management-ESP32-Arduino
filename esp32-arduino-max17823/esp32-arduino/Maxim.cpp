#include "Maxim.h"

void Maxim::init(char modules, BMS_Data *result) {
  this->data = result;
  data->num_modules = modules;
}
BMS_Data Maxim::read_pack() {
  init_values();                                     // reset min/max
  data->errors[data->num_modules + 1] = INCOMPLETE;  // cleared on successful read, packs +1 are global errors

  // setup scan parameters
  if (!single_scan()) {
    Serial.println("Scan timeout, no readings");
    return *data;  // returns data with flagged error "INCOMPLETE"
  }
  for (char i = 0; i < data->num_modules; i++) {
    data->errors[i] = 0;
    cell_V_cal_slave(i);   // Read cell volts
    block_V_cal_slave(i);  // Read module volts
    read_die_temp(i);      // Read die temp
    read_cell_temp(i);     // Read cell temp
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  calculate_soc();

  do_balance();

#ifdef PACK_VOLTS_DIVIDER
  data->pack_volts = data->pack_volts / PACK_VOLTS_DIVIDER;
#endif
  data->errors[data->num_modules + 1] = 0;
  return *data;
}

void Maxim::init_values() {
  data->cell_mv_min = 0xffff;
  data->cell_mv_max = 0;
  data->cell_temp_min = 256.0;
  data->cell_temp_max = -256.0;
  data->pack_volts = 0.0;
  data->num_bal_cells = 0;
}

int *Maxim::spi_read(char module, char reg) {
  char command = READALL;
  switch (module) {
    case ALL:
      break;
    default:
      command = module << 3 | READDEVICE;
  }
  char read_num = (READALL == command) ? (41) : (7);
  // if certain command -> SPI_commands(X, ....) change X and NULL_XX to suit
  PEC_VALUE = pec.pec_code(3, command, reg, DATA_CHECK);  // PEC calculation for BITS READALL,ADDR_SCANCTRL,DATA_CHECK
  bms_SPI.SPI_commands(7, WR_LD_Q0, 6, command, reg, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  // if (READALL == command) {
  //   SPI_return = bms_SPI.SPI_commands(42, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX,
  //                                     NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX,
  //                                     NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX,
  //                                     NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX,
  //                                     NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  // } else {
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  // }
  // SPI command for Read Load Que
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC
  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, 0x00);
  bms_SPI.SPI_commands(2, READ_RX_STATUS, 0x00);
  return SPI_return;
}

int *Maxim::spi_write(char module, char reg, short reg_data) {
  char command = WRITEALL;
  switch (module) {
    case ALL:
      break;
    default:
      command = module << 3 | WRITEDEVICE;
  }

  char lsb = lowByte(reg_data);
  char msb = highByte(reg_data);
  // if certain command -> SPI_commands(X, ....) change X and NULL_XX to suit
  PEC_VALUE = pec.pec_code(4, command, reg, lsb, msb);                                        // PEC calculation
  bms_SPI.SPI_commands(8, WR_LD_Q0, 0x05, command, reg, lsb, msb, PEC_VALUE, ALIVE_COUNTER);  // only 128
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_commands(6, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC
  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);
  bms_SPI.SPI_commands(2, 0x01, NULL_XX);
  return SPI_return;
}

bool Maxim::single_scan() {
  short reg_val = 0x0001;
  spi_write(ALL, ADDR_SCANCTRL, reg_val);
  bool check = false;
  int counter = 0;
  while ((!check) || (counter > 20)) {
    // check for data samples ready
    vTaskDelay(pdMS_TO_TICKS(5));
    SPI_return = spi_read(ALL, ADDR_SCANCTRL);
    int raw = (SPI_return[3]) + (SPI_return[4] << 8);
    check = (raw & 0x8000) && (raw & 0x2000);  // scandone && dataready
    counter++;
  }
  return check;
}


void Maxim::read_die_temp(char module) {
  const float CONV = (230700.0 / 5029581.0);
  SPI_return = spi_read(module, ADDR_DIAGREG);
  int raw = (SPI_return[3]) + (SPI_return[4] << 8);
  float temp = ((raw >> 2) * CONV) - 273.0;  // Calculation
  data->die_temp[module] = (int16_t)temp;
  if ((int16_t)temp > MAX_SLAVE_TEMP) {
    data->errors[module] |= ERROR_TEMP_HIGH;
  }
}
void Maxim::read_cell_temp(char module) {
  SPI_return = spi_read(module, ADDR_AIN1);
  int raw = (SPI_return[3]) + (SPI_return[4] << 8);
  float temp = ((raw >> 4) - 1680) * 0.03125;
  data->cell_temp[module] = (int16_t)temp;
  if (temp > data->cell_temp_max) {
    data->cell_temp_max = temp;
  }
  if (temp < data->cell_temp_min) {
    data->cell_temp_min = temp;
  }
  if ((int16_t)temp > MAX_SLAVE_TEMP) {
    data->errors[module] |= ERROR_TEMP_HIGH;
  }
}

void Maxim::cell_V_cal_slave(char module) {
  for (char cell_pointer = ADDR_CELL1REG; cell_pointer < ADDR_CELL1REG + CELLS_PER_SLAVE; cell_pointer++) {
    SPI_return = spi_read(module, cell_pointer);
    unsigned int raw = ((SPI_return[3]) + (SPI_return[4] << 8) >> 2);  // local variable

    // important, reset min cell reference for balancing LV ADV val at begining of samplingof first slave

    // +10mv = +33 adc
    if ((0 == module) && (cell_pointer == ADDR_CELL1REG)) {
      min_cell_adc_raw = raw + (short)(BALANCE_MV_HYS * 3.3);
    } else {
      if (min_cell_adc_raw > raw) {
        min_cell_adc_raw = raw;
      }
    }
    float cell_voltage = raw * VOLTAGE_REF / VOLTAGE_REF_HEX;
    // Update min and max cell voltage values
    short cell_mv = (short)(cell_voltage * 1000.0);
    char index = cell_pointer - ADDR_CELL1REG + module * CELLS_PER_SLAVE;

    data->cell_millivolts[index] = cell_mv;
    if (cell_mv < data->cell_mv_min) {
      data->cell_mv_min = cell_mv;
    }
    if (cell_mv > data->cell_mv_max) {
      data->cell_mv_max = cell_mv;
    }
    if (cell_mv < MIN_CELL_MILLIVOLTS) {
      data->errors[module] |= ERROR_CELL_VOLT_LOW;
    }
    if (cell_mv > PANIC_MAX_CELL_MILLIVOLTS) {
      data->errors[module] |= ERROR_CELL_VOLT_HIGH;
    }
  }
}

void Maxim::block_V_cal_slave(char module) {
  SPI_return = spi_read(module, ADDR_BLOCKREG);
  unsigned int raw = (SPI_return[3]) + (SPI_return[4] << 8);  // local variable
  float block_voltage = (raw >> 2) * FULL_SCALE_DCIN / FULL_SCALE_DCIN_HEX;
  data->pack_volts += block_voltage;
  if (block_voltage > MAX_SLAVE_VOLTAGE) {
    data->errors[module] |= ERROR_SLAVE_VOLT_HIGH;
  }
  if (block_voltage < MIN_SLAVE_VOLTAGE) {
    data->errors[module] |= ERROR_SLAVE_VOLT_LOW;
  }
}
void Maxim::block_V_cal_all() {
  SPI_return = spi_read(ALL, ADDR_BLOCKREG);
  char module = 0;
  for (char idx = 3; idx < (data->num_modules * 2); idx = idx + 2) {
    unsigned int raw = (SPI_return[idx]) + (SPI_return[idx + 1] << 8);  // local variable
    float block_voltage = (raw >> 2) * FULL_SCALE_DCIN / FULL_SCALE_DCIN_HEX;
    Serial.printf("M:%d idx:%d blockV:%f", (int)module, (int)idx, block_voltage);
    module++;
  }
}

void Maxim::read_balance() {

  for (int i = 0; i < data->num_modules; i++) {
    if (i % 3 == 0) {
      Serial.println();
    }

    SPI_return = spi_read(i, ADDR_BALSWEN);
    // SPI_return = spi_read(i, ADDR_BALUVSTAT);  // auto mode
    short shunts = SPI_return[3] | (short)SPI_return[4] << 8;
    Serial.printf("# %d -> ", i);
    print_shunts(shunts);
  }
  Serial.println();
}

void Maxim::calc_balance_bits(char module) {
  // 2. Host determines which cells to balance and associated balancing time.
  data->balance_bits[module] = 0;  // Clear all balance status
  bits_remainder[module] = 0;      // clear all

  for (char cell = 0; cell < CELLS_PER_SLAVE; cell++) {
    char index = module * CELLS_PER_SLAVE + cell;
    short cell_mv = data->cell_millivolts[index];
    if (cell_balance_conditions(data->cell_mv_min, cell_mv)) {
      // If this cell is within the hysteresis of the lowest cell and above the minimum balance voltage threshold then mark it for balancing.

      data->balance_bits[module] |= 1 << cell;
      data->num_bal_cells++;
    }
  }
}

void Maxim::do_balance() {
  for (int balance_counter = 0; balance_counter < 3; balance_counter++) {
    if (balance_counter == 2) {
      SPI_return = spi_read(ALL, ADDR_WATCHDOG);
      short raw = SPI_return[3] | (short)SPI_return[4] << 8;
      char cbtimer = ((raw & 0xF00) >> 8);
      if (cbtimer < 3) {
        spi_write(ALL, ADDR_WATCHDOG, 0x2500);  // Bal watchdog reset
      }
      spi_write(ALL, ADDR_BALSWEN, OFF);  // turn off shunts for voltage measurement
      return;
    }
    unsigned short bits = 0;
    // Counter is cyclical over 1 second and is generated in the main lool (0->3). Each step represents 250ms.
    if (data->cell_mv_max < BALANCE_MV_THRESHOLD) {  // Whole pack
      Serial.printf("Balancing off - Cells under threshold %dvM", BALANCE_MV_THRESHOLD);
      for (char module = 0; module < data->num_modules; module++) {
        data->balance_bits[module] = 0;  // Clear all balance status
        bits_remainder[module] = 0;      // clear all
      }
      spi_write(ALL, ADDR_BALSWEN, OFF);
      return;
    }

    for (char module = 0; module < data->num_modules; module++) {
      if (balance_counter == 0) {
        calc_balance_bits(module);
      }
      if (data->die_temp[module] > 100) {  // Check module die overtemp
        Serial.printf("SHUNT OVERTEMP! IC temp: %dºC  Switching off module %d shunts | ", data->die_temp[module], module + 1);
        data->balance_bits[module] = 0;  // Clear all balance status
        bits_remainder[module] = 0;      // clear all
      }
      bits = toggle_adjacent_bits(data->balance_bits[module], &bits_remainder[module]);
      spi_write(module, ADDR_BALSWEN, bits);
    }
    vTaskDelay(pdMS_TO_TICKS(SHUNT_ON_TIME_MS));
  }
}

void Maxim::validate_cell_delta() {
  if ((data->cell_mv_max - data->cell_mv_min) > DELTA_CELL_MILLIVOLTS_MAX) {
    data->errors[data->num_modules + 1] |= ERROR_CELL_DELTA_HIGH;
  }
}

void cell_debug(BMS_Data *local_result) {
  char buffer[1024];
  for (char module = 0; module < local_result->num_modules; module++) {
    // Start the module line
    int len = snprintf(buffer, sizeof(buffer), "Slave: #%d ", module + 1);

    // Add cell information for this module
    for (char cell = 0; cell < CELLS_PER_SLAVE; cell++) {
      char index = module * CELLS_PER_SLAVE + cell;
      len += snprintf(buffer + len, sizeof(buffer) - len,
                      "C%d: %dmV B:%d | ",
                      cell + 1,
                      local_result->cell_millivolts[index],
                      (local_result->balance_bits[module] & (1 << cell)) > 0 ? 1 : 0);

      // If buffer is full, send what we have so far
      if (len >= sizeof(buffer)) {
        Serial.print(buffer);
        len = 0;
      }
    }

    // Add temperature information for this module and print the line
    snprintf(buffer + len, sizeof(buffer) - len, "Temp: %d \n", local_result->die_temp[module]);
    Serial.print(buffer);
  }
}

void Maxim::calculate_soc() {
  unsigned int accum = 0;
  char num_cells = data->num_modules * CELLS_PER_SLAVE;
  for (char i = 0; i < num_cells; i++) {
    accum += (unsigned int)data->cell_millivolts[i];
  }
  short voltage = (short)(accum / num_cells);

  int voltages[] = { 4200, 4180, 4150, 4100, 4050, 4000, 3920, 3870, 3820, 3780,
                     3750, 3700, 3670, 3630, 3600, 3570, 3530, 3480, 3420, 3350, 3300 };

  float socs[] = { 100, 95, 90, 85, 80, 75, 70, 65, 60, 55,
                   50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0 };
  int numPoints = sizeof(voltages) / sizeof(voltages[0]);

  // Ensure the voltage is within the range
  if (voltage >= 4200) {
    data->soc = 100.0;
    return;
  }
  if (voltage <= 3300) {
    data->soc = 0.0;
    return;
  }

  // Interpolation
  for (int i = 0; i < numPoints - 1; i++) {
    if (voltage <= voltages[i] && voltage > voltages[i + 1]) {
      // Linear interpolation
      float slope = (socs[i + 1] - socs[i]) / (voltages[i + 1] - voltages[i]);
      float soc = socs[i] + slope * (voltage - voltages[i]);
      data->soc = soc;
      return;
    }
  }
  // In case voltage is exactly at the last point
  data->soc = socs[numPoints - 1];
  return;
}

void Maxim::debug_balance() {
  SPI_return = spi_read(ALL, ADDR_WATCHDOG);
  short raw = SPI_return[3] | (short)SPI_return[4] << 8;
  char cbdiv = ((raw & 0x7000) >> 12);
  char cbtimer = ((raw & 0xF00) >> 8);
  Serial.printf("CBDIV %x | CBTIMER %x", cbdiv, cbtimer);
}

/*

  Automatic cell balancing to a UV threshold is configured by setting the CBMEASEN bits within the BALCTRL register to
  0b11. The UV threshold can be used independently or along side the cell-balancing timer(s) (CBEXP1 or CBEXPn). In
  the case that a timer is programmed, it will serve as a redundant mechanism to ensure that a cell is not overdischarged.

  When all cells have reached the UV threshold, all cell-balancing switches will be disabled, but the cell-balancing timer
  will run until completion. This ensures that the μC can still access the device to confirm balancing operation progress and
  exit status. To use a defined UV threshold, the threshold level must be written to the CBUVTHR in the BALAUTOUVTHR
  register. This register allows for 14 bit values relating to a 305μV LSB.

  Auto Group Mode
  The Auto Group mode performs cell balancing in a controlled manner so that the cells can be discharged as a group for
  a duration and/or to a specific voltage level, as required in the end application. The host initiates an Auto Group mode
  by setting CBMODE to 0b110 (duration is seconds) or 0b111 (duration in minutes), configuring CBEXP1 to the desired
  value (where the LSB = 1 second or minute, respectively), and setting individual BALSWENn bits; a group voltage target
  can also be set using CBUVTHR.
  In Auto Group mode, the balancing switches defined by BALSWEN[n] are automatic controlled through nonoverlapping
  even/odd cycling in accordance with the programmable timer duration (CBEXP1) and/or the undervoltage threshold
  (CBUVTHR). The balancing switch duty cycle can further be controlled using CBDUTY to programmatically set the
  average balancing current. This is indicated as follows:
  Even Cells (2, 4, ... 14):
  BALSWn = BALSWEN[n] & (CBTIMER ≤ CBEXP1) & CBEVEN & ( ((CBMEASEN ==0b11) & (CELLn ≥ CBUVTHR)) |
  (CBMEASEN != 0b11))
  Odd Cells (1, 3, ... 13):
  BALSWn = BALSWEN[n] & (CBTIMER ≤ CBEXP1) & CBODD & ( ((CBMEASEN ==0b11) & (CELLn ≥ CBUVTHR)) |
  (CBMEASEN != 0b11))
  Auto Group modes are identical to Auto Individual modes, except that all timer durations are checked against CBEXP1
  (a single expiration event).


  */

void Maxim::display_error() {
  for (char i = 0; i < data->num_modules; i++) {
    char error = data->errors[i];
    if (error == 0)
      continue;
    Serial.printf("Module %d", i);

    if (error & ERROR_CELL_DELTA_HIGH) {
      Serial.printf("Error: Cell Delta High\n");
    }
    if (error & ERROR_CELL_VOLT_HIGH) {
      Serial.printf("Error: Cell Voltage High\n");
    }
    if (error & ERROR_CELL_VOLT_LOW) {
      Serial.printf("Error: Cell Voltage Low\n");
    }
    if (error & ERROR_TEMP_HIGH) {
      Serial.printf("Error: Temperature High\n");
    }
    if (error & ERROR_SLAVE_VOLT_HIGH) {
      Serial.printf("Error: Slave Voltage High\n");
    }
    if (error & ERROR_SLAVE_VOLT_LOW) {
      Serial.printf("Error: Slave Voltage Low\n");
    }
    if (error & PEC_ERROR) {
      Serial.printf("Error: SPI data PEC Error\n");
    }
    if (error & INCOMPLETE) {
      Serial.printf("Error: Data read incomplete\n");
    }
  }
}

/*
  Helper functions ===============================================
*/

bool data_ready(const BMS_Data *result) {
  return (0 == result->errors[result->num_modules + 1]);
}

bool has_errors(const BMS_Data *result) {
  for (char i = 0; i < result->num_modules + 1; i++) {
    if (result->errors[i] != 0)
      return true;
  }
  return false;
}

inline short toggle_adjacent_bits(const short bits, short *bits_remainder) {
  short result = 0;
  short input = bits;        // mutable copy
  input ^= *bits_remainder;  // use *bits_remainder to get the value
  *bits_remainder = 0;       // reset the value pointed to by bits_remainder

  for (int i = 1; i < 16; i++) {
    int first = ((input >> (i - 1)) & 1) == 1;
    int second = ((input >> i) & 1) == 1;

    if (first && second) {
      result |= 1 << (i - 1);           // keep first
      input ^= 1 << i;                  // drop second
      *bits_remainder |= 1 << (i - 1);  // update mask
    } else {
      if (first) {
        result |= 1 << (i - 1);
      }
      if (second) {
        result |= 1 << i;
      }
    }
  }
  return result;
}

inline bool cell_balance_conditions(short min_cell, short cell_mv) {
  return (cell_mv >= BALANCE_MV_THRESHOLD) && (min_cell < (cell_mv - BALANCE_MV_HYS));
}

void print_shunts(uint16_t value) {
  for (char i = 0; i < CELLS_PER_SLAVE; i++) {
    Serial.printf("%d:%d ", i + 1, (value & (1 << i)) ? 1 : 0);
  }
}

void print_b16(uint16_t value) {
  for (char i = 0; i < 16; i++) {
    Serial.printf("%d", (value & (1 << i)) ? 1 : 0);
  }
  Serial.println();
}