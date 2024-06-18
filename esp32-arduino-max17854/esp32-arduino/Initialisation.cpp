#include "Initialisation.h"

void Initialisation::Arduino_SPI_init() {
  digitalWrite(SHDNL_MAX17841_1, LOW);
  delay(100);
  Serial.begin(BAUDRATE);
  Serial.println("Test code only");
  pinMode(SHDNL_MAX17841_1, OUTPUT);     // _SHDN pin on MAX17841 - high is active - low shutsdown ISO-UART bus
  pinMode(SS1, OUTPUT);                  // Setting the Slave Select Pin as Output (custom)
  digitalWrite(SS1, HIGH);               // disable Slave Select Pin
  digitalWrite(SHDNL_MAX17841_1, HIGH);  // Enable MAX17841
  delay(2);

  SPI.begin(sck, miso, mosi, -1);                                         // Initializing the SPI in Micro-controller with CS disabled
  SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));  // Initializing SPI frequency, SPI mode
}

int Initialisation::MAX178X_init_sequence(char slave_cells) {
  wakeup();                                      // Wake up instructions for start up
  int num_modules = helloall();                  // Send Hello all command
  clear_status_fmea();                           // Clears status and FMEA
  enable_measurement(slave_cells, num_modules);  // SPI commands to enable measurement
  return num_modules;
}

void Initialisation::wakeup() {
  bms_SPI.SPI_commands(2, WRITE_CONFIGURATION_3, KEEP_ALIVE_160_MICRO_SEC);  // Write the configuration 3 register and set keep-alive period to 160us
  bms_SPI.SPI_commands(2, WRITE_RX_INTERRUPT_ENABLE, 0x88);                  // Write Rx_interrupt enable register and set RX_Error_int_enable and RX_Overflow_INT_Enable bits
  bms_SPI.SPI_commands(1, CLR_RX_BUF);                                       // Clear Receive Buffer
  bms_SPI.SPI_commands(2, WRITE_CONFIGURATION_2, 0x30);                      // WRITE_CONFIGURATION_2 and Enable transmit preambles mode
  bms_SPI.SPI_commands(1, READ_RX_STATUS, NULL_XX);                          // Read RX_Status regiter and check for RX_Status for 0x21, Otherwise repeat transactions
  bms_SPI.SPI_commands(2, WRITE_CONFIGURATION_2, 0x10);                      // WRITE_CONFIGURATION_2 and Disable transmit preambles mode
  bms_SPI.SPI_commands(1, READ_RX_STATUS);                                   // Read RX_Status register
  bms_SPI.SPI_commands(1, CLR_TX_BUF);                                       // Clear transmit buffer
  bms_SPI.SPI_commands(1, CLR_RX_BUF);                                       // Clear receive buffer
}

int Initialisation::helloall() {
  int check = 0;
  Serial.print("Scanning ISO bus for slaves");
  while (check == 0) {
    delay(50);
    bms_SPI.SPI_commands(5, WR_LD_Q0, 0x03, HELLOALL, NULL_XX, NULL_XX);  // Send the Helloall command to initialize the address of the BMS daizy chain
    bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);                                // Write Next queue
    SPI_return = bms_SPI.SPI_commands(6, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
    bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);  // Read RX_interrupt flags
    bms_SPI.SPI_commands(2, READ_RX_STATUS, NULL_XX);           // Read RX status
    check = SPI_return[3];
    Serial.print('.');
  }
  Serial.println();
  if (check > 32) {
    Serial.printf("Too many slaves detected: %d \n", (int)check);
    Serial.println("ESP32 will reset in 3 seconds...");
    delay(3000);
    ESP.restart();  // Restart the ESP32
  };
  Serial.print("Number of slaves in daisy chain is : ");
  Serial.println(SPI_return[3]);
  return check;
}

void Initialisation::clear_status_fmea() {
  // SPI TRANSACTION : WRITE ADDR_STATUS1 TO 0X00 TO CLEAR FLAGS

  PEC_VALUE = pec.pec_code(4, WRITEALL, ADDR_STATUS1, 0x00, 0x00);  // PEC calculation for BITS WRITEALL, ADDR_STATUS1N, 0x00, 0x00
  bms_SPI.SPI_commands(8, WR_LD_Q0, 0x06, WRITEALL, ADDR_STATUS1, 0x00, 0x00, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);  // Write Next queue
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC

  // SPI TRANSACTION : READ ADDR_STATUS1 FOR RETURNED RESULTS

  PEC_VALUE = pec.pec_code(3, READALL, ADDR_STATUS1, DATA_CHECK);  // PEC calculation for BITS READALL,ADDR_STATUS1,DATA_CHECK
  bms_SPI.SPI_commands(7, WR_LD_Q0, 0x06, READALL, ADDR_STATUS1, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);  // SPI command for Read Load Que
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);                                                               // Checks the calculated and hardware returned PEC
  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);                                                        // Read RX_interrupt flags
  bms_SPI.SPI_commands(2, READ_RX_STATUS, NULL_XX);                                                                 // Read RX status

  // SPI TRANSACTION : WRITE ADDR_FMEA1 TO 0X00 TO CLEAR FLAGS

  PEC_VALUE = pec.pec_code(4, WRITEALL, ADDR_FMEA1, 0x00, 0x00);  // PEC calculation for BITS WRITEALL, ADDR_FMEA1, 0x00, 0x00
  bms_SPI.SPI_commands(8, WR_LD_Q0, 0x06, WRITEALL, ADDR_FMEA1, 0x00, 0x00, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);  // Write Next queue
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC

  // SPI TRANSACTION : READ ADDR_FMEA1 FOR RETURNED RESULTS

  PEC_VALUE = pec.pec_code(3, READALL, ADDR_FMEA1, DATA_CHECK);  // PEC calculation for BITS READALL,ADDR_FMEA1,DATA_CHECK
  bms_SPI.SPI_commands(7, WR_LD_Q0, 0x06, READALL, ADDR_FMEA1, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);

  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);         // Checks the calculated and hardware returned PEC
  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);  // Read RX_interrupt flags

  bms_SPI.SPI_commands(2, READ_RX_STATUS, NULL_XX);  // Read RX status

  // SPI TRANSACTION : WRITE ADDR_FMEA2 TO 0X00 TO CLEAR FLAGS

  PEC_VALUE = pec.pec_code(4, WRITEALL, ADDR_FMEA2, 0x00, 0x00);  // PEC calculation for BITS WRITEALL, ADDR_FMEA2,0x00, 0x00
  bms_SPI.SPI_commands(8, WR_LD_Q0, 0x06, WRITEALL, ADDR_FMEA2, 0x00, 0x00, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC

  // SPI TRANSACTION : READ ADDR_FMEA2 FOR RETURNED RESULTS

  PEC_VALUE = pec.pec_code(3, READALL, ADDR_FMEA2, DATA_CHECK);  // PEC calculation for BITS READALL,ADDR_MEASUREEN,DATA_CHECK
  bms_SPI.SPI_commands(7, WR_LD_Q0, 0x06, READALL, ADDR_FMEA2, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);  // SPI command for Read Load Que
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);                                                               // Checks the calculated and hardware returned PEC

  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);  // Read RX_interrupt flags
  bms_SPI.SPI_commands(2, READ_RX_STATUS, NULL_XX);           // Read RX status
}

void Initialisation::enable_measurement(char slave_cells, char modules) {
  // SPI TRANSACTION : WRITE MEASURE_ENABLE1 WITH ALL ENABLE ie 0xFF 0xFF
  // Limit the maximum ammout of cells to be read in each slave.
  // reg_val as bits represents each cell ADC
  // ie. 0b000000000111111 meausures the first 6 cells

  uint16_t reg_val = 0;
#ifdef CELL_CONFIGURATION
  reg_val = CELL_CONFIGURATION;
#else
  reg_val = (1 << slave_cells) - 1;  // 4 cells = 0b000000001111
#endif
  reg_val |= (1 << 14);  // add block volts (total slave voltage) read bit - Page 255 of MAX17854

  PEC_VALUE = pec.pec_code(4, WRITEALL, ADDR_MEASUREEN1, lowByte(reg_val), highByte(reg_val));  // PEC calculation for BITS WRITEALL, ADDR_MEASUREEN1, 0xFF, 0xFF
  bms_SPI.SPI_commands(8, WR_LD_Q0, 0x06, WRITEALL, ADDR_MEASUREEN1, lowByte(reg_val), highByte(reg_val), PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);  // Write Next queue
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);  // Checks the calculated and hardware returned PEC

  PEC_VALUE = pec.pec_code(3, READALL, ADDR_MEASUREEN1, DATA_CHECK);  // PEC calculation for BITS READALL,ADDR_MEASUREEN,DATA_CHECK
  bms_SPI.SPI_commands(7, WR_LD_Q0, 0x06, READALL, ADDR_MEASUREEN1, DATA_CHECK, PEC_VALUE, ALIVE_COUNTER);
  bms_SPI.SPI_commands(1, WR_NXT_LD_Q0);
  SPI_return = bms_SPI.SPI_commands(8, RD_NXT_MSG, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX, NULL_XX);
  PEC_check_status = pec.PEC_Check(1, 5, SPI_return);         // Checks the calculated and hardware returned PEC
  bms_SPI.SPI_commands(2, READ_RX_INTERRUPT_FLAGS, NULL_XX);  // Read RX_interrupt flags
  bms_SPI.SPI_commands(2, READ_RX_STATUS, NULL_XX);           // Read RX status
  delay(1);
}
