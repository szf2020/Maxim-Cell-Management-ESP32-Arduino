#ifndef INITIALIZATION_H
#define INITIALIZATION_H
#include "Arduino.h"
#include "PEC.h"
#include "BMS_SPI.h"
#include "Max17841.h"
#include "configuration.h"
#include "Max17823.h"

#define BAUDRATE 115200        // Baud rate for Serial UART communication PC to Microcontroller
#define SPI_FREQUENCY 4000000  // SPI frequency

extern int num_modules;
class Initialisation {
public:
  void Arduino_SPI_init();
  int MAX178X_init_sequence(char slave_cells);
  void wakeup();
  int helloall();
  void clear_status_fmea();
  void enable_measurement(char slave_cells, char modules);
};
extern Initialisation initialisation;
#endif
