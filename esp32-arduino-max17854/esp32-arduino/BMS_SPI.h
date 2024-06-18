#ifndef BMS_SPI_H
#define BMS_SPI_H
#include <stdarg.h>
#include <SPI.h>
#include "Arduino.h"
#define SS1 7  // Actual slave select in microcontroller

extern int *SPI_return;

class BMS_SPI {
public:
  int *SPI_commands(int num, ...);
};

extern BMS_SPI bms_SPI;

#endif
