#ifndef PEC_H
#define PEC_H
#include "Arduino.h"
#include <stdarg.h>

extern int PEC_VALUE;
extern int PEC_check_status;

class PEC
{
public:
  int pec_code(int n, ...);
  bool PEC_Check(int Start, int End, int return_value[]);
};
extern PEC pec;
#endif
