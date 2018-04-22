#ifndef _PS3_
#define _PS3_

#include "mbed.h"
#include "Macro.h"

#define START_BYTE 255
#define DATA_NUM 13
#define CHECKSUM_BIT 13

class PS3
{
public:
  // constructor
  PS3(PinName tx, PinName rx);

  // instance
  RawSerial port;

  // variable
  uint8_t data[DATA_NUM];

  uint8_t sum, readData, data_start, byte,
      buffer[DATA_NUM];

  // function
  void begin();
  int isFailsafe();
  int getStickVal(int axis);
  int getAnalogButtonVal(int num);
  int getButtonVal(int num);
  void receiveData();

private:
};

#endif