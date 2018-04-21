#ifndef _SBUS_
#define _SBUS_

#include "mbed.h"
#include "Macro.h"

#define start_byte 255
#define data_num 13
#define checksum_bit 13

class SBUS
{
public:
  // constructor
  SBUS(PinName tx, PinName rx);

  // instance
  RawSerial port;
  // Ticker timer;

  // variable
  int data[20];
  int val;
  int counter;
  uint8_t sum;
  // int checkedData[13];

  int read, data_start, checksum, byte,
      buffer[data_num];

  // function
  void begin();
  // void checkData();
  int isFailsafe();
  int getStickVal(int axis);
  int getSwitchVal(int num);
  int getAnalogButtonVal(int num);
  void receiveData();

  void eee();

private:
};

#endif