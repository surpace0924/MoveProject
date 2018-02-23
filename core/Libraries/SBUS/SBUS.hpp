#ifndef _SBUS_
#define _SBUS_

#include "mbed.h"
#include "Macro.h"

class SBUS
{
  public:
    // constructor
    SBUS(PinName tx, PinName rx);

    // instance
    RawSerial port;
    Ticker timer;

    // variable
    int data[9];
    int val;
    int counter;
    uint8_t sum;
    int checkedData[9];

    // function
    void begin();
    void checkData();
    int isFailsafe();
    int getStickVal(int axis);
    int getSwitchVal(int num);
    void eee();

  private:
};

#endif