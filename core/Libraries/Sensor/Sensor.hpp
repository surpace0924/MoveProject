#ifndef _SENSOR_
#define _SENSOR_

#include "mbed.h"
#include "Macro.h"

class Sensor
{
  public:
    // constructor
    Sensor(PinName tx, PinName rx);

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
    int getVal(int num);
    void eee();

  private:
};

#endif