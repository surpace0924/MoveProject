#ifndef _RS485_
#define _RS485_

#include "mbed.h"

#define start_signal 0b11111111
#define wait_time 0.0007
#define STX 255

class RS485
{
  public:
    int useDeviceNum;
    int data[31][100];
    int length[31];
    uint8_t id;
    RS485(PinName mbed_tx, PinName mbed_rx);

    void begin(int baudrate);

    void put(int id, int _data[], int _size);

    void put_time(int targetId);

  private:
    RawSerial Port;
    // Ticker timer;

    short flag;
    int counter;
    int m1_data, m2_data, checkSum;
    int ID;

};

#endif