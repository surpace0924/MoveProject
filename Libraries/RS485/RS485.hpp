#ifndef _RS485_
#define _RS485_
 
#include "mbed.h"
 
#define start_signal    0b11111111
#define wait_time       0.0005
 
class RS485
{
public :
    RS485(PinName mbed_tx, PinName mbed_rx);
 
    void begin(int baudrate);
 
    void put(short id, float m1_duty, float m2_duty);
 
private :
    Serial Port;
    Ticker timer;
 
    short flag;
    int counter;
    int m1_data,m2_data,check_sum;
    int ID;
 
    void put_time();
};
 
#endif