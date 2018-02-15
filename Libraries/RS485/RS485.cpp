#include "RS485.hpp"

RS485::RS485(PinName mbed_tx, PinName mbed_rx) : Port(mbed_tx,mbed_rx)
{
}
 
void RS485::begin(int baudrate)
{
    Port.baud(baudrate);
    timer.attach(this, &RS485::put_time, wait_time);
}
 
void RS485::put(short id, float m1_duty, float m2_duty)
{
    if (flag == 0) {
        ID = id;
        if (m1_duty < 0) {
            m1_data = 0b10000000;
        } else {
            m1_data = 0;
        }
        if (m2_duty < 0) {
            m2_data = 0b10000000;
        } else {
            m2_data = 0;
        }
        m1_data += fabs(m1_duty) * 100;
        m2_data += fabs(m2_duty) * 100;
 
        check_sum = (id + m1_data + m2_data) & 0b01111111;
    }
    flag = 1;
}
 
void RS485::put_time()
{
    switch (counter) {
        case 0 :
            Port.putc(start_signal);
            break;
        case 1 :
            Port.putc(ID);
            break;
        case 2 :
            Port.putc(m1_data);
            break;
        case 3 :
            Port.putc(m2_data);
            break;
        case 4 :
            Port.putc(check_sum);
            break;
        default :
            break;
    }
    counter ++;
    if(counter >= 5) {
        counter = 0;
        flag = 0;
    }
}
 