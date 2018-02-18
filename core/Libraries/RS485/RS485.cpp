#include "RS485.hpp"

RS485::RS485(PinName mbed_tx, PinName mbed_rx) : Port(mbed_tx, mbed_rx)
{
}

void RS485::begin(int baudrate)
{
    Port.baud(baudrate);
    timer.attach(this, &RS485::put_time, wait_time);
}

void RS485::put(int id, int _data[], int _size)
{

    length[id] = _size * 2;
    for (int i = 0; i < _size; i += 2)
    {
        int sign = (((_data[i] > 0) - (_data[i] < 0)) == -1) ? 1 : 0;

        checkSum ^= (data[id][i] = (sign << 6) | (_data[i] >> 7));
        checkSum ^= (data[id][i + 1] = (_data[i] & 0b1111111));
    }
}

void RS485::put_time()
{
    static int targetId = 0;
    if (targetId == 1)
    {
        Port.putc(STX);
        Port.putc(2);
        Port.putc(3);
        Port.putc(4);
        Port.putc(5);
        Port.putc(6);
        Port.putc(7);
        Port.putc(8);
        Port.putc(9);
        Port.putc(10);
        Port.putc(11);
        Port.putc(12);
        Port.putc(13);
        Port.putc(14);
        Port.putc(15);
        Port.putc(16);
        Port.putc(17);
        Port.putc(18);
        Port.putc(19);
        Port.putc(20);
    }
    else
    {
        Port.putc(0);
        Port.putc(2);
        Port.putc(3);
        Port.putc(4);
        Port.putc(5);
        Port.putc(6);
        Port.putc(7);
        Port.putc(8);
        Port.putc(9);
        Port.putc(10);
        Port.putc(11);
        Port.putc(12);
        Port.putc(13);
        Port.putc(14);
        Port.putc(15);
        Port.putc(16);
        Port.putc(17);
        Port.putc(18);
        Port.putc(19);
        Port.putc(20);
    }

    targetId = (targetId == 8) ? 0 : targetId + 1;
}
