#include "RS485.hpp"

RS485::RS485(PinName mbed_tx, PinName mbed_rx) : Port(mbed_tx, mbed_rx)
{
}

void RS485::begin(int baudrate)
{
    Port.baud(baudrate);
    // timer.attach(this, &RS485::put_time, wait_time);
}

void RS485::put(int id, int _data[], int _size)
{

    length[id] = _size * 2;
    for (int i = 0; i < _size; i++)
    {
        int sign = (((_data[i] > 0) - (_data[i] < 0)) == -1) ? 1 : 0;

        data[id][2 * i] = (sign << 6) | (abs(_data[i]) >> 7);
        data[id][2 * i + 1] = (abs(_data[i]) & 0b01111111);
    }
}

void RS485::put_time(int targetId)
{
    checkSum = 0;

    Port.putc(STX);
    Port.putc(targetId);
    Port.putc(length[targetId]);

    for (int i = 0; i < length[targetId] / 2; i++)
    {
        checkSum ^= Port.putc(data[targetId][2 * i]);
        checkSum ^= Port.putc(data[targetId][2 * i + 1]);
    }

    Port.putc(checkSum);
}
