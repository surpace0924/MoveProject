#include "mbed.h"
#include "Sensor.hpp"

Sensor::Sensor(PinName tx, PinName rx) : port(tx, rx)
{
    counter = 0;
    sum = 0;
    val = 0;
    for (int i = 0; i < 9; i++)
    {
        data[i] = 0;
        checkedData[i] = 0;
    }
}

void Sensor::begin()
{
    port.baud(57600);
    port.attach(this, &Sensor::eee, Serial::RxIrq);
    timer.attach(this, &Sensor::checkData, 0.005);
}

void Sensor::checkData()
{
    sum = 0;
    for (int i = 0; i <= 7; i++)
    {
        sum ^= data[i];
    }

    if (sum == 255 || sum == 254)
    {
        for (int i = 0; i <= 7; i++)
        {
            checkedData[i] = data[i];
        }
    }
}

int Sensor::getVal(int num)
{
    if (num == 0)
        return ((checkedData[1] << 7) | checkedData[2]);

    else if (num == 1)
        return ((checkedData[3] << 7) | checkedData[4]);

    else if (num == 2)
        return ((checkedData[5] << 7) | checkedData[6]);
}

void Sensor::eee()
{
    val = port.getc();
    if (counter == 0 && val != 255)
        return;

    data[counter] = val;

    if (counter == 7)
    {
        counter = 0;
        return;
    }

    counter++;
}