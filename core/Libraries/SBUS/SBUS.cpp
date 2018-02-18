#include "mbed.h"
#include "SBUS.hpp"

SBUS::SBUS(PinName tx, PinName rx) : port(tx, rx)
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

void SBUS::begin()
{
    port.baud(57600);
    port.attach(this, &SBUS::eee, Serial::RxIrq);
    timer.attach(this, &SBUS::checkData, 0.005);
}

void SBUS::checkData()
{
    sum = 0;
    for (int i = 0; i <= 8; i++)
    {
        sum ^= data[i];
    }

    if (sum == 255 || sum == 254)
    {
        for (int i = 0; i <= 8; i++)
        {
            checkedData[i] = data[i];
        }
    }
}

int SBUS::isFailsafe()
{
    return checkedData[1];
}

int SBUS::getStickVal(int axis)
{
    return ((((checkedData[2] >> axis) & 1) == 0) ? 1 : -1) * checkedData[axis + 3];
}

int SBUS::getSwitchVal(int num)
{
    if (num == 4)
    {
        return checkedData[2] >> 4;
    }
    else
    {
        return (checkedData[7] >> (num * 2)) & 0b11;
    }
}

void SBUS::eee()
{
    val = port.getc();
    if (counter == 0 && val != 255)
        return;

    data[counter] = val;

    if (counter == 8)
    {
        counter = 0;
        return;
    }

    counter++;
}