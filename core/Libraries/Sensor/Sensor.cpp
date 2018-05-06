#include "mbed.h"
#include "Sensor.hpp"

Sensor::Sensor(PinName tx, PinName rx) : port(tx, rx)
{
    sum = 0;
    for (int i = 0; i < DATA_NUM; i++)
    {
        data[i] = 0;
    }
}

void Sensor::begin()
{
    port.baud(57600);
    port.attach(this, &Sensor::receiveData, Serial::RxIrq);
}

void Sensor::checkData()
{
    // sum = 0;
    // for (int i = 0; i <= 7; i++)
    // {
    //     sum ^= data[i];
    // }

    // if (sum == 255 || sum == 254)
    // {
    //     for (int i = 0; i <= 7; i++)
    //     {
    //         data[i] = data[i];
    //     }
    // }
}

int Sensor::getVal(int num)
{
    if (num == 0)
        return ((buffer[1] << 7) | buffer[2]);

    else if (num == 1)
        return ((buffer[3] << 7) | buffer[4]);

    else if (num == 2)
        return ((buffer[5] << 7) | buffer[6]);
}

void Sensor::receiveData()
{
    readData = port.getc();
    if (readData == START_BYTE)
    {
        data_start = 1;
        byte = 1;
        buffer[0] = readData;
    }
    else
    {

        // port.printf("%d\n", buffer[2]);
        if (data_start == 1)
        {
            buffer[byte] = readData;
            byte++;
        }

        if (byte >= CHECKSUM_BIT)
        {
            int i;
            for (i = 0; i < CHECKSUM_BIT; i++)
            {
                sum ^= buffer[i];
            }
            // if (sum == 255 || sum == 254)
            // {
            //     for (i = 0; i < 13; i++)
            //     {
            //         data[i] = buffer[i];
            //     }
            // }
            byte = 0;
            data_start = 0;
        }
    }
}
