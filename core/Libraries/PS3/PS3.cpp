#include "mbed.h"
#include "PS3.hpp"


PS3::PS3(PinName tx, PinName rx) : port(tx, rx)
{
    sum = 0;
    for (int i = 0; i < 13; i++)
    {
        data[i] = 0;
    }
}

void PS3::begin()
{
    port.baud(57600);
    port.attach(this, &PS3::receiveData, Serial::RxIrq);
}

int PS3::isFailsafe()
{
    return data[1];
}

int PS3::getStickVal(int num)
{
    // 符号を比較
    if (((buffer[2] >> num) & 1) == 0)
        return buffer[num + 3];
    else
        return -buffer[num + 3];
}

int PS3::getAnalogButtonVal(int num)
{
    return buffer[num + 7];
}

int PS3::getButtonVal(int num)
{
    if (num <= 6 && num >= 0)
    {
        return (buffer[9] >> num) & 1;
    }
    else if (num <= 13)
    {
        return (buffer[10] >> (num - 7)) & 1;
    }
    else if (num <= 15)
    {
        return (buffer[11] >> (num - 14)) & 1;
    }
    else
    {
        return 0;
    }
}

void PS3::receiveData()
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
                if (i == 1)
                    continue;

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
