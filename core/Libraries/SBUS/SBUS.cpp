#include "mbed.h"
#include "SBUS.hpp"

RawSerial pc(USBTX, USBRX);

SBUS::SBUS(PinName tx, PinName rx) : port(tx, rx)
{
    pc.baud(57600);
    counter = 0;
    sum = 0;
    val = 0;
    for (int i = 0; i < 13; i++)
    {
        data[i] = 0;
        // data[i] = 0;
    }
    data[5] = 150;
}

void SBUS::begin()
{
    port.baud(57600);
    port.attach(this, &SBUS::receiveData, Serial::RxIrq);
    // timer.attach(this, &SBUS::checkData, 0.01);
}

// void SBUS::checkData()
// {
//     sum = 0;
//     for (int i = 0; i <= 12; i++)
//     {
//         if (i == 1)
//         {
//             continue;
//         }
//         sum ^= data[i];
//     }

//     if (sum == 255 || sum == 254)
//     {
//         for (int i = 0; i <= 12; i++)
//         {
//             data[i] = data[i];
//         }
//     }
// }

int SBUS::isFailsafe()
{
    return data[1];
}


int SBUS::getStickVal(int num)
{
    // 符号を比較
    if (((buffer[2] >> num) & 1) == 0)
        return buffer[num + 3];
    else
        return -buffer[num + 3];
}

int SBUS::getAnalogButtonVal(int num)
{
    return buffer[num + 7];
}

int SBUS::getSwitchVal(int num)
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

void SBUS::receiveData()
{
    read = port.getc();
    if (read == start_byte)
    {
        data_start = 1;
        checksum = 0;
        byte = 1;
        buffer[0] = read;
    }
    else
    {
        if (data_start == 1)
        {
            buffer[byte] = read;
            byte++;
        }

        if (byte >= checksum_bit)
        {
            int i;
            for (i = 0; i < checksum_bit; i++)
            {
                if (i == 1)
                    continue;

                sum ^= buffer[i];
            }
            // pc.printf("%d\t%d\t%d\n", sum, buffer[4], data[4]);
            if (sum == 255 || sum == 254)
            {
                for (i = 0; i < 13; i++)
                {
                    // data[i] = buffer[i];
                }
                // check();
            }
            byte = 0;
            data_start = 0;
        }
    }
}
