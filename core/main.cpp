#include <mbed.h>
#include "Config.hpp"
#include "Macro.h"
#include "Libraries/SBUS/SBUS.hpp"
#include "Libraries/MPU6050/MPU6050.hpp"
#include "Libraries/Mecanum/Mecanum.hpp"
#include "Libraries/RS485/RS485.hpp"

DigitalOut led1(LED1);
MPU6050 mpu(PB_9, PB_8);  // I2C1
SBUS propo(PC_10, PC_11); // Serial3
RS485 rs485(PC_6, PC_7);  // Serial6

Serial xbee(PC_12, PD_2); // Serial5
Serial pc(USBTX, USBRX);

Timer deltaTimer;
void debug();

void calculateRobotAngle(double gyroZ);

namespace robotAngle
{
double now = 0;
double target = 0;
}

int main()
{
    xbee.baud(baud::XBEE);

    propo.begin();

    deltaTimer.start();

    rs485.begin(baud::RS485);

    led1 = 0;

    while (1)
    {
        double acc[3] = {0};
        double gyro[3] = {0};
        mpu.getAccelero(acc);
        mpu.getGyro(gyro);

        calculateRobotAngle(gyro[2]);

        // if (abs(propo.getStickVal(2)) > 10)
        // {
        //     robotAngle::target = robotAngle::now;
        // }

        int aaa[4] = {-455, -323, -1, 515};
        rs485.put(1, aaa, 4);
        for (int i = 0; i < 4; i++)
            xbee.printf("%d\t", rs485.data[1][i]);
        rs485.put_time(1);

        debug();
        led1 = !led1;
        wait(cycle::LOOP);
    }
}

void debug()
{
    if (propo.isFailsafe() == true)
        xbee.printf("[NG]\t");
    else
        xbee.printf("[OK]\t");

    for (int i = 0; i < 3; i++)
        xbee.printf("%d\t", propo.getStickVal(i));

    xbee.printf("%.3lf", robotAngle::now);

    xbee.printf("\n");
}

void calculateRobotAngle(double gyroZ)
{
    static double lastGyroZ = 0;

    if (abs(gyroZ) < 0.3)
        gyroZ = 0;

    robotAngle::now += ((gyroZ + lastGyroZ) * (long double)((deltaTimer.read_us()) / (2.0 * 1000000.0)));

    // 前回値とする
    deltaTimer.reset();
    lastGyroZ = gyroZ;

    // 角度は(0 <= angle < 360)
    robotAngle::now += (robotAngle::now < 0) ? 360 : 0;
    robotAngle::now -= (robotAngle::now >= 360) ? 360 : 0;
}
