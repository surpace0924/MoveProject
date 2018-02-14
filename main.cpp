#include <mbed.h>
#include "Macro/Macro.h"
#include "SBUS/SBUS.hpp"
#include "MPU6050/MPU6050.hpp"

namespace cycle
{
const double DEBUG_OUTPUT = 0.010;
const double LOOP = 0.010;
}

DigitalOut led1(LED1);
MPU6050 mpu(PB_9, PB_8);
SBUS propo(PC_10, PC_11);
Serial xbee(PC_12, PD_2);
Serial pc(USBTX, USBRX);

Timer deltaTimer;
Ticker debugOutputTimer;

void calculateRobotAngle(double gyroZ);
void debug();

namespace robotAngle
{
double now = 0;
double target = 0;
}

int main()
{
    xbee.baud(57600);
    deltaTimer.start();

    debugOutputTimer.attach(&debug, cycle::DEBUG_OUTPUT);

    led1 = 0;

    while (1)
    {
        double acc[3] = {0};
        double gyro[3] = {0};
        mpu.getAccelero(acc);
        mpu.getGyro(gyro);

        calculateRobotAngle(gyro[2]);

        if (abs(propo.getStickVal(2)) > 10)
        {
            robotAngle::target = robotAngle::now;
        }

        led1 = !led1;
        wait(cycle::LOOP);
    }
}

void debug()
{
    if (propo.failSafe == true)
        xbee.printf("[NG]\t");
    else
        xbee.printf("[OK]\t");

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
