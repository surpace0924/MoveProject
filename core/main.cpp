#include <mbed.h>
#include "Config.hpp"
#include "Macro.h"
#include "Libraries/SBUS/SBUS.hpp"
#include "Libraries/MPU6050/MPU6050.hpp"
#include "Libraries/Mecanum/Mecanum.hpp"
#include "Libraries/RS485/RS485.hpp"

DigitalOut led1(LED1);

RawSerial xbee(PC_12, PD_2); // Serial5
// Serial pc(USBTX, USBRX);

MPU6050 mpu(PB_9, PB_8);  // I2C1
SBUS propo(PC_10, PC_11); // Serial3
RS485 rs485(PA_9, PA_10); // Serial1

Mecanum mecanum;

Timer deltaTimer;

void debug();
void calculateYawAngle(double gyroZ);

namespace yawAngle
{
double now = 0;
double target = 0;
}

namespace velocity
{
int body[3] = {0};
int wheel[4] = {0};
}

int main()
{
    xbee.baud(baud::XBEE);

    propo.begin();
    rs485.begin(baud::RS485);

    deltaTimer.start();

    led1 = 0;

    while (1)
    {
        double acc[3] = {0};
        double gyro[3] = {0};
        mpu.getAccelero(acc);
        mpu.getGyro(gyro);
        calculateYawAngle(gyro[2]);

        for (int i = 0; i < 3; i++)
            velocity::body[i] = (abs(propo.getStickVal(i)) > 4) ? propo.getStickVal(i) / 2.5 : 0;

        if (abs(propo.getStickVal(2)) > 4)
        {
            yawAngle::target = yawAngle::now;
        }

        if ((yawAngle::target - yawAngle::now) > 180)
        {
            velocity::body[2] += 1.2 * (((yawAngle::target - 360) - yawAngle::now));
        }
        else if ((yawAngle::target - yawAngle::now) < -180)
        {
            velocity::body[2] += 1.2 * ((yawAngle::target - (yawAngle::now - 360)));
        }
        else
        {
            velocity::body[2] += 1.2 * (yawAngle::target - yawAngle::now);
        }

        mecanum.calculate(velocity::body, 254, yawAngle::target, velocity::wheel);

        int motorTermOutput[2][2] = {
            {velocity::wheel[0], velocity::wheel[1]},
            {velocity::wheel[2], velocity::wheel[3]}};

        rs485.set(1, motorTermOutput[0], 2);
        rs485.send(1);

        rs485.set(2, motorTermOutput[1], 2);
        rs485.send(2);

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
        xbee.printf("%d\t", velocity::body[i]);

    xbee.printf("%.3lf\t", yawAngle::now);
    // xbee.printf("%.3lf\t", yawAngle::target);

    for (int i = 0; i < 4; i++)
        xbee.printf("%d\t", velocity::wheel[i]);

    xbee.printf("\n");
}

void calculateYawAngle(double gyroZ)
{
    static double lastGyroZ = 0;

    if (abs(gyroZ) < 0.3)
        gyroZ = 0;

    yawAngle::now += ((gyroZ + lastGyroZ) * (long double)((deltaTimer.read_us()) / (2.0 * 1000000.0)));

    // 前回値とする
    deltaTimer.reset();
    lastGyroZ = gyroZ;

    // 角度は(0 <= angle < 360)
    yawAngle::now += (yawAngle::now < 0) ? 360 : 0;
    yawAngle::now -= (yawAngle::now >= 360) ? 360 : 0;
}
