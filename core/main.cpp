#include <mbed.h>
#include "Config.hpp"
#include "Macro.h"
#include "Libraries/SBUS/SBUS.hpp"
#include "Libraries/MPU6050/MPU6050.hpp"
#include "Libraries/Mecanum/Mecanum.hpp"
#include "Libraries/Steering/Steering.hpp"
#include "Libraries/RS485/RS485.hpp"

#define MOTOR_MAXPOWER 250

// 回転方向系
#define FREE 0
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3

// 方向の算出
#define directions(output) ((output == 0) ? (BRAKE) : ((output > 0) ? (FORWARD) : (BACKWARD)))

DigitalOut led1(LED1);

RawSerial xbee(PC_12, PD_2); // Serial5
// Serial pc(USBTX, USBRX);

MPU6050 mpu(PB_9, PB_8);  // I2C1
SBUS propo(PC_10, PC_11); // Serial3
// RS485 rs485(PA_9, PA_10); // Serial1
RawSerial port1(PA_9, PA_10); // Serial1

Mecanum mecanum;
Steering steer;

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

int arg[4] = {0};

int main()
{
    xbee.baud(baud::XBEE);

    propo.begin();
    port1.baud(baud::RS485);
    // rs485.begin(baud::RS485);

    deltaTimer.start();

    led1 = 0;

    while (1)
    {
        double acc[3] = {0};
        double gyro[3] = {0};
        mpu.getAccelero(acc);
        mpu.getGyro(gyro);

        if (propo.getSwitchVal(0) != 0)
        {
            yawAngle::now = 0;
            yawAngle::target = 0;
        }

        calculateYawAngle(gyro[2]);

        for (int i = 0; i < 3; i++)
            velocity::body[i] = (abs(propo.getStickVal(i)) > 10) ? propo.getStickVal(i) : 0;

        velocity::body[2] *= -1;

        if (abs(propo.getStickVal(2)) > 10)
        {
            yawAngle::target = yawAngle::now;
        }

        int gain;

        if ((yawAngle::target - yawAngle::now) > 180)
        {
            gain = 3.6 * (((yawAngle::target - 360) - yawAngle::now));
        }
        else if ((yawAngle::target - yawAngle::now) < -180)
        {
            gain = 3.6 * ((yawAngle::target - (yawAngle::now - 360)));
        }
        else
        {
            gain = 3.6 * (yawAngle::target - yawAngle::now);
        }

        velocity::body[2] += (abs(gain) > 20) ? gain : 0;

        // mecanum.calculate(velocity::body, 254, yawAngle::target, velocity::wheel);

        if (!propo.isFailsafe() && (velocity::body[0] != 0 || velocity::body[1] != 0 || velocity::body[2] != 0))
        {
            steer.calculate(velocity::body, 254, yawAngle::target, velocity::wheel, arg);
        }
        else
        {
            arg[0] = arg[2] = 45;
            arg[1] = arg[3] = 135;
            for (int i = 0; i < 4; i++)
            {
                velocity::wheel[i] = 0;
            }
        }

        // int motorTermOutput[2][2] = {
        //     {velocity::wheel[0], velocity::wheel[1]},
        //     {velocity::wheel[2], velocity::wheel[3]}};

        // rs485.set(1, motorTermOutput[0], 2);
        // rs485.send(1);

        // rs485.set(2, motorTermOutput[1], 2);
        // rs485.send(2);

        unsigned char packet[15] = {0}; // 送信データ（パケット）
        int sum = 0;
        int i, j = 0;

        int absPwm[4] = {0};
        for (int i = 0; i < 4; i++)
            absPwm[i] = abs(velocity::wheel[i]);

        packet[0] = 255; // スタートバイト
        packet[1] = 12;  // データのバイト長

        i = 2;
        j = 0;
        while (i <= 8)
        {
            packet[i++] = directions(velocity::wheel[j]);
            packet[i++] = absPwm[j++];
        }

        for (int i = 0; i < 4; i++)
            packet[i + 10] = arg[i];

        // チェックサムの計算
        for (i = 2; i <= 13; i++)
            sum ^= packet[i];

        // チェックサムが255にならないようにする
        packet[14] = (sum != 255) ? sum : 254;

        for (i = 0; i <= 14; i++)
        {
            port1.putc(packet[i]);
        }
        // Serial.println();

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
    {
        xbee.printf("%d\t", arg[i]);
        xbee.printf("%d\t", velocity::wheel[i]);
    }

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
