#include <mbed.h>
#include "Config.hpp"
#include "Macro.h"
#include "Libraries/PS3/PS3.hpp"
#include "Libraries/Sensor/Sensor.hpp"
#include "Libraries/MPU6050/MPU6050.hpp"
#include "Libraries/Mecanum/Mecanum.hpp"
#include "Libraries/Steering/Steering.hpp"
#include "Libraries/RS485/RS485.hpp"

DigitalOut led1(LED1);

DigitalOut ledR(PC_1);
DigitalOut ledG(PC_0);
DigitalOut ledB(PB_0);

RawSerial xbee(USBTX, USBRX); // Serial2

MPU6050 mpu(PB_9, PB_8); // I2C1
PS3 ps3(PB_6, PB_7);  // Serial6

RawSerial port1(PA_11, PA_12); // Serial1

Mecanum mecanum;
// Steering steer;

Timer deltaTimer;
Timer aTimer;
Timer bTimer;

void debug();
void calculateYawAngle(double gyroZ);
void setBodyVelocity();
void setTargetAngle();
void decideAngleCorrection();
void resetAngles();
void sendDataToMD();

double sign(double A);

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

namespace distance
{
double toFence = 0;
}

int arg[4] = {0};

bool flag = false;
bool bFlag = false;
int main()
{
    xbee.baud(baud::XBEE);

    ps3.begin();

    port1.baud(baud::RS485);

    deltaTimer.start();

    led1 = 0;
    ledR = 0;
    ledG = 0;
    ledB = 0;

    while (1)
    {
        double acc[3] = {0};
        double gyro[3] = {0};
        mpu.getAccelero(acc);
        mpu.getGyro(gyro);

        resetAngles();

        calculateYawAngle(gyro[2]);

        if (!ps3.isFailsafe())
        {
            setBodyVelocity();

            setTargetAngle();
            decideAngleCorrection();

            if (ps3.getButtonVal(6) == 1)
            {
                // 立ち上がり判定
                if (flag == false)
                {
                    flag = true;
                    aTimer.reset();
                    aTimer.start();
                }

                if (aTimer.read_ms() < 500)
                {
                    velocity::body[0] = 100 * (1 - cos(aTimer.read_ms() * M_PI / 500)) / 2;
                }
                else
                {
                    velocity::body[0] = 100;
                }

                bFlag = false;
            }
            else
            {
                if (bFlag == false)
                {
                    bFlag = true;
                    bTimer.reset();
                    bTimer.start();
                }

                if (bTimer.read_ms() < 500)
                {
                    velocity::body[0] = 100 * ((-(1 - cos(bTimer.read_ms() * M_PI / 500)) / 2) + 1);
                }

                flag = false;
                if (ps3.getButtonVal(12) == 1)
                {
                    velocity::body[0] = 255;
                }
            }
            // =$D$1*((-(1-COS(B2*PI()/$E$1))/2)+1)

            if (!ps3.isFailsafe() && (velocity::body[0] != 0 || velocity::body[1] != 0 || velocity::body[2] != 0))
            {
                mecanum.calculate(velocity::body, 254, yawAngle::target, velocity::wheel);
            }
            else
            {
                for (int i = 0; i < 4; i++)
                {
                    velocity::wheel[i] = 0;
                }
            }

            sendDataToMD();

            debug();
            led1 = !led1;
            wait(cycle::LOOP);
        }
    }
}

double sign(double A)
{
    return (A > 0) - (A < 0);
}

void debug()
{
    if (ps3.isFailsafe() == true)
        xbee.printf("[NG]\t");
    else
        xbee.printf("[OK]\t");

    for (int i = 0; i < 3; i++)
        xbee.printf("%d\t", velocity::body[i]);

    xbee.printf("%.3lf\t", yawAngle::now);
    xbee.printf("%.3lf\t", distance::toFence);

    for (int i = 0; i < 4; i++)
    {
        xbee.printf("%d\t", arg[i]);
    }

    for (int i = 0; i < 4; i++)
    {
        xbee.printf("%d\t", velocity::wheel[i]);
    }

    xbee.printf("\n");
}

void calculateYawAngle(double gyroZ)
{
    static double lastGyroZ = 0;

    if (abs(gyroZ) < 0.4)
        gyroZ = 0;

    yawAngle::now += ((gyroZ + lastGyroZ) * (long double)((deltaTimer.read_us()) / (2.0 * 1000000.0)));

    // 前回値とする
    deltaTimer.reset();
    lastGyroZ = gyroZ;

    // 角度は(-180 < angle <= 180)
    yawAngle::now += (yawAngle::now <= -180) ? 360 : 0;
    yawAngle::now -= (yawAngle::now > 180) ? 360 : 0;
}

void setBodyVelocity()
{
    if (!ps3.isFailsafe())
    {
        for (int i = 0; i < 3; i++)
            velocity::body[i] = (abs(ps3.getStickVal(i)) > 20) ? ps3.getStickVal(i) : 0;
    }
}

void setTargetAngle()
{
    if (abs(ps3.getStickVal(2)) > 20)
    {
        yawAngle::target = yawAngle::now;
        // int temp = (int)(yawAngle::now / 15);
        // yawAngle::target = temp * 15;
    }

    if (ps3.getButtonVal(1))
    {
        yawAngle::target = 0;
    }

    if (ps3.getButtonVal(2))
    {
        yawAngle::target = -90;
    }

    if (ps3.getButtonVal(3))
    {
        yawAngle::target = 180;
    }

    if (ps3.getButtonVal(4))
    {
        yawAngle::target = 90;
    }
}

void resetAngles()
{
    if (ps3.getButtonVal(0) != 0)
    {
        yawAngle::now = 0;
        yawAngle::target = 0;
    }
}

void decideAngleCorrection()
{
    int gain;
    gain = angleGain::GYRO_P * (yawAngle::target - yawAngle::now);

    velocity::body[2] += gain;
}

void sendDataToMD()
{
    uint8_t packet[15] = {0}; // 送信データ（パケット）
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
}
