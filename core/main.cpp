#include <mbed.h>
#include "Config.hpp"
#include "Macro.h"
#include "Libraries/PS3/PS3.hpp"
#include "Libraries/MPU6050/MPU6050.hpp"
#include "Libraries/Mecanum/Mecanum.hpp"
#include "Libraries/DebugSerial/DebugSerial.h"
#include "Libraries/Pid/Pid.hpp"
#include "Libraries/Sensor/Sensor.hpp"

PwmOut led1(LED1);

DigitalOut ledR(PC_1);
DigitalOut ledG(PC_0);
DigitalOut ledB(PB_0);

DebugSerial debugger(USBTX, USBRX, 57600); // Serial2

MPU6050 mpu(PB_9, PB_8); // I2C1
PS3 ps3(PB_6, PB_7);     // Serial6

RawSerial port1(PA_11, PA_12); // Serial1
Sensor sensor(USBTX, USBRX);   // Serial3

Mecanum mecanum;

Timer aTimer;
Timer bTimer;

int swVal[8];
DigitalIn sw[8] = {
    DigitalIn(PB_10),
    DigitalIn(PB_4),
    DigitalIn(PB_5),
    DigitalIn(PB_3),
    DigitalIn(PC_0),
    DigitalIn(PC_1),
    DigitalIn(PB_0),
    DigitalIn(PA_4)};

void debug();
void calculateOrientation(double gyroZ);
void setBodyVelocity();
void setTargetOrientation();
void decideOrientationCorrection();
void resetAngles();
void sendDataToMD();

double targetOrientation[3] = {0.};
double Orientation[3] = {0.};

namespace velocity
{
int body[3] = {0};
int wheel[4] = {0};
}

namespace distance
{
double toFence = 0;
}

bool flag = false;
bool bFlag = false;

int main()
{
    for (int i = 0; i < 8; i++)
        sw[i].mode(PullDown);

    debugger.baud(baud::debugger);

    ps3.begin();

    port1.baud(baud::RS485);
    sensor.begin();

    led1 = 0;
    ledR = 0;
    ledG = 0;
    ledB = 0;

    while (1)
    {
        // debugger.cls();
        double acc[3] = {0};
        double gyro[3] = {0};
        mpu.getAccelero(acc);
        mpu.getGyro(gyro);

        for (int i = 0; i < 8; i++)
            swVal[i] = sw[i];

        resetAngles();
        calculateOrientation(gyro[2]);

        if (!ps3.isFailsafe())
        {
            setBodyVelocity();
            setTargetOrientation();
            decideOrientationCorrection();

            targetOrientation[0] = 550.0;
            targetOrientation[1] = 500.0;

            int maxSpeed = 100;
            int maxAccelTime = 500;
            int maxdecelTime = 300;

            int decelTime;

            int lastVelo;

            int decelStartVelo;

            // if (ps3.getButtonVal(6) == 1)
            // {
            //     // 立ち上がり判定
            //     if (flag == false)
            //     {
            //         flag = true;
            //         aTimer.reset();
            //         aTimer.start();
            //     }

            //     if (aTimer.read_ms() < maxAccelTime)
            //     {
            //         velocity::body[0] = maxSpeed * (1 - cos(aTimer.read_ms() * M_PI / maxAccelTime)) / 2;
            //     }
            //     else
            //     {
            //         velocity::body[0] = maxSpeed;
            //     }
            //     lastVelo = velocity::body[0];
            //     bFlag = false;
            // }
            // else
            // {
            //     if (bFlag == false)
            //     {
            //         bFlag = true;
            //         decelStartVelo = lastVelo;
            //         decelTime = (int)((double)decelStartVelo / maxSpeed * maxdecelTime);
            //         bTimer.reset();
            //         bTimer.start();
            //     }

            //     if (bTimer.read_ms() < decelTime)
            //     {
            //         velocity::body[0] = decelStartVelo * ((-(1 - cos(bTimer.read_ms() * M_PI / decelTime)) / 2) + 1);
            //     }
            //     else if (ps3.getButtonVal(12) == 1)
            //     {
            //         velocity::body[0] = maxSpeed;
            //     }
            //     else
            //     {
            //         velocity::body[0] = (abs(ps3.getStickVal(0)) > 20) ? ps3.getStickVal(0) / 3 : 0;
            //     }

            //     flag = false;
            // }

            if (!ps3.isFailsafe() && (velocity::body[0] != 0 || velocity::body[1] != 0 || velocity::body[2] != 0))
            {
                mecanum.calculate(velocity::body, 254, targetOrientation[2], velocity::wheel);
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

            led1 = (double)velocity::body[0] / 100.;
            wait(cycle::LOOP);
        }
    }
}

void debug()
{
    if (ps3.isFailsafe() == true)
        debugger.printf("[NG]\t");
    else
        debugger.printf("[OK]\t");

    // センサ値
    if (swVal[4] == 1)
    {
        debugger.printf("%d\t", sensor.getVal(0));
        debugger.printf("%d\t", sensor.getVal(1));
        debugger.printf("%d\t", sensor.getVal(2));
    }

    // 位置情報
    if (swVal[5] == 1)
    {
        for (int i = 0; i < 3; i++)
            debugger.printf("%4.2lf\t", Orientation[i]);
    }

    // 速度情報
    if (swVal[6] == 1)
    {
        for (int i = 0; i < 3; i++)
            debugger.printf("%d\t", velocity::body[i]);
    }

    if (swVal[7] == 1)
    {
        for (int i = 0; i < 4; i++)
            debugger.printf("%d\t", velocity::wheel[i]);
    }

    for (int i = 7; i >= 0; i--)
        debugger.printf("%d", swVal[i]);

    debugger.printf("\n");
}

void calculateOrientation(double gyroZ)
{
    Orientation[0] = 850 - sensor.getVal(0) * 10;
    Orientation[1] = sensor.getVal(2) * 10;

    static Timer deltaTimer;
    deltaTimer.start();
    static double lastGyroZ = 0;

    if (abs(gyroZ) < 0.4)
        gyroZ = 0;

    Orientation[2] += ((gyroZ + lastGyroZ) * (long double)((deltaTimer.read_us()) / (2.0 * 1000000.0)));

    // 前回値とする
    deltaTimer.reset();
    lastGyroZ = gyroZ;

    // 角度は(-180 < angle <= 180)
    Orientation[2] += (Orientation[2] <= -180) ? 360 : 0;
    Orientation[2] -= (Orientation[2] > 180) ? 360 : 0;
}

void setBodyVelocity()
{
    if (!ps3.isFailsafe())
    {
        for (int i = 0; i < 3; i++)
            velocity::body[i] = (abs(ps3.getStickVal(i)) > 20) ? ps3.getStickVal(i) / 3 : 0;
    }
}

void resetAngles()
{
    if (ps3.getButtonVal(0) != 0)
    {
        Orientation[2] = 0;
        targetOrientation[2] = 0;
    }
}

void setTargetOrientation()
{
    if (abs(ps3.getStickVal(2)) > 20)
    {
        targetOrientation[2] = Orientation[2];
        // int temp = (int)(Orientation[2] / 15);
        // targetOrientation[2] = temp * 15;
    }

    if (ps3.getButtonVal(1))
    {
        targetOrientation[2] = 0;
    }

    if (ps3.getButtonVal(2))
    {
        targetOrientation[2] = -90;
    }

    if (ps3.getButtonVal(3))
    {
        targetOrientation[2] = 180;
    }

    if (ps3.getButtonVal(4))
    {
        targetOrientation[2] = 90;
    }
}

void decideOrientationCorrection()
{
    static Pid xPid(0.1, 0., 0.);
    static Pid yPid(0.1, 0., 0.);
    static Pid anglePid(angleGain::GYRO_P, angleGain::GYRO_I, 0.);

    if (swVal[1] == 1)
    {
        velocity::body[0] += xPid.calculate(targetOrientation[0], Orientation[0]);
    }

    if (swVal[2] == 1)
    {
        velocity::body[1] += yPid.calculate(targetOrientation[1], Orientation[1]);
    }

    if (swVal[3] == 1)
    {
        velocity::body[2] += anglePid.calculate(targetOrientation[2], Orientation[2]);
    }

    for (int i = 0; i < 3; i++)
    {
        if (velocity::body[2] > 254)
        {
            velocity::body[2] = 254;
        }
        else if (velocity::body[2] < -254)
        {
            velocity::body[2] = -254;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// 2017MDと通信するための関数
// どうせ新しくなるだろうから即席で作った
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

    // 角度
    for (int i = 0; i < 4; i++)
        packet[i + 10] = 0;

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
