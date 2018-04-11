#include <mbed.h>
#include "Config.hpp"
#include "Macro.h"
#include "Libraries/SBUS/SBUS.hpp"
#include "Libraries/Sensor/Sensor.hpp"
#include "Libraries/MPU6050/MPU6050.hpp"
#include "Libraries/Mecanum/Mecanum.hpp"
#include "Libraries/Steering/Steering.hpp"
#include "Libraries/RS485/RS485.hpp"

DigitalOut led1(LED1);

DigitalOut ledR(PC_1);
DigitalOut ledG(PC_0);
DigitalOut ledB(PB_0);

// Serial xbee(PC_12, PD_2); // Serial5
RawSerial xbee(USBTX, USBRX);

MPU6050 mpu(PB_9, PB_8);  // I2C1
SBUS propo(PC_10, PC_11); // Serial3

Sensor sensor(PC_6, PC_7); // Serial3

// RS485 rs485(PA_9, PA_10); // Serial1
RawSerial port1(PA_9, PA_10); // Serial1

Mecanum mecanum;
Steering steer;

Timer deltaTimer;

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
double sensor[2] = {0};
double sensorAve = 0;
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

bool availableSensor = false;

int main()
{
    xbee.baud(baud::XBEE);

    propo.begin();

    sensor.begin();

    port1.baud(baud::RS485);
    rs485.begin(baud::RS485);

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

        yawAngle::sensor[0] = degrees(atan((sensor.getVal(1) - sensor.getVal(0)) / 100.0));
        yawAngle::sensor[1] = degrees(atan((sensor.getVal(2) - sensor.getVal(1)) / 100.0));

        if (abs(yawAngle::sensor[1] - yawAngle::sensor[0]) < 10.0)
        {
            yawAngle::sensorAve = (yawAngle::sensor[0] + yawAngle::sensor[1]) / 2;
            distance::toFence = (sensor.getVal(1) + length::SENSOR_TO_CENTER) * cos(radians(yawAngle::sensorAve));

            if (distance::toFence < 2500)
            {
                availableSensor = true;
            }
            else
            {
                availableSensor = false;
            }
        }
        else
        {
            availableSensor = false;
        }

        setBodyVelocity();

        if (propo.getSwitchVal(1) == 0)
        {
            yawAngle::target = 0;
            if (availableSensor)
            {
                int gain = 0.25 * (800 - distance::toFence);
                gain = constrain(gain, -50, 50);
                velocity::body[0] += (abs(gain) > 5) ? gain : 0;

                gain = angleGain::SONIC_P * (yawAngle::target - yawAngle::sensorAve);
                velocity::body[2] += (abs(gain) > 5) ? gain : 0;
            }
            else
            {
                decideAngleCorrection();
            }
            ledB = 1;
        }
        else
        {
            setTargetAngle();
            decideAngleCorrection();
            ledB = 0;
        }

        velocity::body[2] = sign(velocity::body[2]) * map(abs(velocity::body[2]), 0, 255, 40, 170);

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

        sendDataToMD();

        debug();
        led1 = !led1;
        ledG = availableSensor;
        ledR = (abs(yawAngle::sensorAve - yawAngle::now) < 10);
        wait(cycle::LOOP);
    }
}

double sign(double A)
{
    return (A > 0) - (A < 0);
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
    xbee.printf("%.3lf\t", yawAngle::sensorAve);
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
    for (int i = 0; i < 3; i++)
        velocity::body[i] = (abs(propo.getStickVal(i)) > 7) ? propo.getStickVal(i) : 0;

    velocity::body[2] *= -1;
}

void setTargetAngle()
{
    if (abs(propo.getStickVal(2)) > 7)
    {
        yawAngle::target = yawAngle::now;
        // int temp = (int)(yawAngle::now / 15);
        // yawAngle::target = temp * 15;
    }
}

void resetAngles()
{
    if (propo.getSwitchVal(0) != 0)
    {
        yawAngle::now = 0;
        yawAngle::target = 0;
    }
}

void decideAngleCorrection()
{
    int gain;
    gain = angleGain::GYRO_P * (yawAngle::target - yawAngle::now);

    velocity::body[2] += (abs(gain) > 10) ? gain : 0;
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

    // before
    // int motorTermOutput[2][2] = {
    //     {velocity::wheel[0], velocity::wheel[1]},
    //     {velocity::wheel[2], velocity::wheel[3]}};
    // rs485.set(1, motorTermOutput[0], 2);
    // rs485.send(1);
    // rs485.set(2, motorTermOutput[1], 2);
    // rs485.send(2);
}
