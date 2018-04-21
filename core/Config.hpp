#ifndef _CONFIG_
#define _CONFIG_

#define MOTOR_MAXPOWER 250

// 回転方向系
#define FREE 0
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3

// 方向の算出
#define directions(output) ((output == 0) ? (BRAKE) : ((output > 0) ? (FORWARD) : (BACKWARD)))

namespace cycle
{
const double DEBUG_OUTPUT = 0.010;
const double LOOP = 0.010;
}

namespace baud
{
const int XBEE = 57600;
const int RS485 = 57600;
}

namespace angleGain
{
const double GYRO_P = -1.2;
const double SONIC_P = 1.5;
}

namespace length
{
// [mm]
const double SENSOR_TO_CENTER = 350;
}

#endif
