/*Use #define MPU6050_ES before you include this file if you have an engineering sample (older EVBs will have them), to find out if you have one, check your accelerometer output.
If it is half of what you expected, and you still are on the correct planet, you got an engineering sample
*/


#ifndef MPU6050_H
#define MPU6050_H

/**
 * Includes
 */
#include "mbed.h"
#include "Arduino.h"

/**
 * Defines
 */
#ifndef MPU6050_ADDRESS
#define MPU6050_ADDRESS              0x68  // address pin low (GND), default for InvenSense evaluation board
#endif

#ifdef MPU6050_ES
#define DOUBLE_ACCELERO
#endif

/**
 * Registers
 */
#define MPU6050_CONFIG_REG         0x1A
#define MPU6050_GYRO_CONFIG_REG    0x1B
#define MPU6050_ACCELERO_CONFIG_REG    0x1C

#define MPU6050_INT_PIN_CFG        0x37

#define MPU6050_ACCEL_XOUT_H_REG   0x3B
#define MPU6050_ACCEL_YOUT_H_REG   0x3D
#define MPU6050_ACCEL_ZOUT_H_REG   0x3F

#define MPU6050_TEMP_H_REG         0x41

#define MPU6050_GYRO_XOUT_H_REG    0x43
#define MPU6050_GYRO_YOUT_H_REG    0x45
#define MPU6050_GYRO_ZOUT_H_REG    0x47



#define MPU6050_PWR_MGMT_1_REG     0x6B
#define MPU6050_WHO_AM_I_REG       0x75



/**
 * Definitions
 */
#define MPU6050_SLP_BIT             6
#define MPU6050_BYPASS_BIT         1

#define MPU6050_BW_256              0
#define MPU6050_BW_188              1
#define MPU6050_BW_98               2
#define MPU6050_BW_42               3
#define MPU6050_BW_20               4
#define MPU6050_BW_10               5
#define MPU6050_BW_5                6

#define MPU6050_ACCELERO_RANGE_2G   0
#define MPU6050_ACCELERO_RANGE_4G   1
#define MPU6050_ACCELERO_RANGE_8G   2
#define MPU6050_ACCELERO_RANGE_16G  3

#define MPU6050_GYRO_RANGE_250      0
#define MPU6050_GYRO_RANGE_500      1
#define MPU6050_GYRO_RANGE_1000     2
#define MPU6050_GYRO_RANGE_2000     3

class MPU6050
{
public:
    MPU6050(PinName sda, PinName scl);
    void getAccelero( double *data );
    void getGyro( double *data);
    double getTemp( void );
    void setSleepMode( bool state );
    void write( char address, char data);
    char read( char adress);
    void read( char adress, char *data, int length);

private:
    I2C connection;
    long double offset[6];
};



#endif
