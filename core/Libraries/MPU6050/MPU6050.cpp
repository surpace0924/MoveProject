#include "MPU6050.hpp"

MPU6050::MPU6050(PinName sda, PinName scl) : connection(sda, scl)
{
    this->setSleepMode(false);
    this->write(0x1C, 0x18);
    this->write(0x1B, 0x18);


    char temp[6];
    
    for (int i = 0; i < 6; i++)
        offset[i] = 0;
    
    for (int i = 0; i < 1000; i++)
    {
        this->read(MPU6050_ACCEL_XOUT_H_REG, temp, 6);
        offset[0] += (int)(short)((temp[0]<<8) + temp[1]) / 2048.0 * 9.81;
        offset[1] += (int)(short)((temp[2]<<8) + temp[3]) / 2048.0 * 9.81;
        offset[2] += (int)(short)((temp[4]<<8) + temp[5]) / 2048.0 * 9.81;
        
        this->read(MPU6050_GYRO_XOUT_H_REG, temp, 6);
        offset[3] += (int)(short)((temp[0]<<8) + temp[1]) / 16.4;
        offset[4] += (int)(short)((temp[2]<<8) + temp[3]) / 16.4;
        offset[5] += (int)(short)((temp[4]<<8) + temp[5]) / 16.4;
        
        wait_ms(1);
    }
    
    for (int i = 0; i < 6; i++)
        offset[i] /= 1000;
}

void MPU6050::write(char address, char data)
{
    char temp[2];
    temp[0]=address;
    temp[1]=data;

    connection.write(MPU6050_ADDRESS * 2,temp,2);
}

char MPU6050::read(char address)
{
    char retval;
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, &retval, 1);
    return retval;
}

void MPU6050::read(char address, char *data, int length)
{
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, data, length);
}

void MPU6050::setSleepMode(bool state)
{
    char temp;
    temp = this->read(MPU6050_PWR_MGMT_1_REG);
    if (state == true)
        temp |= 1<<MPU6050_SLP_BIT;
    if (state == false)
        temp &= ~(1<<MPU6050_SLP_BIT);
    this->write(MPU6050_PWR_MGMT_1_REG, temp);
}


void MPU6050::getAccelero( double *data )
{
    char temp[6];
    this->read(MPU6050_ACCEL_XOUT_H_REG, temp, 6);
    data[0] = (int)(short)((temp[0]<<8) + temp[1]) / 2048.0 * 9.81 - offset[0];
    data[1] = (int)(short)((temp[2]<<8) + temp[3]) / 2048.0 * 9.81 - offset[1];
    data[2] = (int)(short)((temp[4]<<8) + temp[5]) / 2048.0 * 9.81 - offset[2];
}

void MPU6050::getGyro( double *data )
{
    char temp[6];
    this->read(MPU6050_GYRO_XOUT_H_REG, temp, 6);
    data[0] = (int)(short)((temp[0]<<8) + temp[1]) / 16.4 - offset[3];
    data[1] = (int)(short)((temp[2]<<8) + temp[3]) / 16.4 - offset[4];
    data[2] = (int)(short)((temp[4]<<8) + temp[5]) / 16.4 - offset[5];
}

double MPU6050::getTemp( void )
{
    short retval;
    char data[2];
    this->read(MPU6050_TEMP_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (retval+12421.0)/340.0;
}

