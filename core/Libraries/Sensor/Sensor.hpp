#ifndef _SENSOR_
#define _SENSOR_

#define START_BYTE 255
#define DATA_NUM 8
#define CHECKSUM_BIT 8

class Sensor
{
public:
  // constructor
  Sensor(PinName tx, PinName rx);

  // instance
  RawSerial port;

  // variable
  uint8_t data[DATA_NUM];

  uint8_t sum, readData, data_start, byte,
      buffer[DATA_NUM];

    // function
    void begin();
    void checkData();
    int getVal(int num);
    void receiveData();

  private:
};

#endif