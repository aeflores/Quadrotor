// Vcc-->3.3V
// Gnd-->Gnd
// SDA--> A4
// SCL--> A5

#ifndef ACELEROMETRO_H_
#define ACELEROMETRO_H_

#include "MPU9250.h"

class Acelerometro {
  private:
    int status = -1;
    // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
    MPU9250 IMU;
  public:
    Acelerometro();
    void magnetometro_cal();
    void acelerometro_cal();
    void gyroscope_cal();
    void initialize();
    void default_cal();
    void settings();
    void get_raw_val(float Acc_val[3][3]);
};

#endif
