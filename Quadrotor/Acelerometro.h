


#ifndef ACELEROMETRO_H_
#define ACELEROMETRO_H_
#include <MPU9250.h>
// Vcc-->3.3V
// Gnd-->Gnd
// SDA--> A4
// SCL--> A5

class Acelerometro {
private:
  int status = -1;
  // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
  MPU9250 IMU;

public:
  Acelerometro(TwoWire Wire): IMU(Wire,0x68){};
  void initialize();
  void default_cal();
  void magnetometro_cal();
  void acelerometro_cal();
  void gyroscope_cal();
  void settings();
  float *get_raw_val();
};

#endif /* ACELEROMETRO_H */
