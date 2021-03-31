// Vcc-->3.3V
// Gnd-->Gnd
// SDA--> A4
// SCL--> A5

#ifndef IMU_H_
#define IMU_H_

#include <Wire.h>
#include <MedianFilterLib.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

MedianFilter<int> magxfilt(3), magyfilt(3), magzfilt(3);

class IMU {
  private:
    int magxbias = -59, magybias = 52, magzbias = 106;
    int accxbias = 0, accybias = 0, acczbias = 0;
    int gyroxbias = -47, gyroybias = -50, gyrozbias = 30;


    int magxSF = 45, magySF = 43, magzSF = 39;
    int accxSF = 1, accySF = 1, acczSF = 1;
    int gyroxSF = 1, gyroySF = 1, gyrozSF = 1;
    
    
    
  public:
    void initialize();
    void readsensor(int16_t imusensor[3][3]);
    void applycalibration(int16_t imusensor[3][3]);
};

#endif
