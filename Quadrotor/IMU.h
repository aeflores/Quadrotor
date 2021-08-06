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


class IMU {
  private:
    // int magxbias = -169, magybias = 17, magzbias = 18;
    int magxbias = -173, magybias = 24, magzbias = 15;       
    int accxbias = 0, accybias = 0, acczbias = 0;
    int gyroxbias = -40, gyroybias = -20, gyrozbias = 24;
    // int gyroxbias = 0, gyroybias = 0, gyrozbias = 0;


//    int magxSF = 44, magySF = 43, magzSF = 38;   
//    int magxSF = 42, magySF = 40, magzSF = 38;
    int magxSF = 46, magySF = 43, magzSF = 42;
    int accxSF = 1, accySF = 1, acczSF = 1;
    int gyroxSF = 1, gyroySF = 1, gyrozSF = 1;
  public:
    void initialize();
    void readsensor(int16_t imusensor[3][3]);
    void applycalibration(int16_t imusensor[3][3]);
};

#endif
