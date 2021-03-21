# include "IMU.h"

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

void IMU :: initialize(){
   // Arduino initializations
  Wire.begin(); 
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

  
}



void IMU :: applycalibration(int16_t imusensor[3][3]){
  imusensor[0][0] = accxSF*(imusensor[0][0] - accxbias);
  imusensor[0][1] = accySF*(imusensor[0][1] - accybias);
  imusensor[0][2] = acczSF*(imusensor[0][2] - acczbias);

  imusensor[1][0] = gyroxSF*(imusensor[1][0] - gyroxbias);
  imusensor[1][1] = gyroySF*(imusensor[1][1] - gyroybias);
  imusensor[1][2] = gyrozSF*(imusensor[1][2] - gyrozbias);

  imusensor[2][0] = magxfilt.AddValue(magxSF*(imusensor[2][0] - magxbias));
  imusensor[2][1] = magyfilt.AddValue(magySF*(imusensor[2][1] - magybias));
  imusensor[2][2] = magzfilt.AddValue(magzSF*(imusensor[2][2] - magzbias));

//  imusensor[2][0] = magxSF*(imusensor[2][0] - magxbias);
//  imusensor[2][1] = magySF*(imusensor[2][1] - magybias);
//  imusensor[2][2] = magzSF*(imusensor[2][2] - magzbias);
  
}




void IMU :: readsensor(int16_t imusensor[3][3]){
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  imusensor[0][0]=-(Buf[0]<<8 | Buf[1]);
  imusensor[0][1]=-(Buf[2]<<8 | Buf[3]);
  imusensor[0][2]=Buf[4]<<8 | Buf[5];

  // Gyroscope
  imusensor[1][0]=-(Buf[8]<<8 | Buf[9]);
  imusensor[1][1]=-(Buf[10]<<8 | Buf[11]);
  imusensor[1][2]=Buf[12]<<8 | Buf[13];

  
  // _____________________
  // :::  Magnetometer ::: 
  // Read register Status 1 and wait for the DRDY: Data Ready
//  uint8_t ST1;
//  
//  do
//  {
//    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
//  }
//  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  // Magnetometer
  imusensor[2][0]=-(Mag[3]<<8 | Mag[2]);
  imusensor[2][1]=-(Mag[1]<<8 | Mag[0]);
  imusensor[2][2]=-(Mag[5]<<8 | Mag[4]);

  applycalibration(imusensor);

  
}
