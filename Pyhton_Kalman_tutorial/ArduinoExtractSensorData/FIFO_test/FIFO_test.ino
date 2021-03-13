// Including libraries
#include "Acelerometro.h"
// Defining objects
Acelerometro acelerometro;

// Global variables
float Acc_raw_val[3][3]; // Raw IMU measurements accelerometer, magnetometer and gyroscope

unsigned long timer = 0;
long loopTime = 10000;   // microseconds


typedef union
{
 float number;
 uint8_t bytes[4];
} FLOATUNION_t;

void setup() {
  Serial.begin(115200);
  acelerometro.initialize();
  // acelerometro.acelerometro_cal();
  // acelerometro.magnetometro_cal();
  // acelerometro.gyroscope_cal();
  acelerometro.default_cal();
  acelerometro.settings();
  timer = micros();
}

void loop() {
  timeSync(loopTime);
  //  sensor.getAngVel(&gyroData);  // Gyro Data
  //  sensor.getAccel(&accelData);  // Accelerometer Data
  //  sensor.getMag(&magData);      // Magnetometer Data
  acelerometro.get_raw_val(Acc_raw_val);
  
//  sendToPC(Acc_raw_val[1][0],  Acc_raw_val[1][1], Acc_raw_val[1][2],
//           Acc_raw_val[0][0],  Acc_raw_val[0][1], Acc_raw_val[0][2],
//           Acc_raw_val[2][0],  Acc_raw_val[2][1], Acc_raw_val[2][2]);
  sendToPC(double(Acc_raw_val[1][0]),  double(Acc_raw_val[1][1]), double(Acc_raw_val[1][2]),
           double(Acc_raw_val[0][0]),  double(Acc_raw_val[0][1]), double(Acc_raw_val[0][2]),
           double(Acc_raw_val[2][0]),  double(Acc_raw_val[2][1]), double(Acc_raw_val[2][2]));
}

void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
    // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}

void sendToPC(float data1, float data2, float data3, 
              float data4, float data5, float data6,
              float data7, float data8, float data9)
{
  byte buf[36];
  float data[9] = {data1, data2, data3, data4, data5, data6, data7, data8, data9};
  for (int i = 0; i < 9; i++) {
    FLOATUNION_t myFloat;
    myFloat.number = data[i];
    for (int j = 0; j < 4; j++) {
      buf[i*4 + j] = myFloat.bytes[j];
    }
  }
//  byte* byteData1 = byte(data1);
//  byte* byteData2 = byte(data2);
//  byte* byteData3 = byte(data3);
//  byte* byteData4 = byte(data4);
//  byte* byteData5 = byte(data5);
//  byte* byteData6 = byte(data6);
//  byte* byteData7 = byte(data7);
//  byte* byteData8 = byte(data8);
//  byte* byteData9 = byte(data9);
//  byte buf[36] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
//                  byteData2[0], byteData2[1], byteData2[2], byteData2[3],
//                  byteData3[0], byteData3[1], byteData3[2], byteData3[3],
//                  byteData4[0], byteData4[1], byteData4[2], byteData4[3],
//                  byteData5[0], byteData5[1], byteData5[2], byteData5[3],
//                  byteData6[0], byteData6[1], byteData6[2], byteData6[3],
//                  byteData7[0], byteData7[1], byteData7[2], byteData7[3],
//                  byteData8[0], byteData8[1], byteData8[2], byteData8[3],
//                  byteData9[0], byteData9[1], byteData9[2], byteData9[3]};
  Serial.write(buf, 36);
}

//
//void sendToPC(int* data1, int* data2, int* data3,
//              int* data4, int* data5, int* data6,
//              int* data7, int* data8, int* data9)
//{
//  byte* byteData1 = (byte*)(data1);
//  byte* byteData2 = (byte*)(data2);
//  byte* byteData3 = (byte*)(data3);
//  byte* byteData4 = (byte*)(data4);
//  byte* byteData5 = (byte*)(data5);
//  byte* byteData6 = (byte*)(data6);
//  byte* byteData7 = (byte*)(data7);
//  byte* byteData8 = (byte*)(data8);
//  byte* byteData9 = (byte*)(data9);
//  byte buf[18] = {byteData1[0], byteData1[1],
//                 byteData2[0], byteData2[1],
//                 byteData3[0], byteData3[1],
//                 byteData4[0], byteData4[1],
//                 byteData5[0], byteData5[1],
//                 byteData6[0], byteData6[1],
//                 byteData7[0], byteData7[1],
//                 byteData8[0], byteData8[1],
//                 byteData9[0], byteData9[1]};
//  Serial.write(buf, 18);
//}
