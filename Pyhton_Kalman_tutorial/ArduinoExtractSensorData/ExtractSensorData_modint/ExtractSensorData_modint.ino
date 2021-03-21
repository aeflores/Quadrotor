
#include "IMU.h"
IMU imu;
int16_t imusensor[3][3];

unsigned long timer = 0;
long loopTime = 10000;   // microseconds


typedef union
{
 int16_t number;
 uint8_t bytes[2];
} INTUNION_t;

// Initializations
void setup()
{
  Serial.begin(115200);
  imu.initialize();
  timer = micros();
  
}

void loop() {
  timeSync(loopTime);
  imu.readsensor(imusensor);
  
//  sendToPC(Acc_raw_val[1][0],  Acc_raw_val[1][1], Acc_raw_val[1][2],
//           Acc_raw_val[0][0],  Acc_raw_val[0][1], Acc_raw_val[0][2],
//           Acc_raw_val[2][0],  Acc_raw_val[2][1], Acc_raw_val[2][2]);
  sendToPC(imusensor[1][0],  imusensor[1][1], imusensor[1][2],
           imusensor[0][0],  imusensor[0][1], imusensor[0][2],
           imusensor[2][0],  imusensor[2][1], imusensor[2][2]);
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

void sendToPC(int16_t data1, int16_t data2, int16_t data3, 
              int16_t data4, int16_t data5, int16_t data6,
              int16_t data7, int16_t data8, int16_t data9)
{
  byte buf[18];
  int16_t data[9] = {data1, data2, data3, data4, data5, data6, data7, data8, data9};
  for (int i = 0; i < 9; i++) {
    INTUNION_t myint;
    myint.number = data[i];
    for (int j = 0; j < 2; j++) {
      buf[i*2 + j] = myint.bytes[j];
    }
  }
  Serial.write(buf, 18);
}
