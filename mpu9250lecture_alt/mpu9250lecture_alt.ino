
#include "IMU.h"

IMU imu;
int16_t imusensor[3][3];





// Initializations
void setup()
{
  Serial.begin(115200);
  imu.initialize();
}

unsigned long time_new, time_old, delta_t;

void loop()
{ 
  
  imu.readsensor(imusensor);
  
  time_new = millis();
  delta_t = time_new - time_old;
  time_old = time_new;
  
  Serial.print (delta_t);
  Serial.print ('\t');
//  // Accelerometer
//  Serial.print (imusensor[0][0],DEC); 
//  Serial.print ("\t");
//  Serial.print (imusensor[0][1],DEC);
//  Serial.print ("\t");
//  Serial.print (imusensor[0][2],DEC);  
//  Serial.print ("\t");
//  // Gyroscope
//  Serial.print (imusensor[1][0],DEC); 
//  Serial.print ("\t");
//  Serial.print (imusensor[1][1],DEC);
//  Serial.print ("\t");
//  Serial.print (imusensor[1][2],DEC);  
//  Serial.print ("\t");
  // Magnetometer
  Serial.print (imusensor[2][0],DEC); 
  Serial.print ("\t");
  Serial.print (imusensor[2][1],DEC);
  Serial.print ("\t");
  Serial.print (imusensor[2][2],DEC);  
  Serial.print ("\t");
  // End of line
  Serial.println("");  
}
