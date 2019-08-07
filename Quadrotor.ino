#include "MPU9250.h"
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
#include "Acelerometro.h"

//unsigned long tiempo=millis();
float radtodeg=180/acos(-1);
int first_iteration=0;

float *Acc_raw_val;
float *yawpitchroll_int;
float *yawpitchroll_triad;
float *yawpitchroll;


void setup() {
  Serial.begin(115200);
  // start communication with IMU
  IMU_ini();
  IMU_default_cal();
  IMU_settings();
  //Acelerometro_cal();
  //Magnetometro_cal();
  //Gyroscope_cal();
}

void loop() {
  
  Acc_raw_val=get_raw_val();
  yawpitchroll_triad=get_ypr_triad(Acc_raw_val);
  
  if (first_iteration<=10){
      yawpitchroll_int=Integrator(Acc_raw_val,yawpitchroll_triad);
      first_iteration=first_iteration+1;
  }
  else {
      yawpitchroll_int=Integrator(Acc_raw_val,yawpitchroll_int);   
  }

  yawpitchroll=filter(Acc_raw_val,yawpitchroll_triad,yawpitchroll_int);

//  Serial.print("Yaw: ");
//  Serial.print(yawpitchroll_triad[0]*radtodeg,3);
//  Serial.print("  ");
//  Serial.print("Pitch: ");  
//  Serial.print(yawpitchroll_triad[1]*radtodeg,3);
//  Serial.print("  ");
//  Serial.print("Roll: ");  
//  Serial.print(yawpitchroll_triad[2]*radtodeg,3);
//
//  Serial.print("Yaw: ");
//  Serial.print(yawpitchroll_int[0]*radtodeg,3);
//  Serial.print("  ");
//  Serial.print("Pitch: ");  
//  Serial.print(yawpitchroll_int[1]*radtodeg,3);
//  Serial.print("  ");
//  Serial.print("Roll: ");  
//  Serial.println(yawpitchroll_int[2]*radtodeg,3);

  Serial.print("Yaw: ");
  Serial.print(yawpitchroll[0]*radtodeg,3);
  Serial.print("  ");
  Serial.print("Pitch: ");  
  Serial.print(yawpitchroll[1]*radtodeg,3);
  Serial.print("  ");
  Serial.print("Roll: ");  
  Serial.println(yawpitchroll[2]*radtodeg,3);
  
//  Serial.print("AccelX: ");
//  Serial.print(Acc_raw_val[0],3);
//  Serial.print("  ");
//  Serial.print("AccelY: ");  
//  Serial.print(Acc_raw_val[1],3);
//  Serial.print("  ");
//  Serial.print("AccelZ: ");  
//  Serial.println(Acc_raw_val[2],3);

//  Serial.print("GyroX: ");
//  Serial.print(Acc_raw_val[3],6);
//  Serial.print("  ");
//  Serial.print("GyroY: ");  
//  Serial.print(Acc_raw_val[4],6);
//  Serial.print("  ");
//  Serial.print("GyroZ: ");  
//  Serial.println(Acc_raw_val[5],6);
////
//  Serial.print("MagX: ");  
//  Serial.print(Acc_raw_val[6],6);
//  Serial.print("  ");  
//  Serial.print("MagY: ");
//  Serial.print(Acc_raw_val[7],6);
//  Serial.print("  ");
//  Serial.print("MagZ: ");  
//  Serial.println(Acc_raw_val[8],6);
} 

