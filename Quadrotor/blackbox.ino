#include "blackbox.h"



// Declaramos el pin CS
#define CS_PIN A3

#include <SPI.h>
#include <SdFat.h>
SdFat sd;

void blackbox::init() {

  while (!sd.begin(CS_PIN)) {  // SPI_FULL_SPEED
    Serial.println("error");
  }

  File dataFile = sd.open("datalog.txt", FILE_WRITE);
  // Writing file heading
  // Files in a directory must be listed an a new txt must be created
  // Here the accelerometer parameters should be written. This comprises calibration of Gyro and Mag (Both linear coefficients) and settings
  // Also it might be interesting to writte radio cycling parameteres
  if (dataFile) {
    dataFile.println("Test");
    dataFile.close();
  }
}


void blackbox::calibration() {
  // When calibration state is entered, PID coefficients must be registered
  
}


void blackbox::savedata() {
  // This function should be called exclusively in flightmode
  // To be written:
  // Delta_t AccX AccY AccZ GyroX GyroY GyroZ MagX MagY MagZ yawpitchroll_triad yawpithroll_int yawpitchroll error_pitch error_roll derivative_error_pitch derivative_error_roll

  
}
