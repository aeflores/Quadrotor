#include "MPU9250.h"
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);


void Acelerometro::initialize() {
  while (status < 0) {
    status = IMU.begin();
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
    } else {
      Serial.println("IMU initialization successful");
    }
  }
}

void Acelerometro::default_cal() {

  // Magnetometer settings
  // X-axis bias and scale factor
  float hxb = -4.257;    // mag bias in uT
  float hxs = 0.9838756; // mag scale factor
  IMU.setMagCalX(hxb, hxs);
  // Y-axis bias and scale factor
  float hyb = 27.517288; // mag bias in uT
  float hys = 1.0328;    // mag scale factor
  IMU.setMagCalY(hyb, hys);
  // Z-axis bias and scale factor
  float hzb = -2.302750; // mag bias in uT
  float hzs = 1.0;       // mag scale factor
  IMU.setMagCalZ(hzb, hzs);

  // Gyroscope settings
  // X-axis bias
  float gxb = 0.011; // gyro bias
  IMU.setGyroBiasX_rads(gxb);
  // Y-axis bias
  float gyb = 0.02215; // gyro bias of 0.001 rad/s
  IMU.setGyroBiasY_rads(gyb);
  // Z-axis bias
  float gzb = -0.012746; // gyro bias of 0.001 rad/s
  IMU.setGyroBiasZ_rads(gzb);

  // GyroBiasX: 0.010979  GyroBiasY: 0.022302  GyroBiasZ: -0.012534
  // GyroBiasX: 0.011440  GyroBiasY: 0.022078  GyroBiasZ: -0.012826
  // GyroBiasX: 0.010926  GyroBiasY: 0.022447  GyroBiasZ: -0.012552 

  
}

void Acelerometro::settings() {
  // Low Pass Filter configuration
  // Digital Low Pass Filter (DLPF) bandwidth.
  // By default, if this function is not called, a DLPF bandwidth of 184 Hz is
  // used. The following DLPF bandwidths are supported: 184 Hz, 92 Hz, 41 Hz, 20 Hz, 10 Hz, 5 Hz
  status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);

  // This function sets the accelerometer full scale range to the given value.
  // By default, if this function is not called, a full scale range of +/- 16 g
  // will be used. The enumerated accelerometer full scale ranges are: +/-
  // 2g,+/- 4g,+/- 8g,+/- 16g,
  status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);

  // This function sets the gyroscope full scale range to the given value.
  // By default, if this function is not called, a full scale range of +/- 2000
  // deg/s will be used. The enumerated gyroscope full scale ranges are: +/-
  // 250deg/s,+/- 500deg/s,+/- 1000deg/s,+/- 2000deg/s,
  status = IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
}

void Acelerometro::magnetometro_cal() {
  status = -1;
  while (status < 0) {
    Serial.println("IMU Magnetometer calibration starting in 2 seconds, please "
                   "move de IMU");
    delay(2000);
    status = IMU.calibrateMag();
    if (status < 0) {
      Serial.println("IMU calibration unsuccessful");
      Serial.print("Status: ");
      Serial.println(status);
    } else {
      Serial.println("IMU calibration successful");
      float hxb = IMU.getMagBiasX_uT();
      float hxs = IMU.getMagScaleFactorX();
      float hyb = IMU.getMagBiasY_uT();
      float hys = IMU.getMagScaleFactorY();
      float hzb = IMU.getMagBiasZ_uT();
      float hzs = IMU.getMagScaleFactorZ();
      Serial.print("MagBiasX: ");
      Serial.print(hxb, 6);
      Serial.print("  ");
      Serial.print("MagSFX: ");
      Serial.print(hxs, 6);
      Serial.print("  ");

      Serial.print("MagBiasY: ");
      Serial.print(hyb, 6);
      Serial.print("  ");
      Serial.print("MagSFY: ");
      Serial.print(hys, 6);
      Serial.print("  ");

      Serial.print("MagBiasZ: ");
      Serial.print(hzb, 6);
      Serial.print("  ");
      Serial.print("MagSFZ: ");
      Serial.println(hzs, 6);
    }
  }
}

void Acelerometro::acelerometro_cal() {
  status = -1;
  while (status < 0) {
    Serial.println("IMU Accelerometer calibration starting in 2 seconds, do "
                   "not move the IMU");
    delay(2000);
    status = IMU.calibrateAccel();
    if (status < 0) {
      Serial.println("IMU calibration unsuccessful");
      Serial.print("Status: ");
      Serial.println(status);
    } else {
      Serial.println("IMU calibration successful");
      float axb = IMU.getAccelBiasX_mss();
      float axs = IMU.getAccelScaleFactorX();
      float ayb = IMU.getAccelBiasY_mss();
      float ays = IMU.getAccelScaleFactorY();
      float azb = IMU.getAccelBiasZ_mss();
      float azs = IMU.getAccelScaleFactorZ();
      Serial.print("AccBiasX: ");
      Serial.print(axb, 6);
      Serial.print("  ");
      Serial.print("AccSFX: ");
      Serial.print(axs, 6);
      Serial.print("  ");

      Serial.print("AccBiasY: ");
      Serial.print(ayb, 6);
      Serial.print("  ");
      Serial.print("AccSFY: ");
      Serial.print(ays, 6);
      Serial.print("  ");

      Serial.print("AccBiasZ: ");
      Serial.print(azb, 6);
      Serial.print("  ");
      Serial.print("AccSFZ: ");
      Serial.print(azs, 6);
      Serial.println("  ");
    }
  }
}

void Acelerometro::gyroscope_cal() {

  status = -1;
  while (status < 0) {
    Serial.println(
      "IMU Gyroscope calibration starting in 2 seconds, do not move the IMU");
    delay(2000);
    status = IMU.calibrateGyro();
    if (status < 0) {
      Serial.println("IMU calibration unsuccessful");
      Serial.print("Status: ");
      Serial.println(status);
    } else {
      Serial.println("IMU calibration successful");
      float gxb = IMU.getGyroBiasX_rads();
      float gyb = IMU.getGyroBiasY_rads();
      float gzb = IMU.getGyroBiasZ_rads();
      Serial.print("GyroBiasX: ");
      Serial.print(gxb, 6);
      Serial.print("  ");
      Serial.print("GyroBiasY: ");
      Serial.print(gyb, 6);
      Serial.print("  ");
      Serial.print("GyroBiasZ: ");
      Serial.print(gzb, 6);
      Serial.println("  ");
    }
  }
}

void Acelerometro::get_raw_val(float (&Acc_val)[9]) {
  // Read the sensor
  IMU.readSensor();
  // Get Acceleration
  float Acc_x = IMU.getAccelX_mss();
  float Acc_y = IMU.getAccelY_mss();
  float Acc_z = IMU.getAccelZ_mss();
  // Get angular velocities
  float Gyro_x = IMU.getGyroX_rads();
  float Gyro_y = IMU.getGyroY_rads();
  float Gyro_z = IMU.getGyroZ_rads();
  // Get magnetic field components in the body reference
  float Mag_x = IMU.getMagX_uT();
  float Mag_y = IMU.getMagY_uT();
  float Mag_z = IMU.getMagZ_uT();
  Acc_val[0] = Acc_x;
  Acc_val[1] = Acc_y;
  Acc_val[2] = Acc_z;
  Acc_val[3] = Gyro_x;
  Acc_val[4] = Gyro_y;
  Acc_val[5] = Gyro_z;
  Acc_val[6] = Mag_x;
  Acc_val[7] = Mag_y;
  Acc_val[8] = Mag_z;
  //  Serial.print("Temperature in C: ");
  //  Serial.println(IMU.getTemperature_C(),6);
  //  Serial.println();
}

// MagBiasX: -7.138206  MagSFX: 1.079588  MagBiasY: 18.255805  MagSFY: 0.946956
// MagBiasZ: 6.438561  MagSFZ: 0.982602 MagBiasX: 13.060284  MagSFX: 1.174667
// MagBiasY: 14.025960  MagSFY: 1.094046  MagBiasZ: 18.988445  MagSFZ: 0.809942
// MagBiasX: -4.390320  MagSFX: 0.984479  MagBiasY: 23.044439  MagSFY: 1.076520
// MagBiasZ: -2.302750  MagSFZ: 0.947584 MagBiasX: -3.515854  MagSFX: 0.936396
// MagBiasY: 27.973655  MagSFY: 1.062817  MagBiasZ: 5.071450  MagSFZ: 1.008898
// MagBiasX: -4.876480  MagSFX: 1.030752  MagBiasY: 27.517288  MagSFY: 0.959087
// MagBiasZ: 3.004229  MagSFZ: 1.012990 MagBiasX: 10.368086  MagSFX: 1.615734
// MagBiasY: 32.904544  MagSFY: 0.650060  MagBiasZ: 19.129512  MagSFZ: 1.186567
