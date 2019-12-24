unsigned long t_0 = 0;

float *get_ypr_triad(float Acc_raw_val[]) {
  // Magnetic field components in the inertial system reference uT
  // Madrid
    float Mag_x_I = 25.6547;
    float Mag_y_I=-0.1469 ;
    float Mag_z_I=36.8374;
  // Almeria
//  float Mag_x_I = 27.4347;
//  float Mag_y_I = -0.0751;
//  float Mag_z_I = 33.9364;
  // Paris, Palaiseau
//  float Mag_x_I = 20.9587;
//  float Mag_y_I = 0.3459;
//  float Mag_z_I = 43.3234;
  float Mag_mod_I = sqrt(pow(Mag_x_I, 2) + pow(Mag_y_I, 2) + pow(Mag_z_I, 2));
  // Gravity components in the inertial system reference m/s2
  float Acc_x_I = 0;
  float Acc_y_I = 0;
  float Acc_z_I = 9.81;
  float Acc_mod_I = sqrt(pow(Acc_x_I, 2) + pow(Acc_y_I, 2) + pow(Acc_z_I, 2));

  float v1[3] = {Mag_x_I / Mag_mod_I, Mag_y_I / Mag_mod_I, Mag_z_I / Mag_mod_I};
  float v2[3] = {Acc_x_I / Acc_mod_I, Acc_y_I / Acc_mod_I, Acc_z_I / Acc_mod_I};

  // Magnetic field components in the body system reference nT
  float Mag_x_B = Acc_raw_val[6];
  float Mag_y_B = Acc_raw_val[7];
  float Mag_z_B = Acc_raw_val[8];
  float Mag_mod_B = sqrt(pow(Mag_x_B, 2) + pow(Mag_y_B, 2) + pow(Mag_z_B, 2));
  // Gravity components in the inertial body reference m/s2
  float Acc_x_B = -Acc_raw_val[0];
  float Acc_y_B = -Acc_raw_val[1];
  float Acc_z_B = -Acc_raw_val[2];
  float Acc_mod_B = sqrt(pow(Acc_x_B, 2) + pow(Acc_y_B, 2) + pow(Acc_z_B, 2));

  float w1[3] = {Mag_x_B / Mag_mod_B, Mag_y_B / Mag_mod_B, Mag_z_B / Mag_mod_B};
  float w2[3] = {Acc_x_B / Acc_mod_B, Acc_y_B / Acc_mod_B, Acc_z_B / Acc_mod_B};

  // TRIAD algorithm

  // Triada R
  float r1[3], r2[3], r3[3], s1[3], s2[3], s3[3];
  r1[0] = v1[0];
  r1[1] = v1[1];
  r1[2] = v1[2];

  r2[0] = v1[1] * v2[2] - v1[2] * v2[1];
  r2[1] = -v1[0] * v2[2] + v1[2] * v2[0];
  r2[2] = v1[0] * v2[1] - v1[1] * v2[0];

  float r2_mod = sqrt(pow(r2[0], 2) + pow(r2[1], 2) + pow(r2[2], 2));

  r2[0] = r2[0] / r2_mod;
  r2[1] = r2[1] / r2_mod;
  r2[2] = r2[2] / r2_mod;

  r3[0] = r1[1] * r2[2] - r1[2] * r2[1];
  r3[1] = -r1[0] * r2[2] + r1[2] * r2[0];
  r3[2] = r1[0] * r2[1] - r1[1] * r2[0];
  // B matrix
  float B[3][3];

  B[0][0] = r1[0];
  B[1][0] = r1[1];
  B[2][0] = r1[2];

  B[0][1] = r2[0];
  B[1][1] = r2[1];
  B[2][1] = r2[2];

  B[0][2] = r3[0];
  B[1][2] = r3[1];
  B[2][2] = r3[2];

  // Triada S
  s1[0] = w1[0];
  s1[1] = w1[1];
  s1[2] = w1[2];

  s2[0] = w1[1] * w2[2] - w1[2] * w2[1];
  s2[1] = -w1[0] * w2[2] + w1[2] * w2[0];
  s2[2] = w1[0] * w2[1] - w1[1] * w2[0];
  float s2_mod = sqrt(pow(s2[0], 2) + pow(s2[1], 2) + pow(s2[2], 2));

  s2[0] = s2[0] / s2_mod;
  s2[1] = s2[1] / s2_mod;
  s2[2] = s2[2] / s2_mod;

  s3[0] = s1[1] * s2[2] - s1[2] * s2[1];
  s3[1] = -s1[0] * s2[2] + s1[2] * s2[0];
  s3[2] = s1[0] * s2[1] - s1[1] * s2[0];
  // A matrix
  float A[3][3];
  A[0][0] = s1[0];
  A[1][0] = s1[1];
  A[2][0] = s1[2];

  A[0][1] = s2[0];
  A[1][1] = s2[1];
  A[2][1] = s2[2];

  A[0][2] = s3[0];
  A[1][2] = s3[1];
  A[2][2] = s3[2];

  // Matriz R=A*B'

  float R[3][3];
  R[0][0] = A[0][0] * B[0][0] + A[0][1] * B[0][1] + A[0][2] * B[0][2];
  R[0][1] = A[0][0] * B[1][0] + A[0][1] * B[1][1] + A[0][2] * B[1][2];
  R[0][2] = A[0][0] * B[2][0] + A[0][1] * B[2][1] + A[0][2] * B[2][2];

  R[1][0] = A[1][0] * B[0][0] + A[1][1] * B[0][1] + A[1][2] * B[0][2];
  R[1][1] = A[1][0] * B[1][0] + A[1][1] * B[1][1] + A[1][2] * B[1][2];
  R[1][2] = A[1][0] * B[2][0] + A[1][1] * B[2][1] + A[1][2] * B[2][2];

  R[2][0] = A[2][0] * B[0][0] + A[2][1] * B[0][1] + A[2][2] * B[0][2];
  R[2][1] = A[2][0] * B[1][0] + A[2][1] * B[1][1] + A[2][2] * B[1][2];
  R[2][2] = A[2][0] * B[2][0] + A[2][1] * B[2][1] + A[2][2] * B[2][2];

  static float yawpitchroll[3];
  yawpitchroll[0] = atan2(R[0][1], R[0][0]);
  yawpitchroll[1] = atan2(-R[0][2], sqrt(pow(R[1][2], 2) + pow(R[2][2], 2)));
  yawpitchroll[2] = atan2(R[1][2], R[2][2]);
  return yawpitchroll;
}

float *Integrator(float Acc_raw_val[], float yawpitchroll[]) {
  unsigned long t_1 = millis();
  static float yawpitchroll_integrator[3];
  double delta_t;
  delta_t = (t_1 - t_0) / 1000.;
  // Serial.println(delta_t,6);

  float p = Acc_raw_val[3]; // gyro_x
  float q = Acc_raw_val[4]; // gyro_y
  float r = Acc_raw_val[5]; // gyro_z

  //  Serial.print(p,6);
  //  Serial.print("   ");
  //  Serial.print(q,6);
  //  Serial.print("   ");
  //  Serial.println(r,6);

  float yaw_angle = yawpitchroll[0];
  float pitch_angle = yawpitchroll[1];
  float roll_angle = yawpitchroll[2];

  //  Serial.print(yaw_angle,6);
  //  Serial.print("   ");
  //  Serial.print(pitch_angle,6);
  //  Serial.print("   ");
  //  Serial.println(roll_angle,6);

  float yaw_dot =
      (q * sin(roll_angle) + r * cos(roll_angle)) / cos(pitch_angle);
  float pitch_dot = q * cos(roll_angle) - r * sin(roll_angle);
  float roll_dot =
      p + (q * sin(roll_angle) + r * cos(roll_angle)) * tan(pitch_angle);

  //  Serial.print(yaw_dot,6);
  //  Serial.print("   ");
  //  Serial.print(pitch_dot,6);
  //  Serial.print("   ");
  //  Serial.println(roll_dot,6);

  float delta_yaw = yaw_dot * delta_t;
  float delta_angle = pitch_dot * delta_t;
  float delta_roll = +roll_dot * delta_t;

  //  Serial.print(delta_yaw,6);
  //  Serial.print("   ");
  //  Serial.print(delta_angle,6);
  //  Serial.print("   ");
  //  Serial.println(delta_roll,6);

  yaw_angle = yaw_angle + delta_yaw;
  pitch_angle = pitch_angle + delta_angle;
  roll_angle = roll_angle + delta_roll;

  yawpitchroll_integrator[0] = yaw_angle;
  yawpitchroll_integrator[1] = pitch_angle;
  yawpitchroll_integrator[2] = roll_angle;
  t_0 = t_1;
  return yawpitchroll_integrator;
}

float *filter(float Acc_raw_val[], float yawpitchroll_triad[],
              float yawpitchroll_int[]) {
  static float yawpitchroll[3];
  float rotation_treshold = 10; // rad/s;
  float pqr_module = sqrt(pow(Acc_raw_val[3], 2) + pow(Acc_raw_val[4], 2) +
                          pow(Acc_raw_val[5], 2));
  float dinamic_coef = 1 / (1 + exp(-(pqr_module - rotation_treshold / 2)));

  //yawpitchroll[0] = (1 - dinamic_coef) * yawpitchroll_triad[0] +
  //                  dinamic_coef * yawpitchroll_int[0];
  yawpitchroll[0]=yawpitchroll_int[0];
  yawpitchroll[1] = (1 - dinamic_coef) * yawpitchroll_triad[1] +
                    dinamic_coef * yawpitchroll_int[1];
  yawpitchroll[2] = (1 - dinamic_coef) * yawpitchroll_triad[2] +
                    dinamic_coef * yawpitchroll_int[2];

  //  yawpitchroll[0]=0.5*yawpitchroll_triad[0]+0.5*yawpitchroll_int[0];
  //  yawpitchroll[1]=0.5*yawpitchroll_triad[1]+0.5*yawpitchroll_int[1];
  //  yawpitchroll[2]=0.5*yawpitchroll_triad[2]+0.5*yawpitchroll_int[2];

  return yawpitchroll;
}
