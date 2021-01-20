#include "Attitude.h"

float Euler::yaw_deg() const {
  return yaw * radtodeg;
}
float Euler::pitch_deg() const {
  return pitch * radtodeg;
}
float Euler::roll_deg() const {
  return roll * radtodeg;
}

float sign(const float value) {
  return ((value > 0) - (value < 0));
}

float vector_module(const float v[3]) {
  return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
}
void normalize_vector(float v[3]) {
  float mod = vector_module(v);
  for (int i = 0; i < 3; i++) {
    v[i] = v[i] / mod;
  }
}
void vectprod(const float v1[], const  float v2[], float r2[]) {
  r2[0] = v1[1] * v2[2] - v1[2] * v2[1];
  r2[1] = -v1[0] * v2[2] + v1[2] * v2[0];
  r2[2] = v1[0] * v2[1] - v1[1] * v2[0];
}
void transpose(float M[3][3]) {
  float tmp;
  tmp = M[0][1];
  M[0][1] = M[1][0];
  M[1][0] = tmp;

  tmp = M[0][2];
  M[0][2] = M[2][0];
  M[2][0] = tmp;

  tmp = M[1][2];
  M[1][2] = M[2][1];
  M[2][1] = tmp;
}

void triada(const float v1[3], const float v2[3], float M[3][3]) {
  for (int i = 0; i < 3; i++) {
    M[0][i] = v1[i];
  }
  vectprod(v1, v2, M[1]);
  normalize_vector(M[1]);
  vectprod(M[0], M[1], M[2]);
  transpose(M);
}


void Attitude::initial_cond() {
  // Triad Algorithm parameters are computed
  // Magnetic field components in the inertial system reference uT
  // Madrid
  float Mag_I[3] = {25.6547, -0.1469 , 36.8374};
  // Gravity components in the inertial system reference m/s2
  float Acc_I[3] = { 0.0, 0.0, 9.81};
  normalize_vector(Mag_I);
  normalize_vector(Acc_I);
  // Triada R
  triada(Mag_I, Acc_I, B);
  // Triad algorithm is run serveral times to get initial conditions
  int iterations = 100;
  float qinit[4];
  for (int i = 0; i < iterations; i++) {
    acelerometro.get_raw_val(Acc_raw_val);
    triad_algorithm(Acc_raw_val[0], Acc_raw_val[2], qinit);
    q0[0] += qinit[0];
    q0[1] += qinit[1];
    q0[2] += qinit[2];
    q0[3] += qinit[3];
  }
  q0[0] = q0[0] / iterations;
  q0[1] = q0[1] / iterations;
  q0[2] = q0[2] / iterations;
  q0[3] = q0[3] / iterations;
  Euler yawpitchroll_init;
  Q2E(q0,  yawpitchroll_init);
  Serial.print("Initial conditions");
  Serial.print("  Yaw =  ");
  Serial.print(yawpitchroll_init.yaw_deg(), 3);
  Serial.print("  pitch =  ");
  Serial.print(yawpitchroll_init.pitch_deg(), 3);
  Serial.print("  roll =  ");
  Serial.print(yawpitchroll_init.roll_deg(), 3);
  Serial.print("\t");
}

void Attitude::triad_algorithm(const float Acc[3], const float Mag[3], float q[4]) {
  // Magnetic field components in the body system reference nT
  float w1[3] = {Mag[0], Mag[1], Mag[2]};
  normalize_vector(w1);
  // Gravity components in the inertial body reference m/s2
  float w2[3] = { -Acc[0], -Acc[1], -Acc[2]};
  normalize_vector(w2);
  // TRIAD algorithm
  // Triada S
  float A[3][3];
  triada(w1, w2, A);
  // Matrix R=A*B'
  float R[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R[i][j] = A[i][0] * B[j][0] + A[i][1] * B[j][1] + A[i][2] * B[j][2];
    }
  }
  // Rotation matrix to quaternion
  q[0] = sqrt(R[0][0] + R[1][1] + R[2][2] + 1) / 2.0;
  q[1] = sign(R[1][2] - R[2][1]) * sqrt(R[0][0] - R[1][1] - R[2][2] + 1) / 2.0;
  q[2] = sign(R[2][0] - R[0][2]) * sqrt(-R[0][0] + R[1][1] - R[2][2] + 1) / 2.0;
  q[3] = sign(R[0][1] - R[1][0]) * sqrt(-R[0][0] - R[1][1] + R[2][2] + 1) / 2.0;
  //  yawpitchroll.yaw = atan2(R[0][1], R[0][0]);
  //  yawpitchroll.pitch = atan2(-R[0][2], sqrt(pow(R[1][2], 2) + pow(R[2][2], 2)));
  //  yawpitchroll.roll = atan2(R[1][2], R[2][2]);

}

void integrate(const float gyro[3], const float qk0[4], float delta_t,  float qk1[4]) {
  float omega_mod         = vector_module(gyro);
  float omega_mod_delta   = omega_mod * delta_t / 2.0;
  float omega_mod_sin_mod = sin(omega_mod_delta) / omega_mod;
  float omega_mod_cos     = cos(omega_mod_delta);
  qk1[0] = qk0[0] * omega_mod_cos - gyro[0] * qk0[1] * omega_mod_sin_mod  - gyro[1] * qk0[2] * omega_mod_sin_mod  - gyro[2] * qk0[3] * omega_mod_sin_mod;
  qk1[1] = qk0[1] * omega_mod_cos + gyro[0] * qk0[0] * omega_mod_sin_mod  + gyro[1] * qk0[3] * omega_mod_sin_mod  + gyro[2] * qk0[2] * omega_mod_sin_mod;
  qk1[2] = qk0[2] * omega_mod_cos + gyro[0] * qk0[3] * omega_mod_sin_mod  + gyro[1] * qk0[0] * omega_mod_sin_mod  - gyro[2] * qk0[1] * omega_mod_sin_mod;
  qk1[3] = qk0[3] * omega_mod_cos - gyro[0] * qk0[2] * omega_mod_sin_mod  + gyro[1] * qk0[1] * omega_mod_sin_mod  + gyro[2] * qk0[0] * omega_mod_sin_mod;

}

void offset_attitude(const float qsi[4], const float qoffset[4],  float qout[4]) {
  // Q3 = Q2 * Q1
  // Qsi= Qsf* Qfi
  // Qsf⁻¹ * Qsi = Qfi
  // qoffset = Qsf⁻¹
  // qout = qoffset * qsi
  qout[0] = qoffset[0]*qsi[0] - (qoffset[1]*qsi[1] + qoffset[2]*qsi[2] + qoffset[3]*qsi[3]);
  qout[1] = qoffset[0]*qsi[1] +  qoffset[1]*qsi[0] + qoffset[2]*qsi[3] - qoffset[3]*qsi[2];
  qout[2] = qoffset[0]*qsi[2] +  qoffset[2]*qsi[0] + qoffset[3]*qsi[1] - qoffset[1]*qsi[3];
  qout[3] = qoffset[0]*qsi[3] +  qoffset[3]*qsi[0] + qoffset[1]*qsi[2] - qoffset[2]*qsi[1];
}

void Q2E(const float qk1[4], Euler& yawpitchroll) {
  yawpitchroll.yaw   =    atan2(2 * (qk1[1] * qk1[2] + qk1[0] * qk1[3]), pow(qk1[0], 2)  +  pow(qk1[1], 2)  -  pow(qk1[2], 2) -  pow(qk1[3], 2));
  yawpitchroll.pitch =  - asin(2 * (qk1[1] * qk1[3] - qk1[0] * qk1[2]));
  yawpitchroll.roll  =    atan2(2 * (qk1[2] * qk1[3] + qk1[0] * qk1[1]), pow(qk1[0], 2)  -  pow(qk1[1], 2)  -  pow(qk1[2], 2) +  pow(qk1[3], 2));

  if (yawpitchroll.yaw < 0.0) {
    yawpitchroll.yaw = 2 * pi + yawpitchroll.yaw;
  }
}


void Attitude::get_attitude(const float Acc_raw_val[3][3], Euler& yawpitchroll, float delta_t) {
  float q1integ[4];
  float q1triad[4];
  triad_algorithm(Acc_raw_val[0], Acc_raw_val[2],  q1triad);
  integrate(Acc_raw_val[1], q0, delta_t,  q1integ);
  q0[0] = 0.01 * q1triad[0] + 0.99 * q1integ[0];
  q0[1] = 0.01* q1triad[1] + 0.99 * q1integ[1];
  q0[2] = 0.01 * q1triad[2] + 0.99 * q1integ[2];
  q0[3] = 0.01 * q1triad[3] + 0.99 * q1integ[3];
  offset_attitude(q0, qoffset, q1);

      Serial.print("  q0 = ");
    Serial.print(q0[0]);
    Serial.print("\t");
    Serial.print(q0[1]);
    Serial.print("\t");
    Serial.print(q0[2]);
    Serial.print("\t");
    Serial.print(q0[3]);
    Serial.print("\t");
  // Una vez que tenemos la orientacion del frame con respecto al mundo convertimos nuestro cuaternio en angulos de euler devolvemos eso
  Q2E(q1,  yawpitchroll);
}

//    Serial.print("Initial conditions");
//    Serial.print("  Yaw =  ");
//    Serial.print(yawpitchroll_init.yaw_deg(), 3);
//    Serial.print("  pitch =  ");
//    Serial.print(yawpitchroll_init.pitch_deg(), 3);
//    Serial.print("  roll =  ");
//    Serial.print(yawpitchroll_init.roll_deg(), 3);
//    Serial.print("\t");

//    Serial.print("  q0 = ");
//    Serial.print(q0[0]);
//    Serial.print("\t");
//    Serial.print(q0[1]);
//    Serial.print("\t");
//    Serial.print(q0[2]);
//    Serial.print("\t");
//    Serial.print(q0[3]);
//    Serial.print("\t");
//    Serial.print("\t");
//    Serial.print(Acc_raw_val[1][0], 6);
//    Serial.print("\t");
//    Serial.print(Acc_raw_val[1][1], 6);
//    Serial.print("\t");
//    Serial.print(Acc_raw_val[1][2], 6);
//    Serial.print("\t");

//    Serial.println("Acc");
//    Serial.print(Acc_raw_val[0][0]);
//    Serial.print("\t");
//    Serial.print(Acc_raw_val[0][1]);
//    Serial.print("\t");
//    Serial.println(Acc_raw_val[0][2]);
//    Serial.println("Mag");
//    Serial.print(Acc_raw_val[2][0]);
//    Serial.print("\t");
//    Serial.print(Acc_raw_val[2][1]);
//    Serial.print("\t");
//    Serial.println(Acc_raw_val[2][2]);
//    Serial.println(" ");
//    Serial.print("Initial conditions   ");
//    Serial.print(" q0 = ");
//    Serial.print(q0[0]);
//    Serial.print(" q1 = ");
//    Serial.print(q0[1]);
//    Serial.print(" q2 = ");
//    Serial.print(q0[2]);
//    Serial.print(" q3 = ");
//    Serial.print(q0[3]);
//    Serial.println("B");
//    Serial.print(B[0][0],4);
//    Serial.print("\t");
//    Serial.print(B[0][1],4);
//    Serial.print("\t");
//    Serial.println(B[0][2],4);
//    Serial.print(B[1][0],4);
//    Serial.print("\t");
//    Serial.print(B[1][1],4);
//    Serial.print("\t");
//    Serial.println(B[1][2],4);
//    Serial.print(B[2][0],4);
//    Serial.print("\t");
//    Serial.print(B[2][1],4);
//    Serial.print("\t");
//    Serial.println(B[2][2],4);
