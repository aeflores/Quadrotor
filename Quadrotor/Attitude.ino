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
  float Acc_I[3] = {0.0, 0.0, 9.81};
  normalize_vector(Mag_I);
  normalize_vector(Acc_I);
  // Triada R
  triada(Mag_I, Acc_I, B);
  // Triad algorithm is run serveral times to get initial conditions
  int iterations = 100;
  float qinit[4], theta = 0, k[3] = {0., 0., 0.};
  for (int i = 0; i < iterations; i++) {
    imu.readsensor(imusensor);
    triad_algorithm(imusensor[0], imusensor[2], qinit);
    theta += 2 * acos(qinit[0]) / iterations;
    k[0]  += qinit[1] / sin(acos(qinit[0]))/iterations;
    k[1]  += qinit[2] / sin(acos(qinit[0]))/iterations;
    k[2]  += qinit[3] / sin(acos(qinit[0]))/iterations;
//    Serial.print(i);
//    Serial.print("\t");
//    Serial.print(theta*radtodeg, 3);
//    Serial.print("\t");
//    Serial.print(k[0], 3);
//    Serial.print("\t");
//    Serial.print(k[1], 3);
//    Serial.print("\t");
//    Serial.print(k[2], 3);
//    Serial.println("\t");
  }
//  Serial.print("Initial condition =   ");
//  Serial.print(theta*radtodeg, 3);
//  Serial.print("\t");
//  Serial.print(k[0], 3);
//  Serial.print("\t");
//  Serial.print(k[1], 3);
//  Serial.print("\t");
//  Serial.print(k[2], 3);
//  Serial.println("\t");
  normalize_vector(k);
  q0[0] = cos(theta / 2);
  q0[1] = k[0] * sin(theta / 2);
  q0[2] = k[1] * sin(theta / 2);
  q0[3] = k[2] * sin(theta / 2);
  Euler yawpitchroll_init;
  Q2E(q0,  yawpitchroll_init);
}

void Attitude::triad_algorithm(const int Acc[3], const int Mag[3], float q[4]) {
  // Magnetic field components in the body system reference nT
  float w1[3] = {float(Mag[0]), float(Mag[1]), float(Mag[2])};
  normalize_vector(w1);
  // Gravity components in the inertial body reference m/s2
  // float w2[3] = { -Acc[0], -Acc[1], -Acc[2]};
  float w2[3] = {float(Acc[0]), float(Acc[1]), float(Acc[2])};
  normalize_vector(w2);

//  Serial.print("Mag = ");
//  Serial.print("\t");
//  Serial.print(w1[0],3);
//  Serial.print("\t");
//  Serial.print(w1[1],3);  
//  Serial.print("\t");
//  Serial.print(w1[2],3);
//  Serial.print("\t");
//
//
//  Serial.print("Acc = ");
//  Serial.print("\t");
//  Serial.print(w2[0],3);
//  Serial.print("\t");
//  Serial.print(w2[1],3);  
//  Serial.print("\t");
//  Serial.print(w2[2],3);  
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
  float qnew[4];
  // Rotation matrix to quaternion
  qnew[0] = sqrt(R[0][0] + R[1][1] + R[2][2] + 1) / 2.0;
  qnew[1] = sign(R[1][2] - R[2][1]) * sqrt(R[0][0] - R[1][1] - R[2][2] + 1) / 2.0;
  qnew[2] = sign(R[2][0] - R[0][2]) * sqrt(-R[0][0] + R[1][1] - R[2][2] + 1) / 2.0;
  qnew[3] = sign(R[0][1] - R[1][0]) * sqrt(-R[0][0] - R[1][1] + R[2][2] + 1) / 2.0;

  if (isnan(qnew[0]) || isnan(qnew[1])|| isnan(qnew[2])|| isnan(qnew[3])){
  }
  else{
    q[0] = qnew[0];
    q[1] = qnew[1];
    q[2] = qnew[2];
    q[3] = qnew[3];
  }
//  Serial.print(q[0], 3);
//  Serial.print("\t");
//  Serial.print(q[1], 3);
//  Serial.print("\t");
//  Serial.print(q[2], 3);
//  Serial.print("\t");
//  Serial.print(q[3], 3);
//  Serial.print("\t");
  
  //  yawpitchroll.yaw = atan2(R[0][1], R[0][0]);
  //  yawpitchroll.pitch = atan2(-R[0][2], sqrt(pow(R[1][2], 2) + pow(R[2][2], 2)));
  //  yawpitchroll.roll = atan2(R[1][2], R[2][2]);

}

void integrate(const int gyroint[3], const float qk0[4], int delta_t,  float qk1[4]) {
  float gyro[3];
  gyro[0] = gyroint[0]  * deg2rad / 32.7675f;
  gyro[1] = gyroint[1]  * deg2rad / 32.7675f;
  gyro[2] = gyroint[2]  * deg2rad / 32.7675f;

  float omega_mod         = vector_module(gyro);
  float omega_mod_delta   = omega_mod * delta_t * 1e-6 / 2.0;
  float omega_mod_sin_mod = sin(omega_mod_delta) / omega_mod;
  float omega_mod_cos     = cos(omega_mod_delta);

//  Serial.print(omega_mod,8);
//  Serial.print("\t");
//  Serial.print(omega_mod_delta,8);
//  Serial.print("\t");
//  Serial.print(omega_mod_sin_mod,8);
//  Serial.print("\t");
//  Serial.print(omega_mod_sin_mod,8);
//  Serial.print("\t");
  
  qk1[0] = qk0[0] * omega_mod_cos - gyro[0] * qk0[1] * omega_mod_sin_mod  - gyro[1] * qk0[2] * omega_mod_sin_mod  - gyro[2] * qk0[3] * omega_mod_sin_mod;
  qk1[1] = qk0[1] * omega_mod_cos + gyro[0] * qk0[0] * omega_mod_sin_mod  + gyro[1] * qk0[3] * omega_mod_sin_mod  + gyro[2] * qk0[2] * omega_mod_sin_mod;
  qk1[2] = qk0[2] * omega_mod_cos + gyro[0] * qk0[3] * omega_mod_sin_mod  + gyro[1] * qk0[0] * omega_mod_sin_mod  - gyro[2] * qk0[1] * omega_mod_sin_mod;
  qk1[3] = qk0[3] * omega_mod_cos - gyro[0] * qk0[2] * omega_mod_sin_mod  + gyro[1] * qk0[1] * omega_mod_sin_mod  + gyro[2] * qk0[0] * omega_mod_sin_mod;

  if (isnan(qk1[0]) || isnan(qk1[1])|| isnan(qk1[2])|| isnan(qk1[3]) || omega_mod<1e-6){
    qk1[0] = qk0[0];
    qk1[1] = qk0[1];
    qk1[2] = qk0[2];
    qk1[3] = qk0[3];
    
  }
//  Serial.print(qk1[0], 3);
//  Serial.print("\t");
//  Serial.print(qk1[1], 3);
//  Serial.print("\t");
//  Serial.print(qk1[2], 3);
//  Serial.print("\t");
//  Serial.print(qk1[3], 3);
//  Serial.print("\t");

}

void offset_attitude(const float qsi[4], const float qoffset[4],  float qout[4]) {
  // Q3 = Q2 * Q1
  // Qsi= Qsf* Qfi
  // Qsf⁻¹ * Qsi = Qfi
  // qoffset = Qsf⁻¹
  // qout = qoffset * qsi
  qout[0] = qoffset[0] * qsi[0] -  qoffset[1] * qsi[1] - qoffset[2] * qsi[2] - qoffset[3] * qsi[3];
  qout[1] = qoffset[0] * qsi[1] +  qoffset[1] * qsi[0] + qoffset[2] * qsi[3] - qoffset[3] * qsi[2];
  qout[2] = qoffset[0] * qsi[2] +  qoffset[2] * qsi[0] + qoffset[3] * qsi[1] - qoffset[1] * qsi[3];
  qout[3] = qoffset[0] * qsi[3] +  qoffset[3] * qsi[0] + qoffset[1] * qsi[2] - qoffset[2] * qsi[1];
}

void Q2E(const float qk1[4], Euler& yawpitchroll) {
  yawpitchroll.yaw   =    atan2(2 * (qk1[1] * qk1[2] + qk1[0] * qk1[3]), pow(qk1[0], 2)  +  pow(qk1[1], 2)  -  pow(qk1[2], 2) -  pow(qk1[3], 2));
  yawpitchroll.pitch =  - asin(2 * (qk1[1] * qk1[3] - qk1[0] * qk1[2]));
  yawpitchroll.roll  =    atan2(2 * (qk1[2] * qk1[3] + qk1[0] * qk1[1]), pow(qk1[0], 2)  -  pow(qk1[1], 2)  -  pow(qk1[2], 2) +  pow(qk1[3], 2));

  if (yawpitchroll.yaw < 0.0) {
    yawpitchroll.yaw = 2 * pi + yawpitchroll.yaw;
  }
}

void combinequat(const float q1[4], const float q2[4], float alpha, float q3[4]) {

//  Serial.print(q1[0],2);
//  Serial.print("\t");
//  Serial.print(q1[1],2);
//  Serial.print("\t");
//  Serial.print(q1[2],2);
//  Serial.print("\t");
//  Serial.print(q1[3],2);
//  Serial.print("\t\t");
//  Serial.print(q2[0],2);
//  Serial.print("\t");
//  Serial.print(q2[1],2);
//  Serial.print("\t");
//  Serial.print(q2[2],2);
//  Serial.print("\t");
//  Serial.print(q2[3],2);
//  Serial.print("\t\t");

  float theta1;
  float k1[3], k2[3], k3[3];

  if (q1[0]<0){
    theta1 = 2 * acos(-q1[0]);
    k1[0] = -q1[1] / sin(theta1 / 2.);
    k1[1] = -q1[2] / sin(theta1 / 2.);
    k1[2] = -q1[3] / sin(theta1 / 2.);
  }
  else{
    theta1 = 2 * acos(q1[0]);
    k1[0] = q1[1] / sin(theta1 / 2.);
    k1[1] = q1[2] / sin(theta1 / 2.);
    k1[2] = q1[3] / sin(theta1 / 2.);
  }
  float theta2 = 2 * acos(q2[0]);
  k2[0] = q2[1] / sin(theta2 / 2.);
  k2[1] = q2[2] / sin(theta2 / 2.);
  k2[2] = q2[3] / sin(theta2 / 2.);


//  Serial.print("\t");
//  Serial.print(theta1*radtodeg,2);
//  Serial.print("\t");
//  Serial.print(k1[0],2);
//  Serial.print("\t");
//  Serial.print(k1[1],2);
//  Serial.print("\t");
//  Serial.print(k1[2],2);
//  Serial.print("\t\t");
//  Serial.print(theta2*radtodeg,2);
//  Serial.print("\t");
//  Serial.print(k2[0],2);
//  Serial.print("\t");
//  Serial.print(k2[1],2);
//  Serial.print("\t");
//  Serial.print("\t");
//  Serial.print(k2[2],2);
//  Serial.print("\t");
  float theta3 = alpha * theta1 + (1 - alpha) * theta2;
  k3[0] = alpha * k1[0] + (1 - alpha) * k2[0];
  k3[1] = alpha * k1[1] + (1 - alpha) * k2[1];
  k3[2] = alpha * k1[2] + (1 - alpha) * k2[2];
  normalize_vector(k3);
  q3[0] = cos(theta3 / 2);
  q3[1] = sin(theta3 / 2) * k3[0];
  q3[2] = sin(theta3 / 2) * k3[1];
  q3[3] = sin(theta3 / 2) * k3[2];
  
}


void Attitude::get_attitude(const int imusensor[3][3], Euler& yawpitchroll, int delta_t) {
//  Serial.print(imusensor[0][0], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[0][1], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[0][2], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[1][0], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[1][1], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[1][2], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[2][0], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[2][1], DEC);
//  Serial.print("\t");
//  Serial.print(imusensor[2][2], DEC);
//  Serial.print("\t");


  
  float q1integ[4];
  float q1triad[4];
  triad_algorithm(imusensor[0], imusensor[2],  q1triad);
  integrate(imusensor[1], q0, delta_t,  q1integ);

//  if (isnan(q1integ[0]) || isnan(q1integ[1])|| isnan(q1integ[2])|| isnan(q1integ[3])){
//    q1integ[0] = q1triad[0];
//    q1integ[1] = q1triad[1];
//    q1integ[2] = q1triad[2];
//    q1integ[3] = q1triad[3];
//    
//  }

//  Serial.print(q1triad[0], 3);
//  Serial.print("\t");
//  Serial.print(q1triad[1], 3);
//  Serial.print("\t");
//  Serial.print(q1triad[2], 3);
//  Serial.print("\t");
//  Serial.print(q1triad[3], 3);
//  Serial.print("\t");
//
//  Serial.print(q1integ[0], 3);
//  Serial.print("\t");
//  Serial.print(q1integ[1], 3);
//  Serial.print("\t");
//  Serial.print(q1integ[2], 3);
//  Serial.print("\t");
//  Serial.print(q1integ[3], 3);
//  Serial.print("\t");


  
  combinequat(q1integ, q1triad, alpha, q0);
  // offset_attitude(q0, qoffset, q1);
  // Una vez que tenemos la orientacion del frame con respecto al mundo convertimos nuestro cuaternio en angulos de euler devolvemos eso
  // Q2E(q1,  yawpitchroll);
  Q2E(q0,  yawpitchroll);
}
