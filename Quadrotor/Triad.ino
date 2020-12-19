
#include "Triad.h"


// no se el nombre adecuado para esta funcion
void transform(const float v1[],const  float v2[], float r2[]){
  r2[0] = v1[1] * v2[2] - v1[2] * v2[1];
  r2[1] = -v1[0] * v2[2] + v1[2] * v2[0];
  r2[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

float vector_module(const float v[3]){
  return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
}

void normalize_vector(float v[3]){
  float mod= vector_module(v);
  for(int i=0; i<3; i++){
    v[i]= v[i]/mod;
  }
}

void triada(const float v1[3],const float v2[3], float M[3][3]){
  
  float r[3][3];
  for(int i=0; i<3; i++){
    r[0][i] = v1[i];
  }
  transform(v1,v2,r[1]);
  normalize_vector(r[1]);
  transform(r[0],r[1],r[2]);
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      M[j][i]=r[i][j];    
    }
  }

}



void Triad::initialize(){
  // Magnetic field components in the inertial system reference uT
  // Madrid
    float Mag_I[3] = {25.6547f, -0.1469f ,36.8374f};
  // Almeria
//  float Mag_x_I = 27.4347;
//  float Mag_y_I = -0.0751;
//  float Mag_z_I = 33.9364;
  // Paris, Palaiseau
//  float Mag_x_I = 20.9587;
//  float Mag_y_I = 0.3459;
//  float Mag_z_I = 43.3234;
  // Gravity components in the inertial system reference m/s2
  float Acc_I[3] ={ 0.0f, 0.0f, 9.81f};
  normalize_vector(Mag_I);
  normalize_vector(Acc_I);
// Triada R
  triada(Mag_I,Acc_I,B);
}




void Triad::get_ypr_triad(const float Acc_raw_val[3][3],Attitude &yawpitchroll) {
  // Magnetic field components in the body system reference nT
  float w1[3]= {Acc_raw_val[2][0], Acc_raw_val[2][1], Acc_raw_val[2][2]};
  normalize_vector(w1);
  // Gravity components in the inertial body reference m/s2
  float w2[3]= {Acc_raw_val[0][0], Acc_raw_val[0][1], Acc_raw_val[0][2]};
  normalize_vector(w2);

  // TRIAD algorithm

  // Triada S
  float A[3][3];
  triada(w1,w2,A);

  // Matriz R=A*B'
  float R[3][3];
  for(int i=0; i<3; i++){
   for (int j=0; j<3; j++){
    R[i][j] = A[i][0] * B[j][0] + A[i][1] * B[j][1] + A[i][2] * B[j][2];
   } 
  }
  
  yawpitchroll.yaw = atan2(R[0][1], R[0][0]);
  yawpitchroll.pitch = atan2(-R[0][2], sqrt(pow(R[1][2], 2) + pow(R[2][2], 2)));
  yawpitchroll.roll = atan2(R[1][2], R[2][2]);

  if (yawpitchroll.yaw < 0.0) {
    yawpitchroll.yaw = 2 * pi + yawpitchroll.yaw;
  }
}

void Triad::integrator(const float gyro[3], const Attitude &yawpitchroll, float delta_t, Attitude &yawpitchroll_integrator) {


  //  Serial.print(p,6);
  //  Serial.print("   ");
  //  Serial.print(q,6);
  //  Serial.print("   ");
  //  Serial.println(r,6);
//

  float qk0[4], qk1[4];
  float yaw_angle_cos = cos(yawpitchroll.yaw/2.0f);
  float roll_angle_cos = cos(yawpitchroll.roll/2.0f);
  float pitch_angle_cos = cos(yawpitchroll.pitch/2.0f);
  float yaw_angle_sin = sin(yawpitchroll.yaw/2.0f);
  float roll_angle_sin = sin(yawpitchroll.roll/2.0f);
  float pitch_angle_sin = sin(yawpitchroll.pitch/2.0f);
  
  qk0[0]    = roll_angle_cos*pitch_angle_cos*yaw_angle_cos + roll_angle_sin*pitch_angle_sin*yaw_angle_sin;
  qk0[1]    = roll_angle_sin*pitch_angle_cos*yaw_angle_cos - roll_angle_cos*pitch_angle_sin*yaw_angle_sin;
  qk0[2]    = roll_angle_cos*pitch_angle_sin*yaw_angle_cos + roll_angle_sin*pitch_angle_cos*yaw_angle_sin;
  qk0[3]    = roll_angle_cos*pitch_angle_cos*yaw_angle_sin - roll_angle_sin*pitch_angle_sin*yaw_angle_cos;

//  float OMEGA[4][4];
//
//  OMEGA[0][1] = -p/2.;
//  OMEGA[0][2] = -q/2.;
//  OMEGA[0][3] = -r/2.;
//
//  OMEGA[1][0] = p/2.;
//  OMEGA[1][2] = r/2.;
//  OMEGA[1][3] = q/2.;
//
//  OMEGA[2][0] = q/2.;
//  OMEGA[2][1] = -r/2.;
//  OMEGA[2][3] = p/2.;
//
//  OMEGA[3][0] = r/2.;
//  OMEGA[3][1] = q/2.;
//  OMEGA[3][2] = -p/2.;

  float omega_mod = vector_module(gyro);
  float omega_mod_delta = omega_mod * delta_t/2.0;
  float omega_mod_sin_mod= sin(omega_mod_delta)/omega_mod;
  float omega_mod_cos = cos(omega_mod_delta);
  
  qk1[0] = qk0[0]*omega_mod_cos - gyro[0] *qk0[1]*omega_mod_sin_mod  - gyro[1] *qk0[2]*omega_mod_sin_mod  - gyro[2] *qk0[3]*omega_mod_sin_mod;
  qk1[1] = qk0[1]*omega_mod_cos + gyro[0] *qk0[0]*omega_mod_sin_mod  + gyro[1] *qk0[3]*omega_mod_sin_mod  + gyro[2] *qk0[2]*omega_mod_sin_mod;
  qk1[2] = qk0[2]*omega_mod_cos + gyro[0] *qk0[3]*omega_mod_sin_mod  + gyro[1] *qk0[0]*omega_mod_sin_mod  - gyro[2] *qk0[1]*omega_mod_sin_mod;
  qk1[3] = qk0[3]*omega_mod_cos - gyro[0] *qk0[2]*omega_mod_sin_mod  + gyro[1] *qk0[1]*omega_mod_sin_mod  + gyro[2] *qk0[0]*omega_mod_sin_mod;
 
  //  Serial.print(yaw_angle,6);
  //  Serial.print("   ");
  //  Serial.print(pitch_angle,6);
  //  Serial.print("   ");
  //  Serial.println(roll_angle,6);

//  float yaw_dot =
//      (q * sin(roll_angle0) + r * cos(roll_angle0)) / cos(pitch_angle0);
//  float pitch_dot = q * cos(roll_angle0) - r * sin(roll_angle0);
//  float roll_dot =
//      p + (q * sin(roll_angle0) + r * cos(roll_angle0)) * tan(pitch_angle0);

  //  Serial.print(yaw_dot,6);
  //  Serial.print("   ");
  //  Serial.print(pitch_dot,6);
  //  Serial.print("   ");
  //  Serial.println(roll_dot,6);

//  float delta_yaw = yaw_dot * delta_t;
//  float delta_angle = pitch_dot * delta_t;
//  float delta_roll = +roll_dot * delta_t;

  //  Serial.print(delta_yaw,6);
  //  Serial.print("   ");
  //  Serial.print(delta_angle,6);
  //  Serial.print("   ");
  //  Serial.println(delta_roll,6);
//
//  float yaw_angle = yaw_angle0 + delta_yaw;
//  float pitch_angle = pitch_angle0 + delta_angle;
//  float roll_angle = roll_angle0 + delta_roll;
  
  yawpitchroll_integrator.yaw   = atan2(2*(qk1[1]*qk1[2] + qk1[0]*qk1[3]), pow(qk1[0], 2)  +  pow(qk1[1], 2)  -  pow(qk1[2], 2) -  pow(qk1[3], 2));
  yawpitchroll_integrator.pitch = - asin(2*(qk1[1]*qk1[3] - qk1[0]*qk1[2]));
  yawpitchroll_integrator.roll  = atan2(2*(qk1[2]*qk1[3] + qk1[0]*qk1[1]), pow(qk1[0], 2)  -  pow(qk1[1], 2)  -  pow(qk1[2], 2) +  pow(qk1[3], 2));
  
  if (yawpitchroll_integrator.yaw < 0.0) {
    yawpitchroll_integrator.yaw = 2 * pi + yawpitchroll_integrator.yaw;
  }
  
//  if (yaw_angle0 <= 2 * pi && yaw_angle > 2 * pi) {
//    yaw_angle = yaw_angle - 2 * pi;
//  }
//  else if (yaw_angle0 > 0.0 && yaw_angle <= 0.0) {
//    yaw_angle = 2 * pi + yaw_angle;
//  }

}

void Triad::filter(const Attitude yawpitchroll_triad,
              const Attitude &yawpitchroll_int, Attitude &yawpitchroll) {

  if (yawpitchroll_triad.yaw > pi && yawpitchroll_int.yaw < pi) {
    yawpitchroll.yaw = yawpitchroll_triad.yaw;
  }
  else if (yawpitchroll_triad.yaw < pi && yawpitchroll_int.yaw > pi) {
    yawpitchroll.yaw = yawpitchroll_triad.yaw;
  }
  else {
    yawpitchroll.yaw = 0.1 * yawpitchroll_triad.yaw + 0.9 * yawpitchroll_int.yaw;
  }

  //  yawpitchroll.yaw=0.95*yawpitchroll_triad[0]+0.05*yawpitchroll_int[0];
    yawpitchroll.pitch=0.1*yawpitchroll_triad.pitch+0.9*yawpitchroll_int.pitch;
    yawpitchroll.roll=0.1*yawpitchroll_triad.roll+0.9*yawpitchroll_int.roll;

}
