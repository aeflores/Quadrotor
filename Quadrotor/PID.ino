#include "PID.h"



void PID::init(float Kp, float Kd, float Ki) {
  parameters.Kp = Kp;
  parameters.Kd = Kd;
  parameters.Ki = Ki;
  parameters.saturation = 50;
  parameters.alpha      = 0.3;
}

void PID :: get_output(float error, float derivative, float integral, PIDparameters& parameters, float output) {
  output = parameters.Kp * error + parameters.Kd * derivative + parameters.Ki * integral;
}

void PID :: pid_step(const float state, const float reference, float output, int delta_t) {

  error = parameters.alpha*(reference - state) + (1 - parameters.alpha)*error_n_1;
  derivative = (error - error_n_1)*dt*parameters.alpha + derivative_n_1*(1 - parameters.alpha);
  Serial.print(reference - state);
  Serial.print("\t");
  Serial.print((error - error_n_1)*dt);
  Serial.print("\t");
  error_n_1 = error;
  derivative_n_1 = derivative;

  if (!integral_saturation) {
    integral = integral + error*delta_t*1e-6;
  }
  if (abs(integral) > parameters.saturation && integral>=0){
    integral= parameters.saturation;
  }
  else if (abs(integral) > parameters.saturation && integral<0){
    integral = -parameters.saturation;
  }

  Serial.print(error);
  Serial.print("\t");
  Serial.print(derivative);
  Serial.print("\t");
  Serial.print(integral);
  Serial.println("\t");

}

//  derivative = (error - error_old)/delta_t/1e-6;
//  Serial.print(derivative);
//  Serial.print("\t");
//  derivative = (3*error/2.0 - 2*error_1 + error_2/2.0)/delta_t*1e6;
//  Serial.print(derivative);
//  Serial.print("\t");
//  derivative = (25./12.*error - 4.*error_1 + 3.*error_2 - 4./3. * error_3 + error_4/4.)/delta_t*1e6;
//  
//  error_4 = error_3;
//  error_3 = error_2;
//  error_2 = error_1;
