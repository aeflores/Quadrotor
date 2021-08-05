#include "PID.h"


//float sign(const float value) {
//  return ((value > 0) - (value < 0));
//}


void PID::init(float Kp, float Kd, float Ki, float error_fc, float error_dot_fc, float saturation) {
  parameters.Kp = Kp;
  parameters.Kd = Kd;
  parameters.Ki = Ki;
  parameters.error_fc = error_fc;
  parameters.error_dot_fc = error_dot_fc;
  parameters.saturation = saturation;
  integral = 0;
}


float PID :: pid_step(const float state, const float reference, const int delta_t) {
  float output;
  float dt = (float)delta_t*1e-6;

  
  float RC_error = 1/(2*pi*parameters.error_fc);
  float RC_error_dot = 1/(2*pi*parameters.error_dot_fc);


  
  error = error_n_1 +  ((dt/(RC_error + dt))*((reference - state) - error_n_1));
  //error = parameters.alpha*(reference - state) + (1 - parameters.alpha)*error_n_1;
  derivative = derivative_n_1 + ((dt/(RC_error_dot + dt))*((error - error_n_1)/dt - derivative_n_1));
  
  
  
  //derivative = (error - error_n_1)/dt*parameters.alpha + derivative_n_1*(1 - parameters.alpha);
  
  error_n_1 = error;
  derivative_n_1 = derivative;


  if (!integral_saturation) {
    integral += error*dt;
  }

  float controlsignal = parameters.Kp*error  +  parameters.Kd*derivative  +  parameters.Ki*integral;

  if (abs(controlsignal)>=parameters.saturation){
    integral_saturation = true;
    // integral = 0;
    output = sign(controlsignal)*parameters.saturation;
  }
  else{
    output = controlsignal;
    integral_saturation = false;
  }

//  Serial.print(output);
//  Serial.print("\t");
//  Serial.print(integral_saturation);
//  Serial.print("\t");

  return output;

  


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
