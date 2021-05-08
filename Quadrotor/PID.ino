#include "PID.h"


//float sign(const float value) {
//  return ((value > 0) - (value < 0));
//}


void PID::init(float Kp, float Kd, float Ki, float saturation) {
  parameters.Kp = Kp;
  parameters.Kd = Kd;
  parameters.Ki = Ki;
  parameters.saturation = saturation;
  parameters.alpha      = 0.3;
}


float PID :: pid_step(const float state, const float reference, const int delta_t) {
  float output;
  error = parameters.alpha*(reference - state) + (1 - parameters.alpha)*error_n_1;
  derivative = (error - error_n_1)*dt*parameters.alpha + derivative_n_1*(1 - parameters.alpha);
  error_n_1 = error;
  derivative_n_1 = derivative;

  if (!integral_saturation) {
    integral += error*delta_t*1e-6;
  }

  float controlsignal = parameters.Kp*error  +  parameters.Kd*derivative  +  parameters.Ki*integral;

  if (abs(controlsignal)>=parameters.saturation){
    integral_saturation = true;
    output = sign(controlsignal)*parameters.saturation;
  }
  else{
    output = controlsignal;
  }

  return output;
//  Serial.print(output);
//  Serial.print("\t");
  


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
