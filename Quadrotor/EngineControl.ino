#include "EngineControl.h"


void EngineControl::init(){
    for(int i=0;i<4;i++){
        engine[i].attach(engine_port[i]);
        engine[i].writeMicroseconds(engine_speed[i]);
    }
    pitchPID.init(5.0, 0.5, 0.0, 100);
    rollPID.init(5.0, 0.5, 0.0, 100);
    yawratePID.init(5.0, 0.0, 0., 20);
    heightPID.init(1.0, 0.1, 0.3, 700);

}

void EngineControl::updateEngines(){
    for(int i=0;i<4;i++){
    engine[i].writeMicroseconds(engine_speed[i]);
    }
}

void EngineControl::testControl(const int control[5]){
    for(int i=0;i<4;i++){
        engine_speed[i]=(control[i]-512)*2+ 1000;
        if (engine_speed[i]<MIN_SPEED)
            engine_speed[i]=MIN_SPEED;
        if (engine_speed[i]>MAX_SPEED)
            engine_speed[i]=MAX_SPEED;
    }
}

void EngineControl::configure(ControllerConfiguration &conf){
  upperUnbalanceRange             = conf.upperUnbalanceRange;
  lowerUnbalanceRange             = conf.lowerUnbalanceRange;
  error2CorrectionCoeff           = conf.error2CorrectionCoeff;
  derivativeError2CorrectionCoeff = conf.derivativeError2CorrectionCoeff;
  feedforwardunbalance14          = conf.feedforwardunbalance14;
  feedforwardunbalance23          = conf.feedforwardunbalance23;
}

void EngineControl::computeReference(const int control[5]){
    // Calculating the pitch and roll references from the joystick data
    reference.pitch = -(-float(control[3])+515.)/1024.*20.;
    reference.roll = (-float(control[2])+502.)/1024.*20.;
    // Calculating the height rate and the roll variation rate references from data
    //reference.altitude_rate = (-float(control[1])+518.)/1024.*20.; // º/s
    reference.height = control[1]; // for testing
    reference.yaw_rate = (-float(control[0])+518.)/1024.*10.; // cm/s
    
//    Serial.print(reference.pitch);
//    Serial.print("\t");
//    Serial.print(reference.roll);
//    Serial.print("\t");
//    Serial.print(reference.yaw_rate);
//    Serial.print("\t");
//    Serial.print(reference.height);
//    Serial.println("\t");
}

int EngineControl::altitudeRate2Power(float altitude_rate){
  int power= altitude_rate * alt2powerCoeff + alt2powerBase;
  if (power > alt2powerBase + powerRange)
    return alt2powerBase + powerRange;
  if (power < alt2powerBase - powerRange)
    return alt2powerBase - powerRange;
  return power;
}

int EngineControl::error2Correction(float error, float derivative_error){
  
  float unbalance=error * error2CorrectionCoeff + derivative_error * derivativeError2CorrectionCoeff;
  if (unbalance > upperUnbalanceRange)
    return upperUnbalanceRange;
  if (unbalance < -lowerUnbalanceRange)
    return -lowerUnbalanceRange;
  return unbalance;
}


void EngineControl::pdControl(const int control[4], const Euler &yawpitchroll, const QuadState &state, const int delta_t){
    computeReference(control);
    float pitchinput, rollinput, yaw_rateinput, heightinput;
    
    float unbalance_pitch    = pitchPID.pid_step(state.pitch, reference.pitch, delta_t);
    float unbalance_roll     = rollPID.pid_step(state.roll, reference.roll, delta_t);
    float unbalance_yaw_rate = yawratePID.pid_step(state.yaw_rate, reference.yaw_rate, delta_t);
    float power              = 1000 +  heightPID.pid_step(state.height, reference.height, delta_t);
    
    // power= altitudeRate2Power(reference.height);

    engine_speed[0]= max(int(power + unbalance_pitch - unbalance_roll + feedforwardunbalance14), 1000);
    engine_speed[1]= max(int(power - unbalance_pitch - unbalance_roll + feedforwardunbalance23), 1000);
    engine_speed[2]= max(int(power - unbalance_pitch + unbalance_roll - feedforwardunbalance23), 1000);
    engine_speed[3]= max(int(power + unbalance_pitch + unbalance_roll - feedforwardunbalance14), 1000);
    Serial.print(unbalance_pitch);
    Serial.print("\t");
    Serial.print(unbalance_roll);
    Serial.print("\t");
    Serial.print(unbalance_yaw_rate);
    Serial.print("\t");
    Serial.print(power-1000);
    Serial.println("\t");


//    
//    float old_error_pitch, old_error_roll, old_derivative_error_pitch, old_derivative_error_roll;
//    // save old values
//    old_error_pitch = error_pitch;
//    old_error_roll = error_roll;
//
//    old_derivative_error_pitch = derivative_error_pitch;
//    old_derivative_error_roll = derivative_error_roll;   
//
//    float derivative_pitch_filter_coeff = 0.2;
//    float derivative_roll_filter_coeff = 0.2;
//
//    float pitch_filter_coeff = 0.8;
//    float roll_filter_coeff = 0.05;
//    
//    // compute new values
//    error_pitch = reference.pitch - yawpitchroll.pitch_deg();
//    error_roll = reference.roll - yawpitchroll.roll_deg();
//    // Filtering errors
//    error_pitch = pitch_filter_coeff*error_pitch + (1-pitch_filter_coeff)*old_error_pitch;
//    error_roll = roll_filter_coeff*error_roll + (1-roll_filter_coeff)*old_error_roll;
//    
//    // compute derivative
//    derivative_error_pitch = (error_pitch - old_error_pitch) *1000000 / delta_t;
//    derivative_error_roll = (error_roll - old_error_roll) *1000000 / delta_t;
//    // Filtering error derivatives
//    derivative_error_pitch = derivative_pitch_filter_coeff *derivative_error_pitch + (1 - derivative_pitch_filter_coeff)*old_derivative_error_pitch;
//    derivative_error_roll = derivative_roll_filter_coeff *derivative_error_roll + (1 - derivative_roll_filter_coeff)*old_derivative_error_roll;
//    // compute power based on error
//    int unbalance_pitch=error2Correction(error_pitch, derivative_error_pitch);
//    int unbalance_roll=error2Correction(error_roll, derivative_error_roll);

}

void EngineControl::stop(){
     for(int i=0;i<4;i++){
         engine_speed[i]=MIN_SPEED;
     }
}
