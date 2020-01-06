
#include "EngineControl.h"


void EngineControl::init(){
    for(int i=0;i<4;i++){
        engine[i].attach(engine_port[i]);
        engine[i].writeMicroseconds(engine_speed[i]);
    }
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
  upperUnbalanceRange=conf.upperUnbalanceRange;
  lowerUnbalanceRange=conf.lowerUnbalanceRange;
  error2CorrectionCoeff=conf.error2CorrectionCoeff;
}

void EngineControl::computeReference(const int control[5]){
    // Calculating the pitch and roll references from the joystick data
    reference.pitch = -(-float(control[3])+515.)/1024.*30.;
    reference.roll = (-float(control[2])+502.)/1024.*30.;
    // Calculating the height rate and the roll variation rate references from data
    //reference.altitude_rate = (-float(control[1])+518.)/1024.*20.; // º/s
    reference.altitude_rate = control[1]; // for testing
    reference.yaw_rate = (-float(control[0])+518.)/1024.*10.; // cm/s
}

int EngineControl::altitudeRate2Power(float altitude_rate){
  int power= altitude_rate * alt2powerCoeff + alt2powerBase;
  if (power > alt2powerBase + powerRange)
    return alt2powerBase + powerRange;
  if (power < alt2powerBase - powerRange)
    return alt2powerBase - powerRange;
  return power;
}

int EngineControl::error2Correction(float error){
  float unbalance=error * error2CorrectionCoeff;
  if (unbalance > upperUnbalanceRange)
    return upperUnbalanceRange;
  if (unbalance < -lowerUnbalanceRange)
    return -lowerUnbalanceRange;
  return unbalance;
}

void EngineControl::proportionalControl(const int control[4], const Attitude &yawpitchroll_deg){
    computeReference(control);
    error_pitch= reference.pitch - yawpitchroll_deg.pitch;
    error_roll= reference.roll - yawpitchroll_deg.roll;
    int unbalance_pitch=error2Correction(error_pitch);
    int unbalance_roll=error2Correction(error_roll);
    power= altitudeRate2Power(reference.altitude_rate);

    engine_speed[0]= power - unbalance_pitch - unbalance_roll;
    engine_speed[1]= power - unbalance_pitch + unbalance_roll;
    engine_speed[2]= power + unbalance_pitch - unbalance_roll;
    engine_speed[3]= power + unbalance_pitch + unbalance_roll;
}

void EngineControl::stop(){
     for(int i=0;i<4;i++){
         engine_speed[i]=MIN_SPEED;
     }
}
