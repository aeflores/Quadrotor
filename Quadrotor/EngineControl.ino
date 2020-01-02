
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


void EngineControl::computeReference(const int control[5]){
    // Calculating the pitch and roll references from the joystick data
    reference.pitch = (-float(control[3])+515.)/1024.*30.;
    reference.roll = (-float(control[2])+502.)/1024.*30.;
    // Calculating the height rate and the roll variation rate references from data
    reference.altitude_rate = (-float(control[1])+518.)/1024.*20.; // º/s
    reference.yaw_rate = (-float(control[0])+518.)/1024.*10.; // cm/s
}

int EngineControl::altitudeRate2Power(float altitude_rate){
  return altitude_rate * 10 + 1000;
}

void EngineControl::proportionalControl(const int control[5], const Attitude &yawpitchroll){
    computeReference(control);
    error_pitch=yawpitchroll.pitch*radtodeg - reference.pitch;
    error_roll=yawpitchroll.roll*radtodeg - reference.roll;
    power= altitudeRate2Power(reference.altitude_rate);
    engine_speed[0]= power + error_pitch + error_roll;
    engine_speed[1]= power + error_pitch + error_roll;
    engine_speed[2]= power + error_pitch + error_roll;
    engine_speed[3]= power + error_pitch + error_roll;
}

void EngineControl::stop(){
     for(int i=0;i<4;i++){
         engine_speed[i]=MIN_SPEED;
     }
}
