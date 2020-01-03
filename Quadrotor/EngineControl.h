

#ifndef ENGINE_CONTROL_H_
#define ENGINE_CONTROL_H_

#include <Servo.h> 
#include "Triad.h"

const int MIN_SPEED=1000;
const int MAX_SPEED=2000;

struct ControlReference
{
    float pitch, roll, yaw_rate, altitude_rate;
};

class EngineControl{
private:
    const short engine_port[4]={3,9,5,6};
    Servo engine[4];
    float alt2powerCoeff=1;
    float alt2powerBase=1000;
    float powerRange=1000;
    //float powerRange=300;
    float error2CorrectionCoeff=10;
    float unbalanceRange=100;

    void computeReference(const int control[5]);
    int altitudeRate2Power(float altitude_rate);
    int error2Correction(float error);
public:
    int power;
    float error_pitch, error_roll;
    int engine_speed[4]={MIN_SPEED,MIN_SPEED,MIN_SPEED,MIN_SPEED};
    ControlReference reference;
    void init();
    void updateEngines();
    void testControl(const int control[5]);
    void proportionalControl(const int control[5],const Attitude &yawpitchroll);
    void stop();

};

#endif /* ENGINE_CONTROL_H_*/
