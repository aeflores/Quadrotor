

#ifndef ENGINE_CONTROL_H_
#define ENGINE_CONTROL_H_

#include <Servo.h> 
#include "Triad.h"
#include "TransmitData.h"

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

    void computeReference(const int control[5]);
    int altitudeRate2Power(float altitude_rate);
    int error2Correction(float error, float derivative_error);
public:
    float error2CorrectionCoeff = 5;
    float derivativeError2CorrectionCoeff = 0;
    float upperUnbalanceRange=50;
    float lowerUnbalanceRange=20;
    
    float feedforwardunbalance14 = 0;
    float feedforwardunbalance23 = 10;

    int power;
    float error_pitch, error_roll, derivative_error_pitch, derivative_error_roll;
    int engine_speed[4]={MIN_SPEED,MIN_SPEED,MIN_SPEED,MIN_SPEED};
    ControlReference reference;
    void init();
    void updateEngines();
    void testControl(const int control[5]);
    void configure(ControllerConfiguration &conf);
    void pdControl(const int control[4], const Attitude &yawpitchroll, const int delta_t);
    void stop();

};

#endif /* ENGINE_CONTROL_H_*/
