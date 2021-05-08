

#ifndef ENGINE_CONTROL_H_
#define ENGINE_CONTROL_H_

#include <Servo.h> 
#include "Attitude.h"
#include "TransmitData.h"
#include "PID.h"

const int MIN_SPEED=1000;
const int MAX_SPEED=2000;

struct QuadState
{
    float pitch, roll, yaw_rate, height;
};

class EngineControl{
private:
    const short engine_port[4]={6,3,9,5};
    Servo engine[4];
    float alt2powerCoeff=1;
    float alt2powerBase=1000;
    float powerRange=1000;
    //float powerRange=300;

    void computeReference(const int control[5]);
    int altitudeRate2Power(float altitude_rate);
    int error2Correction(float error, float derivative_error);
public:
    float error2CorrectionCoeff           = 5;
    float derivativeError2CorrectionCoeff = 0;
    float upperUnbalanceRange             = 100;
    float lowerUnbalanceRange             = 100;
    float feedforwardunbalance14          = 0;
    float feedforwardunbalance23          = 0;

    int power;
    float error_pitch, error_roll, derivative_error_pitch, derivative_error_roll;
    int engine_speed[4]={MIN_SPEED,MIN_SPEED,MIN_SPEED,MIN_SPEED};
    void init();
    void updateEngines();
    void testControl(const int control[5]);
    void configure(ControllerConfiguration &conf);
    void pdControl(const int control[4], const Euler &yawpitchroll, const QuadState &state, const int delta_t);
    void stop();
    PID pitchPID, rollPID, yawratePID, heightPID;
    QuadState reference;

};

#endif /* ENGINE_CONTROL_H_*/
