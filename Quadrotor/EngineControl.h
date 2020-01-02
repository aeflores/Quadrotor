

#ifndef ENGINE_CONTROL_H_
#define ENGINE_CONTROL_H_

#include <Servo.h> 

const int MIN_SPEED=1000;
const int MAX_SPEED=2000;

class EngineControl{
private:
    const short engine_port[4]={3,9,5,6};
    int engine_speed[4]={MIN_SPEED,MIN_SPEED,MIN_SPEED,MIN_SPEED};
    Servo engine[4];
public:
    void init();
    void updateEngines();
    void testControl(int control[5]);
    void stop();

};

#endif /* ENGINE_CONTROL_H_*/