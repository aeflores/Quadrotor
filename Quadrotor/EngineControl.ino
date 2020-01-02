
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

void EngineControl::testControl(int control[5]){
    for(int i=0;i<4;i++){
        engine_speed[i]=(control[i]-512)*2+ 1000;
        if (engine_speed[i]<MIN_SPEED)
            engine_speed[i]=MIN_SPEED;
        if (engine_speed[i]>MAX_SPEED)
            engine_speed[i]=MAX_SPEED;
    }
}

void EngineControl::stop(){
     for(int i=0;i<4;i++){
         engine_speed[i]=MIN_SPEED;
     }
}