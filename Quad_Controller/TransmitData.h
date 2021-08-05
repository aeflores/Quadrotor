



#ifndef TRANSMIT_DATA_H_
#define TRANSMIT_DATA_H_


enum State { STANDBY = 0, CALIBRATION = 1, FLYMODE = 2, ABORT = 3 };
const int NUM_STATES = 4;

enum StateChange{ NO = 0, NEXT = 1, PREV = 2};

struct ControlData
{
  int movement[5];
  StateChange change;
  bool moreData;
};

struct TransmitData
{
    State state;
    int delta_t;
    int quadstate[4];
    int errors[2];
    int engines[4];


    void print(HardwareSerial& Serial){
       switch (state) {
        case STANDBY:
          Serial.print("STANDBY ");
          break;
        case CALIBRATION:
          Serial.print("CALIBRATION ");
          break;
        case FLYMODE:
          Serial.print("FLYMODE ");
          break;
        case ABORT:
          Serial.print("ABORT ");
          break;
        }
        Serial.print("\t"); 
        Serial.print(" Yawrate= ");
        // Serial.print("\t"); 
        Serial.print(float(quadstate[0])/100., 2);
        Serial.print("\t"); 
        Serial.print(" Pitch= ");
        // Serial.print("\t"); 
        Serial.print(float(quadstate[1])/100., 2);
        Serial.print("\t"); 
        Serial.print(" Roll= ");
        // Serial.print("\t"); 
        Serial.print(float(quadstate[2])/100., 2);
        Serial.print("\t");
        Serial.print(" Height= ");
        Serial.print(float(quadstate[3])/100., 2);
        Serial.print("\t"); 
        Serial.print(" Error pitch= ");
        // Serial.print("\t"); 
        Serial.print(float(errors[0])/100., 2);
        Serial.print("\t"); 
        Serial.print(" Error roll= ");
        // Serial.print("\t"); 
        Serial.print(float(errors[1])/100., 2);
        Serial.print("\t"); 

        for(int i=0; i<4; i++){
            Serial.print(" Eng");
            Serial.print(i+1);
            Serial.print("= "); 
            Serial.print(engines[i]);
            Serial.print("\t");
        }
        Serial.print("Delta T=");
        // Serial.print("\t"); 
        Serial.print(delta_t);
    }
};

struct ControllerConfiguration{
    float PIDattitude_Kp = 2;
    float PIDattitude_Kd = 0.5;
    float PIDattitude_Ki = 0.0;

    float PIDheight_Kp = 2;
    float PIDheight_Kd = 0.5;
    float PIDheight_Ki = 2;
    float PIDheight_error_fc = 1;
    float PIDheight_error_dot_fc =  1;
    float PIDheight_saturation = 700;

    void print(HardwareSerial& Serial){
        Serial.print("\t"); 
        Serial.print(" PIDattitudeKp= ");
        Serial.print("\t"); 
        Serial.print(PIDattitude_Kp, 2);
        Serial.print("\t"); 
        Serial.print(" PIDattitudeKd= ");
        Serial.print("\t"); 
        Serial.print(PIDattitude_Kd, 2);
        Serial.print("\t");
        Serial.print(" PIDattitudeKi= ");
        Serial.print("\t"); 
        Serial.print(PIDattitude_Ki, 2);
        Serial.print("\t");

        Serial.print("\t"); 
        Serial.print(" PIDheightKp= ");
        Serial.print("\t"); 
        Serial.print(PIDheight_Kp, 2);
        Serial.print("\t"); 
        Serial.print(" PIDheightKd= ");
        Serial.print("\t"); 
        Serial.print(PIDheight_Kd, 2);
        Serial.print("\t");
        Serial.print(" PIDheightKi= ");
        Serial.print("\t"); 
        Serial.print(PIDheight_Ki, 2);
        Serial.print("\t");
        Serial.print(" PIDheight_error_fc= ");
        Serial.print("\t"); 
        Serial.print(PIDheight_error_fc, 2);
        Serial.print("\t");
        Serial.print(" PIDheight_error_dot_fc= ");
        Serial.print("\t"); 
        Serial.print(PIDheight_error_dot_fc, 2);
        Serial.print("\t");
        Serial.print(" PIDheight_saturation= ");
        Serial.print("\t"); 
        Serial.print(PIDheight_saturation, 2);
        Serial.print("\t");
    }



    
//    float error2CorrectionCoeff=0;
//    float derivativeError2CorrectionCoeff=0;
//    float upperUnbalanceRange=100;
//    float lowerUnbalanceRange=100;
//    float feedforwardunbalance14 = 0;
//    float feedforwardunbalance23 = 0;
};


#endif
